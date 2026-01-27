// spsc_queue.h
// Single Producer Single Consumer (SPSC) lock-free queue
//
// A fixed-size ring buffer optimized for single producer / single consumer
// scenarios. Uses C++20 atomic wait/notify for efficient blocking waits.
//
// Thread Safety:
//   - Producer thread: begin_write(), end_write()
//   - Consumer thread: wait_for_item(), get_latest(), get_sequence(), get_item_count()
//   - Any thread: signal_exit()
//   - Any thread (when no threads are blocked in wait_for_item()): reset_exit_signal()
//
// Memory Model:
//   - Uses acquire/release semantics for correct synchronization
//   - Cache-line aligned atomics to prevent false sharing
//
// Pointer Lifetime:
//   Pointers returned by get_latest() and wait_for_item() are valid only until
//   the producer wraps around and overwrites that slot. Because the queue uses
//   a fixed ring buffer of N slots, a pointer remains valid through N-1
//   subsequent end_write() calls. After N writes, the slot is overwritten.
//   Consumers should copy data out promptly and not retain pointers.

#ifndef spsc_queue_h
#define spsc_queue_h

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace imujoco {

/// Single Producer Single Consumer lock-free queue.
///
/// @tparam T The element type stored in the queue
/// @tparam N The capacity of the queue (number of slots)
///
/// This queue uses a ring buffer with triple buffering by default (N=3).
/// The producer writes to one slot while the consumer reads from another,
/// with a third slot available to absorb timing variations.
///
/// @warning Pointers returned by get_latest() and wait_for_item() point into
/// the internal ring buffer. They remain valid through N-1 subsequent writes;
/// after N writes, the slot is overwritten. Consumers must copy data out
/// promptly and not cache these pointers.
///
/// Example usage:
/// @code
/// SpscQueue<FrameData, 3> queue;
///
/// // Producer thread
/// auto* slot = queue.begin_write();
/// slot->data = compute_data();
/// queue.end_write();
///
/// // Consumer thread
/// uint64_t last_item_count = 0;
/// auto* item = queue.wait_for_item(last_item_count);
/// if (item) {
///     last_item_count = queue.get_item_count();  // Update for next wait
///     FrameData copy = *item;  // Copy out immediately
///     process(copy);
/// }
/// @endcode
template <typename T, std::size_t N = 3>
class SpscQueue {
public:
    static_assert(N >= 2, "Queue must have at least 2 slots");

    SpscQueue()
        : write_index_(0),
          read_index_(0),
          sequence_(0),
          item_count_(0),
          exit_signaled_(false) {}

    // Non-copyable, non-movable
    SpscQueue(const SpscQueue&) = delete;
    SpscQueue& operator=(const SpscQueue&) = delete;
    SpscQueue(SpscQueue&&) = delete;
    SpscQueue& operator=(SpscQueue&&) = delete;

    // =========================================================================
    // Producer Interface (single thread only)
    // =========================================================================

    /// Begin writing to the next slot.
    /// @return Pointer to the slot to write to
    /// @note Must be followed by end_write() after writing is complete
    T* begin_write() {
        return &buffers_[write_index_.load(std::memory_order_relaxed)];
    }

    /// Complete writing and publish the slot to consumers.
    /// @note Makes the written data visible to the consumer thread
    void end_write() {
        std::size_t idx = write_index_.load(std::memory_order_relaxed);
        // Publish the written slot to consumers
        read_index_.store(idx, std::memory_order_release);
        // Advance to next write slot
        write_index_.store((idx + 1) % N, std::memory_order_relaxed);
        // Increment sequence (for legacy API compatibility)
        sequence_.fetch_add(1, std::memory_order_release);
        // Increment item count and notify waiting consumers.
        // We wait/notify on item_count_ (not sequence_) to ensure the
        // notification is tied to the predicate checked in wait_for_item().
        item_count_.fetch_add(1, std::memory_order_release);
        item_count_.notify_one();
    }

    // =========================================================================
    // Consumer Interface (single thread only)
    // =========================================================================

    /// Wait for a new item to be available.
    /// @param last_item_count The item count from the last successful read (from get_item_count())
    /// @return Pointer to the latest item, or nullptr if exit was signaled
    /// @note Blocks until item_count > last_item_count or exit is signaled.
    ///       The returned pointer is valid only until the producer wraps around
    ///       and overwrites this slot. Copy data out immediately.
    const T* wait_for_item(uint64_t last_item_count) {
        // Loop to handle spurious wakeups per C++ spec
        for (;;) {
            // Check exit first to avoid unnecessary waiting
            if (exit_signaled_.load(std::memory_order_acquire)) {
                return nullptr;
            }
            // Check if a NEW item has been written since last_item_count.
            uint64_t current_items = item_count_.load(std::memory_order_acquire);
            if (current_items > last_item_count) {
                return &buffers_[read_index_.load(std::memory_order_acquire)];
            }
            // Wait on item_count_ directly to avoid missed-wakeup issues.
            // We wait/notify on the same atomic as our predicate check.
            item_count_.wait(current_items, std::memory_order_acquire);
        }
    }

    /// Get the latest available item without blocking.
    /// @return Pointer to the latest item, or nullptr if no items written yet.
    ///         The returned pointer is valid only until the producer wraps around
    ///         and overwrites this slot. Copy data out immediately.
    /// @note Returns nullptr only when no items have been written via end_write().
    ///       This check is independent of signal_exit() which only affects sequence_.
    const T* get_latest() const {
        if (item_count_.load(std::memory_order_acquire) == 0) {
            return nullptr;
        }
        return &buffers_[read_index_.load(std::memory_order_acquire)];
    }

    /// Get the current synchronization sequence number.
    /// @return Monotonically increasing value used internally to wake waiters.
    ///         Increments on each end_write() and on signal_exit() to ensure
    ///         threads blocked in wait_for_item() are notified.
    /// @note This is a synchronization sequence used for sleeping/waking
    ///       consumers (including exit notifications), not a pure "items
    ///       written" counter. For detecting new data availability, callers
    ///       should compare get_item_count() against their own last_item_count.
    uint64_t get_sequence() const {
        return sequence_.load(std::memory_order_acquire);
    }

    /// Get the number of items written.
    /// @return Number of completed end_write() calls since construction.
    /// @note Consumers should use this together with a caller-maintained
    ///       last_item_count value to detect new data between calls to
    ///       wait_for_item(). Unlike get_sequence(), this is not incremented
    ///       by signal_exit().
    uint64_t get_item_count() const {
        return item_count_.load(std::memory_order_acquire);
    }

    // =========================================================================
    // Lifecycle Control (thread-safe)
    // =========================================================================

    /// Signal consumers to exit.
    /// Wakes up any threads blocked in wait_for_item().
    /// @note Notifies on item_count_ to wake waiters (does not increment item_count_).
    void signal_exit() {
        bool expected = false;
        if (exit_signaled_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            // Increment sequence for legacy API compatibility
            sequence_.fetch_add(1, std::memory_order_release);
            // Notify on item_count_ to wake consumers blocked in wait_for_item()
            item_count_.notify_all();
        }
    }

    /// Reset the exit signal for reuse.
    /// @note Only call when no threads are waiting
    void reset_exit_signal() {
        exit_signaled_.store(false, std::memory_order_release);
    }

    /// Check if exit has been signaled.
    bool is_exit_signaled() const {
        return exit_signaled_.load(std::memory_order_acquire);
    }

    // =========================================================================
    // Legacy API (for compatibility during migration)
    // =========================================================================

    /// Legacy alias for get_sequence().
    [[deprecated("Use get_sequence() instead")]]
    uint64_t get_count() const { return get_sequence(); }

private:
    std::array<T, N> buffers_;

    // Cache-line aligned to prevent false sharing between producer and consumer
    alignas(64) std::atomic<std::size_t> write_index_;
    alignas(64) std::atomic<std::size_t> read_index_;
    alignas(64) std::atomic<uint64_t> sequence_;
    // Tracks actual items written (not incremented by signal_exit)
    alignas(64) std::atomic<uint64_t> item_count_;

    std::atomic<bool> exit_signaled_;
};

}  // namespace imujoco

#endif  // spsc_queue_h
