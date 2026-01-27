// spsc_queue.h
// Single Producer Single Consumer (SPSC) lock-free queue with multi-reader support
//
// A fixed-size ring buffer optimized for single producer scenarios. While named
// "SPSC", multiple reader threads may call wait_for_item() concurrently if each
// maintains its own last_item_count state. All readers observe the same latest
// item (this is NOT a work-stealing queue). Uses C++20 atomic wait/notify.
//
// Thread Safety:
//   - Producer thread (single): begin_write(), end_write()
//   - Consumer thread(s): wait_for_item(), get_latest(), get_sequence(), get_item_count()
//     Multiple threads may call wait_for_item() concurrently if each maintains its own
//     last_item_count state. All concurrent readers will receive the same latest item.
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

/// Single Producer lock-free queue with multi-reader support.
///
/// @tparam T The element type stored in the queue
/// @tparam N The capacity of the queue (number of slots)
///
/// This queue uses a ring buffer with triple buffering by default (N=3).
/// Multiple reader threads may call wait_for_item() concurrently if each
/// maintains its own last_item_count state. All readers observe the same
/// latest item (this is NOT a work-stealing queue).
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
        // Advance to next write slot
        std::size_t idx = write_index_.load(std::memory_order_relaxed);
        write_index_.store((idx + 1) % N, std::memory_order_relaxed);
        // Increment item count (tracks actual writes).
        // Consumers derive the read index from item_count_ to ensure consistency:
        // latest_index = (item_count - 1) % N
        item_count_.fetch_add(1, std::memory_order_release);
        // Increment sequence and notify waiting consumers.
        // We wait/notify on sequence_ because it's modified by BOTH end_write()
        // and signal_exit(), avoiding missed-wakeup deadlocks.
        // Use notify_all() to support wrappers that may have multiple waiters.
        sequence_.fetch_add(1, std::memory_order_release);
        sequence_.notify_all();
    }

    // =========================================================================
    // Consumer Interface (multiple readers supported with independent state)
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
                // Derive read index from item_count to ensure consistency.
                // This guarantees the returned pointer corresponds to the observed write.
                std::size_t read_idx = static_cast<std::size_t>((current_items - 1) % N);
                return &buffers_[read_idx];
            }
            // Wait on sequence_ (not item_count_) because sequence_ is modified
            // by BOTH end_write() and signal_exit(). This avoids missed-wakeup
            // deadlocks where signal_exit() notifies between our predicate check
            // and the wait call.
            uint64_t current_seq = sequence_.load(std::memory_order_acquire);
            sequence_.wait(current_seq, std::memory_order_acquire);
        }
    }

    /// Get the latest available item without blocking.
    /// @return Pointer to the latest item, or nullptr if no items written yet.
    ///         The returned pointer is valid only until the producer wraps around
    ///         and overwrites this slot. Copy data out immediately.
    /// @note Returns nullptr only when no items have been written via end_write().
    ///       This check is independent of signal_exit() which only affects sequence_.
    const T* get_latest() const {
        uint64_t count = item_count_.load(std::memory_order_acquire);
        if (count == 0) {
            return nullptr;
        }
        // Derive read index from item_count to ensure consistency
        std::size_t read_idx = static_cast<std::size_t>((count - 1) % N);
        return &buffers_[read_idx];
    }

    /// Get the current synchronization sequence number.
    /// @return Monotonically increasing value incremented on end_write() and signal_exit().
    /// @note This is the atomic used for wait/notify in wait_for_item(). It increments
    ///       on both writes and exit signals to ensure waiters are always woken.
    ///       For detecting new data availability, compare get_item_count() against
    ///       your own last_item_count value.
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
    /// @note Increments and notifies on sequence_ to wake waiters.
    void signal_exit() {
        bool expected = false;
        if (exit_signaled_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            // Increment sequence and notify to wake consumers blocked in wait_for_item()
            sequence_.fetch_add(1, std::memory_order_release);
            sequence_.notify_all();
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
    alignas(64) std::atomic<uint64_t> sequence_;
    // Tracks actual items written (not incremented by signal_exit)
    alignas(64) std::atomic<uint64_t> item_count_;

    alignas(64) std::atomic<bool> exit_signaled_;
};

}  // namespace imujoco

#endif  // spsc_queue_h
