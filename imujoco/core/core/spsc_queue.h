// spsc_queue.h
// Single Producer Single Consumer (SPSC) lock-free queue
//
// A fixed-size ring buffer optimized for single producer / single consumer
// scenarios. Uses C++20 atomic wait/notify for efficient blocking waits.
//
// Thread Safety:
//   - Producer thread: begin_write(), end_write()
//   - Consumer thread: wait_for_item(), get_latest(), get_sequence()
//   - Any thread: signal_exit(), reset_exit_signal()
//
// Memory Model:
//   - Uses acquire/release semantics for correct synchronization
//   - Cache-line aligned atomics to prevent false sharing
//
// Pointer Lifetime:
//   Pointers returned by get_latest() and wait_for_item() are valid only until
//   the next call to begin_write() that would overwrite that slot. Because the
//   queue uses a fixed ring buffer of N slots, a pointer becomes invalid after
//   N-1 subsequent writes. Consumers should copy data out immediately and not
//   retain pointers across multiple read operations.

#ifndef IMUJOCO_CORE_SPSC_QUEUE_H_
#define IMUJOCO_CORE_SPSC_QUEUE_H_

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
/// the internal ring buffer. They remain valid only until the producer wraps
/// around and overwrites that slot (after N-1 writes). Consumers must copy
/// data out promptly and not cache these pointers.
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
/// auto* item = queue.wait_for_item(last_seq);
/// if (item) {
///     FrameData copy = *item;  // Copy out immediately
///     process(copy);
/// }
/// @endcode
template <typename T, std::size_t N = 3>
class SpscQueue {
public:
    static_assert(N >= 2, "Queue must have at least 2 slots");

    SpscQueue() : write_index_(0), read_index_(0), sequence_(0), exit_signaled_(false) {}

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
        // Increment sequence and notify waiting consumers
        sequence_.fetch_add(1, std::memory_order_release);
        sequence_.notify_one();
    }

    // =========================================================================
    // Consumer Interface (single thread only)
    // =========================================================================

    /// Wait for a new item to be available.
    /// @param last_sequence The sequence value from the last successful read
    /// @return Pointer to the latest item, or nullptr if exit was signaled
    /// @note Blocks until sequence > last_sequence or exit is signaled.
    ///       The returned pointer is valid only until the producer wraps around
    ///       and overwrites this slot. Copy data out immediately.
    const T* wait_for_item(uint64_t last_sequence) {
        sequence_.wait(last_sequence, std::memory_order_acquire);
        if (exit_signaled_.load(std::memory_order_acquire)) {
            return nullptr;
        }
        return &buffers_[read_index_.load(std::memory_order_acquire)];
    }

    /// Get the latest available item without blocking.
    /// @return Pointer to the latest item, or nullptr if no items written yet.
    ///         The returned pointer is valid only until the producer wraps around
    ///         and overwrites this slot. Copy data out immediately.
    const T* get_latest() const {
        if (sequence_.load(std::memory_order_acquire) == 0) {
            return nullptr;
        }
        return &buffers_[read_index_.load(std::memory_order_acquire)];
    }

    /// Get the current sequence number.
    /// @return Monotonically increasing sequence used for synchronization.
    ///         Increments on each end_write() and on signal_exit().
    ///         Use with wait_for_item() to detect new data availability.
    /// @note This is a synchronization sequence, not strictly "items written"
    ///       since signal_exit() also increments it to wake waiting consumers.
    uint64_t get_sequence() const {
        return sequence_.load(std::memory_order_acquire);
    }

    // =========================================================================
    // Lifecycle Control (thread-safe)
    // =========================================================================

    /// Signal consumers to exit.
    /// Wakes up any threads blocked in wait_for_item().
    /// @note Increments sequence to ensure waiters wake up.
    void signal_exit() {
        bool expected = false;
        if (exit_signaled_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            // Increment sequence to wake up waiting consumers
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

    /// @deprecated Use get_sequence() instead
    uint64_t get_count() const { return get_sequence(); }

private:
    std::array<T, N> buffers_;

    // Cache-line aligned to prevent false sharing between producer and consumer
    alignas(64) std::atomic<std::size_t> write_index_;
    alignas(64) std::atomic<std::size_t> read_index_;
    alignas(64) std::atomic<uint64_t> sequence_;

    std::atomic<bool> exit_signaled_;
};

}  // namespace imujoco

#endif  // IMUJOCO_CORE_SPSC_QUEUE_H_
