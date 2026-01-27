// spsc_queue.h
// Single Producer Single Consumer (SPSC) lock-free queue
//
// A fixed-size ring buffer optimized for single producer / single consumer
// scenarios. Uses C++20 atomic wait/notify for efficient blocking waits.
//
// Thread Safety:
//   - Producer thread: begin_write(), end_write()
//   - Consumer thread: wait_for_item(), get_latest(), get_count()
//   - Any thread: signal_exit(), reset_exit_signal()
//
// Memory Model:
//   - Uses acquire/release semantics for correct synchronization
//   - Cache-line aligned atomics to prevent false sharing

#ifndef IMUJOCO_CORE_SPSC_QUEUE_H_
#define IMUJOCO_CORE_SPSC_QUEUE_H_

#include <array>
#include <atomic>
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
/// auto* item = queue.wait_for_item(last_count);
/// if (item) process(*item);
/// @endcode
template <typename T, size_t N = 3>
class SpscQueue {
public:
    static_assert(N >= 2, "Queue must have at least 2 slots");

    SpscQueue() : write_index_(0), read_index_(0), count_(0), exit_signaled_(false) {}

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
        int idx = write_index_.load(std::memory_order_relaxed);
        // Publish the written slot to consumers
        read_index_.store(idx, std::memory_order_release);
        // Advance to next write slot
        write_index_.store((idx + 1) % static_cast<int>(N), std::memory_order_relaxed);
        // Increment count and notify waiting consumers
        count_.fetch_add(1, std::memory_order_release);
        count_.notify_one();
    }

    // =========================================================================
    // Consumer Interface (single thread only)
    // =========================================================================

    /// Wait for a new item to be available.
    /// @param last_count The count value from the last successful read
    /// @return Pointer to the latest item, or nullptr if exit was signaled
    /// @note Blocks until count > last_count or exit is signaled
    const T* wait_for_item(uint64_t last_count) {
        count_.wait(last_count, std::memory_order_acquire);
        if (exit_signaled_.load(std::memory_order_acquire)) {
            return nullptr;
        }
        return &buffers_[read_index_.load(std::memory_order_acquire)];
    }

    /// Get the latest available item without blocking.
    /// @return Pointer to the latest item, or nullptr if no items written yet
    const T* get_latest() const {
        if (count_.load(std::memory_order_acquire) == 0) {
            return nullptr;
        }
        return &buffers_[read_index_.load(std::memory_order_acquire)];
    }

    /// Get the current item count.
    /// @return Number of items written since construction
    uint64_t get_count() const {
        return count_.load(std::memory_order_acquire);
    }

    // =========================================================================
    // Lifecycle Control (thread-safe)
    // =========================================================================

    /// Signal consumers to exit.
    /// Wakes up any threads blocked in wait_for_item().
    void signal_exit() {
        bool expected = false;
        if (exit_signaled_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            // Increment count to wake up waiting consumers
            count_.fetch_add(1, std::memory_order_release);
            count_.notify_all();
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

private:
    std::array<T, N> buffers_;

    // Cache-line aligned to prevent false sharing between producer and consumer
    alignas(64) std::atomic<int> write_index_;
    alignas(64) std::atomic<int> read_index_;
    alignas(64) std::atomic<uint64_t> count_;

    std::atomic<bool> exit_signaled_;
};

}  // namespace imujoco

#endif  // IMUJOCO_CORE_SPSC_QUEUE_H_
