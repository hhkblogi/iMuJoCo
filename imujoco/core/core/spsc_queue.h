// spsc_queue.h
// Single-Producer "Latest Value" Mutex-Free Queue
//
// NAMING: The class is named SpscQueue for API compatibility with the original
// single-consumer implementation. However, the current implementation supports
// multiple concurrent readers (each maintaining independent last_item_count state).
// All readers observe the same latest item - this is NOT a work-stealing queue.
// Semantically, this is an SPMR (Single-Producer Multi-Reader) "latest value" queue.
//
// INTENDED USE CASE:
//   This queue is designed for real-time scenarios where consumers want the LATEST
//   value, not every value. Typical use cases include:
//   - Physics simulation → Renderer (consumer wants latest frame, skipping is OK)
//   - Sensor data → Processing (consumer wants freshest reading)
//   - Game state → Network sync (consumer wants current state)
//
//   If you need every item delivered (no skipping), use a traditional FIFO queue.
//
// DESIGN RATIONALE:
//   - Zero-copy for minimal latency: returns pointers into internal buffer
//   - Mutex-free: no locks; producer is wait-free, consumers may block in wait_for_item()
//   - Fixed-size for predictable memory: no allocations after construction
//   - "Latest value" semantics: slow consumers skip to newest data
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
//   - Producer's end_write() releases; consumer's wait_for_item() acquires
//   - This guarantees all writes to the slot are visible before the pointer is returned
//   - Cache-line aligned atomics to prevent false sharing
//
// Pointer Lifetime & Safety Contract:
//   Pointers returned by get_latest() and wait_for_item() point into the internal
//   ring buffer. They are guaranteed valid through N-1 subsequent end_write() calls.
//   After N writes, the slot is overwritten and the pointer becomes invalid.
//
//   CONSUMER RESPONSIBILITY: Copy data out promptly after receiving a pointer.
//   The queue cannot enforce this - it's a contract. For a 3-slot queue (N=3),
//   you have 2 writes of safety margin. For real-time frame delivery at 60Hz
//   consumer / 1000Hz producer, use N >= 16 to provide adequate margin.
//
//   This is inherent to zero-copy ring buffer design. If you need automatic
//   lifetime management, consider a different data structure (e.g., shared_ptr
//   per slot), but that adds overhead inappropriate for real-time use.

#ifndef spsc_queue_h
#define spsc_queue_h

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace imujoco {

/// Single-Producer "Latest Value" mutex-free queue (supports multiple readers).
///
/// @tparam T The element type stored in the queue
/// @tparam N The capacity of the queue (number of slots)
///
/// This queue uses a ring buffer with triple buffering by default (N=3).
/// The producer is wait-free; consumers may block in wait_for_item() via atomic::wait.
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
    static_assert(std::is_default_constructible_v<T>,
                  "T must be default-constructible (storage uses std::array<T, N>)");

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
            // CRITICAL: Capture sequence BEFORE checking predicates to avoid missed wakeup.
            // If end_write()/signal_exit() increments sequence_ after this load but before
            // our wait(), we'll see a stale expected_seq and wait() will return immediately
            // (because current value != expected). This prevents the deadlock where we
            // load an already-incremented sequence and block forever.
            uint64_t expected_seq = sequence_.load(std::memory_order_acquire);

            // Check exit signal
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
            // Wait on the sequence value captured BEFORE predicate checks.
            // If sequence_ changed between our load and now, wait() returns immediately.
            sequence_.wait(expected_seq, std::memory_order_acquire);
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
    uint64_t get_sequence() const noexcept {
        return sequence_.load(std::memory_order_acquire);
    }

    /// Get the number of items written.
    /// @return Number of completed end_write() calls since construction.
    /// @note Consumers should use this together with a caller-maintained
    ///       last_item_count value to detect new data between calls to
    ///       wait_for_item(). Unlike get_sequence(), this is not incremented
    ///       by signal_exit().
    uint64_t get_item_count() const noexcept {
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
    bool is_exit_signaled() const noexcept {
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
