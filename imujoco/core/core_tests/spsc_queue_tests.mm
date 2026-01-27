// spsc_queue_tests.mm
// Tests for SpscQueue (Single-Producer Multi-Reader "latest value" queue)

#import <XCTest/XCTest.h>

#include "spsc_queue.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

// Import only the specific type needed (avoids broad `using namespace`)
using imujoco::SpscQueue;

// Simple test struct
struct TestData {
    int value = 0;
    uint64_t sequence = 0;
    char padding[64] = {};  // Make it non-trivial size
};

@interface SpscQueueTests : XCTestCase
@end

@implementation SpscQueueTests

#pragma mark - Basic Tests

- (void)test_initial_state {
    SpscQueue<TestData, 3> queue;

    XCTAssertEqual(queue.get_sequence(), 0ULL, @"Initial sequence should be 0");
    XCTAssertTrue(queue.get_latest() == nullptr, @"get_latest should return nullptr when empty");
    XCTAssertFalse(queue.is_exit_signaled(), @"Exit should not be signaled initially");
}

- (void)test_single_write_read {
    SpscQueue<TestData, 3> queue;

    // Write one item
    auto* slot = queue.begin_write();
    slot->value = 42;
    slot->sequence = 1;
    queue.end_write();

    XCTAssertEqual(queue.get_sequence(), 1ULL, @"Sequence should be 1 after one write");

    // Read it back
    const auto* item = queue.get_latest();
    XCTAssertNotEqual(item, nullptr, @"get_latest should return non-null after write");
    XCTAssertEqual(item->value, 42, @"Value should match what was written");
    XCTAssertEqual(item->sequence, 1ULL, @"Sequence should match what was written");
}

- (void)test_multiple_writes {
    SpscQueue<TestData, 3> queue;

    // Write multiple items
    for (int i = 1; i <= 10; i++) {
        auto* slot = queue.begin_write();
        slot->value = i * 10;
        slot->sequence = i;
        queue.end_write();
    }

    XCTAssertEqual(queue.get_sequence(), 10ULL, @"Sequence should be 10 after 10 writes");

    // get_latest should return the most recent
    const auto* item = queue.get_latest();
    XCTAssertNotEqual(item, nullptr);
    XCTAssertEqual(item->value, 100, @"Latest value should be 100");
    XCTAssertEqual(item->sequence, 10ULL, @"Latest sequence should be 10");
}

- (void)test_ring_buffer_wrap_around {
    SpscQueue<TestData, 3> queue;  // 3-slot buffer

    // Write more items than slots to test wrap-around
    for (int i = 1; i <= 100; i++) {
        auto* slot = queue.begin_write();
        slot->value = i;
        slot->sequence = i;
        queue.end_write();
    }

    XCTAssertEqual(queue.get_sequence(), 100ULL, @"Sequence should track total writes");

    const auto* item = queue.get_latest();
    XCTAssertNotEqual(item, nullptr);
    XCTAssertEqual(item->value, 100, @"Latest value should be 100 after wrap-around");
}

#pragma mark - Exit Signal Tests

- (void)test_exit_signal {
    SpscQueue<TestData, 3> queue;

    XCTAssertFalse(queue.is_exit_signaled());

    queue.signal_exit();

    XCTAssertTrue(queue.is_exit_signaled(), @"Exit should be signaled");
}

- (void)test_get_latest_nullptr_after_exit_on_empty_queue {
    // Regression test: get_latest() must return nullptr on an empty queue
    // even after signal_exit() increments the sequence counter.
    SpscQueue<TestData, 3> queue;

    // Verify empty state before exit
    XCTAssertTrue(queue.get_latest() == nullptr,
                  @"get_latest should return nullptr on empty queue");
    XCTAssertEqual(queue.get_item_count(), 0ULL,
                   @"item_count should be 0 before any writes");

    // Signal exit (this increments sequence_ but NOT item_count_)
    queue.signal_exit();

    // get_latest() must still return nullptr because no items were written
    XCTAssertTrue(queue.get_latest() == nullptr,
                  @"get_latest must return nullptr after signal_exit on empty queue");
    XCTAssertEqual(queue.get_item_count(), 0ULL,
                   @"item_count should remain 0 after signal_exit");
    // sequence_ is incremented by signal_exit, but that's for wakeup only
    XCTAssertEqual(queue.get_sequence(), 1ULL,
                   @"sequence should be 1 after signal_exit");
}

- (void)test_exit_signal_reset {
    SpscQueue<TestData, 3> queue;

    queue.signal_exit();
    XCTAssertTrue(queue.is_exit_signaled());

    queue.reset_exit_signal();
    XCTAssertFalse(queue.is_exit_signaled(), @"Exit signal should be reset");
}

- (void)test_wait_for_item_after_exit_reset_on_empty_queue {
    // Regression test: wait_for_item() must not return a pointer to uninitialized
    // data when signal_exit() followed by reset_exit_signal() is called on an
    // empty queue. The sequence_ is incremented by signal_exit(), so after reset
    // the `current > last_sequence` check could pass without any items written.
    SpscQueue<TestData, 3> queue;

    // Signal exit on empty queue (increments sequence_ to 1)
    queue.signal_exit();
    XCTAssertEqual(queue.get_sequence(), 1ULL);
    XCTAssertEqual(queue.get_item_count(), 0ULL, @"No items written yet");

    // Reset exit signal for "reuse"
    queue.reset_exit_signal();
    XCTAssertFalse(queue.is_exit_signaled());

    // Now wait_for_item(0) should block (not return uninitialized data)
    // because item_count_ is still 0, even though sequence_ > 0.
    // We test this by having a consumer thread that should block until we
    // either write an item or signal exit again.
    std::atomic<bool> consumer_returned{false};
    std::atomic<bool> got_valid_item{false};

    std::thread consumer([&]() {
        const auto* item = queue.wait_for_item(0);
        consumer_returned = true;
        // If we got an item (not nullptr from exit), record it
        if (item != nullptr) {
            got_valid_item = true;
        }
    });

    // Give consumer time to potentially (incorrectly) return
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Consumer should still be blocked because no items were written
    XCTAssertFalse(consumer_returned.load(),
                   @"Consumer should be blocked waiting - no items written yet");

    // Now write an actual item
    auto* slot = queue.begin_write();
    slot->value = 42;
    slot->sequence = 1;
    queue.end_write();

    // Wait for consumer to return
    consumer.join();

    XCTAssertTrue(consumer_returned, @"Consumer should have returned after write");
    XCTAssertTrue(got_valid_item, @"Consumer should have received valid item, not nullptr");
}

- (void)test_exit_signal_idempotent {
    SpscQueue<TestData, 3> queue;

    uint64_t seq_before = queue.get_sequence();

    // Signal exit multiple times
    queue.signal_exit();
    queue.signal_exit();
    queue.signal_exit();

    // Sequence should only increment by 1 (first signal)
    XCTAssertEqual(queue.get_sequence(), seq_before + 1,
                   @"Multiple exit signals should only increment sequence once");
}

- (void)test_wait_for_item_after_exit_reset_on_nonempty_queue {
    // Regression test: wait_for_item() must not return an already-consumed item
    // after signal_exit()/reset_exit_signal() on a non-empty queue.
    // This tests that we compare against item_count_, not sequence_.
    SpscQueue<TestData, 3> queue;

    // Write some items
    for (int i = 1; i <= 3; i++) {
        auto* slot = queue.begin_write();
        slot->value = i * 10;
        slot->sequence = i;
        queue.end_write();
    }

    // Consumer reads latest item
    uint64_t last_item_count = queue.get_item_count();  // = 3
    XCTAssertEqual(last_item_count, 3ULL);

    const auto* item = queue.get_latest();
    XCTAssertNotEqual(item, nullptr);
    XCTAssertEqual(item->value, 30, @"Latest item should have value 30");

    // Now signal_exit() (increments sequence_ to 4, item_count_ stays 3)
    queue.signal_exit();
    XCTAssertEqual(queue.get_sequence(), 4ULL);
    XCTAssertEqual(queue.get_item_count(), 3ULL);

    // Reset for "reuse"
    queue.reset_exit_signal();

    // Consumer calls wait_for_item(3) - should block because no NEW items
    // were written (item_count_ is still 3, not > 3)
    std::atomic<bool> consumer_returned{false};
    std::atomic<int> received_value{0};

    std::thread consumer([&]() {
        const auto* new_item = queue.wait_for_item(last_item_count);
        consumer_returned = true;
        if (new_item) {
            received_value = new_item->value;
        }
    });

    // Give consumer time to potentially (incorrectly) return the old item
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Consumer should still be blocked - no new items written
    XCTAssertFalse(consumer_returned.load(),
                   @"Consumer should block - no new items since last_item_count=3");

    // Write a NEW item (value=40)
    auto* slot = queue.begin_write();
    slot->value = 40;
    slot->sequence = 4;
    queue.end_write();

    // Wait for consumer
    consumer.join();

    XCTAssertTrue(consumer_returned, @"Consumer should return after new write");
    XCTAssertEqual(received_value.load(), 40,
                   @"Consumer should receive NEW item (40), not old item (30)");
}

#pragma mark - Threading Tests

- (void)test_producer_consumer_basic {
    SpscQueue<TestData, 3> queue;

    constexpr int kNumItems = 1000;
    std::atomic<bool> consumer_ready{false};
    std::atomic<bool> consumer_done{false};
    std::atomic<int> max_observed{0};
    std::vector<int> received_values;
    received_values.reserve(kNumItems);

    // Consumer thread
    std::thread consumer([&]() {
        uint64_t last_item_count = 0;
        int last_value = 0;

        // Signal that consumer is ready before entering the wait loop
        consumer_ready.store(true, std::memory_order_release);

        while (true) {
            const auto* item = queue.wait_for_item(last_item_count);
            if (!item) break;  // Exit signaled

            // IMPORTANT: Copy item immediately to avoid data race.
            // The producer may overwrite this slot after N-1 more writes.
            TestData local_copy = *item;

            // Update last_item_count from the queue's actual item count
            last_item_count = queue.get_item_count();

            // Verify ordering (values should be monotonically increasing)
            if (local_copy.value > last_value) {
                received_values.push_back(local_copy.value);
                last_value = local_copy.value;
                max_observed.store(local_copy.value, std::memory_order_release);
            }

            // Exit once we've observed the final item
            if (local_copy.value == kNumItems) break;
        }
        consumer_done = true;
    });

    // Wait for consumer to be ready before producing
    while (!consumer_ready.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }

    // Producer thread (main)
    for (int i = 1; i <= kNumItems; i++) {
        auto* slot = queue.begin_write();
        slot->value = i;
        slot->sequence = i;
        queue.end_write();

        // Small delay to simulate real-world timing
        if (i % 100 == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    // Wait with timeout for consumer to observe final item.
    // signal_exit() is only used as a deadlock-avoidance fallback.
    constexpr int kTimeoutMs = 5000;
    bool timeout_triggered = false;
    auto start = std::chrono::steady_clock::now();
    while (!consumer_done.load(std::memory_order_acquire)) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > std::chrono::milliseconds(kTimeoutMs)) {
            // Timeout: signal exit to unblock consumer and prevent test hang
            timeout_triggered = true;
            queue.signal_exit();
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    consumer.join();

    XCTAssertFalse(timeout_triggered,
                   @"Test should not timeout - consumer should observe final item");
    XCTAssertTrue(consumer_done, @"Consumer should complete");
    XCTAssertGreaterThan(received_values.size(), 0UL, @"Should have received some values");
    XCTAssertEqual(max_observed.load(), kNumItems,
                   @"Consumer should have observed final value");

    // Verify monotonic ordering
    for (size_t i = 1; i < received_values.size(); i++) {
        XCTAssertGreaterThan(received_values[i], received_values[i-1],
                             @"Values should be monotonically increasing");
    }
}

- (void)test_wait_for_item_with_exit {
    SpscQueue<TestData, 3> queue;

    std::atomic<bool> consumer_started{false};
    std::atomic<bool> consumer_exited{false};
    const TestData* result = nullptr;

    // Consumer thread that waits
    std::thread consumer([&]() {
        consumer_started = true;
        result = queue.wait_for_item(0);
        consumer_exited = true;
    });

    // Wait for consumer to start waiting
    while (!consumer_started) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    XCTAssertFalse(consumer_exited, @"Consumer should be blocked waiting");

    // Signal exit
    queue.signal_exit();

    consumer.join();

    XCTAssertTrue(consumer_exited, @"Consumer should have exited");
    XCTAssertTrue(result == nullptr, @"wait_for_item should return nullptr on exit");
}

- (void)test_high_frequency_producer {
    // Use a larger buffer to reduce wrap-around during stress test
    SpscQueue<TestData, 16> queue;

    constexpr int kNumItems = 10000;
    std::atomic<int> max_received{0};
    std::atomic<bool> consumer_ready{false};

    // Consumer thread using wait_for_item() to avoid data races
    std::thread consumer([&]() {
        uint64_t last_item_count = 0;
        consumer_ready = true;

        while (true) {
            const auto* item = queue.wait_for_item(last_item_count);
            if (!item) break;  // Exit signaled

            // Copy value immediately before it can be overwritten
            int value = item->value;
            last_item_count = queue.get_item_count();

            if (value > max_received.load(std::memory_order_relaxed)) {
                max_received.store(value, std::memory_order_relaxed);
            }

            // Exit once we've observed the final item
            if (value == kNumItems) break;
        }
    });

    // Wait for consumer to be ready
    while (!consumer_ready.load()) {
        std::this_thread::yield();
    }

    // High-frequency producer
    for (int i = 1; i <= kNumItems; i++) {
        auto* slot = queue.begin_write();
        slot->value = i;
        queue.end_write();
    }

    // Use signal_exit() as fallback to prevent deadlock if consumer misses final item.
    // Wait up to a bounded deadline for the consumer to observe the final value.
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
    while (max_received.load(std::memory_order_relaxed) != kNumItems &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (max_received.load(std::memory_order_relaxed) != kNumItems) {
        queue.signal_exit();
    }
    consumer.join();

    XCTAssertEqual(max_received.load(), kNumItems,
                   @"Consumer should eventually see final value");
}

#pragma mark - Edge Cases

- (void)test_get_latest_returns_same_until_new_write {
    SpscQueue<TestData, 3> queue;

    auto* slot = queue.begin_write();
    slot->value = 42;
    queue.end_write();

    const auto* item1 = queue.get_latest();
    const auto* item2 = queue.get_latest();

    XCTAssertEqual(item1, item2, @"get_latest should return same pointer until new write");
    XCTAssertEqual(item1->value, 42);
}

- (void)test_large_data_type {
    // Test with a larger struct
    struct LargeData {
        double values[1000];
        int id;
    };

    SpscQueue<LargeData, 3> queue;

    auto* slot = queue.begin_write();
    slot->id = 123;
    for (int i = 0; i < 1000; i++) {
        slot->values[i] = i * 1.5;
    }
    queue.end_write();

    const auto* item = queue.get_latest();
    XCTAssertNotEqual(item, nullptr);
    XCTAssertEqual(item->id, 123);
    XCTAssertEqualWithAccuracy(item->values[500], 750.0, 0.001);
}

@end
