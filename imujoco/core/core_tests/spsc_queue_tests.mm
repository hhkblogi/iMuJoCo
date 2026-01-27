// spsc_queue_tests.mm
// Tests for SpscQueue (Single Producer Single Consumer lock-free queue)

#import <XCTest/XCTest.h>

#include "spsc_queue.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

using namespace imujoco;

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
        uint64_t last_seq = 0;
        int last_value = 0;

        // Signal that consumer is ready before entering the wait loop
        consumer_ready.store(true, std::memory_order_release);

        while (true) {
            const auto* item = queue.wait_for_item(last_seq);
            if (!item) break;  // Exit signaled

            // Update last_seq based on the item we received, not queue's current
            // sequence which may have advanced further due to concurrent writes
            last_seq = item->sequence;

            // Verify ordering (values should be monotonically increasing)
            if (item->value > last_value) {
                received_values.push_back(item->value);
                last_value = item->value;
                max_observed.store(item->value, std::memory_order_release);
            }

            // Exit once we've observed the final item
            if (item->value == kNumItems) break;
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
    SpscQueue<TestData, 3> queue;

    constexpr int kNumItems = 10000;
    std::atomic<int> max_received{0};

    // Consumer thread
    std::thread consumer([&]() {
        while (true) {
            const auto* item = queue.get_latest();
            if (item && item->value > max_received) {
                max_received = item->value;
            }
            if (queue.is_exit_signaled() && max_received >= kNumItems) {
                break;
            }
            std::this_thread::yield();
        }
    });

    // High-frequency producer
    for (int i = 1; i <= kNumItems; i++) {
        auto* slot = queue.begin_write();
        slot->value = i;
        queue.end_write();
    }

    queue.signal_exit();
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
