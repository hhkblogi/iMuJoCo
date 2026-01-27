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
    std::atomic<bool> consumer_done{false};
    std::vector<int> received_values;
    received_values.reserve(kNumItems);

    // Consumer thread
    std::thread consumer([&]() {
        uint64_t last_seq = 0;
        int last_value = 0;

        while (true) {
            const auto* item = queue.wait_for_item(last_seq);
            if (!item) break;  // Exit signaled

            last_seq = queue.get_sequence();

            // Verify ordering (values should be monotonically increasing)
            if (item->value > last_value) {
                received_values.push_back(item->value);
                last_value = item->value;
            }

            if (item->value == kNumItems) break;
        }
        consumer_done = true;
    });

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

    consumer.join();

    XCTAssertTrue(consumer_done, @"Consumer should complete");
    XCTAssertGreaterThan(received_values.size(), 0UL, @"Should have received some values");
    XCTAssertEqual(received_values.back(), kNumItems, @"Should have received final value");

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
