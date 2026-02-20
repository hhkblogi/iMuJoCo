// spmc_queue_tests.cc
// GoogleTest tests for SpmcQueue (Single-Producer Multi-Consumer "latest value" queue)

#include "spmc_queue.h"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

using imujoco::SpmcQueue;

// Simple test struct
struct TestData {
    int value = 0;
    uint64_t sequence = 0;
    char padding[64] = {};  // Make it non-trivial size
};

// =============================================================================
// Basic Tests
// =============================================================================

TEST(SpmcQueueTest, InitialState) {
    SpmcQueue<TestData, 3> queue;

    EXPECT_EQ(queue.get_sequence(), 0ULL);
    EXPECT_EQ(queue.get_latest(), nullptr);
    EXPECT_FALSE(queue.is_exit_signaled());
}

TEST(SpmcQueueTest, SingleWriteRead) {
    SpmcQueue<TestData, 3> queue;

    auto* slot = queue.begin_write();
    slot->value = 42;
    slot->sequence = 1;
    queue.end_write();

    EXPECT_EQ(queue.get_sequence(), 1ULL);

    const auto* item = queue.get_latest();
    ASSERT_NE(item, nullptr);
    EXPECT_EQ(item->value, 42);
    EXPECT_EQ(item->sequence, 1ULL);
}

TEST(SpmcQueueTest, MultipleWrites) {
    SpmcQueue<TestData, 3> queue;

    for (int i = 1; i <= 10; i++) {
        auto* slot = queue.begin_write();
        slot->value = i * 10;
        slot->sequence = i;
        queue.end_write();
    }

    EXPECT_EQ(queue.get_sequence(), 10ULL);

    const auto* item = queue.get_latest();
    ASSERT_NE(item, nullptr);
    EXPECT_EQ(item->value, 100);
    EXPECT_EQ(item->sequence, 10ULL);
}

TEST(SpmcQueueTest, RingBufferWrapAround) {
    SpmcQueue<TestData, 3> queue;  // 3-slot buffer

    for (int i = 1; i <= 100; i++) {
        auto* slot = queue.begin_write();
        slot->value = i;
        slot->sequence = i;
        queue.end_write();
    }

    EXPECT_EQ(queue.get_sequence(), 100ULL);

    const auto* item = queue.get_latest();
    ASSERT_NE(item, nullptr);
    EXPECT_EQ(item->value, 100);
}

// =============================================================================
// Exit Signal Tests
// =============================================================================

TEST(SpmcQueueTest, ExitSignal) {
    SpmcQueue<TestData, 3> queue;

    EXPECT_FALSE(queue.is_exit_signaled());

    queue.signal_exit();

    EXPECT_TRUE(queue.is_exit_signaled());
}

TEST(SpmcQueueTest, GetLatestNullptrAfterExitOnEmptyQueue) {
    SpmcQueue<TestData, 3> queue;

    EXPECT_EQ(queue.get_latest(), nullptr);
    EXPECT_EQ(queue.get_item_count(), 0ULL);

    queue.signal_exit();

    // get_latest() must still return nullptr — no items were written
    EXPECT_EQ(queue.get_latest(), nullptr);
    EXPECT_EQ(queue.get_item_count(), 0ULL);
    EXPECT_EQ(queue.get_sequence(), 1ULL);
}

TEST(SpmcQueueTest, ExitSignalReset) {
    SpmcQueue<TestData, 3> queue;

    queue.signal_exit();
    EXPECT_TRUE(queue.is_exit_signaled());

    queue.reset_exit_signal();
    EXPECT_FALSE(queue.is_exit_signaled());
}

TEST(SpmcQueueTest, WaitForItemAfterExitResetOnEmptyQueue) {
    // Regression: wait_for_item() must not return uninitialized data after
    // signal_exit()/reset_exit_signal() on an empty queue.
    SpmcQueue<TestData, 3> queue;

    queue.signal_exit();
    EXPECT_EQ(queue.get_sequence(), 1ULL);
    EXPECT_EQ(queue.get_item_count(), 0ULL);

    queue.reset_exit_signal();
    EXPECT_FALSE(queue.is_exit_signaled());

    std::atomic<bool> consumer_returned{false};
    std::atomic<bool> got_valid_item{false};

    std::thread consumer([&]() {
        const auto* item = queue.wait_for_item(0);
        consumer_returned = true;
        if (item != nullptr) {
            got_valid_item = true;
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Consumer should still be blocked
    EXPECT_FALSE(consumer_returned.load());

    // Write an item to unblock
    auto* slot = queue.begin_write();
    slot->value = 42;
    slot->sequence = 1;
    queue.end_write();

    consumer.join();

    EXPECT_TRUE(consumer_returned.load());
    EXPECT_TRUE(got_valid_item.load());
}

TEST(SpmcQueueTest, ExitSignalIdempotent) {
    SpmcQueue<TestData, 3> queue;

    uint64_t seq_before = queue.get_sequence();

    queue.signal_exit();
    queue.signal_exit();
    queue.signal_exit();

    // Sequence should only increment by 1 (first signal)
    EXPECT_EQ(queue.get_sequence(), seq_before + 1);
}

TEST(SpmcQueueTest, WaitForItemAfterExitResetOnNonemptyQueue) {
    // Regression: wait_for_item() must not return an already-consumed item
    // after signal_exit()/reset_exit_signal() on a non-empty queue.
    SpmcQueue<TestData, 3> queue;

    for (int i = 1; i <= 3; i++) {
        auto* slot = queue.begin_write();
        slot->value = i * 10;
        slot->sequence = i;
        queue.end_write();
    }

    uint64_t last_item_count = queue.get_item_count();  // = 3
    EXPECT_EQ(last_item_count, 3ULL);

    const auto* item = queue.get_latest();
    ASSERT_NE(item, nullptr);
    EXPECT_EQ(item->value, 30);

    queue.signal_exit();
    EXPECT_EQ(queue.get_sequence(), 4ULL);
    EXPECT_EQ(queue.get_item_count(), 3ULL);

    queue.reset_exit_signal();

    std::atomic<bool> consumer_returned{false};
    std::atomic<int> received_value{0};

    std::thread consumer([&]() {
        const auto* new_item = queue.wait_for_item(last_item_count);
        consumer_returned = true;
        if (new_item) {
            received_value = new_item->value;
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Consumer should still be blocked — no NEW items
    EXPECT_FALSE(consumer_returned.load());

    // Write a new item
    auto* slot = queue.begin_write();
    slot->value = 40;
    slot->sequence = 4;
    queue.end_write();

    consumer.join();

    EXPECT_TRUE(consumer_returned.load());
    EXPECT_EQ(received_value.load(), 40);
}

// =============================================================================
// Threading Tests
// =============================================================================

TEST(SpmcQueueTest, ProducerConsumerBasic) {
    SpmcQueue<TestData, 3> queue;

    constexpr int kNumItems = 1000;
    std::atomic<bool> consumer_ready{false};
    std::atomic<bool> consumer_done{false};
    std::atomic<int> max_observed{0};
    std::vector<int> received_values;
    received_values.reserve(kNumItems);

    std::thread consumer([&]() {
        uint64_t last_item_count = 0;
        int last_value = 0;

        consumer_ready.store(true, std::memory_order_release);

        while (true) {
            const auto* item = queue.wait_for_item(last_item_count);
            if (!item) break;

            TestData local_copy = *item;
            last_item_count = local_copy.sequence;

            if (local_copy.value > last_value) {
                received_values.push_back(local_copy.value);
                last_value = local_copy.value;
                max_observed.store(local_copy.value, std::memory_order_release);
            }

            if (local_copy.value == kNumItems) break;
        }
        consumer_done = true;
    });

    while (!consumer_ready.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }

    for (int i = 1; i <= kNumItems; i++) {
        auto* slot = queue.begin_write();
        slot->value = i;
        slot->sequence = i;
        queue.end_write();

        if (i % 100 == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    constexpr int kTimeoutMs = 5000;
    bool timeout_triggered = false;
    auto start = std::chrono::steady_clock::now();
    while (!consumer_done.load(std::memory_order_acquire)) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > std::chrono::milliseconds(kTimeoutMs)) {
            timeout_triggered = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    queue.signal_exit();
    consumer.join();

    EXPECT_FALSE(timeout_triggered) << "Test should not timeout";
    EXPECT_TRUE(consumer_done.load());
    EXPECT_GT(received_values.size(), 0UL);
    EXPECT_EQ(max_observed.load(), kNumItems);

    // Verify monotonic ordering
    for (size_t i = 1; i < received_values.size(); i++) {
        EXPECT_GT(received_values[i], received_values[i - 1]);
    }
}

TEST(SpmcQueueTest, WaitForItemWithExit) {
    SpmcQueue<TestData, 3> queue;

    std::atomic<bool> consumer_started{false};
    std::atomic<bool> consumer_exited{false};
    const TestData* result = nullptr;

    std::thread consumer([&]() {
        consumer_started.store(true, std::memory_order_release);
        result = queue.wait_for_item(0);
        consumer_exited.store(true, std::memory_order_release);
    });

    while (!consumer_started.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    EXPECT_FALSE(consumer_exited.load());

    queue.signal_exit();

    consumer.join();

    EXPECT_TRUE(consumer_exited.load());
    EXPECT_EQ(result, nullptr);
}

TEST(SpmcQueueTest, HighFrequencyProducer) {
    SpmcQueue<TestData, 16> queue;

    constexpr int kNumItems = 10000;
    std::atomic<int> max_received{0};
    std::atomic<bool> consumer_ready{false};

    std::thread consumer([&]() {
        uint64_t last_item_count = 0;
        consumer_ready = true;

        while (true) {
            const auto* item = queue.wait_for_item(last_item_count);
            if (!item) break;

            int value = item->value;
            last_item_count = static_cast<uint64_t>(value);

            if (value > max_received.load(std::memory_order_relaxed)) {
                max_received.store(value, std::memory_order_relaxed);
            }

            if (value == kNumItems) break;
        }
    });

    while (!consumer_ready.load()) {
        std::this_thread::yield();
    }

    for (int i = 1; i <= kNumItems; i++) {
        auto* slot = queue.begin_write();
        slot->value = i;
        queue.end_write();
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
    while (max_received.load(std::memory_order_relaxed) != kNumItems &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (max_received.load(std::memory_order_relaxed) != kNumItems) {
        queue.signal_exit();
    }
    consumer.join();

    EXPECT_EQ(max_received.load(), kNumItems);
}

// =============================================================================
// Edge Cases
// =============================================================================

TEST(SpmcQueueTest, GetLatestReturnsSameUntilNewWrite) {
    SpmcQueue<TestData, 3> queue;

    auto* slot = queue.begin_write();
    slot->value = 42;
    queue.end_write();

    const auto* item1 = queue.get_latest();
    const auto* item2 = queue.get_latest();

    EXPECT_EQ(item1, item2);
    EXPECT_EQ(item1->value, 42);
}

TEST(SpmcQueueTest, LargeDataType) {
    struct LargeData {
        double values[1000];
        int id;
    };

    SpmcQueue<LargeData, 3> queue;

    auto* slot = queue.begin_write();
    slot->id = 123;
    for (int i = 0; i < 1000; i++) {
        slot->values[i] = i * 1.5;
    }
    queue.end_write();

    const auto* item = queue.get_latest();
    ASSERT_NE(item, nullptr);
    EXPECT_EQ(item->id, 123);
    EXPECT_NEAR(item->values[500], 750.0, 0.001);
}

TEST(SpmcQueueTest, MultipleConcurrentReaders) {
    SpmcQueue<TestData, 8> queue;

    constexpr int kNumReaders = 4;
    constexpr int kNumItems = 100;

    std::atomic<int> readers_ready{0};
    std::atomic<int> readers_done{0};
    std::array<std::atomic<int>, kNumReaders> max_observed{};
    for (auto& m : max_observed) m.store(0);

    std::vector<std::thread> readers;
    for (int r = 0; r < kNumReaders; ++r) {
        readers.emplace_back([&, r]() {
            uint64_t last_item_count = 0;
            readers_ready.fetch_add(1, std::memory_order_release);

            while (true) {
                const auto* item = queue.wait_for_item(last_item_count);
                if (!item) break;

                int value = item->value;
                last_item_count = static_cast<uint64_t>(value);

                if (value > max_observed[r].load(std::memory_order_relaxed)) {
                    max_observed[r].store(value, std::memory_order_relaxed);
                }

                if (value == kNumItems) break;
            }
            readers_done.fetch_add(1, std::memory_order_release);
        });
    }

    while (readers_ready.load(std::memory_order_acquire) < kNumReaders) {
        std::this_thread::yield();
    }

    for (int i = 1; i <= kNumItems; ++i) {
        auto* slot = queue.begin_write();
        slot->value = i;
        slot->sequence = i;
        queue.end_write();

        if (i % 10 == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (readers_done.load(std::memory_order_acquire) < kNumReaders &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (readers_done.load(std::memory_order_acquire) < kNumReaders) {
        queue.signal_exit();
    }

    for (auto& t : readers) {
        t.join();
    }

    for (int r = 0; r < kNumReaders; ++r) {
        EXPECT_EQ(max_observed[r].load(), kNumItems) << "Reader " << r;
    }
}

// =============================================================================
// Cancellation Predicate Tests
// =============================================================================

TEST(SpmcQueueTest, WaitForItemWithCancellation) {
    SpmcQueue<TestData, 3> queue;

    std::atomic<bool> consumer_started{false};
    std::atomic<bool> consumer_returned{false};
    std::atomic<bool> cancel_flag{false};
    const TestData* result = reinterpret_cast<const TestData*>(1);  // Sentinel

    std::thread consumer([&]() {
        consumer_started.store(true, std::memory_order_release);
        result = queue.wait_for_item(0, [&] {
            return cancel_flag.load(std::memory_order_acquire);
        });
        consumer_returned.store(true, std::memory_order_release);
    });

    while (!consumer_started.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    EXPECT_FALSE(consumer_returned.load(std::memory_order_acquire));

    cancel_flag.store(true, std::memory_order_release);
    queue.reset_exit_signal();  // Bumps sequence_ and notifies; exit stays false

    consumer.join();

    EXPECT_FALSE(queue.is_exit_signaled());
    EXPECT_TRUE(consumer_returned.load(std::memory_order_acquire));
    EXPECT_EQ(result, nullptr);
}

TEST(SpmcQueueTest, ExitResetDoesNotStrandWaiters) {
    SpmcQueue<TestData, 3> queue;
    std::atomic<uint64_t> epoch{0};

    std::atomic<bool> consumer_started{false};
    std::atomic<bool> consumer_returned{false};
    const TestData* result = reinterpret_cast<const TestData*>(1);  // Sentinel

    uint64_t epoch_before = epoch.load(std::memory_order_acquire);

    std::thread consumer([&]() {
        consumer_started.store(true, std::memory_order_release);
        result = queue.wait_for_item(0, [&] {
            return epoch.load(std::memory_order_acquire) != epoch_before;
        });
        consumer_returned.store(true, std::memory_order_release);
    });

    while (!consumer_started.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    EXPECT_FALSE(consumer_returned.load(std::memory_order_acquire));

    // Simulate Stop(): signal_exit + epoch advance
    queue.signal_exit();
    epoch.fetch_add(1, std::memory_order_release);

    // Simulate Start(): reset exit signal (wakes waiters via sequence bump)
    queue.reset_exit_signal();

    constexpr int kTimeoutMs = 5000;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kTimeoutMs);
    while (!consumer_returned.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!consumer_returned.load(std::memory_order_acquire)) {
        queue.signal_exit();
    }
    consumer.join();

    EXPECT_TRUE(consumer_returned.load(std::memory_order_acquire));
    EXPECT_EQ(result, nullptr);
}
