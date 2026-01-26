// fragment_test.cc
// Unit tests for UDP fragmentation and reassembly

#include "fragment.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <random>

namespace imujoco::driver {
namespace {

// Test CRC-16 computation
TEST(FragmentTest, CRC16) {
    // Test with known data
    const uint8_t data[] = {0x31, 0x32, 0x33, 0x34, 0x35,
                            0x36, 0x37, 0x38, 0x39};  // "123456789"
    uint16_t crc = ComputeCRC16(data, sizeof(data));
    // Expected CRC-16-CCITT for "123456789" is 0x29B1
    EXPECT_EQ(crc, 0x29B1);
}

// Test header checksum
TEST(FragmentTest, HeaderChecksum) {
    FragmentHeader header;
    header.magic = kFragmentMagic;
    header.message_id = 1234;
    header.fragment_index = 0;
    header.fragment_count = 1;
    header.total_size = 100;
    header.payload_size = 100;
    header.checksum = 0;
    header.checksum = ComputeHeaderChecksum(&header);

    EXPECT_TRUE(ValidateHeaderChecksum(&header));

    // Corrupt the header
    header.message_id = 5678;
    EXPECT_FALSE(ValidateHeaderChecksum(&header));
}

// Test single fragment (small message)
TEST(FragmentTest, SingleFragment) {
    FragmentedSender sender;

    std::vector<uint8_t> message(100, 0xAB);
    auto fragments = sender.FragmentMessage(message.data(), message.size());

    ASSERT_EQ(fragments.size(), 1);
    EXPECT_EQ(fragments[0].size(), kFragmentHeaderSize + 100);

    // Verify header
    const auto* header =
        reinterpret_cast<const FragmentHeader*>(fragments[0].data());
    EXPECT_EQ(header->magic, kFragmentMagic);
    EXPECT_EQ(header->fragment_index, 0);
    EXPECT_EQ(header->fragment_count, 1);
    EXPECT_EQ(header->total_size, 100);
    EXPECT_EQ(header->payload_size, 100);
    EXPECT_TRUE(ValidateHeaderChecksum(header));

    // Verify payload
    for (size_t i = 0; i < 100; i++) {
        EXPECT_EQ(fragments[0][kFragmentHeaderSize + i], 0xAB);
    }
}

// Test multi-fragment (large message)
TEST(FragmentTest, MultiFragment) {
    FragmentedSender sender;

    // Create a message that needs 3 fragments
    size_t message_size = kMaxFragmentPayload * 2 + 500;
    std::vector<uint8_t> message(message_size);
    std::iota(message.begin(), message.end(), 0);

    auto fragments = sender.FragmentMessage(message.data(), message.size());

    ASSERT_EQ(fragments.size(), 3);

    // Verify each fragment
    for (size_t i = 0; i < fragments.size(); i++) {
        const auto* header =
            reinterpret_cast<const FragmentHeader*>(fragments[i].data());
        EXPECT_EQ(header->magic, kFragmentMagic);
        EXPECT_EQ(header->fragment_index, i);
        EXPECT_EQ(header->fragment_count, 3);
        EXPECT_EQ(header->total_size, message_size);
        EXPECT_TRUE(ValidateHeaderChecksum(header));

        if (i < 2) {
            EXPECT_EQ(header->payload_size, kMaxFragmentPayload);
        } else {
            EXPECT_EQ(header->payload_size, 500);
        }
    }
}

// Test reassembly of single fragment
TEST(FragmentTest, ReassemblySingleFragment) {
    FragmentedSender sender;
    ReassemblyManager reassembler;

    std::vector<uint8_t> original(100, 0xCD);
    auto fragments = sender.FragmentMessage(original.data(), original.size());

    ASSERT_EQ(fragments.size(), 1);

    auto result = reassembler.ProcessFragment(fragments[0].data(),
                                               fragments[0].size());

    ASSERT_TRUE(result.complete);
    ASSERT_EQ(result.size, original.size());
    EXPECT_EQ(std::memcmp(result.data, original.data(), original.size()), 0);
}

// Test reassembly of multi-fragment message
TEST(FragmentTest, ReassemblyMultiFragment) {
    FragmentedSender sender;
    ReassemblyManager reassembler;

    // Create test message
    size_t message_size = kMaxFragmentPayload * 2 + 500;
    std::vector<uint8_t> original(message_size);
    std::iota(original.begin(), original.end(), 0);

    auto fragments = sender.FragmentMessage(original.data(), original.size());
    ASSERT_EQ(fragments.size(), 3);

    // Process fragments in order
    for (size_t i = 0; i < fragments.size() - 1; i++) {
        auto result = reassembler.ProcessFragment(fragments[i].data(),
                                                   fragments[i].size());
        EXPECT_FALSE(result.complete);
    }

    // Last fragment should complete reassembly
    auto result = reassembler.ProcessFragment(fragments.back().data(),
                                               fragments.back().size());
    ASSERT_TRUE(result.complete);
    ASSERT_EQ(result.size, original.size());
    EXPECT_EQ(std::memcmp(result.data, original.data(), original.size()), 0);
}

// Test out-of-order reassembly
TEST(FragmentTest, OutOfOrderReassembly) {
    FragmentedSender sender;
    ReassemblyManager reassembler;

    size_t message_size = kMaxFragmentPayload * 3;
    std::vector<uint8_t> original(message_size);
    std::iota(original.begin(), original.end(), 0);

    auto fragments = sender.FragmentMessage(original.data(), original.size());
    ASSERT_EQ(fragments.size(), 3);

    // Process in reverse order
    auto result = reassembler.ProcessFragment(fragments[2].data(),
                                               fragments[2].size());
    EXPECT_FALSE(result.complete);

    result = reassembler.ProcessFragment(fragments[0].data(),
                                          fragments[0].size());
    EXPECT_FALSE(result.complete);

    result = reassembler.ProcessFragment(fragments[1].data(),
                                          fragments[1].size());
    ASSERT_TRUE(result.complete);
    ASSERT_EQ(result.size, original.size());
    EXPECT_EQ(std::memcmp(result.data, original.data(), original.size()), 0);
}

// Test duplicate fragment handling
TEST(FragmentTest, DuplicateFragments) {
    FragmentedSender sender;
    ReassemblyManager reassembler;

    std::vector<uint8_t> original(kMaxFragmentPayload * 2);
    std::iota(original.begin(), original.end(), 0);

    auto fragments = sender.FragmentMessage(original.data(), original.size());
    ASSERT_EQ(fragments.size(), 2);

    // Process first fragment twice
    auto result = reassembler.ProcessFragment(fragments[0].data(),
                                               fragments[0].size());
    EXPECT_FALSE(result.complete);

    result = reassembler.ProcessFragment(fragments[0].data(),
                                          fragments[0].size());
    EXPECT_FALSE(result.complete);  // Still not complete (duplicate ignored)

    // Now process second fragment
    result = reassembler.ProcessFragment(fragments[1].data(),
                                          fragments[1].size());
    ASSERT_TRUE(result.complete);
    EXPECT_EQ(std::memcmp(result.data, original.data(), original.size()), 0);
}

// Test checksum validation
TEST(FragmentTest, ChecksumValidation) {
    FragmentedSender sender;
    ReassemblyManager reassembler;

    std::vector<uint8_t> original(100, 0xEF);
    auto fragments = sender.FragmentMessage(original.data(), original.size());

    ASSERT_EQ(fragments.size(), 1);

    // Corrupt the checksum
    auto corrupted = fragments[0];
    corrupted[14]++;  // Modify checksum byte

    auto result = reassembler.ProcessFragment(corrupted.data(),
                                               corrupted.size());
    EXPECT_FALSE(result.complete);
    EXPECT_EQ(result.data, nullptr);
}

// Test slot eviction (LRU)
TEST(FragmentTest, SlotEviction) {
    FragmentedSender sender;
    ReassemblyManager reassembler;

    // Create incomplete messages to fill all slots
    for (size_t i = 0; i < kReassemblySlots + 2; i++) {
        std::vector<uint8_t> msg(kMaxFragmentPayload * 2, static_cast<uint8_t>(i));
        auto fragments = sender.FragmentMessage(msg.data(), msg.size());

        // Only send first fragment (incomplete)
        reassembler.ProcessFragment(fragments[0].data(), fragments[0].size());
    }

    // Should have evicted oldest slots
    EXPECT_LE(reassembler.ActiveSlotCount(), static_cast<int>(kReassemblySlots));
}

// Test message ID wrap-around
TEST(FragmentTest, MessageIdWrapAround) {
    FragmentedSender sender;

    // Send many messages to wrap the ID
    for (int i = 0; i < 70000; i++) {
        std::vector<uint8_t> msg(10, static_cast<uint8_t>(i));
        sender.FragmentMessage(msg.data(), msg.size());
    }

    // Next message should still work
    std::vector<uint8_t> msg(100, 0xFF);
    auto fragments = sender.FragmentMessage(msg.data(), msg.size());
    EXPECT_EQ(fragments.size(), 1);
}

// Test empty message
TEST(FragmentTest, EmptyMessage) {
    FragmentedSender sender;

    auto fragments = sender.FragmentMessage(nullptr, 0);
    EXPECT_TRUE(fragments.empty());

    std::vector<uint8_t> empty;
    fragments = sender.FragmentMessage(empty.data(), 0);
    EXPECT_TRUE(fragments.empty());
}

// Test message too large
TEST(FragmentTest, MessageTooLarge) {
    FragmentedSender sender;

    std::vector<uint8_t> huge(kMaxMessageSize + 1);
    auto fragments = sender.FragmentMessage(huge.data(), huge.size());
    EXPECT_TRUE(fragments.empty());
}

// Test invalid fragment data
TEST(FragmentTest, InvalidFragmentData) {
    ReassemblyManager reassembler;

    // Too short
    std::vector<uint8_t> short_data(kFragmentHeaderSize - 1);
    auto result = reassembler.ProcessFragment(short_data.data(),
                                               short_data.size());
    EXPECT_FALSE(result.complete);

    // Wrong magic
    std::vector<uint8_t> bad_magic(kFragmentHeaderSize + 10);
    reinterpret_cast<FragmentHeader*>(bad_magic.data())->magic = 0xDEADBEEF;
    result = reassembler.ProcessFragment(bad_magic.data(), bad_magic.size());
    EXPECT_FALSE(result.complete);

    // Null data
    result = reassembler.ProcessFragment(nullptr, 100);
    EXPECT_FALSE(result.complete);
}

}  // namespace
}  // namespace imujoco::driver
