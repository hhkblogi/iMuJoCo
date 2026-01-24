// fragment_tests.mm
// Tests for UDP packet fragmentation and reassembly

#import <XCTest/XCTest.h>

#include "mjc_fragment.h"
#include "imujoco/schema/state_generated.h"
#include "imujoco/schema/control_generated.h"

#include <vector>
#include <cstring>
#include <random>

using namespace imujoco;

@interface FragmentTests : XCTestCase
@end

@implementation FragmentTests

#pragma mark - CRC-16 Tests

- (void)test_crc16_basic {
    // Test with known data
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = mj_fragment_crc16(data, sizeof(data));
    XCTAssertTrue(crc != 0, @"CRC should be non-zero for non-empty data");
}

- (void)test_crc16_consistency {
    // Same data should produce same CRC
    uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint16_t crc1 = mj_fragment_crc16(data, sizeof(data));
    uint16_t crc2 = mj_fragment_crc16(data, sizeof(data));
    XCTAssertEqual(crc1, crc2, @"Same data should produce same CRC");
}

- (void)test_crc16_different_data {
    uint8_t data1[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t data2[] = {0x01, 0x02, 0x03, 0x05};  // Last byte different
    uint16_t crc1 = mj_fragment_crc16(data1, sizeof(data1));
    uint16_t crc2 = mj_fragment_crc16(data2, sizeof(data2));
    XCTAssertNotEqual(crc1, crc2, @"Different data should produce different CRC");
}

#pragma mark - Fragment Header Tests

- (void)test_fragment_header_size {
    XCTAssertEqual(sizeof(MJFragmentHeader), (size_t)MJ_FRAGMENT_HEADER_SIZE,
                   @"Header struct should be 16 bytes");
}

- (void)test_fragment_header_checksum {
    MJFragmentHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = MJ_PACKET_MAGIC_FRAG;
    header.message_id = 123;
    header.fragment_index = 0;
    header.fragment_count = 3;
    header.total_size = 5000;
    header.payload_size = 1456;
    header.checksum = 0;

    uint16_t checksum = mj_fragment_header_checksum(&header);
    header.checksum = checksum;

    XCTAssertTrue(mj_fragment_header_valid(&header), @"Header with correct checksum should be valid");
}

- (void)test_fragment_header_invalid_checksum {
    MJFragmentHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = MJ_PACKET_MAGIC_FRAG;
    header.message_id = 123;
    header.fragment_index = 0;
    header.fragment_count = 3;
    header.total_size = 5000;
    header.payload_size = 1456;
    header.checksum = 0xDEAD;  // Wrong checksum

    XCTAssertFalse(mj_fragment_header_valid(&header), @"Header with wrong checksum should be invalid");
}

- (void)test_fragment_header_invalid_magic {
    MJFragmentHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = 0x12345678;  // Wrong magic
    header.message_id = 123;
    header.checksum = mj_fragment_header_checksum(&header);

    XCTAssertFalse(mj_fragment_header_valid(&header), @"Header with wrong magic should be invalid");
}

#pragma mark - ReassemblySlot Tests

- (void)test_reassembly_slot_fragment_received {
    ReassemblySlot slot;
    slot.Reset();

    // Mark some fragments as received
    slot.MarkFragmentReceived(0);
    slot.MarkFragmentReceived(5);
    slot.MarkFragmentReceived(63);  // Last in primary mask
    slot.MarkFragmentReceived(64);  // First in extended mask
    slot.MarkFragmentReceived(200);

    XCTAssertTrue(slot.IsFragmentReceived(0), @"Fragment 0 should be marked");
    XCTAssertTrue(slot.IsFragmentReceived(5), @"Fragment 5 should be marked");
    XCTAssertTrue(slot.IsFragmentReceived(63), @"Fragment 63 should be marked");
    XCTAssertTrue(slot.IsFragmentReceived(64), @"Fragment 64 should be marked");
    XCTAssertTrue(slot.IsFragmentReceived(200), @"Fragment 200 should be marked");

    XCTAssertFalse(slot.IsFragmentReceived(1), @"Fragment 1 should not be marked");
    XCTAssertFalse(slot.IsFragmentReceived(100), @"Fragment 100 should not be marked");
}

#pragma mark - ReassemblyManager Tests

- (void)test_reassembly_single_fragment {
    ReassemblyManager manager;

    // Create a small message that fits in one fragment
    std::vector<uint8_t> payload(100, 0xAB);

    // Build fragment packet
    std::vector<uint8_t> packet(MJ_FRAGMENT_HEADER_SIZE + payload.size());
    MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet.data());
    header->magic = MJ_PACKET_MAGIC_FRAG;
    header->message_id = 1;
    header->fragment_index = 0;
    header->fragment_count = 1;
    header->total_size = static_cast<uint32_t>(payload.size());
    header->payload_size = static_cast<uint16_t>(payload.size());
    header->checksum = 0;
    header->checksum = mj_fragment_header_checksum(header);

    memcpy(packet.data() + MJ_FRAGMENT_HEADER_SIZE, payload.data(), payload.size());

    // Process fragment
    size_t message_size = 0;
    const uint8_t* result = manager.ProcessFragment(packet.data(), packet.size(), &message_size);

    XCTAssertTrue(result != nullptr, @"Single fragment should complete immediately");
    XCTAssertEqual(message_size, payload.size(), @"Message size should match payload size");
    XCTAssertEqual(memcmp(result, payload.data(), payload.size()), 0,
                   @"Reassembled data should match original");
}

- (void)test_reassembly_multiple_fragments_in_order {
    ReassemblyManager manager;

    // Create a message that needs 3 fragments
    size_t total_size = MJ_MAX_FRAGMENT_PAYLOAD * 2 + 500;  // ~3400 bytes
    std::vector<uint8_t> original(total_size);
    for (size_t i = 0; i < total_size; i++) {
        original[i] = static_cast<uint8_t>(i & 0xFF);
    }

    uint16_t msg_id = 42;
    uint8_t fragment_count = 3;
    const uint8_t* result = nullptr;
    size_t message_size = 0;

    for (uint8_t i = 0; i < fragment_count; i++) {
        size_t offset = i * MJ_MAX_FRAGMENT_PAYLOAD;
        size_t payload_size = std::min(static_cast<size_t>(MJ_MAX_FRAGMENT_PAYLOAD),
                                       total_size - offset);

        std::vector<uint8_t> packet(MJ_FRAGMENT_HEADER_SIZE + payload_size);
        MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet.data());
        header->magic = MJ_PACKET_MAGIC_FRAG;
        header->message_id = msg_id;
        header->fragment_index = i;
        header->fragment_count = fragment_count;
        header->total_size = static_cast<uint32_t>(total_size);
        header->payload_size = static_cast<uint16_t>(payload_size);
        header->checksum = 0;
        header->checksum = mj_fragment_header_checksum(header);

        memcpy(packet.data() + MJ_FRAGMENT_HEADER_SIZE, original.data() + offset, payload_size);

        result = manager.ProcessFragment(packet.data(), packet.size(), &message_size);

        if (i < fragment_count - 1) {
            XCTAssertTrue(result == nullptr, @"Should not complete before last fragment");
        }
    }

    XCTAssertTrue(result != nullptr, @"Should complete after last fragment");
    XCTAssertEqual(message_size, total_size, @"Message size should match");
    XCTAssertEqual(memcmp(result, original.data(), total_size), 0,
                   @"Reassembled data should match original");
}

- (void)test_reassembly_out_of_order {
    ReassemblyManager manager;

    size_t total_size = MJ_MAX_FRAGMENT_PAYLOAD * 2 + 100;
    std::vector<uint8_t> original(total_size);
    for (size_t i = 0; i < total_size; i++) {
        original[i] = static_cast<uint8_t>(i & 0xFF);
    }

    uint16_t msg_id = 99;
    uint8_t fragment_count = 3;

    // Send fragments in reverse order: 2, 1, 0
    int order[] = {2, 1, 0};
    const uint8_t* result = nullptr;
    size_t message_size = 0;

    for (int idx = 0; idx < 3; idx++) {
        uint8_t i = order[idx];
        size_t offset = i * MJ_MAX_FRAGMENT_PAYLOAD;
        size_t payload_size = std::min(static_cast<size_t>(MJ_MAX_FRAGMENT_PAYLOAD),
                                       total_size - offset);

        std::vector<uint8_t> packet(MJ_FRAGMENT_HEADER_SIZE + payload_size);
        MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet.data());
        header->magic = MJ_PACKET_MAGIC_FRAG;
        header->message_id = msg_id;
        header->fragment_index = i;
        header->fragment_count = fragment_count;
        header->total_size = static_cast<uint32_t>(total_size);
        header->payload_size = static_cast<uint16_t>(payload_size);
        header->checksum = 0;
        header->checksum = mj_fragment_header_checksum(header);

        memcpy(packet.data() + MJ_FRAGMENT_HEADER_SIZE, original.data() + offset, payload_size);

        result = manager.ProcessFragment(packet.data(), packet.size(), &message_size);
    }

    XCTAssertTrue(result != nullptr, @"Should complete after all fragments received");
    XCTAssertEqual(message_size, total_size, @"Message size should match");
    XCTAssertEqual(memcmp(result, original.data(), total_size), 0,
                   @"Out-of-order reassembly should match original");
}

- (void)test_reassembly_duplicate_fragment {
    ReassemblyManager manager;

    size_t total_size = MJ_MAX_FRAGMENT_PAYLOAD * 2;
    std::vector<uint8_t> original(total_size, 0x55);

    uint16_t msg_id = 77;

    // Send fragment 0 twice, then fragment 1
    for (int attempt = 0; attempt < 2; attempt++) {
        // Fragment 0
        std::vector<uint8_t> packet0(MJ_FRAGMENT_HEADER_SIZE + MJ_MAX_FRAGMENT_PAYLOAD);
        MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet0.data());
        header->magic = MJ_PACKET_MAGIC_FRAG;
        header->message_id = msg_id;
        header->fragment_index = 0;
        header->fragment_count = 2;
        header->total_size = static_cast<uint32_t>(total_size);
        header->payload_size = MJ_MAX_FRAGMENT_PAYLOAD;
        header->checksum = 0;
        header->checksum = mj_fragment_header_checksum(header);
        memcpy(packet0.data() + MJ_FRAGMENT_HEADER_SIZE, original.data(), MJ_MAX_FRAGMENT_PAYLOAD);

        size_t message_size = 0;
        const uint8_t* result = manager.ProcessFragment(packet0.data(), packet0.size(), &message_size);
        XCTAssertTrue(result == nullptr, @"Should not complete with only fragment 0");
    }

    // Fragment 1
    std::vector<uint8_t> packet1(MJ_FRAGMENT_HEADER_SIZE + MJ_MAX_FRAGMENT_PAYLOAD);
    MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet1.data());
    header->magic = MJ_PACKET_MAGIC_FRAG;
    header->message_id = msg_id;
    header->fragment_index = 1;
    header->fragment_count = 2;
    header->total_size = static_cast<uint32_t>(total_size);
    header->payload_size = MJ_MAX_FRAGMENT_PAYLOAD;
    header->checksum = 0;
    header->checksum = mj_fragment_header_checksum(header);
    memcpy(packet1.data() + MJ_FRAGMENT_HEADER_SIZE,
           original.data() + MJ_MAX_FRAGMENT_PAYLOAD, MJ_MAX_FRAGMENT_PAYLOAD);

    size_t message_size = 0;
    const uint8_t* result = manager.ProcessFragment(packet1.data(), packet1.size(), &message_size);

    XCTAssertTrue(result != nullptr, @"Should complete after all unique fragments");
    XCTAssertEqual(message_size, total_size, @"Message size should match");
}

- (void)test_reassembly_invalid_checksum {
    ReassemblyManager manager;

    std::vector<uint8_t> packet(MJ_FRAGMENT_HEADER_SIZE + 100);
    MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet.data());
    header->magic = MJ_PACKET_MAGIC_FRAG;
    header->message_id = 1;
    header->fragment_index = 0;
    header->fragment_count = 1;
    header->total_size = 100;
    header->payload_size = 100;
    header->checksum = 0xBAD0;  // Wrong checksum

    size_t message_size = 0;
    const uint8_t* result = manager.ProcessFragment(packet.data(), packet.size(), &message_size);

    XCTAssertTrue(result == nullptr, @"Invalid checksum should be rejected");
}

- (void)test_reassembly_invalid_fragment_index {
    ReassemblyManager manager;

    std::vector<uint8_t> packet(MJ_FRAGMENT_HEADER_SIZE + 100);
    MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet.data());
    header->magic = MJ_PACKET_MAGIC_FRAG;
    header->message_id = 1;
    header->fragment_index = 5;   // Invalid: >= fragment_count
    header->fragment_count = 3;
    header->total_size = 100;
    header->payload_size = 100;
    header->checksum = 0;
    header->checksum = mj_fragment_header_checksum(header);

    size_t message_size = 0;
    const uint8_t* result = manager.ProcessFragment(packet.data(), packet.size(), &message_size);

    XCTAssertTrue(result == nullptr, @"Invalid fragment index should be rejected");
}

- (void)test_reassembly_concurrent_messages {
    ReassemblyManager manager;

    // Two messages interleaved
    size_t size1 = MJ_MAX_FRAGMENT_PAYLOAD + 100;
    size_t size2 = MJ_MAX_FRAGMENT_PAYLOAD + 200;
    std::vector<uint8_t> msg1(size1, 0x11);
    std::vector<uint8_t> msg2(size2, 0x22);

    // msg1 fragment 0
    {
        std::vector<uint8_t> packet(MJ_FRAGMENT_HEADER_SIZE + MJ_MAX_FRAGMENT_PAYLOAD);
        MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet.data());
        header->magic = MJ_PACKET_MAGIC_FRAG;
        header->message_id = 1;
        header->fragment_index = 0;
        header->fragment_count = 2;
        header->total_size = static_cast<uint32_t>(size1);
        header->payload_size = MJ_MAX_FRAGMENT_PAYLOAD;
        header->checksum = 0;
        header->checksum = mj_fragment_header_checksum(header);
        memcpy(packet.data() + MJ_FRAGMENT_HEADER_SIZE, msg1.data(), MJ_MAX_FRAGMENT_PAYLOAD);

        size_t message_size = 0;
        manager.ProcessFragment(packet.data(), packet.size(), &message_size);
    }

    // msg2 fragment 0
    {
        std::vector<uint8_t> packet(MJ_FRAGMENT_HEADER_SIZE + MJ_MAX_FRAGMENT_PAYLOAD);
        MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet.data());
        header->magic = MJ_PACKET_MAGIC_FRAG;
        header->message_id = 2;
        header->fragment_index = 0;
        header->fragment_count = 2;
        header->total_size = static_cast<uint32_t>(size2);
        header->payload_size = MJ_MAX_FRAGMENT_PAYLOAD;
        header->checksum = 0;
        header->checksum = mj_fragment_header_checksum(header);
        memcpy(packet.data() + MJ_FRAGMENT_HEADER_SIZE, msg2.data(), MJ_MAX_FRAGMENT_PAYLOAD);

        size_t message_size = 0;
        manager.ProcessFragment(packet.data(), packet.size(), &message_size);
    }

    XCTAssertEqual(manager.GetActiveSlotCount(), 2, @"Should have 2 active slots");

    // msg1 fragment 1 - should complete msg1
    {
        size_t remaining = size1 - MJ_MAX_FRAGMENT_PAYLOAD;
        std::vector<uint8_t> packet(MJ_FRAGMENT_HEADER_SIZE + remaining);
        MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet.data());
        header->magic = MJ_PACKET_MAGIC_FRAG;
        header->message_id = 1;
        header->fragment_index = 1;
        header->fragment_count = 2;
        header->total_size = static_cast<uint32_t>(size1);
        header->payload_size = static_cast<uint16_t>(remaining);
        header->checksum = 0;
        header->checksum = mj_fragment_header_checksum(header);
        memcpy(packet.data() + MJ_FRAGMENT_HEADER_SIZE,
               msg1.data() + MJ_MAX_FRAGMENT_PAYLOAD, remaining);

        size_t message_size = 0;
        const uint8_t* result = manager.ProcessFragment(packet.data(), packet.size(), &message_size);

        XCTAssertTrue(result != nullptr, @"msg1 should complete");
        XCTAssertEqual(message_size, size1, @"msg1 size should match");
        XCTAssertEqual(result[0], 0x11, @"msg1 content should match");
    }
}

#pragma mark - FlatBuffers Integration Tests

- (void)test_state_packet_fragmentation_roundtrip {
    // Create a large StatePacket that needs fragmentation
    flatbuffers::FlatBufferBuilder builder(8192);

    // Create large arrays (simulating a complex robot)
    std::vector<double> qpos(100);  // 100 generalized positions
    std::vector<double> qvel(100);  // 100 generalized velocities
    std::vector<double> ctrl(50);   // 50 actuators
    std::vector<double> sensordata(200);  // 200 sensor values

    for (int i = 0; i < 100; i++) {
        qpos[i] = static_cast<double>(i) * 0.1;
        qvel[i] = static_cast<double>(i) * 0.01;
    }
    for (int i = 0; i < 50; i++) {
        ctrl[i] = static_cast<double>(i) * 0.5;
    }
    for (int i = 0; i < 200; i++) {
        sensordata[i] = static_cast<double>(i) * 1.0;
    }

    auto qpos_vec = builder.CreateVector(qpos);
    auto qvel_vec = builder.CreateVector(qvel);
    auto ctrl_vec = builder.CreateVector(ctrl);
    auto sensor_vec = builder.CreateVector(sensordata);

    uint32_t sequence = 12345;
    double time = 1.234;
    double energy_pot = 100.5;
    double energy_kin = 50.25;

    auto state = imujoco::schema::CreateStatePacket(builder,
        sequence, time, energy_pot, energy_kin,
        qpos_vec, qvel_vec, ctrl_vec, sensor_vec);

    builder.Finish(state, imujoco::schema::StatePacketIdentifier());

    const uint8_t* original_data = builder.GetBufferPointer();
    size_t original_size = builder.GetSize();

    // Verify it needs fragmentation
    XCTAssertTrue(original_size > MJ_MAX_FRAGMENT_PAYLOAD,
                  @"StatePacket should be larger than one fragment (%zu bytes)", original_size);

    // Fragment the message
    uint8_t fragment_count = static_cast<uint8_t>(
        (original_size + MJ_MAX_FRAGMENT_PAYLOAD - 1) / MJ_MAX_FRAGMENT_PAYLOAD);
    uint16_t msg_id = 999;

    // Reassemble
    ReassemblyManager manager;
    const uint8_t* result = nullptr;
    size_t result_size = 0;

    for (uint8_t i = 0; i < fragment_count; i++) {
        size_t offset = i * MJ_MAX_FRAGMENT_PAYLOAD;
        size_t payload_size = std::min(static_cast<size_t>(MJ_MAX_FRAGMENT_PAYLOAD),
                                       original_size - offset);

        std::vector<uint8_t> packet(MJ_FRAGMENT_HEADER_SIZE + payload_size);
        MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(packet.data());
        header->magic = MJ_PACKET_MAGIC_FRAG;
        header->message_id = msg_id;
        header->fragment_index = i;
        header->fragment_count = fragment_count;
        header->total_size = static_cast<uint32_t>(original_size);
        header->payload_size = static_cast<uint16_t>(payload_size);
        header->checksum = 0;
        header->checksum = mj_fragment_header_checksum(header);

        memcpy(packet.data() + MJ_FRAGMENT_HEADER_SIZE, original_data + offset, payload_size);

        result = manager.ProcessFragment(packet.data(), packet.size(), &result_size);
    }

    XCTAssertTrue(result != nullptr, @"Should reassemble complete message");
    XCTAssertEqual(result_size, original_size, @"Size should match");

    // Verify the reassembled FlatBuffers is valid
    flatbuffers::Verifier verifier(result, result_size);
    XCTAssertTrue(imujoco::schema::VerifyStatePacketBuffer(verifier),
                  @"Reassembled buffer should be valid FlatBuffers");

    // Verify contents
    auto packet = imujoco::schema::GetStatePacket(result);
    XCTAssertEqual(packet->sequence(), sequence);
    XCTAssertEqualWithAccuracy(packet->time(), time, 0.0001);
    XCTAssertEqualWithAccuracy(packet->energy_potential(), energy_pot, 0.0001);
    XCTAssertEqualWithAccuracy(packet->energy_kinetic(), energy_kin, 0.0001);
    XCTAssertEqual(packet->qpos()->size(), 100u);
    XCTAssertEqual(packet->qvel()->size(), 100u);
    XCTAssertEqual(packet->ctrl()->size(), 50u);
    XCTAssertEqual(packet->sensordata()->size(), 200u);

    // Spot check some values
    XCTAssertEqualWithAccuracy(packet->qpos()->Get(50), 5.0, 0.0001);
    XCTAssertEqualWithAccuracy(packet->sensordata()->Get(100), 100.0, 0.0001);
}

- (void)test_control_packet_no_fragmentation_needed {
    // Small ControlPacket shouldn't need fragmentation
    flatbuffers::FlatBufferBuilder builder(256);

    std::vector<double> ctrl = {0.1, 0.2, 0.3, 0.4, 0.5};
    auto ctrl_vec = builder.CreateVector(ctrl);

    auto control = imujoco::schema::CreateControlPacket(builder, 42, ctrl_vec);
    builder.Finish(control, imujoco::schema::ControlPacketIdentifier());

    size_t size = builder.GetSize();
    XCTAssertTrue(size < MJ_MAX_FRAGMENT_PAYLOAD,
                  @"Small ControlPacket should fit in one fragment (%zu bytes)", size);
}

@end
