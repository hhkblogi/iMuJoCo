// echo_server.cc
// Simple state receiver - prints StatePackets received from iMuJoCo
//
// Usage: ./echo_server --host 192.168.65.111 --port 9000

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <vector>

#include <flatbuffers/flatbuffers.h>
#include "imujoco/driver/args.h"
#include "control_generated.h"
#include "state_generated.h"
#include "fragment.h"

using namespace imujoco::driver;

std::atomic<bool> running{true};

void signalHandler(int) {
    running = false;
}

int main(int argc, char* argv[]) {
    Args args(argc, argv);

    std::string host = args.get("host", "127.0.0.1");
    uint16_t port = static_cast<uint16_t>(args.get_int("port", 9000));

    if (args.has("help")) {
        std::cout << "Usage: " << args.program()
                  << " [--host HOST] [--port PORT]\n"
                  << "  --host  Simulation host (default: 127.0.0.1)\n"
                  << "  --port  Simulation port (default: 9000)\n";
        return 0;
    }

    std::cout << "State Receiver - Connecting to " << host << ":" << port << std::endl;

    // Create socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return 1;
    }

    // Bind to ephemeral port
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = 0;

    if (bind(sock, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(sock);
        return 1;
    }

    // Set up remote address
    sockaddr_in remote_addr{};
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &remote_addr.sin_addr) != 1) {
        std::cerr << "Invalid host address: " << host << std::endl;
        close(sock);
        return 1;
    }

    // Set up signal handler
    std::signal(SIGINT, signalHandler);

    // Buffers
    std::vector<uint8_t> recv_buffer(kMaxUDPPayload);
    flatbuffers::FlatBufferBuilder builder(256);
    FragmentedSender sender;
    ReassemblyManager reassembler;

    int packets_received = 0;
    uint32_t sequence = 0;

    std::cout << "Receiving StatePackets (ctrl shows what iMuJoCo is applying)..." << std::endl;
    std::cout << "Press Ctrl+C to exit.\n" << std::endl;

    while (running) {
        // Send a minimal control packet to keep receiving states
        // (iMuJoCo only sends StatePackets to clients that send ControlPackets)
        builder.Clear();
        auto ctrl_vec = builder.CreateVector(std::vector<double>{});
        auto control = imujoco::schema::CreateControlPacket(builder, sequence++, ctrl_vec);
        imujoco::schema::FinishControlPacketBuffer(builder, control);

        auto fragments = sender.FragmentMessage(builder.GetBufferPointer(), builder.GetSize());
        for (const auto& frag : fragments) {
            sendto(sock, frag.data(), frag.size(), 0,
                   reinterpret_cast<const sockaddr*>(&remote_addr), sizeof(remote_addr));
        }

        // Receive with timeout
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sock, &readfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 100000;  // 100ms

        int sel = select(sock + 1, &readfds, nullptr, nullptr, &tv);
        if (sel <= 0) continue;

        sockaddr_in sender_addr{};
        socklen_t sender_len = sizeof(sender_addr);
        ssize_t recv_len = recvfrom(sock, recv_buffer.data(), recv_buffer.size(), 0,
                                     reinterpret_cast<sockaddr*>(&sender_addr), &sender_len);

        if (recv_len <= 0) continue;

        // Process fragment
        auto result = reassembler.ProcessFragment(recv_buffer.data(), static_cast<size_t>(recv_len));
        if (!result.complete) continue;

        // Verify and parse StatePacket
        flatbuffers::Verifier verifier(result.data, result.size);
        if (!imujoco::schema::VerifyStatePacketBuffer(verifier)) {
            std::cerr << "Invalid StatePacket" << std::endl;
            continue;
        }

        auto state = imujoco::schema::GetStatePacket(result.data);
        packets_received++;

        // Print received state
        std::cout << "[" << packets_received << "] "
                  << "seq=" << state->sequence()
                  << " time=" << state->time();

        if (state->qpos() && state->qpos()->size() > 0) {
            std::cout << " qpos=[";
            for (size_t i = 0; i < state->qpos()->size() && i < 6; i++) {
                std::cout << (i > 0 ? ", " : "") << state->qpos()->Get(i);
            }
            if (state->qpos()->size() > 6) std::cout << ", ...";
            std::cout << "]";
        }

        if (state->ctrl() && state->ctrl()->size() > 0) {
            std::cout << " ctrl=[";
            for (size_t i = 0; i < state->ctrl()->size() && i < 4; i++) {
                std::cout << (i > 0 ? ", " : "") << state->ctrl()->Get(i);
            }
            if (state->ctrl()->size() > 4) std::cout << ", ...";
            std::cout << "]";
        }

        std::cout << std::endl;
    }

    std::cout << "\nExiting. Packets received: " << packets_received << std::endl;

    close(sock);
    return 0;
}
