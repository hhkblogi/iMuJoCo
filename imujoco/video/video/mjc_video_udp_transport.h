// mjc_video_udp_transport.h
// Raw UDP video transport using FragmentedSender from mjc_fragment.h
//
// Sends [MJVideoFrameDesc + pixel data] as a single fragmented message.
// The receiver reassembles fragments and extracts the descriptor + pixels.
//
// Receiver discovery: waits for a "hello" packet from any client, then
// streams to that client's address. This mirrors the control channel's
// implicit client registration pattern.

#ifndef IMUJOCO_VIDEO_UDP_TRANSPORT_H_
#define IMUJOCO_VIDEO_UDP_TRANSPORT_H_

#include "mjc_video_transport.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <netinet/in.h>
#include <vector>

// Swift C++ interop: reference semantics for non-copyable types
#if __has_attribute(swift_attr)
#define MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE __attribute__((swift_attr("import_reference"))) __attribute__((swift_attr("retain:immortal"))) __attribute__((swift_attr("release:immortal")))
#else
#define MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE
#endif

// Forward declarations (avoid including mjc_fragment.h in the header)
namespace imujoco { class FragmentedSender; }

/// Raw UDP video transport.
///
/// Binds a UDP socket on the specified port and waits for a receiver
/// to send any packet (even 1 byte) to register its address. Once a
/// receiver is known, SendFrame() fragments the frame and sends it.
///
/// Thread safety:
///   - Start()/Stop(): call from any thread, not concurrent with SendFrame
///   - SendFrame(): call from video capture thread only
///   - HasReceiver()/IsActive(): safe from any thread
class MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE MJVideoUDPTransport : public MJVideoTransport {
public:
    /// Create a new transport instance. Caller must call destroy() when done.
    static MJVideoUDPTransport* create();
    /// Destroy a transport instance.
    static void destroy(MJVideoUDPTransport* transport);

    ~MJVideoUDPTransport() override;

    bool Start(uint16_t port) override;
    void Stop() override;
    bool SendFrame(const MJVideoFrameDesc& desc,
                   const uint8_t* data, size_t size) override;
    bool HasReceiver() const override;
    bool IsActive() const override;

private:
    MJVideoUDPTransport();

    /// Poll for incoming "hello" packets to discover receiver address.
    /// Called at the start of each SendFrame to check for new/changed receiver.
    void PollForReceiver();

    int socket_fd_;
    uint16_t port_;
    std::atomic<bool> active_;
    std::atomic<bool> has_receiver_;

    // Receiver address (set by first incoming packet)
    struct sockaddr_in receiver_addr_;

    // Fragment sender (owns its own buffer)
    std::unique_ptr<imujoco::FragmentedSender> sender_;

    // Scratch buffer for [desc + pixel data] concatenation
    std::vector<uint8_t> send_buffer_;
};

#endif  // IMUJOCO_VIDEO_UDP_TRANSPORT_H_
