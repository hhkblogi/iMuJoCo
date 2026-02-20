// mjc_video_rtp_transport.h
// RTP video transport (RFC 3550) with JPEG payload (RFC 2435)
//
// Sends JPEG-encoded frames as RTP packets with proper sequencing,
// timestamping (90kHz clock), and RFC 2435 JPEG payload format.
// Compatible with VLC media player and other standard RTP receivers.
//
// Used in conjunction with the RTSP server for session negotiation.

#ifndef IMUJOCO_VIDEO_RTP_TRANSPORT_H_
#define IMUJOCO_VIDEO_RTP_TRANSPORT_H_

#include "mjc_video_transport.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <netinet/in.h>
#include <vector>

// Swift C++ interop: reference semantics for non-copyable types
#if __has_attribute(swift_attr)
#define MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE __attribute__((swift_attr("import_reference"))) __attribute__((swift_attr("retain:immortal"))) __attribute__((swift_attr("release:immortal")))
#endif
#ifndef MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE
#define MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE
#endif

/// RTP video transport with JPEG payload (RFC 2435).
///
/// Sends JPEG frames over RTP/UDP. Each JPEG frame is split into
/// RTP packets with proper JPEG payload headers. The 90kHz RTP
/// timestamp clock is derived from simulation time.
///
/// Thread safety:
///   - Start()/Stop(): call from any thread, not concurrent with SendFrame
///   - SendFrame(): call from video capture thread only
///   - AddClient()/RemoveClient(): safe from any thread (RTSP server thread)
///   - HasReceiver()/IsActive(): safe from any thread
class MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE MJVideoRTPTransport : public MJVideoTransport {
public:
    /// Create a new RTP transport instance.
    static MJVideoRTPTransport* create();
    /// Destroy an RTP transport instance.
    static void destroy(MJVideoRTPTransport* transport);

    ~MJVideoRTPTransport() override;

    bool Start(uint16_t port) override;
    void Stop() override;
    bool SendFrame(const MJVideoFrameDesc& desc,
                   const uint8_t* data, size_t size) override;
    bool HasReceiver() const override;
    bool IsActive() const override;

    /// Add an RTP client (called by RTSP server on PLAY).
    /// @param addr Client address (IP + port for RTP data)
    void AddClient(const struct sockaddr_in& addr);

    /// Remove an RTP client (called by RTSP server on TEARDOWN).
    /// @param addr Client address to remove
    void RemoveClient(const struct sockaddr_in& addr);

    /// Get the current RTP server port (for SDP generation).
    uint16_t GetPort() const { return port_; }

private:
    MJVideoRTPTransport();

    /// Send a single RTP packet.
    void SendRTPPacket(const uint8_t* payload, size_t payload_size,
                       uint32_t timestamp, bool marker,
                       const struct sockaddr_in& dest);

    /// Extract JPEG quantization tables from JFIF data.
    /// Returns offset to start of scan data (after SOS marker).
    static size_t FindJPEGScanData(const uint8_t* data, size_t size,
                                    std::vector<uint8_t>& luma_qt,
                                    std::vector<uint8_t>& chroma_qt);

    int socket_fd_;
    uint16_t port_;
    std::atomic<bool> active_;

    // RTP state
    uint16_t sequence_number_;
    uint32_t ssrc_;

    // Client list (protected by mutex for RTSP thread safety)
    mutable std::mutex clients_mutex_;
    std::vector<struct sockaddr_in> clients_;

    // Scratch buffer for RTP packet assembly
    std::vector<uint8_t> packet_buffer_;
};

#endif  // IMUJOCO_VIDEO_RTP_TRANSPORT_H_
