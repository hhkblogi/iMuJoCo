// mjc_video_rtsp_server.h
// Minimal RTSP server for VLC-compatible video streaming
//
// Handles RTSP session negotiation (DESCRIBE/SETUP/PLAY/TEARDOWN)
// and delegates RTP data delivery to MJVideoRTPTransport.
//
// VLC connects via: rtsp://<device-ip>:8554/camera0
//
// This is intentionally minimal — just enough for VLC/ffplay compatibility.
// Not a full RTSP implementation (no Range, no record, single media stream).

#ifndef IMUJOCO_VIDEO_RTSP_SERVER_H_
#define IMUJOCO_VIDEO_RTSP_SERVER_H_

#include "mjc_video_rtp_transport.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

// Swift C++ interop: reference semantics for non-copyable types
#if __has_attribute(swift_attr)
#define MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE __attribute__((swift_attr("import_reference"))) __attribute__((swift_attr("retain:immortal"))) __attribute__((swift_attr("release:immortal")))
#endif
#ifndef MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE
#define MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE
#endif

/// RTSP session state.
struct MJRTSPSession {
    std::string session_id;
    struct sockaddr_in client_addr;
    uint16_t client_rtp_port;     ///< Client's RTP port (from Transport header)
    uint16_t client_rtcp_port;    ///< Client's RTCP port
    bool playing;
};

/// Minimal RTSP server for video stream negotiation.
///
/// Listens on TCP port (default 8554) for RTSP requests. When a client
/// sends SETUP+PLAY, the server registers the client's RTP port with
/// the RTP transport, which then streams JPEG frames.
///
/// Thread safety:
///   - Start()/Stop(): call from any thread, not concurrently
///   - Other methods: internal (called from accept thread)
class MJC_VIDEO_SWIFT_IMMORTAL_REFERENCE MJVideoRTSPServer {
public:
    /// Create a new RTSP server.
    /// @param rtpTransport The RTP transport to register clients with
    static MJVideoRTSPServer* create(MJVideoRTPTransport* rtpTransport);
    /// Destroy an RTSP server.
    static void destroy(MJVideoRTSPServer* server);

    ~MJVideoRTSPServer();

    /// Start listening for RTSP connections.
    /// @param port TCP port to listen on (default 8554)
    /// @param width Stream width for SDP
    /// @param height Stream height for SDP
    /// @return true if started successfully
    bool Start(uint16_t port = 8554, uint16_t width = 256, uint16_t height = 256);

    /// Stop the RTSP server and close all connections.
    void Stop();

    /// Whether the server is running.
    bool IsActive() const;

private:
    explicit MJVideoRTSPServer(MJVideoRTPTransport* rtpTransport);

    /// Accept loop (runs on dedicated thread).
    void AcceptLoop();

    /// Handle a single client connection.
    void HandleClient(int client_fd, struct sockaddr_in client_addr);

    /// Parse an RTSP request line and headers.
    struct RTSPRequest {
        std::string method;
        std::string uri;
        int cseq;
        std::string session;
        std::string transport;
    };
    static RTSPRequest ParseRequest(const std::string& request);

    /// Build RTSP response.
    std::string BuildResponse(int cseq, int status_code, const std::string& status_text,
                              const std::string& headers = "",
                              const std::string& body = "") const;

    /// Generate SDP for DESCRIBE response.
    std::string GenerateSDP(const std::string& uri) const;

    /// Generate a unique session ID.
    static std::string GenerateSessionId();

    MJVideoRTPTransport* rtp_transport_;  // Not owned
    int listen_fd_;
    uint16_t port_;
    std::atomic<bool> active_;
    std::thread accept_thread_;

    // Active sessions
    mutable std::mutex sessions_mutex_;
    std::unordered_map<std::string, MJRTSPSession> sessions_;
};

#endif  // IMUJOCO_VIDEO_RTSP_SERVER_H_
