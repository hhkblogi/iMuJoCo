// mjc_video_mjpeg_server.h
// MJPEG-over-HTTP server for VLC/browser streaming.
//
// Serves JPEG frames as a multipart/x-mixed-replace HTTP stream.
// VLC plays it via: http://<device-ip>:<port>/
// Much simpler and more reliable than RTP/RTSP for JPEG streaming.

#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#if __has_attribute(swift_attr)
#define MJC_MJPEG_SWIFT_IMMORTAL_REFERENCE                                     \
  __attribute__((swift_attr("import_reference")))                              \
  __attribute__((swift_attr("retain:immortal")))                               \
  __attribute__((swift_attr("release:immortal")))
#else
#define MJC_MJPEG_SWIFT_IMMORTAL_REFERENCE
#endif

/// MJPEG-over-HTTP server.
///
/// Accepts TCP connections, sends HTTP response headers with
/// `Content-Type: multipart/x-mixed-replace`, then streams JPEG frames
/// as multipart parts. Supports multiple simultaneous clients.
class MJC_MJPEG_SWIFT_IMMORTAL_REFERENCE MJVideoMJPEGServer {
public:
  static MJVideoMJPEGServer *create();
  static void destroy(MJVideoMJPEGServer *server);

  /// Start listening on the given TCP port.
  bool Start(uint16_t port);

  /// Stop the server and disconnect all clients.
  void Stop();

  /// Whether the server is actively listening.
  bool IsActive() const;

  /// Whether any client is connected.
  bool HasReceiver() const;

  /// Send a JPEG frame to all connected clients.
  /// Disconnects clients that fail to receive.
  bool SendJPEG(const uint8_t *data, size_t size);

  /// Get the listening port.
  uint16_t GetPort() const { return port_; }

private:
  MJVideoMJPEGServer();
  ~MJVideoMJPEGServer();

  void AcceptLoop();

  int listen_fd_;
  uint16_t port_;
  std::atomic<bool> active_;
  std::thread accept_thread_;

  mutable std::mutex clients_mutex_;
  std::vector<int> client_fds_;

  static constexpr const char *kBoundary = "mjpeg_frame_boundary";
};
