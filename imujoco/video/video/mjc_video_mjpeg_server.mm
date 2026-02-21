// mjc_video_mjpeg_server.mm
// MJPEG-over-HTTP server implementation.
//
// HTTP response format (multipart/x-mixed-replace):
//   HTTP/1.1 200 OK\r\n
//   Content-Type: multipart/x-mixed-replace; boundary=<boundary>\r\n
//   \r\n
//   --<boundary>\r\n
//   Content-Type: image/jpeg\r\n
//   Content-Length: <size>\r\n
//   \r\n
//   <JPEG data>\r\n
//   --<boundary>\r\n
//   ...

#include "mjc_video_mjpeg_server.h"

#include <arpa/inet.h>
#include <cstring>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <os/log.h>

// MARK: - Factory Methods

MJVideoMJPEGServer *MJVideoMJPEGServer::create() {
  return new MJVideoMJPEGServer();
}

void MJVideoMJPEGServer::destroy(MJVideoMJPEGServer *server) { delete server; }

// MARK: - Construction / Destruction

MJVideoMJPEGServer::MJVideoMJPEGServer()
    : listen_fd_(-1), port_(0), active_(false) {}

MJVideoMJPEGServer::~MJVideoMJPEGServer() { Stop(); }

// MARK: - Start / Stop

bool MJVideoMJPEGServer::Start(uint16_t port) {
  if (active_.load(std::memory_order_acquire))
    return true;

  listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    os_log_error(OS_LOG_DEFAULT, "MJPEG: Failed to create socket");
    return false;
  }

  int reuse = 1;
  setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);

  if (bind(listen_fd_, reinterpret_cast<struct sockaddr *>(&addr),
           sizeof(addr)) < 0) {
    os_log_error(OS_LOG_DEFAULT, "MJPEG: Failed to bind port %u", port);
    close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }

  if (listen(listen_fd_, 4) < 0) {
    os_log_error(OS_LOG_DEFAULT, "MJPEG: Failed to listen");
    close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }

  port_ = port;
  active_.store(true, std::memory_order_release);
  accept_thread_ = std::thread([this] { AcceptLoop(); });

  os_log_info(OS_LOG_DEFAULT, "MJPEG: Server started on port %u", port);
  return true;
}

void MJVideoMJPEGServer::Stop() {
  active_.store(false, std::memory_order_release);

  if (listen_fd_ >= 0) {
    close(listen_fd_);
    listen_fd_ = -1;
  }

  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }

  std::lock_guard<std::mutex> lock(clients_mutex_);
  for (int fd : client_fds_) {
    close(fd);
  }
  client_fds_.clear();

  os_log_info(OS_LOG_DEFAULT, "MJPEG: Server stopped");
}

bool MJVideoMJPEGServer::IsActive() const {
  return active_.load(std::memory_order_acquire);
}

bool MJVideoMJPEGServer::HasReceiver() const {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  return !client_fds_.empty();
}

// MARK: - Accept Loop

void MJVideoMJPEGServer::AcceptLoop() {
  while (active_.load(std::memory_order_acquire)) {
    struct sockaddr_in client_addr {};
    socklen_t addr_len = sizeof(client_addr);

    int client_fd =
        accept(listen_fd_, reinterpret_cast<struct sockaddr *>(&client_addr),
               &addr_len);
    if (client_fd < 0) {
      if (active_.load(std::memory_order_acquire)) {
        os_log_error(OS_LOG_DEFAULT, "MJPEG: accept() failed");
      }
      break;
    }

    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, addr_str, sizeof(addr_str));
    os_log_info(OS_LOG_DEFAULT, "MJPEG: Client connected from %s:%u",
                addr_str, ntohs(client_addr.sin_port));

    // Suppress SIGPIPE on this socket (Apple platforms)
    int nosigpipe = 1;
    setsockopt(client_fd, SOL_SOCKET, SO_NOSIGPIPE, &nosigpipe,
               sizeof(nosigpipe));

    // TCP_NODELAY for lower latency
    int nodelay = 1;
    setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    // Send timeout: prevent slow clients from blocking the capture thread
    struct timeval snd_tv;
    snd_tv.tv_sec = 0;
    snd_tv.tv_usec = 100000;  // 100ms
    setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &snd_tv, sizeof(snd_tv));

    // Read the HTTP request (consume it, we don't need the content)
    char buf[4096];
    recv(client_fd, buf, sizeof(buf) - 1, 0);

    // Send HTTP response header
    std::string header = "HTTP/1.1 200 OK\r\n"
                         "Content-Type: multipart/x-mixed-replace; boundary=";
    header += kBoundary;
    header += "\r\n"
              "Cache-Control: no-cache, no-store\r\n"
              "Connection: keep-alive\r\n"
              "Pragma: no-cache\r\n"
              "\r\n";

    ssize_t sent = send(client_fd, header.c_str(), header.size(), 0);
    if (sent <= 0) {
      os_log_error(OS_LOG_DEFAULT, "MJPEG: Failed to send HTTP header");
      close(client_fd);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      client_fds_.push_back(client_fd);
    }
  }
}

// MARK: - Send JPEG Frame

/// Helper to send all bytes on a socket. Returns false on failure.
static bool SendAll(int fd, const void *data, size_t size) {
  const uint8_t *ptr = static_cast<const uint8_t *>(data);
  size_t remaining = size;
  while (remaining > 0) {
    ssize_t sent = send(fd, ptr, remaining, 0);
    if (sent <= 0)
      return false;
    ptr += sent;
    remaining -= static_cast<size_t>(sent);
  }
  return true;
}

bool MJVideoMJPEGServer::SendJPEG(const uint8_t *data, size_t size) {
  if (!active_.load(std::memory_order_acquire))
    return false;

  // Build multipart part header
  char part_header[256];
  int hdr_len =
      snprintf(part_header, sizeof(part_header),
               "--%s\r\n"
               "Content-Type: image/jpeg\r\n"
               "Content-Length: %zu\r\n"
               "\r\n",
               kBoundary, size);

  std::lock_guard<std::mutex> lock(clients_mutex_);
  if (client_fds_.empty())
    return false;

  // Send to all clients, remove dead ones
  auto it = client_fds_.begin();
  while (it != client_fds_.end()) {
    bool ok = SendAll(*it, part_header, static_cast<size_t>(hdr_len)) &&
              SendAll(*it, data, size) && SendAll(*it, "\r\n", 2);
    if (!ok) {
      os_log_info(OS_LOG_DEFAULT, "MJPEG: Client disconnected");
      close(*it);
      it = client_fds_.erase(it);
    } else {
      ++it;
    }
  }

  return true;
}
