// mjc_video_rtsp_server.mm
// Minimal RTSP server implementation for VLC compatibility
//
// Supports: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN
// SDP describes a single JPEG video stream over RTP/UDP

#include "mjc_video_rtsp_server.h"

#include <arpa/inet.h>
#include <cstring>
#include <random>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

#include <os/log.h>

static constexpr uint8_t kJPEGPayloadType = 26;

// MARK: - Factory Methods

MJVideoRTSPServer* MJVideoRTSPServer::create(MJVideoRTPTransport* rtpTransport) {
    return new MJVideoRTSPServer(rtpTransport);
}

void MJVideoRTSPServer::destroy(MJVideoRTSPServer* server) {
    delete server;
}

// MARK: - Construction / Destruction

MJVideoRTSPServer::MJVideoRTSPServer(MJVideoRTPTransport* rtpTransport)
    : rtp_transport_(rtpTransport),
      listen_fd_(-1),
      port_(0),
      active_(false) {}

MJVideoRTSPServer::~MJVideoRTSPServer() {
    Stop();
}

// MARK: - Start / Stop

bool MJVideoRTSPServer::Start(uint16_t port, uint16_t width, uint16_t height) {
    if (active_.load(std::memory_order_acquire)) return true;

    (void)width;   // Reserved for future SDP use
    (void)height;

    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        os_log_error(OS_LOG_DEFAULT, "RTSP: Failed to create TCP socket");
        return false;
    }

    int reuse = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(listen_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        os_log_error(OS_LOG_DEFAULT, "RTSP: Failed to bind port %u", port);
        close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    if (listen(listen_fd_, 4) < 0) {
        os_log_error(OS_LOG_DEFAULT, "RTSP: Failed to listen");
        close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    port_ = port;
    active_.store(true, std::memory_order_release);

    accept_thread_ = std::thread([this] { AcceptLoop(); });

    os_log_info(OS_LOG_DEFAULT, "RTSP: Server started on port %u", port);
    return true;
}

void MJVideoRTSPServer::Stop() {
    active_.store(false, std::memory_order_release);

    // Close listen socket to unblock accept()
    if (listen_fd_ >= 0) {
        close(listen_fd_);
        listen_fd_ = -1;
    }

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }

    // Clean up sessions
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        for (auto& [id, session] : sessions_) {
            if (session.playing && rtp_transport_) {
                struct sockaddr_in rtp_addr = session.client_addr;
                rtp_addr.sin_port = htons(session.client_rtp_port);
                rtp_transport_->RemoveClient(rtp_addr);
            }
        }
        sessions_.clear();
    }

    os_log_info(OS_LOG_DEFAULT, "RTSP: Server stopped");
}

bool MJVideoRTSPServer::IsActive() const {
    return active_.load(std::memory_order_acquire);
}

// MARK: - Accept Loop

void MJVideoRTSPServer::AcceptLoop() {
    while (active_.load(std::memory_order_acquire)) {
        struct sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);

        int client_fd = accept(listen_fd_,
                               reinterpret_cast<struct sockaddr*>(&client_addr),
                               &addr_len);
        if (client_fd < 0) {
            if (active_.load(std::memory_order_acquire)) {
                os_log_error(OS_LOG_DEFAULT, "RTSP: accept() failed");
            }
            break;
        }

        char addr_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, addr_str, sizeof(addr_str));
        os_log_info(OS_LOG_DEFAULT, "RTSP: Client connected from %s:%u",
                    addr_str, ntohs(client_addr.sin_port));

        // Handle client in-line (simple sequential handling)
        // For multiple VLC clients, they each get their own TCP connection
        // but we handle them sequentially (fine for our use case).
        HandleClient(client_fd, client_addr);
    }
}

// MARK: - Client Handler

void MJVideoRTSPServer::HandleClient(int client_fd, struct sockaddr_in client_addr) {
    // Set receive timeout so we don't block forever
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::string buffer;
    buffer.reserve(4096);
    char recv_buf[2048];

    while (active_.load(std::memory_order_acquire)) {
        ssize_t n = recv(client_fd, recv_buf, sizeof(recv_buf) - 1, 0);
        if (n <= 0) break;

        recv_buf[n] = '\0';
        buffer.append(recv_buf, static_cast<size_t>(n));

        // Cap buffer to prevent unbounded growth from misbehaving clients
        if (buffer.size() > 16384) {
            os_log_error(OS_LOG_DEFAULT, "RTSP: Client buffer exceeded 16KB, disconnecting");
            break;
        }

        // Process complete requests (delimited by \r\n\r\n)
        size_t end_pos;
        while ((end_pos = buffer.find("\r\n\r\n")) != std::string::npos) {
            std::string request = buffer.substr(0, end_pos + 4);
            buffer.erase(0, end_pos + 4);

            auto req = ParseRequest(request);
            std::string response;

            if (req.method == "OPTIONS") {
                response = BuildResponse(req.cseq, 200, "OK",
                    "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n");

            } else if (req.method == "DESCRIBE") {
                std::string sdp = GenerateSDP(req.uri);
                response = BuildResponse(req.cseq, 200, "OK",
                    "Content-Type: application/sdp\r\n"
                    "Content-Length: " + std::to_string(sdp.size()) + "\r\n",
                    sdp);

            } else if (req.method == "SETUP") {
                // Parse client_port from Transport header
                // Example: Transport: RTP/AVP;unicast;client_port=5000-5001
                uint16_t rtp_port = 0, rtcp_port = 0;
                auto cp_pos = req.transport.find("client_port=");
                if (cp_pos != std::string::npos) {
                    std::string ports = req.transport.substr(cp_pos + 12);
                    if (sscanf(ports.c_str(), "%hu-%hu", &rtp_port, &rtcp_port) < 1) {
                        rtp_port = 0;
                    }
                    if (rtcp_port == 0) rtcp_port = rtp_port + 1;
                }

                if (rtp_port == 0) {
                    response = BuildResponse(req.cseq, 461, "Unsupported Transport");
                } else {
                    std::string session_id = GenerateSessionId();

                    MJRTSPSession session;
                    session.session_id = session_id;
                    session.client_addr = client_addr;
                    session.client_rtp_port = rtp_port;
                    session.client_rtcp_port = rtcp_port;
                    session.playing = false;

                    {
                        std::lock_guard<std::mutex> lock(sessions_mutex_);
                        sessions_[session_id] = session;
                    }

                    uint16_t server_rtp_port = rtp_transport_ ? rtp_transport_->GetPort() : 0;
                    response = BuildResponse(req.cseq, 200, "OK",
                        "Transport: RTP/AVP;unicast;client_port=" +
                        std::to_string(rtp_port) + "-" + std::to_string(rtcp_port) +
                        ";server_port=" + std::to_string(server_rtp_port) +
                        "-" + std::to_string(server_rtp_port + 1) + "\r\n"
                        "Session: " + session_id + ";timeout=60\r\n");
                }

            } else if (req.method == "PLAY") {
                std::lock_guard<std::mutex> lock(sessions_mutex_);
                auto it = sessions_.find(req.session);
                if (it != sessions_.end() && rtp_transport_) {
                    it->second.playing = true;

                    // Register client's RTP address with the transport
                    struct sockaddr_in rtp_addr = it->second.client_addr;
                    rtp_addr.sin_port = htons(it->second.client_rtp_port);
                    rtp_transport_->AddClient(rtp_addr);

                    response = BuildResponse(req.cseq, 200, "OK",
                        "Session: " + req.session + "\r\n");
                } else {
                    response = BuildResponse(req.cseq, 454, "Session Not Found");
                }

            } else if (req.method == "TEARDOWN") {
                std::lock_guard<std::mutex> lock(sessions_mutex_);
                auto it = sessions_.find(req.session);
                if (it != sessions_.end()) {
                    if (it->second.playing && rtp_transport_) {
                        struct sockaddr_in rtp_addr = it->second.client_addr;
                        rtp_addr.sin_port = htons(it->second.client_rtp_port);
                        rtp_transport_->RemoveClient(rtp_addr);
                    }
                    sessions_.erase(it);
                }
                response = BuildResponse(req.cseq, 200, "OK");
                send(client_fd, response.c_str(), response.size(), 0);
                shutdown(client_fd, SHUT_WR);  // Flush response before close
                close(client_fd);
                return;

            } else {
                response = BuildResponse(req.cseq, 405, "Method Not Allowed");
            }

            send(client_fd, response.c_str(), response.size(), 0);
        }
    }

    close(client_fd);
}

// MARK: - Request Parsing

MJVideoRTSPServer::RTSPRequest MJVideoRTSPServer::ParseRequest(const std::string& request) {
    RTSPRequest req;
    req.cseq = 0;

    std::istringstream stream(request);
    std::string line;

    // First line: METHOD URI RTSP/1.0
    if (std::getline(stream, line)) {
        // Trim \r
        if (!line.empty() && line.back() == '\r') line.pop_back();

        auto sp1 = line.find(' ');
        auto sp2 = line.find(' ', sp1 + 1);
        if (sp1 != std::string::npos) {
            req.method = line.substr(0, sp1);
            if (sp2 != std::string::npos) {
                req.uri = line.substr(sp1 + 1, sp2 - sp1 - 1);
            }
        }
    }

    // Parse headers
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) break;

        auto colon = line.find(':');
        if (colon == std::string::npos) continue;

        std::string key = line.substr(0, colon);
        std::string value = line.substr(colon + 1);
        // Trim leading whitespace from value
        auto vstart = value.find_first_not_of(' ');
        if (vstart != std::string::npos) value = value.substr(vstart);

        if (key == "CSeq") {
            try { req.cseq = std::stoi(value); } catch (...) { req.cseq = 0; }
        } else if (key == "Session") {
            // Session may have ";timeout=60" suffix
            auto semi = value.find(';');
            req.session = (semi != std::string::npos) ? value.substr(0, semi) : value;
        } else if (key == "Transport") {
            req.transport = value;
        }
    }

    return req;
}

// MARK: - Response Building

std::string MJVideoRTSPServer::BuildResponse(int cseq, int status_code,
                                               const std::string& status_text,
                                               const std::string& headers,
                                               const std::string& body) const {
    std::string resp = "RTSP/1.0 " + std::to_string(status_code) + " " + status_text + "\r\n";
    resp += "CSeq: " + std::to_string(cseq) + "\r\n";
    resp += headers;
    resp += "\r\n";
    if (!body.empty()) {
        resp += body;
    }
    return resp;
}

// MARK: - SDP Generation

std::string MJVideoRTSPServer::GenerateSDP(const std::string& uri) const {
    uint16_t rtp_port = rtp_transport_ ? rtp_transport_->GetPort() : 0;

    std::ostringstream sdp;
    sdp << "v=0\r\n";
    sdp << "o=- 0 0 IN IP4 0.0.0.0\r\n";
    sdp << "s=iMuJoCo Simulation\r\n";
    sdp << "c=IN IP4 0.0.0.0\r\n";
    sdp << "t=0 0\r\n";
    sdp << "m=video " << rtp_port << " RTP/AVP " << static_cast<int>(kJPEGPayloadType) << "\r\n";
    sdp << "a=rtpmap:" << static_cast<int>(kJPEGPayloadType) << " JPEG/90000\r\n";
    sdp << "a=framerate:10\r\n";
    sdp << "a=control:" << uri << "\r\n";
    return sdp.str();
}

// MARK: - Session ID Generation

std::string MJVideoRTSPServer::GenerateSessionId() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint64_t> dist;
    uint64_t id = dist(gen);

    char buf[17];
    snprintf(buf, sizeof(buf), "%016llX", static_cast<unsigned long long>(id));
    return std::string(buf);
}

