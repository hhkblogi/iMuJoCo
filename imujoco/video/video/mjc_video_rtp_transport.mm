// mjc_video_rtp_transport.mm
// RTP transport implementation with RFC 7798 HEVC payload
//
// Input: HVCC format (4-byte big-endian length prefix + NAL unit data)
//
// RTP packet format (RFC 3550):
//   [12-byte RTP header][payload]
//
// HEVC payload format (RFC 7798):
//   Single NAL Unit Packet: [RTP header][NAL unit]
//   Fragmentation Unit (FU): [RTP header][PayloadHdr(2)][FU header(1)][FU payload]

#include "mjc_video_rtp_transport.h"

#include <arpa/inet.h>
#include <cstring>
#include <random>
#include <sys/socket.h>
#include <unistd.h>

#include <os/log.h>

static constexpr size_t kRTPHeaderSize = 12;
static constexpr size_t kMaxRTPPayloadSize = 1400; // Max RTP payload bytes (below typical 1500 MTU)
static constexpr uint8_t kRTPVersion = 2;
static constexpr uint8_t kHEVCPayloadType = 96;  // Dynamic payload type for H.265
static constexpr uint8_t kHEVCFUType = 49;        // NAL unit type for Fragmentation Unit

// Input format: HVCC (4-byte big-endian length prefix + NAL unit data)

// MARK: - Factory Methods

MJVideoRTPTransport* MJVideoRTPTransport::create() {
    return new MJVideoRTPTransport();
}

void MJVideoRTPTransport::destroy(MJVideoRTPTransport* transport) {
    delete transport;
}

// MARK: - Construction / Destruction

MJVideoRTPTransport::MJVideoRTPTransport()
    : socket_fd_(-1),
      port_(0),
      active_(false),
      sequence_number_(0),
      ssrc_(0) {
    // Generate random SSRC (RFC 3550 §8.1)
    std::random_device rd;
    ssrc_ = rd();
    // Random initial sequence number (RFC 3550 §5.1)
    sequence_number_ = static_cast<uint16_t>(rd() & 0xFFFF);
}

MJVideoRTPTransport::~MJVideoRTPTransport() {
    Stop();
}

// MARK: - Start / Stop

bool MJVideoRTPTransport::Start(uint16_t port) {
    if (active_.load(std::memory_order_acquire)) {
        return true;
    }

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        os_log_error(OS_LOG_DEFAULT, "RTP: Failed to create socket");
        return false;
    }

    // Allow port reuse
    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Set send buffer size (256KB for burst sending)
    int sndbuf = 256 * 1024;
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    // Bind to port so RTP packets have a predictable source address.
    // Clients use NWConnection to this port for bidirectional UDP flow.
    struct sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    bind_addr.sin_port = htons(port);
    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
        os_log_error(OS_LOG_DEFAULT, "RTP: Failed to bind UDP port %u: %{errno}d", port, errno);
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    port_ = port;
    active_.store(true, std::memory_order_release);
    packet_buffer_.reserve(kRTPHeaderSize + kMaxRTPPayloadSize);

    os_log_info(OS_LOG_DEFAULT, "RTP: HEVC transport started on port %u", port);
    return true;
}

void MJVideoRTPTransport::Stop() {
    active_.store(false, std::memory_order_release);

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.clear();

    os_log_info(OS_LOG_DEFAULT, "RTP: Transport stopped");
}

// MARK: - Client Management

void MJVideoRTPTransport::AddClient(const struct sockaddr_in& addr) {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    // Check for duplicate
    for (const auto& client : clients_) {
        if (client.sin_addr.s_addr == addr.sin_addr.s_addr &&
            client.sin_port == addr.sin_port) {
            return;
        }
    }
    clients_.push_back(addr);
    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &addr.sin_addr, addr_str, sizeof(addr_str));
    os_log_info(OS_LOG_DEFAULT, "RTP: Client added (%s:%u)",
                addr_str, ntohs(addr.sin_port));
}

void MJVideoRTPTransport::RemoveClient(const struct sockaddr_in& addr) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.erase(
        std::remove_if(clients_.begin(), clients_.end(),
            [&](const struct sockaddr_in& c) {
                return c.sin_addr.s_addr == addr.sin_addr.s_addr &&
                       c.sin_port == addr.sin_port;
            }),
        clients_.end());
    os_log_info(OS_LOG_DEFAULT, "RTP: Client removed");
}

bool MJVideoRTPTransport::HasReceiver() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    return !clients_.empty();
}

bool MJVideoRTPTransport::IsActive() const {
    return active_.load(std::memory_order_acquire);
}

// MARK: - SendPacket Helper

void MJVideoRTPTransport::SendPacket(const uint8_t* data, size_t size,
                                      const std::vector<struct sockaddr_in>& clients) {
    for (const auto& client : clients) {
        ssize_t sent = sendto(socket_fd_, data, size, 0,
               reinterpret_cast<const struct sockaddr*>(&client), sizeof(client));
        if (sent < 0) {
            os_log_error(OS_LOG_DEFAULT, "RTP: sendto() failed: %{errno}d", errno);
        }
    }
}

// MARK: - SendFrame (RFC 7798 HEVC Payload)

bool MJVideoRTPTransport::SendFrame(const MJVideoFrameDesc& desc,
                                     const uint8_t* data, size_t size) {
    if (!active_.load(std::memory_order_acquire) || socket_fd_ < 0) return false;

    // Snapshot client list
    std::vector<struct sockaddr_in> clients;
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        if (clients_.empty()) return false;
        clients = clients_;
    }

    // Expect HEVC HVCC data (4-byte length-prefixed NAL units)
    if (desc.format != static_cast<uint8_t>(MJVideoFormat::HEVC)) {
        return false;
    }

    // RTP timestamp: 90kHz clock from simulation time
    uint32_t rtp_timestamp = static_cast<uint32_t>(desc.simulation_time * 90000.0);

    // Count NAL units for marker bit (O(k) where k = NAL count, not O(n))
    size_t nal_count = 0;
    {
        size_t o = 0;
        while (o + 4 <= size) {
            uint32_t l;
            std::memcpy(&l, data + o, 4);
            l = ntohl(l);
            if (o + 4 + l > size) break;
            o += 4 + l;
            nal_count++;
        }
    }

    if (nal_count == 0) return false;

    // One-time diagnostic log
    static bool logged_first_frame = false;
    if (!logged_first_frame) {
        logged_first_frame = true;
        os_log_info(OS_LOG_DEFAULT,
            "RTP: First HEVC frame (HVCC): total_size=%zu nal_units=%zu",
            size, nal_count);
    }

    // Parse and send each NAL unit inline (no vector allocation)
    size_t offset = 0;
    size_t nal_index = 0;
    while (offset + 4 <= size) {
        // Read 4-byte big-endian NAL length
        uint32_t nal_length;
        std::memcpy(&nal_length, data + offset, 4);
        nal_length = ntohl(nal_length);
        offset += 4;
        if (offset + nal_length > size) {
            os_log_error(OS_LOG_DEFAULT, "RTP: Invalid NAL length %u at offset %zu (frame size %zu)",
                         nal_length, offset - 4, size);
            break;
        }

        const uint8_t* nal_data = data + offset;
        size_t nal_size = nal_length;
        offset += nal_length;
        nal_index++;
        bool is_last_nal = (nal_index == nal_count);

        if (nal_size < 2) continue;  // HEVC NAL header is 2 bytes minimum

        // Read HEVC NAL unit header (2 bytes)
        // Byte 0: forbidden(1) | type(6) | layerID_hi(1)
        // Byte 1: layerID_lo(5) | tid(3)
        uint8_t nal_byte0 = nal_data[0];
        uint8_t nal_byte1 = nal_data[1];

        if (nal_size <= kMaxRTPPayloadSize) {
            // Single NAL Unit Packet: fits in one RTP packet
            packet_buffer_.resize(kRTPHeaderSize + nal_size);
            uint8_t* pkt = packet_buffer_.data();

            // RTP header (12 bytes)
            pkt[0] = (kRTPVersion << 6);  // V=2, P=0, X=0, CC=0
            pkt[1] = kHEVCPayloadType | (is_last_nal ? 0x80 : 0);  // M bit on last NAL of frame
            pkt[2] = static_cast<uint8_t>(sequence_number_ >> 8);
            pkt[3] = static_cast<uint8_t>(sequence_number_ & 0xFF);
            pkt[4] = static_cast<uint8_t>(rtp_timestamp >> 24);
            pkt[5] = static_cast<uint8_t>(rtp_timestamp >> 16);
            pkt[6] = static_cast<uint8_t>(rtp_timestamp >> 8);
            pkt[7] = static_cast<uint8_t>(rtp_timestamp);
            pkt[8]  = static_cast<uint8_t>(ssrc_ >> 24);
            pkt[9]  = static_cast<uint8_t>(ssrc_ >> 16);
            pkt[10] = static_cast<uint8_t>(ssrc_ >> 8);
            pkt[11] = static_cast<uint8_t>(ssrc_);

            // Payload: raw NAL unit (including its 2-byte header)
            std::memcpy(pkt + kRTPHeaderSize, nal_data, nal_size);

            SendPacket(packet_buffer_.data(), packet_buffer_.size(), clients);
            sequence_number_++;

        } else {
            // Fragmentation Unit (FU): NAL too large for single packet
            // RFC 7798 §4.4.3
            //
            // PayloadHdr (2 bytes): same as NAL header but Type=49 (FU)
            //   Byte 0: forbidden(1) | Type=49(6) | layerID_hi(1)
            //   Byte 1: layerID_lo(5) | tid(3)
            // FU header (1 byte): S(1) | E(1) | FuType(6)

            uint8_t nal_type = (nal_byte0 >> 1) & 0x3F;

            // Build PayloadHdr: copy original layerID and TID, set type to FU (49)
            uint8_t fu_indicator0 = (nal_byte0 & 0x81) | (kHEVCFUType << 1);  // keep F and layerID_hi, set type=49
            uint8_t fu_indicator1 = nal_byte1;  // keep layerID_lo and TID

            // Fragment the NAL unit body (skip 2-byte NAL header)
            const uint8_t* frag_data = nal_data + 2;
            size_t frag_remaining = nal_size - 2;

            // Each FU packet has: RTP(12) + PayloadHdr(2) + FU_header(1) + payload
            size_t max_frag_payload = kMaxRTPPayloadSize - 3;  // minus PayloadHdr + FU header
            bool first_fragment = true;

            while (frag_remaining > 0) {
                size_t chunk = std::min(max_frag_payload, frag_remaining);
                bool last_fragment = (chunk >= frag_remaining);

                packet_buffer_.resize(kRTPHeaderSize + 3 + chunk);
                uint8_t* pkt = packet_buffer_.data();

                // RTP header
                pkt[0] = (kRTPVersion << 6);
                pkt[1] = kHEVCPayloadType | ((is_last_nal && last_fragment) ? 0x80 : 0);
                pkt[2] = static_cast<uint8_t>(sequence_number_ >> 8);
                pkt[3] = static_cast<uint8_t>(sequence_number_ & 0xFF);
                pkt[4] = static_cast<uint8_t>(rtp_timestamp >> 24);
                pkt[5] = static_cast<uint8_t>(rtp_timestamp >> 16);
                pkt[6] = static_cast<uint8_t>(rtp_timestamp >> 8);
                pkt[7] = static_cast<uint8_t>(rtp_timestamp);
                pkt[8]  = static_cast<uint8_t>(ssrc_ >> 24);
                pkt[9]  = static_cast<uint8_t>(ssrc_ >> 16);
                pkt[10] = static_cast<uint8_t>(ssrc_ >> 8);
                pkt[11] = static_cast<uint8_t>(ssrc_);

                // PayloadHdr (2 bytes)
                pkt[kRTPHeaderSize]     = fu_indicator0;
                pkt[kRTPHeaderSize + 1] = fu_indicator1;

                // FU header (1 byte): S | E | FuType
                uint8_t fu_header = nal_type;  // lower 6 bits = original NAL type
                if (first_fragment) fu_header |= 0x80;  // S bit
                if (last_fragment)  fu_header |= 0x40;  // E bit
                pkt[kRTPHeaderSize + 2] = fu_header;

                // FU payload
                std::memcpy(pkt + kRTPHeaderSize + 3, frag_data, chunk);

                SendPacket(packet_buffer_.data(), packet_buffer_.size(), clients);
                sequence_number_++;

                frag_data += chunk;
                frag_remaining -= chunk;
                first_fragment = false;
            }
        }
    }

    return true;
}
