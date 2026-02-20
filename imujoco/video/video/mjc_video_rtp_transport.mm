// mjc_video_rtp_transport.mm
// RTP transport implementation with RFC 2435 JPEG payload
//
// RTP packet format (RFC 3550):
//   [12-byte RTP header][payload]
//
// JPEG payload format (RFC 2435):
//   [8-byte JPEG header][optional restart marker header][optional quantization header][JPEG data]

#include "mjc_video_rtp_transport.h"

#include <arpa/inet.h>
#include <cstring>
#include <random>
#include <sys/socket.h>
#include <unistd.h>

#include <os/log.h>

static constexpr size_t kRTPHeaderSize = 12;
static constexpr size_t kJPEGHeaderSize = 8;
static constexpr size_t kQuantHeaderSize = 4;  // MBZ + MBZ + Length(16)
static constexpr size_t kMaxRTPPayload = 1400; // Safe UDP payload (below typical 1500 MTU)
static constexpr uint8_t kRTPVersion = 2;
static constexpr uint8_t kJPEGPayloadType = 26;

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

    port_ = port;
    active_.store(true, std::memory_order_release);
    packet_buffer_.reserve(kRTPHeaderSize + kJPEGHeaderSize + 128 + kMaxRTPPayload);

    os_log_info(OS_LOG_DEFAULT, "RTP: Transport started on port %u", port);
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
    os_log_info(OS_LOG_DEFAULT, "RTP: Client added (%s:%u)",
                inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
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

// MARK: - SendFrame (RFC 2435 JPEG Payload)

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

    // RFC 2435 requires JPEG data. If format is not JPEG, skip.
    if (desc.format != static_cast<uint8_t>(MJVideoFormat::JPEG)) {
        return false;
    }

    // Extract quantization tables and find scan data offset
    std::vector<uint8_t> luma_qt, chroma_qt;
    size_t scan_offset = FindJPEGScanData(data, size, luma_qt, chroma_qt);
    if (scan_offset == 0 || scan_offset >= size) {
        // Couldn't parse JPEG — send raw scan data without quant tables
        scan_offset = 0;
    }

    // RTP timestamp: 90kHz clock from simulation time
    uint32_t rtp_timestamp = static_cast<uint32_t>(desc.simulation_time * 90000.0);

    // RFC 2435 type field determines chroma subsampling and restart markers:
    //   Type 0: YCbCr 4:2:0 baseline (matches CoreGraphics JPEG output)
    //   Type 1: YCbCr 4:2:2 baseline
    //   +64: restart marker header present
    // CGImageDestination outputs 4:2:0 by default, no restart markers → type 0
    // Q >= 128 means quantization table header is present (independent of type)
    uint8_t jpeg_type = 0;
    uint8_t jpeg_q = 255;  // Q=255 means quantization tables are present in first packet

    // Width/height in 8-pixel blocks (RFC 2435 §3.1.4-5)
    uint8_t width_blocks = static_cast<uint8_t>(desc.width / 8);
    uint8_t height_blocks = static_cast<uint8_t>(desc.height / 8);

    // Scan data to packetize (skip JFIF headers, send entropy-coded data)
    const uint8_t* scan_data = data + scan_offset;
    size_t scan_size = size - scan_offset;

    // Packetize into RTP packets
    size_t offset = 0;
    bool first_packet = true;

    while (offset < scan_size) {
        // Calculate how much JPEG data fits in this packet
        size_t header_overhead = kRTPHeaderSize + kJPEGHeaderSize;
        if (first_packet && jpeg_q >= 128) {
            // First packet includes quantization table header
            header_overhead += kQuantHeaderSize + luma_qt.size() + chroma_qt.size();
        }

        size_t max_jpeg_data = kMaxRTPPayload - (header_overhead - kRTPHeaderSize);
        size_t chunk_size = std::min(max_jpeg_data, scan_size - offset);
        bool last_packet = (offset + chunk_size >= scan_size);

        // Build packet
        packet_buffer_.clear();
        packet_buffer_.resize(header_overhead + chunk_size);
        uint8_t* pkt = packet_buffer_.data();

        // RTP header (12 bytes)
        pkt[0] = (kRTPVersion << 6);  // V=2, P=0, X=0, CC=0
        pkt[1] = kJPEGPayloadType | (last_packet ? 0x80 : 0);  // M bit on last packet
        pkt[2] = static_cast<uint8_t>(sequence_number_ >> 8);
        pkt[3] = static_cast<uint8_t>(sequence_number_ & 0xFF);
        // Timestamp (big-endian)
        pkt[4] = static_cast<uint8_t>(rtp_timestamp >> 24);
        pkt[5] = static_cast<uint8_t>(rtp_timestamp >> 16);
        pkt[6] = static_cast<uint8_t>(rtp_timestamp >> 8);
        pkt[7] = static_cast<uint8_t>(rtp_timestamp);
        // SSRC (big-endian)
        pkt[8]  = static_cast<uint8_t>(ssrc_ >> 24);
        pkt[9]  = static_cast<uint8_t>(ssrc_ >> 16);
        pkt[10] = static_cast<uint8_t>(ssrc_ >> 8);
        pkt[11] = static_cast<uint8_t>(ssrc_);

        // JPEG header (8 bytes, RFC 2435 §3.1)
        size_t hdr_off = kRTPHeaderSize;
        // Type-specific: 0
        pkt[hdr_off + 0] = 0;
        // Fragment offset (24-bit big-endian)
        pkt[hdr_off + 1] = static_cast<uint8_t>((offset >> 16) & 0xFF);
        pkt[hdr_off + 2] = static_cast<uint8_t>((offset >> 8) & 0xFF);
        pkt[hdr_off + 3] = static_cast<uint8_t>(offset & 0xFF);
        // Type
        pkt[hdr_off + 4] = jpeg_type;
        // Q
        pkt[hdr_off + 5] = jpeg_q;
        // Width (in 8-pixel blocks)
        pkt[hdr_off + 6] = width_blocks;
        // Height (in 8-pixel blocks)
        pkt[hdr_off + 7] = height_blocks;
        hdr_off += kJPEGHeaderSize;

        // Quantization table header (first packet only, Q >= 128)
        if (first_packet && jpeg_q >= 128) {
            uint16_t qt_len = static_cast<uint16_t>(luma_qt.size() + chroma_qt.size());
            pkt[hdr_off + 0] = 0;  // MBZ
            pkt[hdr_off + 1] = 0;  // Precision (0 = 8-bit)
            pkt[hdr_off + 2] = static_cast<uint8_t>(qt_len >> 8);
            pkt[hdr_off + 3] = static_cast<uint8_t>(qt_len & 0xFF);
            hdr_off += kQuantHeaderSize;

            // Copy quantization tables
            if (!luma_qt.empty()) {
                std::memcpy(pkt + hdr_off, luma_qt.data(), luma_qt.size());
                hdr_off += luma_qt.size();
            }
            if (!chroma_qt.empty()) {
                std::memcpy(pkt + hdr_off, chroma_qt.data(), chroma_qt.size());
                hdr_off += chroma_qt.size();
            }
            first_packet = false;
        }

        // Copy JPEG scan data
        std::memcpy(pkt + hdr_off, scan_data + offset, chunk_size);

        // Send to all clients
        for (const auto& client : clients) {
            sendto(socket_fd_, packet_buffer_.data(), packet_buffer_.size(), 0,
                   reinterpret_cast<const struct sockaddr*>(&client), sizeof(client));
        }

        sequence_number_++;
        offset += chunk_size;
    }

    return true;
}

// MARK: - JPEG Parsing

size_t MJVideoRTPTransport::FindJPEGScanData(const uint8_t* data, size_t size,
                                               std::vector<uint8_t>& luma_qt,
                                               std::vector<uint8_t>& chroma_qt) {
    // Parse JFIF markers to extract quantization tables and find SOS marker
    luma_qt.clear();
    chroma_qt.clear();

    if (size < 2 || data[0] != 0xFF || data[1] != 0xD8) {
        return 0;  // Not a valid JPEG
    }

    size_t pos = 2;
    int qt_count = 0;

    while (pos + 1 < size) {
        if (data[pos] != 0xFF) {
            pos++;
            continue;
        }

        uint8_t marker = data[pos + 1];

        // SOS (Start of Scan) — data follows after marker + length + header
        if (marker == 0xDA) {
            if (pos + 3 < size) {
                uint16_t sos_len = (static_cast<uint16_t>(data[pos + 2]) << 8) | data[pos + 3];
                return pos + 2 + sos_len;  // Offset to start of entropy-coded data
            }
            return 0;
        }

        // DQT (Define Quantization Table)
        if (marker == 0xDB) {
            if (pos + 3 >= size) break;
            uint16_t seg_len = (static_cast<uint16_t>(data[pos + 2]) << 8) | data[pos + 3];
            size_t seg_end = pos + 2 + seg_len;
            size_t tpos = pos + 4;  // Skip marker + length

            while (tpos < seg_end && tpos < size) {
                uint8_t pq_tq = data[tpos];  // precision(4) | table_id(4)
                uint8_t precision = (pq_tq >> 4) & 0x0F;
                size_t table_size = (precision == 0) ? 64 : 128;
                tpos++;

                if (tpos + table_size > size) break;

                if (qt_count == 0) {
                    luma_qt.assign(data + tpos, data + tpos + table_size);
                } else {
                    chroma_qt.assign(data + tpos, data + tpos + table_size);
                }
                qt_count++;
                tpos += table_size;
            }

            pos = seg_end;
            continue;
        }

        // Skip other markers with length
        if (marker >= 0xC0 && marker != 0xFF) {
            if (pos + 3 >= size) break;
            uint16_t seg_len = (static_cast<uint16_t>(data[pos + 2]) << 8) | data[pos + 3];
            pos = pos + 2 + seg_len;
        } else {
            pos += 2;
        }
    }

    return 0;
}

// MARK: - SendRTPPacket (unused, packetization is inline in SendFrame)

void MJVideoRTPTransport::SendRTPPacket(const uint8_t* payload, size_t payload_size,
                                         uint32_t timestamp, bool marker,
                                         const struct sockaddr_in& dest) {
    // Not used — RTP packets are assembled inline in SendFrame for efficiency.
    // Kept for potential future use with non-JPEG payloads.
    (void)payload; (void)payload_size; (void)timestamp; (void)marker; (void)dest;
}
