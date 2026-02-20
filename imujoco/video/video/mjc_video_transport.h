// mjc_video_transport.h
// Abstract transport protocol for video streaming
//
// Transports are pluggable backends that deliver video frames to external
// consumers. The interface is kept minimal: Start, Stop, SendFrame, HasReceiver.
// Each transport runs its own I/O internally (e.g., UDP socket, TCP listener).

#ifndef IMUJOCO_VIDEO_TRANSPORT_H_
#define IMUJOCO_VIDEO_TRANSPORT_H_

#include "mjc_video_types.h"

#include <cstddef>
#include <cstdint>

/// Abstract base class for video transports.
///
/// Implementations must be safe to call SendFrame() from a single thread
/// (the video capture thread). Start/Stop may be called from any thread
/// but must not be concurrent with SendFrame.
class MJVideoTransport {
public:
    virtual ~MJVideoTransport() = default;

    /// Start the transport (bind sockets, start listeners, etc.)
    /// @param port The port to listen/send on
    /// @return true if started successfully
    virtual bool Start(uint16_t port) = 0;

    /// Stop the transport and release resources.
    virtual void Stop() = 0;

    /// Send a video frame.
    /// @param desc Frame descriptor (40 bytes, prepended to data)
    /// @param data Pixel data
    /// @param size Size of pixel data in bytes (must match desc.data_size)
    /// @return true if frame was sent (or queued) successfully
    /// @note If the transport cannot keep up, it should drop the frame
    ///       rather than blocking. VLA agents want the latest frame.
    virtual bool SendFrame(const MJVideoFrameDesc& desc,
                           const uint8_t* data, size_t size) = 0;

    /// Check if any receiver is connected / listening.
    /// Transports may use this to skip encoding when nobody is watching.
    virtual bool HasReceiver() const = 0;

    /// Check if the transport is currently active.
    virtual bool IsActive() const = 0;
};

#endif  // IMUJOCO_VIDEO_TRANSPORT_H_
