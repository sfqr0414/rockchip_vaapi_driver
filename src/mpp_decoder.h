#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include <va/va.h>
#include <va/va_drmcommon.h>

#include <rockchip/rk_mpi.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_packet.h>

#include "util/atomic_sync_queue.h"
#include "util/log.h"

namespace rockchip {

enum class CodecProfile {
    H264,
    HEVC,
    VP9,
    AV1,
    Unknown,
};

/// Encapsulates a single decoded surface.
struct DecodedSurface {
    VASurfaceID va_id = VA_INVALID_ID;
    int dmabuf_fd = -1;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    bool is_10bit = false;
    std::atomic<bool> ready{false};
};

struct DecodeJob {
    VASurfaceID target_surface = VA_INVALID_ID;
    std::vector<uint8_t> bitstream;
    std::vector<uint8_t> extra_data; // For SPS/PPS or AV1 Sequence Header
};

struct SurfaceInfo {
    // DMABUF returned to VA (exported by the driver)
    int dmabuf_fd = -1;
    // Imported MPP buffer handle used for decode output.
    MppBuffer buffer = nullptr;

    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    std::shared_ptr<std::atomic<bool>> ready;
    std::shared_ptr<std::atomic<bool>> decode_failed;
};

class MppDecoder {
public:
    MppDecoder();
    ~MppDecoder();

    bool initialize(CodecProfile profile, int width, int height);
    bool isInitialized() const;
    bool enqueueJob(DecodeJob job);

    /// Wait until the surface is ready to be used (decoded).
    bool waitSurfaceReady(VASurfaceID surface, uint32_t timeout_ms = 5000);

    /// Allocates a surface (with an exported dmabuf FD) and tracks readiness.
    bool allocateSurface(VASurfaceID id, DecodedSurface& out, int width, int height);

    /// Update the output surface size/stride when MPP reports a resolution change.
    bool updateSurfaceResolution(VASurfaceID id, int width, int height);

    bool getSurfaceInfo(VASurfaceID id, uint32_t& width, uint32_t& height, uint32_t& stride, int& dmabuf_fd, bool& failed);

    /// Notify the decoder that a surface is ready for display.
    void markSurfaceReady(VASurfaceID surface);

    /// Signals the decoder to tear down.
    void shutdown();

private:
    void decoderThreadMain();
    bool processJob(const DecodeJob& job);
    bool drainFrames(const DecodeJob& job, SurfaceInfo& info);
    CodecProfile profile_{CodecProfile::Unknown};

    MppCtx ctx_ = nullptr;
    MppApi* api_ = nullptr;
    MppBufferGroup group_ = nullptr;

    util::AtomicSyncQueue<DecodeJob, 16> job_queue_;

    std::atomic<bool> running_{false};
    std::thread decoder_thread_;

    // Per-surface readiness indicator.
    std::unordered_map<VASurfaceID, SurfaceInfo> surfaces_;
};

CodecProfile vaProfileToCodec(VAProfile profile);

} // namespace rockchip
