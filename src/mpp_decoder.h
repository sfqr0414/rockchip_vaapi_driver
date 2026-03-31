#pragma once

#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_meta.h>
#include <rockchip/mpp_packet.h>
#include <rockchip/rk_mpi.h>
#include <va/va.h>
#include <va/va_drmcommon.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

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

struct DecodedSurface {
    VASurfaceID va_id = VA_INVALID_ID;
    int dmabuf_fd = -1;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    bool is_10bit = false;
};

struct SurfaceInfo {
    DecodedSurface surface;
    MppBuffer buffer = nullptr;
    MppFrame frame = nullptr;
    std::atomic<bool> ready{false};
    std::atomic<bool> failed{false};
};

struct DecodeJob {
    VASurfaceID target_surface = VA_INVALID_ID;
    std::vector<uint8_t> bitstream;
    std::vector<uint8_t> extra_data;
    bool eos = false;
    uint64_t job_id = 0;
};

class MppDecoder {
   public:
    MppDecoder();
    ~MppDecoder();

    bool initialize(CodecProfile profile, int width, int height);
    bool isInitialized() const;
    bool enqueueJob(DecodeJob job);

    bool allocateSurface(VASurfaceID id, DecodedSurface& out, int width, int height);
    bool updateSurfaceResolution(VASurfaceID id, int width, int height);
    bool getSurfaceInfo(VASurfaceID id, uint32_t& width, uint32_t& height, uint32_t& stride, int& dmabuf_fd, bool& failed);
    bool getSurfaceState(VASurfaceID id, bool& ready, bool& failed);
    bool waitSurfaceReady(VASurfaceID surface, uint32_t timeout_ms = 15000);
    void forceSurfaceReady(VASurfaceID surface);
    void resetSurface(VASurfaceID surface);
    void releaseSurface(VASurfaceID surface);
    void destroySurface(VASurfaceID surface);
    void shutdown();

   private:
    void inputThreadMain(std::stop_token st);
    void outputThreadMain(std::stop_token st);
    void setSurfaceState(VASurfaceID surface, bool ready, bool failed);
    bool dropPendingSurfaceLocked(VASurfaceID surface);
    VASurfaceID dropOldestPendingSurfaceLocked();

    CodecProfile profile_{CodecProfile::Unknown};
    MppCtx ctx_ = nullptr;
    MppApi* api_ = nullptr;
    MppBufferGroup group_ = nullptr;
    util::AtomicSyncQueue<DecodeJob, 16> input_queue_;
    std::atomic<bool> running_{false};
    std::atomic<bool> eos_sent_{false};
    std::atomic<bool> eos_seen_{false};

    std::jthread input_thread_;
    std::jthread output_thread_;

    std::unordered_map<VASurfaceID, SurfaceInfo> surfaces_;
    std::deque<VASurfaceID> pending_surfaces_;
    std::unordered_map<uint64_t, VASurfaceID> pending_surface_pts_;
    std::deque<MppPacket> pending_packets_;
    std::deque<std::shared_ptr<std::vector<uint8_t>>> pending_payloads_;
    size_t pending_count_ = 0;
    std::atomic<uint64_t> last_enqueue_us_{0};
    std::atomic<uint64_t> last_progress_us_{0};
    std::atomic<uint64_t> next_job_id_{1};

    std::mutex surface_mutex_;
    std::mutex pending_mutex_;
    std::condition_variable pending_cv_;

    std::mutex cv_mutex_;
    std::condition_variable cv_;
};

CodecProfile vaProfileToCodec(VAProfile profile);

}  // namespace rockchip
