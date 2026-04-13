#include "mpp_decoder.h"

#include <chrono>

extern "C" {
#include <fcntl.h>
#include <sys/ioctl.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <xf86drm.h>
}

#include <sys/stat.h>
#include <sys/sysmacros.h>

#include <mutex>
#include <optional>
#include <thread>

#include <rockchip/mpp_log.h>

namespace rockchip {

namespace {

bool envEnabled(const char* name) {
    const char* value = std::getenv(name);
    if (!value) return false;
    return std::strcmp(value, "1") == 0 || std::strcmp(value, "true") == 0 || std::strcmp(value, "yes") == 0;
}

bool envDisabled(const char* name) {
    const char* value = std::getenv(name);
    if (!value) return false;
    return std::strcmp(value, "0") == 0 || std::strcmp(value, "false") == 0 || std::strcmp(value, "no") == 0;
}

bool av1ExportAs10BitEnabled() {
    return !envDisabled("ROCKCHIP_VAAPI_AV1_EXPORT_P010");
}

bool hasDumbBufferCap(int fd) {
    uint64_t value = 0;
    return drmGetCap(fd, DRM_CAP_DUMB_BUFFER, &value) == 0 && value != 0;
}

std::optional<unique_fd> openStableExportDrmFd() {
    std::array<drmDevicePtr, 64> devices{};
    const int device_count = drmGetDevices2(0, devices.data(), static_cast<int>(devices.size()));
    if (device_count < 0) {
        return std::nullopt;
    }

    std::optional<unique_fd> preferred;
    std::optional<unique_fd> fallback;
    for (int index = 0; index < device_count; ++index) {
        drmDevicePtr device_info = devices[index];
        if (!device_info) continue;
        if ((device_info->available_nodes & (1 << DRM_NODE_PRIMARY)) == 0) continue;
        const char* node = device_info->nodes[DRM_NODE_PRIMARY];
        if (!node) continue;

        unique_fd fd{open(node, O_RDWR | O_CLOEXEC)};
        if (!fd) continue;
        if (!hasDumbBufferCap(fd.get())) continue;

        drmVersionPtr version = drmGetVersion(fd.get());
        const std::string driver_name = version && version->name ? version->name : "";
        if (version) drmFreeVersion(version);

        if (driver_name.find("rockchip") != std::string::npos) {
            preferred = std::move(fd);
            break;
        }
        if (!fallback) {
            fallback = std::move(fd);
        }
    }

    drmFreeDevices(devices.data(), device_count);
    if (preferred) return preferred;
    if (fallback) return fallback;
    return std::nullopt;
}

bool createStableExportBuffer(MppDecoder::SurfaceInfo& surface_info,
                              int drm_fd,
                              uint32_t width,
                              uint32_t height,
                              bool is_10bit) {
    if (drm_fd < 0 || is_10bit) {
        return false;
    }

    drm_mode_create_dumb create = {};
    create.width = alignUp(width, 64);
    create.height = height + height / 2;
    create.bpp = 8;
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) != 0) {
        return false;
    }

    drm_prime_handle prime = {};
    prime.handle = create.handle;
    prime.flags = DRM_CLOEXEC | DRM_RDWR;
    if (drmIoctl(drm_fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime) != 0) {
        drm_mode_destroy_dumb destroy = {};
        destroy.handle = create.handle;
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
        return false;
    }

    drm_mode_map_dumb map = {};
    map.handle = create.handle;
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map) != 0) {
        ::close(prime.fd);
        drm_mode_destroy_dumb destroy = {};
        destroy.handle = create.handle;
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
        return false;
    }

    void* addr = mmap(nullptr, create.size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, static_cast<off_t>(map.offset));
    if (addr == MAP_FAILED) {
        ::close(prime.fd);
        drm_mode_destroy_dumb destroy = {};
        destroy.handle = create.handle;
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
        return false;
    }

    surface_info.export_map = addr;
    surface_info.export_size = create.size;
    surface_info.export_handle = create.handle;
    surface_info.export_pitch = create.pitch;
    surface_info.dmabuf.reset(prime.fd);
    return true;
}

void destroyStableExportBuffer(MppDecoder::SurfaceInfo& surface_info, int drm_fd) {
    if (surface_info.export_map && surface_info.export_map != MAP_FAILED) {
        munmap(surface_info.export_map, surface_info.export_size);
    }
    surface_info.export_map = nullptr;
    surface_info.export_size = 0;
    surface_info.export_pitch = 0;
    surface_info.dmabuf.reset();
    surface_info.surface.dmabuf_fd = -1;

    if (drm_fd >= 0 && surface_info.export_handle != 0) {
        drm_mode_destroy_dumb destroy = {};
        destroy.handle = surface_info.export_handle;
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    }
    surface_info.export_handle = 0;
}

bool copyFrameToStableExport(void* dst_map,
                             uint32_t dst_pitch,
                             uint32_t width,
                             uint32_t height,
                             uint32_t src_stride_bytes,
                             uint32_t src_ver_stride,
                             const uint8_t* src_base) {
    if (!dst_map || !src_base || width == 0 || height == 0) {
        return false;
    }

    uint8_t* dst_y = static_cast<uint8_t*>(dst_map);
    uint8_t* dst_uv = dst_y + static_cast<size_t>(dst_pitch) * height;
    const uint8_t* src_y = src_base;
    const uint8_t* src_uv = src_base + static_cast<size_t>(src_stride_bytes) * src_ver_stride;

    for (uint32_t row = 0; row < height; ++row) {
        std::memcpy(dst_y + static_cast<size_t>(row) * dst_pitch,
                    src_y + static_cast<size_t>(row) * src_stride_bytes,
                    width);
    }
    for (uint32_t row = 0; row < height / 2; ++row) {
        std::memcpy(dst_uv + static_cast<size_t>(row) * dst_pitch,
                    src_uv + static_cast<size_t>(row) * src_stride_bytes,
                    width);
    }
    return true;
}

}  // namespace

MppDecoder::MppDecoder() { mpp_set_log_level(MPP_LOG_ERROR); }

MppDecoder::~MppDecoder() { shutdown(); }

bool MppDecoder::initialize(CodecProfile profile, int width, int height, int drm_fd) {
    std::scoped_lock lock(init_mutex_);
    profile_ = profile;
    if (session_initialized_) return true;

    stable_export_enabled_ = envEnabled("ROCKCHIP_VAAPI_STABLE_EXPORT") ||
                             (profile == CodecProfile::HEVC && !envEnabled("ROCKCHIP_VAAPI_DISABLE_STABLE_EXPORT"));

    struct stat stat_buf = {};
    if (drm_fd < 0 || fstat(drm_fd, &stat_buf) != 0 || !S_ISCHR(stat_buf.st_mode) || major(stat_buf.st_rdev) != 226 || minor(stat_buf.st_rdev) < 128) {
        return false;
    }

    session_ = MppSession::create();
    if (!session_) {
        return false;
    }

    MppCtx ctx = session_->ctx();
    MppApi* api = session_->api();

    if (mpp_init(ctx, MPP_CTX_DEC, codecProfileToMpp(profile_)) != MPP_OK) {
        session_.reset();
        return false;
    }

    MppFrameFormat output_format = (profile_ == CodecProfile::AV1 && av1ExportAs10BitEnabled())
                                       ? MPP_FMT_YUV420SP_10BIT
                                       : MPP_FMT_YUV420SP;
    static std::atomic<int> init_log_count{0};
    if (init_log_count.fetch_add(1, std::memory_order_relaxed) < 8) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "mpp: initialize profile={} stable_export={} av1_p010={} output_format={} width={} height={}",
                  static_cast<int>(profile_),
                  stable_export_enabled_ ? 1 : 0,
                  av1ExportAs10BitEnabled() ? 1 : 0,
                  static_cast<int>(output_format),
                  width,
                  height);
    }
    api->control(ctx, MPP_DEC_SET_OUTPUT_FORMAT, &output_format);

    MppDecCfg raw_cfg = nullptr;
    if (mpp_dec_cfg_init(&raw_cfg) == MPP_OK && raw_cfg) {
        MppDecCfgHandle cfg{raw_cfg};
        mpp_dec_cfg_set_u32(cfg.get(), "base:split_parse", 1);
        api->control(ctx, MPP_DEC_SET_CFG, cfg.get());
    }

    uint32_t split_parse = 1;
    api->control(ctx, MPP_DEC_SET_PARSER_SPLIT_MODE, &split_parse);

    MppPollType nonblock = MPP_POLL_NON_BLOCK;
    api->control(ctx, MPP_SET_OUTPUT_TIMEOUT, &nonblock);
    session_initialized_ = true;
    return true;
}

bool MppDecoder::isInitialized() const { return session_initialized_; }

bool MppDecoder::allocateSurface(VASurfaceID id, DecodedSurface& out, int width, int height) {
    const bool wants_10bit = out.is_10bit;
    uint32_t stride = alignUp(static_cast<uint32_t>(width), 64);
    if (wants_10bit) {
        stride = alignUp(static_cast<uint32_t>(width), 128);
    }

    DecodedSurface ds;
    ds.va_id = id;
    ds.width = static_cast<uint32_t>(width);
    ds.height = static_cast<uint32_t>(height);
    ds.stride = stride;
    ds.is_10bit = wants_10bit;
    ds.dmabuf_fd = -1;

    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto& entry = surfaces_[id];
    entry.surface = ds;
    destroyStableExportBuffer(entry, export_drm_fd_.get());
    entry.in_flight.store(false);
    entry.ready.store(false);
    entry.failed.store(false);
    out = entry.surface;
    return true;
}

bool MppDecoder::updateSurfaceResolution(VASurfaceID id, int width, int height) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;

    uint32_t stride = alignUp(static_cast<uint32_t>(width), 64);
    it->second.surface.width = static_cast<uint32_t>(width);
    it->second.surface.height = static_cast<uint32_t>(height);
    it->second.surface.stride = stride;
    return true;
}

bool MppDecoder::getSurfaceInfo(VASurfaceID id, uint32_t& width, uint32_t& height, uint32_t& stride, int& dmabuf_fd, bool& failed, bool& pending) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;
    width = it->second.surface.width;
    height = it->second.surface.height;
    stride = it->second.surface.stride;
    dmabuf_fd = it->second.surface.dmabuf_fd;
    failed = it->second.failed.load();
    pending = it->second.in_flight.load();
    return true;
}

bool MppDecoder::getSurfaceState(VASurfaceID id, bool& ready, bool& failed) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;
    ready = it->second.ready.load();
    failed = it->second.failed.load();
    return true;
}

bool MppDecoder::getSurfaceDebugInfo(VASurfaceID id,
                                     uint64_t& last_submitted_job_id,
                                     uint64_t& last_completed_job_id,
                                     uint64_t& last_submit_us,
                                     uint64_t& last_complete_us) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;
    last_submitted_job_id = it->second.last_submitted_job_id.load(std::memory_order_acquire);
    last_completed_job_id = it->second.last_completed_job_id.load(std::memory_order_acquire);
    last_submit_us = it->second.last_submit_us.load(std::memory_order_acquire);
    last_complete_us = it->second.last_complete_us.load(std::memory_order_acquire);
    return true;
}

bool MppDecoder::enqueueJob(DecodeJob job) {
    if (!session_) return false;

    job.job_id = next_job_id_.fetch_add(1, std::memory_order_relaxed);

    bool expected = false;
    if (running_.compare_exchange_strong(expected, true)) {
        eos_sent_.store(false, std::memory_order_relaxed);
        eos_seen_.store(false, std::memory_order_relaxed);
        last_progress_us_.store(steadyMicrosNow(), std::memory_order_relaxed);
        input_thread_ = std::jthread([this](std::stop_token st){ inputThreadMain(st); });
        output_thread_ = std::jthread([this](std::stop_token st){ outputThreadMain(st); });
    }

    if (job.target_surface != VA_INVALID_ID) {
        {
            std::lock_guard<std::mutex> surface_lock(surface_mutex_);
            auto surface_it = surfaces_.find(job.target_surface);
            if (surface_it != surfaces_.end()) {
                surface_it->second.last_submitted_job_id.store(job.job_id, std::memory_order_release);
                surface_it->second.last_submit_us.store(steadyMicrosNow(), std::memory_order_release);
            }
        }

        size_t surface_count = 0;
        {
            std::lock_guard<std::mutex> lock(surface_mutex_);
            surface_count = surfaces_.size();
        }

        std::unique_lock<std::mutex> lock(pending_mutex_);
        const size_t target_in_flight = (surface_count == 0) ? kMaxInFlightJobs : std::min<size_t>(surface_count * 4, kMaxInFlightJobs);
        const size_t max_in_flight = std::max<size_t>(1, target_in_flight);
        static std::atomic<int> enqueue_wait_log_count{0};
        while (pending_count_ >= max_in_flight && running_.load()) {
            if (enqueue_wait_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
                util::log(util::stderr_sink, util::LogLevel::Info,
                          "mpp: enqueueJob waiting surface={} pending={} max_in_flight={}",
                          job.target_surface, pending_count_, max_in_flight);
            }
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            lock.lock();
        }
        static std::atomic<int> dedupe_log_count{0};
        if (dropPendingSurfaceLocked(job.target_surface) &&
            dedupe_log_count.fetch_add(1, std::memory_order_relaxed) < 8) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: enqueueJob replaced older pending work for surface={} with job_id={}",
                      job.target_surface,
                      static_cast<unsigned long long>(job.job_id));
        }
        pending_surfaces_.emplace_back(job.job_id, job.target_surface);
        pending_surface_pts_[job.job_id] = job.target_surface;
        ++pending_count_;
        pending_cv_.notify_all();
    } else if (job.discard_output) {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        discard_output_job_ids_.insert(job.job_id);
    }

    static std::atomic<int> enqueue_log_count{0};
    if (enqueue_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "mpp: enqueueJob surface={} eos={} pending={} input_running={}",
                  job.target_surface, job.eos ? 1 : 0, pending_count_, running_.load() ? 1 : 0);
    }

    input_queue_.push(std::move(job));
    last_enqueue_us_.store(steadyMicrosNow(), std::memory_order_relaxed);
    return true;
}

void MppDecoder::setSurfaceState(VASurfaceID surface, bool ready, bool failed) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end()) return;
    it->second.in_flight.store(false, std::memory_order_release);
    it->second.ready.store(ready, std::memory_order_release);
    it->second.failed.store(failed, std::memory_order_release);
    std::lock_guard<std::mutex> lock2(cv_mutex_);
    cv_.notify_all();
}

std::vector<VASurfaceID> MppDecoder::failPendingSurfaces(VASurfaceID preferred_surface) {
    std::vector<VASurfaceID> failed_surfaces;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        if (preferred_surface != VA_INVALID_ID) {
            failed_surfaces.push_back(preferred_surface);
            dropPendingSurfaceLocked(preferred_surface);
        }

        for (const auto& pending : pending_surfaces_) {
            VASurfaceID surface = pending.second;
            if (surface != VA_INVALID_ID &&
                std::find(failed_surfaces.begin(), failed_surfaces.end(), surface) == failed_surfaces.end()) {
                failed_surfaces.push_back(surface);
            }
        }

        pending_surfaces_.clear();
        discard_output_job_ids_.clear();
        pending_surface_pts_.clear();
        pending_payloads_.clear();
        pending_count_ = 0;
        pending_cv_.notify_all();
    }

    for (VASurfaceID surface : failed_surfaces) {
        setSurfaceState(surface, false, true);
    }
    return failed_surfaces;
}

bool MppDecoder::dropPendingSurfaceLocked(VASurfaceID surface) {
    bool removed = false;
    for (auto it = pending_surfaces_.begin(); it != pending_surfaces_.end();) {
        if (it->second == surface) {
            discard_output_job_ids_.insert(it->first);
            pending_surface_pts_.erase(it->first);
            pending_payloads_.erase(it->first);
            it = pending_surfaces_.erase(it);
            removed = true;
            if (pending_count_ > 0) {
                --pending_count_;
            }
        } else {
            ++it;
        }
    }

    if (removed) {
        pending_cv_.notify_all();
    }
    return removed;
}

bool MppDecoder::dropPendingJobLocked(uint64_t job_id, VASurfaceID* surface) {
    auto surface_it = pending_surface_pts_.find(job_id);
    if (surface_it == pending_surface_pts_.end()) {
        discard_output_job_ids_.erase(job_id);
        return false;
    }

    if (surface) {
        *surface = surface_it->second;
    }
    pending_surface_pts_.erase(surface_it);
    pending_payloads_.erase(job_id);

    for (auto it = pending_surfaces_.begin(); it != pending_surfaces_.end(); ++it) {
        if (it->first == job_id) {
            pending_surfaces_.erase(it);
            if (pending_count_ > 0) {
                --pending_count_;
            }
            pending_cv_.notify_all();
            return true;
        }
    }

    pending_cv_.notify_all();
    return true;
}

VASurfaceID MppDecoder::dropOldestPendingSurfaceLocked() {
    if (pending_surfaces_.empty()) return VA_INVALID_ID;
    const auto [job_id, surface] = pending_surfaces_.front();
    pending_surfaces_.pop_front();
    pending_surface_pts_.erase(job_id);
    pending_payloads_.erase(job_id);
    if (pending_count_ > 0) {
        --pending_count_;
    }
    pending_cv_.notify_all();
    return surface;
}

bool MppDecoder::waitSurfaceReady(VASurfaceID surface, uint32_t timeout_ms) {
    std::atomic<bool>* pending_flag = nullptr;
    std::atomic<bool>* ready_flag = nullptr;
    std::atomic<bool>* failed_flag = nullptr;
    std::atomic<uint64_t>* last_submitted_job_id = nullptr;
    std::atomic<uint64_t>* last_completed_job_id = nullptr;
    std::atomic<uint64_t>* last_submit_us = nullptr;
    std::atomic<uint64_t>* last_complete_us = nullptr;
    {
        std::lock_guard<std::mutex> lock(surface_mutex_);
        auto it = surfaces_.find(surface);
        if (it == surfaces_.end()) return false;
        pending_flag = &it->second.in_flight;
        ready_flag = &it->second.ready;
        failed_flag = &it->second.failed;
        last_submitted_job_id = &it->second.last_submitted_job_id;
        last_completed_job_id = &it->second.last_completed_job_id;
        last_submit_us = &it->second.last_submit_us;
        last_complete_us = &it->second.last_complete_us;
    }

    if (!pending_flag || !ready_flag || !failed_flag || !last_submitted_job_id || !last_completed_job_id || !last_submit_us || !last_complete_us) return false;

    if (ready_flag->load(std::memory_order_acquire) || failed_flag->load(std::memory_order_acquire)) {
        return ready_flag->load(std::memory_order_acquire) && !failed_flag->load(std::memory_order_acquire);
    }

    if (!pending_flag->load(std::memory_order_acquire)) {
        return false;
    }

    static std::atomic<int> wait_start_log_count{0};
    if (wait_start_log_count.fetch_add(1, std::memory_order_relaxed) < 3) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "mpp: waitSurfaceReady start surface={} timeout_ms={}", surface, timeout_ms);
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lock(cv_mutex_);
    cv_.wait_until(lock, deadline, [&] {
        return ready_flag->load(std::memory_order_acquire) || failed_flag->load(std::memory_order_acquire);
    });

    const bool failed = failed_flag->load(std::memory_order_acquire);
    const bool ready = ready_flag->load(std::memory_order_acquire);
    const bool still_pending = pending_flag->load(std::memory_order_acquire);
    const uint64_t submitted_job_id = last_submitted_job_id->load(std::memory_order_acquire);
    const uint64_t completed_job_id = last_completed_job_id->load(std::memory_order_acquire);
    const uint64_t submit_us = last_submit_us->load(std::memory_order_acquire);
    const uint64_t complete_us = last_complete_us->load(std::memory_order_acquire);
    const uint64_t now_us = steadyMicrosNow();
    static std::atomic<int> wait_end_log_count{0};
    if (wait_end_log_count.fetch_add(1, std::memory_order_relaxed) < 3) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "mpp: waitSurfaceReady end surface={} ready={} failed={} pending={} submitted_job={} completed_job={} wait_ms={} since_submit_ms={} since_complete_ms={}",
                  surface,
                  ready ? 1 : 0,
                  failed ? 1 : 0,
                  still_pending ? 1 : 0,
                  static_cast<unsigned long long>(submitted_job_id),
                  static_cast<unsigned long long>(completed_job_id),
                  timeout_ms,
                  submit_us == 0 ? 0ull : static_cast<unsigned long long>((now_us - submit_us) / 1000),
                  complete_us == 0 ? 0ull : static_cast<unsigned long long>((now_us - complete_us) / 1000));
    }

    if (failed) return false;
    return ready;
}

void MppDecoder::forceSurfaceReady(VASurfaceID surface) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end()) return;
    it->second.in_flight.store(false);
    it->second.failed.store(false);
    it->second.ready.store(true);
    std::lock_guard<std::mutex> lock2(cv_mutex_);
    cv_.notify_all();
}

void MppDecoder::resetSurface(VASurfaceID surface) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end()) return;
    it->second.in_flight.store(true);
    it->second.ready.store(false);
    it->second.failed.store(false);
    std::lock_guard<std::mutex> lock2(cv_mutex_);
    cv_.notify_all();
}

void MppDecoder::releaseSurface(VASurfaceID surface) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end()) return;

    if (!stable_export_enabled_) {
        it->second.dmabuf.reset();
    }

    it->second.surface.dmabuf_fd = -1;

    it->second.in_flight.store(false);
    it->second.ready.store(false);
    it->second.failed.store(false);
}

void MppDecoder::destroySurface(VASurfaceID surface) {
    {
        std::lock_guard<std::mutex> lock(surface_mutex_);
        auto it = surfaces_.find(surface);
        if (it != surfaces_.end()) {
            it->second.frame.reset();
            it->second.buffer.reset();
            destroyStableExportBuffer(it->second, export_drm_fd_.get());
            it->second.in_flight.store(false);

            surfaces_.erase(it);
        }
    }

    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        dropPendingSurfaceLocked(surface);
    }
}

void MppDecoder::shutdown() {
    std::shared_ptr<MppSession> session;
    {
        std::scoped_lock lock(init_mutex_);
        if (!session_) {
            session_initialized_ = false;
            return;
        }
        session = session_;
    }

    running_ = false;

    if (!eos_sent_) {
        DecodeJob eos;
        eos.target_surface = VA_INVALID_ID;
        eos.eos = true;
        input_queue_.push(eos);
        eos_sent_ = true;
    }

    input_queue_.shutdown();
    {
        std::scoped_lock lock(pending_mutex_);
        pending_cv_.notify_all();
    }

    if (input_thread_.joinable()) {
        input_thread_.request_stop();
        util::log(util::stderr_sink, util::LogLevel::Info, "joining input thread");
        input_thread_.join();
        util::log(util::stderr_sink, util::LogLevel::Info, "joined input thread");
    }

    // Give the output thread a short window to drain any in-flight frames and
    // observe the EOS frame before we force it to stop.
    for (int i = 0; i < 500; ++i) {
        if (eos_seen_.load()) {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            if (pending_count_ == 0) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (output_thread_.joinable()) {
        output_thread_.request_stop();
        util::log(util::stderr_sink, util::LogLevel::Info, "joining output thread");
        output_thread_.join();
        util::log(util::stderr_sink, util::LogLevel::Info, "joined output thread");
    }

    {
        std::scoped_lock lock(surface_mutex_);
        for (auto& kv : surfaces_) {
            kv.second.frame.reset();
            kv.second.buffer.reset();
            destroyStableExportBuffer(kv.second, export_drm_fd_.get());
            kv.second.in_flight.store(false);
        }
        surfaces_.clear();
    }

    {
        std::scoped_lock lock(pending_mutex_);
        pending_surfaces_.clear();
        pending_surface_pts_.clear();
        pending_payloads_.clear();
        pending_count_ = 0;
        pending_cv_.notify_all();
    }

    if (MppApi* api = session->api(); api && session_initialized_) {
        api->reset(session->ctx());
    }

    {
        std::scoped_lock lock(init_mutex_);
        session_.reset();
        session_initialized_ = false;
    }
}

void MppDecoder::inputThreadMain(std::stop_token st) {
    std::shared_ptr<MppSession> session = session_;
    if (!session) {
        running_ = false;
        eos_seen_ = true;
        return;
    }
    MppCtx ctx = session->ctx();
    MppApi* api = session->api();

    while (!st.stop_requested() && (running_ || !eos_sent_)) {
        auto job_opt = input_queue_.pop();
        if (!job_opt) break;
        DecodeJob job = std::move(*job_opt);

        std::vector<uint8_t> payload;
        const uint8_t* payload_ptr = nullptr;
        size_t payload_size = 0;

        if (job.eos) {
            const bool submitted = submitDecoderEos(ctx, api, running_);
            static std::atomic<int> eos_log_count{0};
            if (eos_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
                util::log(util::stderr_sink, util::LogLevel::Info,
                          "mpp: inputThread EOS submitted={}", submitted ? 1 : 0);
            }
            eos_sent_ = true;
            continue;
        }

        if (!job.extra_data.empty() && !job.bitstream.empty()) {
            payload.reserve(job.extra_data.size() + job.bitstream.size());
            payload.insert(payload.end(), job.extra_data.begin(), job.extra_data.end());
            payload.insert(payload.end(), job.bitstream.begin(), job.bitstream.end());
            payload_ptr = payload.data();
            payload_size = payload.size();
        } else if (!job.extra_data.empty()) {
            payload = std::move(job.extra_data);
            payload_ptr = payload.data();
            payload_size = payload.size();
        } else {
            payload = std::move(job.bitstream);
            payload_ptr = payload.data();
            payload_size = payload.size();
        }

        if (payload_size == 0) continue;

        MppPacket raw_packet = nullptr;
        if (mpp_packet_init(&raw_packet, const_cast<uint8_t*>(payload_ptr), payload_size) != MPP_OK) {
            continue;
        }
        MppPacketHandle packet{raw_packet};

        if (job.job_id > 0) {
            mpp_packet_set_pts(packet.get(), static_cast<RK_S64>(job.job_id));
            mpp_packet_set_dts(packet.get(), static_cast<RK_S64>(job.job_id));
        }

        bool packet_submitted = false;
        while (running_) {
            int ret = api->decode_put_packet(ctx, packet.get());
            if (ret == MPP_OK) {
                packet_submitted = true;
                break;
            }
            if (ret == MPP_ERR_BUFFER_FULL) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            util::log(util::stderr_sink, util::LogLevel::Error,
                      "mpp: decode_put_packet returned {}", ret);
            failPendingSurfaces(job.target_surface);
            running_ = false;
            eos_seen_ = true;
            break;
        }

        if (!packet_submitted) {
            break;
        }

        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_payloads_[job.job_id] = std::make_shared<std::vector<uint8_t>>(std::move(payload));
        }

        static std::atomic<int> input_log_count{0};
        if (input_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: inputThread submitted surface={} bytes={}",
                      job.target_surface, payload_size);
        }
    }
}

void MppDecoder::outputThreadMain(std::stop_token st) {
    std::shared_ptr<MppSession> session = session_;
    if (!session) {
        running_ = false;
        eos_seen_ = true;
        return;
    }
    MppCtx ctx = session->ctx();
    MppApi* api = session->api();

    while (!st.stop_requested() && (running_ || !eos_seen_)) {
        MppFrame frame = nullptr;
        static std::atomic<int> get_log_count{0};
        if (get_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: outputThread calling decode_get_frame pending={} running={}",
                      pending_count_, running_.load() ? 1 : 0);
        }
        int ret = api->decode_get_frame(ctx, &frame);

        if (get_log_count.load(std::memory_order_relaxed) <= 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: outputThread decode_get_frame ret={} frame={}", ret, static_cast<const void*>(frame));
        }

        if (ret == MPP_ERR_DISPLAY_FULL) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (ret == MPP_ERR_TIMEOUT || (ret == MPP_OK && !frame)) {
            if (eos_sent_.load(std::memory_order_relaxed)) {
                std::lock_guard<std::mutex> lock(pending_mutex_);
                if (pending_count_ == 0 && pending_surfaces_.empty()) {
                    eos_seen_ = true;
                    running_ = false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (ret != MPP_OK || !frame) {
            if (ret != MPP_OK) {
                util::log(util::stderr_sink, util::LogLevel::Error,
                          "mpp: decode_get_frame returned {}", ret);
                failPendingSurfaces();
                running_ = false;
                eos_seen_ = true;
                break;
            }
            continue;
        }
        MppFrameHandle frame_handle{frame};

        static std::atomic<int> output_log_count{0};
        if (output_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: outputThread frame eos={} discard={} err={} pending={}",
                      mpp_frame_get_eos(frame) ? 1 : 0,
                      mpp_frame_get_discard(frame) ? 1 : 0,
                      mpp_frame_get_errinfo(frame) ? 1 : 0,
                      pending_count_);
        }

        if (mpp_frame_get_info_change(frame)) {
            uint32_t new_w = mpp_frame_get_width(frame);
            uint32_t new_h = mpp_frame_get_height(frame);
            std::lock_guard<std::mutex> lock(surface_mutex_);
            for (auto& item : surfaces_) {
                item.second.surface.width = new_w;
                item.second.surface.height = new_h;
            }
            api->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
            continue;
        }

        bool is_error = mpp_frame_get_errinfo(frame) || mpp_frame_get_discard(frame);
        bool is_eos_frame = mpp_frame_get_eos(frame);

        const MppBuffer frame_buffer = mpp_frame_get_buffer(frame);
        VASurfaceID surface_id = VA_INVALID_ID;

        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            if (surface_id == VA_INVALID_ID) {
                RK_S64 frame_pts = mpp_frame_get_pts(frame);
                if (frame_pts > 0) {
                    const uint64_t frame_job_id = static_cast<uint64_t>(frame_pts);
                    if (discard_output_job_ids_.erase(frame_job_id) > 0) {
                        static std::atomic<int> discard_log_count{0};
                        if (discard_log_count.fetch_add(1, std::memory_order_relaxed) < 8) {
                            util::log(util::stderr_sink, util::LogLevel::Info,
                                      "mpp: outputThread discarded priming frame job_id={} pts={}",
                                      static_cast<unsigned long long>(frame_job_id),
                                      static_cast<long long>(frame_pts));
                        }
                        continue;
                    }

                    auto it = pending_surface_pts_.find(frame_job_id);
                    if (it != pending_surface_pts_.end()) {
                        surface_id = it->second;
                        dropPendingJobLocked(frame_job_id);
                    }
                }
            }

            if (surface_id == VA_INVALID_ID) {
                surface_id = dropOldestPendingSurfaceLocked();
                static std::atomic<int> fallback_surface_log_count{0};
                if (surface_id != VA_INVALID_ID &&
                    fallback_surface_log_count.fetch_add(1, std::memory_order_relaxed) < 8) {
                    util::log(util::stderr_sink, util::LogLevel::Info,
                              "mpp: outputThread fallback-mapped frame pts={} to oldest pending surface={}",
                              static_cast<long long>(mpp_frame_get_pts(frame)),
                              surface_id);
                }
            }
        }

        if (surface_id != VA_INVALID_ID) {
            std::lock_guard<std::mutex> lock(surface_mutex_);
            auto it = surfaces_.find(surface_id);
            if (it != surfaces_.end()) {
                const MppFrameFormat frame_fmt = mpp_frame_get_fmt(frame);
                const uint32_t frame_mode = mpp_frame_get_mode(frame);
                MppBuffer mpp_buf = mpp_frame_get_buffer(frame);
                const uint32_t output_width = mpp_frame_get_width(frame);
                const uint32_t output_height = mpp_frame_get_height(frame);
                const uint32_t src_stride_pixels = mpp_frame_get_hor_stride_pixel(frame);
                const uint32_t src_stride_bytes = mpp_frame_get_hor_stride(frame);
                const uint32_t src_ver_stride = mpp_frame_get_ver_stride(frame);
                const uint32_t zero_copy_stride = src_stride_pixels ? src_stride_pixels
                                                                    : (it->second.surface.is_10bit ? src_stride_bytes / 2 : src_stride_bytes);
                const int frame_fd = mpp_buf ? mpp_buffer_get_fd(mpp_buf) : -1;
                uint32_t published_stride = zero_copy_stride;
                bool export_ok = false;

                if (stable_export_enabled_ && mpp_buf && output_width > 0 && output_height > 0 && !is_error) {
                    if (!it->second.export_map) {
                        if (!export_drm_fd_) {
                            auto export_fd = openStableExportDrmFd();
                            if (export_fd) {
                                export_drm_fd_ = std::move(*export_fd);
                            }

                            static std::atomic<int> stable_open_log_count{0};
                            if (stable_open_log_count.fetch_add(1, std::memory_order_relaxed) < 4) {
                                util::log(util::stderr_sink,
                                          export_drm_fd_ ? util::LogLevel::Info : util::LogLevel::Warn,
                                          "mpp: stable-export open drm fd ok={}",
                                          export_drm_fd_ ? 1 : 0);
                            }
                        }

                        const bool created = createStableExportBuffer(it->second,
                                                                      export_drm_fd_.get(),
                                                                      output_width,
                                                                      output_height,
                                                                      it->second.surface.is_10bit);
                        static std::atomic<int> stable_alloc_log_count{0};
                        if (stable_alloc_log_count.fetch_add(1, std::memory_order_relaxed) < 4) {
                            util::log(util::stderr_sink, created ? util::LogLevel::Info : util::LogLevel::Warn,
                                      "mpp: stable-export buffer create surface={} out_w={} out_h={} ok={} pitch={} fd={}",
                                      surface_id, output_width, output_height, created ? 1 : 0,
                                      it->second.export_pitch, it->second.dmabuf.get());
                        }
                    }

                    if (it->second.export_map) {
                        const uint8_t* frame_ptr = static_cast<const uint8_t*>(mpp_buffer_get_ptr(mpp_buf));
                        export_ok = copyFrameToStableExport(it->second.export_map,
                                                            it->second.export_pitch,
                                                            output_width,
                                                            output_height,
                                                            src_stride_bytes,
                                                            src_ver_stride,
                                                            frame_ptr);
                        if (export_ok) {
                            it->second.surface.dmabuf_fd = it->second.dmabuf.get();
                            published_stride = it->second.export_pitch;
                        }

                        static std::atomic<int> stable_copy_log_count{0};
                        if (export_ok && stable_copy_log_count.fetch_add(1, std::memory_order_relaxed) < 4) {
                            util::log(util::stderr_sink, util::LogLevel::Info,
                                      "mpp: stable-export copy surface={} out_w={} out_h={} src_stride_bytes={} src_ver_stride={} export_pitch={}",
                                      surface_id, output_width, output_height, src_stride_bytes, src_ver_stride, it->second.export_pitch);
                        }
                    }
                } else if (mpp_buf && frame_fd >= 0 && output_width > 0 && output_height > 0 && !is_error) {
                    mpp_buffer_inc_ref(mpp_buf);
                    it->second.buffer.reset(mpp_buf);
                    it->second.dmabuf.reset(frame_fd >= 0 ? dup(frame_fd) : -1);
                    it->second.surface.dmabuf_fd = it->second.dmabuf.get();
                    export_ok = it->second.surface.dmabuf_fd >= 0;

                    static std::atomic<int> zero_copy_log_count{0};
                    if (export_ok && zero_copy_log_count.fetch_add(1, std::memory_order_relaxed) < 4) {
                        util::log(util::stderr_sink, util::LogLevel::Info,
                                  "mpp: zero-copy frame surface={} fmt={} mode={} out_w={} out_h={} stride_px={} stride_bytes={} ver_stride={} fd={}",
                                  surface_id, static_cast<uint32_t>(frame_fmt), frame_mode,
                                  output_width, output_height, src_stride_pixels, src_stride_bytes, src_ver_stride, frame_fd);
                    }
                }

                if (is_error || !export_ok) {
                    static std::atomic<int> err_log_count{0};
                    if (err_log_count.fetch_add(1, std::memory_order_relaxed) < 4) {
                        util::log(util::stderr_sink, util::LogLevel::Warn,
                                  "mpp: failed to export decoded frame to surface={} width={} height={} stride={} fd={}",
                                  surface_id, output_width, output_height, src_stride_pixels, frame_fd);
                    }
                    it->second.in_flight.store(false);
                    it->second.failed.store(true);
                    it->second.ready.store(false);
                    it->second.last_completed_job_id.store(static_cast<uint64_t>(mpp_frame_get_pts(frame)), std::memory_order_release);
                    it->second.last_complete_us.store(steadyMicrosNow(), std::memory_order_release);
                    {
                        std::lock_guard<std::mutex> lock(cv_mutex_);
                        cv_.notify_all();
                    }
                } else {
                    it->second.surface.width = output_width;
                    it->second.surface.height = output_height;
                    it->second.surface.stride = published_stride;
                    it->second.in_flight.store(false);
                    it->second.failed.store(false);
                    it->second.ready.store(true);
                    it->second.last_completed_job_id.store(static_cast<uint64_t>(mpp_frame_get_pts(frame)), std::memory_order_release);
                    it->second.last_complete_us.store(steadyMicrosNow(), std::memory_order_release);
                    {
                        std::lock_guard<std::mutex> lock(cv_mutex_);
                        cv_.notify_all();
                    }
                }
            }
        }

        last_progress_us_.store(steadyMicrosNow(), std::memory_order_relaxed);
        if (is_eos_frame) {
            eos_seen_ = true;
        }
    }
}

} // namespace rockchip
