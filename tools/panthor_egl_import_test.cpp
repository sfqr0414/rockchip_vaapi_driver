extern "C" {
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <drm_fourcc.h>
#include <xf86drm.h>
}

#include <sys/mman.h>
#include <sys/wait.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <format>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

namespace {

class UniqueFd {
   public:
    UniqueFd() = default;
    explicit UniqueFd(int fd) : fd_(fd) {}

    ~UniqueFd() {
        reset();
    }

    UniqueFd(const UniqueFd&) = delete;
    UniqueFd& operator=(const UniqueFd&) = delete;

    UniqueFd(UniqueFd&& other) noexcept : fd_(other.release()) {}

    UniqueFd& operator=(UniqueFd&& other) noexcept {
        if (this != &other) {
            reset(other.release());
        }
        return *this;
    }

    int get() const {
        return fd_;
    }

    explicit operator bool() const {
        return fd_ >= 0;
    }

    int release() {
        const int fd = fd_;
        fd_ = -1;
        return fd;
    }

    void reset(int fd = -1) {
        if (fd_ >= 0) {
            close(fd_);
        }
        fd_ = fd;
    }

   private:
    int fd_ = -1;
};

template <typename F>
class ScopeExit {
   public:
    explicit ScopeExit(F&& func) : func_(std::forward<F>(func)) {}

    ScopeExit(const ScopeExit&) = delete;
    ScopeExit& operator=(const ScopeExit&) = delete;

    ScopeExit(ScopeExit&& other) noexcept : func_(std::move(other.func_)), active_(other.active_) {
        other.active_ = false;
    }

    ~ScopeExit() {
        if (active_) {
            func_();
        }
    }

   private:
    F func_;
    bool active_ = true;
};

template <typename F>
ScopeExit<F> makeScopeExit(F&& func) {
    return ScopeExit<F>(std::forward<F>(func));
}

[[noreturn]] void fail(std::string_view message) {
    std::cerr << std::format("[Panthor EGL Test] ERROR: {}\n", message);
    std::exit(1);
}

bool containsInsensitive(std::string_view haystack, std::string_view needle) {
    auto to_lower = [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); };
    std::string lowered_haystack(haystack.begin(), haystack.end());
    std::string lowered_needle(needle.begin(), needle.end());
    std::transform(lowered_haystack.begin(), lowered_haystack.end(), lowered_haystack.begin(), to_lower);
    std::transform(lowered_needle.begin(), lowered_needle.end(), lowered_needle.begin(), to_lower);
    return lowered_haystack.find(lowered_needle) != std::string::npos;
}

std::string fourccToString(uint32_t fourcc) {
    std::string out(4, '\0');
    out[0] = static_cast<char>(fourcc & 0xff);
    out[1] = static_cast<char>((fourcc >> 8) & 0xff);
    out[2] = static_cast<char>((fourcc >> 16) & 0xff);
    out[3] = static_cast<char>((fourcc >> 24) & 0xff);
    for (char& ch : out) {
        if (ch == '\0') {
            ch = ' ';
        }
    }
    return out;
}

std::string drmFormatName(uint32_t format) {
    switch (format) {
        case DRM_FORMAT_R8:
            return "R8";
        case DRM_FORMAT_GR88:
            return "GR88";
        default:
            return fourccToString(format);
    }
}

std::string readFdInfo(int fd) {
    std::ifstream stream(std::format("/proc/self/fdinfo/{}", fd));
    if (!stream.is_open()) {
        return std::format("<fdinfo unavailable: {}>", std::strerror(errno));
    }

    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

std::string readFdLinkTarget(int fd) {
    std::array<char, 512> buffer{};
    const auto path = std::format("/proc/self/fd/{}", fd);
    const ssize_t length = readlink(path.c_str(), buffer.data(), buffer.size() - 1);
    if (length < 0) {
        return std::format("<readlink failed: {}>", std::strerror(errno));
    }
    buffer[static_cast<size_t>(length)] = '\0';
    return buffer.data();
}

void printFdInfo(const char* prefix, int fd) {
    std::cout << std::format("{} fdlink[{}]: {}\n", prefix, fd, readFdLinkTarget(fd));
    const std::string content = readFdInfo(fd);
    std::istringstream lines(content);
    std::string line;
    while (std::getline(lines, line)) {
        std::cout << std::format("{} fdinfo[{}]: {}\n", prefix, fd, line);
    }
}

struct Options {
    std::string drm_device;
    int width = 1920;
    int height = 1080;
};

struct DrmDevice {
    UniqueFd fd;
    std::string path;
    std::string driver_name;
};

struct DumbBuffer {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t bpp = 0;
    uint32_t handle = 0;
    uint32_t pitch = 0;
    uint64_t size = 0;
    uint32_t drm_format = 0;
    UniqueFd prime_fd;
    void* map = MAP_FAILED;
};

struct PlaneImport {
    int fd = -1;
    uint32_t drm_format = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pitch = 0;
    uint32_t offset = 0;
    uint64_t modifier = DRM_FORMAT_MOD_LINEAR;
};

struct Nv12Planes {
    std::string allocation_path;
    std::string allocation_driver;
    PlaneImport y;
    PlaneImport uv;
};

void printSyntheticMetadata(const DumbBuffer& buffer, const Nv12Planes& planes) {
    std::cout << "[Panthor Meta] begin descriptor\n";
    std::cout << std::format("[Panthor Meta] frame: width={} height={} fourcc=NV12-like objects=1 layers=2 unique_hint=1\n",
                             planes.y.width,
                             planes.y.height);
    std::cout << std::format("[Panthor Meta] object[0]: fd={} size={} modifier=0x{:x}\n",
                             planes.y.fd,
                             buffer.size,
                             DRM_FORMAT_MOD_LINEAR);
    printFdInfo("[Panthor Meta]", planes.y.fd);
    std::cout << std::format("[Panthor Meta] layer[0]: drm_format={} num_planes=1 pitch0={} offset0={} object0=0 pitch1=0 offset1=0 object1=0\n",
                             drmFormatName(planes.y.drm_format),
                             planes.y.pitch,
                             planes.y.offset);
    std::cout << std::format("[Panthor Meta] layer[1]: drm_format={} num_planes=1 pitch0={} offset0={} object0=0 pitch1=0 offset1=0 object1=0\n",
                             drmFormatName(planes.uv.drm_format),
                             planes.uv.pitch,
                             planes.uv.offset);
    std::cout << std::format("[Panthor Meta] plane[0:0]: object=0 fd={} offset={} pitch={} modifier=0x{:x}\n",
                             planes.y.fd,
                             planes.y.offset,
                             planes.y.pitch,
                             planes.y.modifier);
    std::cout << std::format("[Panthor Meta] plane[1:0]: object=0 fd={} offset={} pitch={} modifier=0x{:x}\n",
                             planes.uv.fd,
                             planes.uv.offset,
                             planes.uv.pitch,
                             planes.uv.modifier);
    std::cout << "[Panthor Meta] end descriptor\n";
}

std::string getDriverName(int fd) {
    drmVersionPtr version = drmGetVersion(fd);
    if (!version) {
        return {};
    }
    const std::string name = version->name ? version->name : "";
    drmFreeVersion(version);
    return name;
}

std::optional<DrmDevice> openSpecificDevice(const std::string& path) {
    UniqueFd fd(open(path.c_str(), O_RDWR | O_CLOEXEC));
    if (!fd) {
        return std::nullopt;
    }
    DrmDevice device;
    device.driver_name = getDriverName(fd.get());
    device.path = path;
    device.fd = std::move(fd);
    return device;
}

Options parseArgs(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string_view arg = argv[index];
        auto require_value = [&](std::string_view flag) -> const char* {
            if (index + 1 >= argc) {
                fail(std::format("missing value for {}", flag));
            }
            return argv[++index];
        };

        if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  --drm-device <path>  DRM primary node to allocate from\n"
                      << "  --width <pixels>     Test image width (default 1920)\n"
                      << "  --height <pixels>    Test image height (default 1080)\n";
            std::exit(0);
        }
        if (arg == "--drm-device") {
            options.drm_device = require_value(arg);
            continue;
        }
        if (arg == "--width") {
            options.width = std::stoi(require_value(arg));
            continue;
        }
        if (arg == "--height") {
            options.height = std::stoi(require_value(arg));
            continue;
        }

        fail(std::format("unknown argument {}", arg));
    }

    if (options.width <= 0 || options.height <= 0) {
        fail("width and height must be positive");
    }
    if ((options.width % 2) != 0 || (options.height % 2) != 0) {
        fail("width and height must be even for NV12-style separate planes");
    }
    return options;
}

bool tryCreateTinyDumbBuffer(int drm_fd) {
    drm_mode_create_dumb create = {};
    create.width = 64;
    create.height = 64;
    create.bpp = 8;
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) != 0) {
        return false;
    }

    drm_mode_destroy_dumb destroy = {};
    destroy.handle = create.handle;
    drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    return true;
}

DrmDevice resolvePanthorPrimaryDevice(const Options& options) {
    if (!options.drm_device.empty()) {
        auto device = openSpecificDevice(options.drm_device);
        if (!device) {
            fail(std::format("failed to open DRM device {}", options.drm_device));
        }
        return std::move(*device);
    }

    drmDevicePtr raw_devices[64] = {};
    const int count = drmGetDevices2(0, raw_devices, static_cast<int>(std::size(raw_devices)));
    if (count < 0) {
        fail("drmGetDevices2 failed while resolving Panthor primary node");
    }

    std::optional<DrmDevice> panthor_without_dumb;
    std::optional<DrmDevice> fallback_with_dumb;
    for (int index = 0; index < count; ++index) {
        drmDevicePtr info = raw_devices[index];
        if (!info || (info->available_nodes & (1 << DRM_NODE_PRIMARY)) == 0 || !info->nodes[DRM_NODE_PRIMARY]) {
            continue;
        }

        auto opened = openSpecificDevice(info->nodes[DRM_NODE_PRIMARY]);
        if (!opened) {
            continue;
        }

        const bool is_panthor = containsInsensitive(opened->driver_name, "panthor");
        const bool has_dumb = tryCreateTinyDumbBuffer(opened->fd.get());
        if (is_panthor && has_dumb) {
            drmFreeDevices(raw_devices, count);
            return std::move(*opened);
        }
        if (is_panthor && !panthor_without_dumb) {
            panthor_without_dumb = std::move(*opened);
            continue;
        }
        if (has_dumb && (!fallback_with_dumb || containsInsensitive(opened->driver_name, "rockchip"))) {
            fallback_with_dumb = std::move(*opened);
        }
    }

    drmFreeDevices(raw_devices, count);
    if (panthor_without_dumb) {
        std::cout << std::format("[Panthor EGL Test] INFO: {} ({}) does not support dumb buffers; falling back to another primary node for allocation\n",
                                 panthor_without_dumb->path,
                                 panthor_without_dumb->driver_name);
    }
    if (fallback_with_dumb) {
        return std::move(*fallback_with_dumb);
    }
    fail("no DRM primary node with dumb-buffer support was found");
}

void drmIoctlOrFail(int fd, unsigned long request, void* arg, std::string_view stage) {
    if (drmIoctl(fd, request, arg) != 0) {
        fail(std::format("{} failed: {}", stage, std::strerror(errno)));
    }
}

DumbBuffer createDumbBuffer(int drm_fd, uint32_t width, uint32_t height, uint32_t bpp, uint32_t drm_format) {
    DumbBuffer buffer;
    buffer.width = width;
    buffer.height = height;
    buffer.bpp = bpp;
    buffer.drm_format = drm_format;

    drm_mode_create_dumb create = {};
    create.width = width;
    create.height = height;
    create.bpp = bpp;
    drmIoctlOrFail(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create, "DRM_IOCTL_MODE_CREATE_DUMB");

    buffer.handle = create.handle;
    buffer.pitch = create.pitch;
    buffer.size = create.size;

    drm_prime_handle prime = {};
    prime.handle = buffer.handle;
    prime.flags = DRM_CLOEXEC | DRM_RDWR;
    drmIoctlOrFail(drm_fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime, "DRM_IOCTL_PRIME_HANDLE_TO_FD");
    buffer.prime_fd.reset(prime.fd);

    drm_mode_map_dumb map = {};
    map.handle = buffer.handle;
    drmIoctlOrFail(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map, "DRM_IOCTL_MODE_MAP_DUMB");

    buffer.map = mmap(nullptr, buffer.size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, static_cast<off_t>(map.offset));
    if (buffer.map == MAP_FAILED) {
        fail(std::format("mmap dumb buffer failed: {}", std::strerror(errno)));
    }
    return buffer;
}

void destroyDumbBuffer(int drm_fd, DumbBuffer& buffer) {
    if (buffer.map != MAP_FAILED) {
        munmap(buffer.map, buffer.size);
        buffer.map = MAP_FAILED;
    }
    buffer.prime_fd.reset();
    if (buffer.handle != 0) {
        drm_mode_destroy_dumb destroy = {};
        destroy.handle = buffer.handle;
        if (drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy) != 0) {
            std::cerr << std::format("[Panthor EGL Test] WARN: DRM_IOCTL_MODE_DESTROY_DUMB failed for handle {}: {}\n",
                                     buffer.handle,
                                     std::strerror(errno));
        }
        buffer.handle = 0;
    }
}

void fillYPlane(DumbBuffer& buffer) {
    auto* bytes = static_cast<uint8_t*>(buffer.map);
    for (uint32_t row = 0; row < buffer.height; ++row) {
        std::memset(bytes + row * buffer.pitch, 0xd0, buffer.width);
    }
}

void fillUvPlane(DumbBuffer& buffer) {
    auto* bytes = static_cast<uint8_t*>(buffer.map);
    for (uint32_t row = 0; row < buffer.height; ++row) {
        auto* line = bytes + row * buffer.pitch;
        for (uint32_t column = 0; column < buffer.width; ++column) {
            line[column * 2 + 0] = 0x80;
            line[column * 2 + 1] = 0x80;
        }
    }
}

void fillNv12LikeBuffer(DumbBuffer& buffer, uint32_t width, uint32_t height) {
    auto* bytes = static_cast<uint8_t*>(buffer.map);
    for (uint32_t row = 0; row < height; ++row) {
        std::memset(bytes + row * buffer.pitch, 0xd0, width);
    }

    auto* uv_base = bytes + static_cast<size_t>(buffer.pitch) * height;
    for (uint32_t row = 0; row < height / 2; ++row) {
        auto* line = uv_base + row * buffer.pitch;
        for (uint32_t column = 0; column < width / 2; ++column) {
            line[column * 2 + 0] = 0x80;
            line[column * 2 + 1] = 0x80;
        }
    }
}

Nv12Planes buildPlanes(const DrmDevice& device, const DumbBuffer& buffer, uint32_t width, uint32_t height) {
    Nv12Planes planes;
    planes.allocation_path = device.path;
    planes.allocation_driver = device.driver_name;
    planes.y.fd = buffer.prime_fd.get();
    planes.y.drm_format = DRM_FORMAT_R8;
    planes.y.width = width;
    planes.y.height = height;
    planes.y.pitch = buffer.pitch;
    planes.y.offset = 0;

    planes.uv.fd = buffer.prime_fd.get();
    planes.uv.drm_format = DRM_FORMAT_GR88;
    planes.uv.width = width / 2;
    planes.uv.height = height / 2;
    planes.uv.pitch = buffer.pitch;
    planes.uv.offset = static_cast<uint32_t>(buffer.pitch * height);
    return planes;
}

namespace egl_runtime {

using EGLBoolean = unsigned int;
using EGLenum = unsigned int;
using EGLint = int;
using EGLDisplay = void*;
using EGLConfig = void*;
using EGLContext = void*;
using EGLSurface = void*;
using EGLImageKHR = void*;
using EGLClientBuffer = void*;
using EGLNativeDisplayType = void*;

using GLenum = unsigned int;
using GLuint = unsigned int;
using GLint = int;
using GLsizei = int;
using GLbitfield = unsigned int;
using GLfloat = float;
using GLboolean = unsigned char;
using GLubyte = unsigned char;
using GLchar = char;

constexpr EGLBoolean EGL_TRUE = 1;
constexpr EGLenum EGL_OPENGL_ES_API = 0x30A0;
constexpr EGLint EGL_NONE = 0x3038;
constexpr EGLint EGL_EXTENSIONS = 0x3055;
constexpr EGLint EGL_HEIGHT = 0x3056;
constexpr EGLint EGL_WIDTH = 0x3057;
constexpr EGLint EGL_SURFACE_TYPE = 0x3033;
constexpr EGLint EGL_PBUFFER_BIT = 0x0001;
constexpr EGLint EGL_RED_SIZE = 0x3024;
constexpr EGLint EGL_GREEN_SIZE = 0x3023;
constexpr EGLint EGL_BLUE_SIZE = 0x3022;
constexpr EGLint EGL_ALPHA_SIZE = 0x3021;
constexpr EGLint EGL_RENDERABLE_TYPE = 0x3040;
constexpr EGLint EGL_OPENGL_ES2_BIT = 0x0004;
constexpr EGLint EGL_CONTEXT_CLIENT_VERSION = 0x3098;
constexpr EGLenum EGL_LINUX_DMA_BUF_EXT = 0x3270;
constexpr EGLint EGL_LINUX_DRM_FOURCC_EXT = 0x3271;
constexpr EGLint EGL_DMA_BUF_PLANE0_FD_EXT = 0x3272;
constexpr EGLint EGL_DMA_BUF_PLANE0_OFFSET_EXT = 0x3273;
constexpr EGLint EGL_DMA_BUF_PLANE0_PITCH_EXT = 0x3274;
constexpr EGLint EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT = 0x3443;
constexpr EGLint EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT = 0x3444;
constexpr EGLint EGL_PLATFORM_SURFACELESS_MESA = 0x31DD;
constexpr EGLDisplay EGL_NO_DISPLAY = nullptr;
constexpr EGLContext EGL_NO_CONTEXT = nullptr;
constexpr EGLSurface EGL_NO_SURFACE = nullptr;
constexpr EGLImageKHR EGL_NO_IMAGE_KHR = nullptr;
constexpr EGLNativeDisplayType EGL_DEFAULT_DISPLAY = nullptr;

constexpr GLenum GL_EXTENSIONS = 0x1F03;
constexpr GLenum GL_VENDOR = 0x1F00;
constexpr GLenum GL_RENDERER = 0x1F01;
constexpr GLenum GL_VERSION = 0x1F02;
constexpr GLenum GL_TEXTURE0 = 0x84C0;
constexpr GLenum GL_TEXTURE1 = 0x84C1;
constexpr GLenum GL_TEXTURE_2D = 0x0DE1;
constexpr GLenum GL_TEXTURE_MIN_FILTER = 0x2801;
constexpr GLenum GL_TEXTURE_MAG_FILTER = 0x2800;
constexpr GLenum GL_TEXTURE_WRAP_S = 0x2802;
constexpr GLenum GL_TEXTURE_WRAP_T = 0x2803;
constexpr GLenum GL_LINEAR = 0x2601;
constexpr GLenum GL_CLAMP_TO_EDGE = 0x812F;
constexpr GLenum GL_COLOR_BUFFER_BIT = 0x00004000;
constexpr GLenum GL_FLOAT = 0x1406;
constexpr GLenum GL_FALSE = 0;
constexpr GLenum GL_RGBA = 0x1908;
constexpr GLenum GL_UNSIGNED_BYTE = 0x1401;
constexpr GLenum GL_TRIANGLE_STRIP = 0x0005;
constexpr GLenum GL_ARRAY_BUFFER = 0x8892;
constexpr GLenum GL_STATIC_DRAW = 0x88E4;
constexpr GLenum GL_VERTEX_SHADER = 0x8B31;
constexpr GLenum GL_FRAGMENT_SHADER = 0x8B30;
constexpr GLenum GL_COMPILE_STATUS = 0x8B81;
constexpr GLenum GL_LINK_STATUS = 0x8B82;

using PFNEGLGETDISPLAYPROC = EGLDisplay (*)(EGLNativeDisplayType);
using PFNEGLINITIALIZEPROC = EGLBoolean (*)(EGLDisplay, EGLint*, EGLint*);
using PFNEGLTERMINATEPROC = EGLBoolean (*)(EGLDisplay);
using PFNEGLBINDAPIPROC = EGLBoolean (*)(EGLenum);
using PFNEGLQUERYSTRINGPROC = const char* (*)(EGLDisplay, EGLint);
using PFNEGLCHOOSECONFIGPROC = EGLBoolean (*)(EGLDisplay, const EGLint*, EGLConfig*, EGLint, EGLint*);
using PFNEGLCREATEPBUFFERSURFACEPROC = EGLSurface (*)(EGLDisplay, EGLConfig, const EGLint*);
using PFNEGLDESTROYSURFACEPROC = EGLBoolean (*)(EGLDisplay, EGLSurface);
using PFNEGLCREATECONTEXTPROC = EGLContext (*)(EGLDisplay, EGLConfig, EGLContext, const EGLint*);
using PFNEGLDESTROYCONTEXTPROC = EGLBoolean (*)(EGLDisplay, EGLContext);
using PFNEGLMAKECURRENTPROC = EGLBoolean (*)(EGLDisplay, EGLSurface, EGLSurface, EGLContext);
using PFNEGLGETERRORPROC = EGLint (*)();
using PFNEGLGETPROCADDRESSPROC = void* (*)(const char*);
using PFNEGLCREATEIMAGEKHRPROC = EGLImageKHR (*)(EGLDisplay, EGLContext, EGLenum, EGLClientBuffer, const EGLint*);
using PFNEGLDESTROYIMAGEKHRPROC = EGLBoolean (*)(EGLDisplay, EGLImageKHR);
using PFNEGLGETPLATFORMDISPLAYEXTPROC = EGLDisplay (*)(EGLenum, void*, const EGLint*);

using PFNGLGETSTRINGPROC = const GLubyte* (*)(GLenum);
using PFNGLGENTEXTURESPROC = void (*)(GLsizei, GLuint*);
using PFNGLBINDTEXTUREPROC = void (*)(GLenum, GLuint);
using PFNGLTEXPARAMETERIPROC = void (*)(GLenum, GLenum, GLint);
using PFNGLCREATESHADERPROC = GLuint (*)(GLenum);
using PFNGLSHADERSOURCEPROC = void (*)(GLuint, GLsizei, const GLchar* const*, const GLint*);
using PFNGLCOMPILESHADERPROC = void (*)(GLuint);
using PFNGLGETSHADERIVPROC = void (*)(GLuint, GLenum, GLint*);
using PFNGLGETSHADERINFOLOGPROC = void (*)(GLuint, GLsizei, GLsizei*, GLchar*);
using PFNGLDELETESHADERPROC = void (*)(GLuint);
using PFNGLCREATEPROGRAMPROC = GLuint (*)();
using PFNGLATTACHSHADERPROC = void (*)(GLuint, GLuint);
using PFNGLBINDATTRIBLOCATIONPROC = void (*)(GLuint, GLuint, const GLchar*);
using PFNGLLINKPROGRAMPROC = void (*)(GLuint);
using PFNGLGETPROGRAMIVPROC = void (*)(GLuint, GLenum, GLint*);
using PFNGLGETPROGRAMINFOLOGPROC = void (*)(GLuint, GLsizei, GLsizei*, GLchar*);
using PFNGLDELETEPROGRAMPROC = void (*)(GLuint);
using PFNGLUSEPROGRAMPROC = void (*)(GLuint);
using PFNGLACTIVETEXTUREPROC = void (*)(GLenum);
using PFNGLGETUNIFORMLOCATIONPROC = GLint (*)(GLuint, const GLchar*);
using PFNGLUNIFORM1IPROC = void (*)(GLint, GLint);
using PFNGLGENBUFFERSPROC = void (*)(GLsizei, GLuint*);
using PFNGLBINDBUFFERPROC = void (*)(GLenum, GLuint);
using PFNGLBUFFERDATAPROC = void (*)(GLenum, std::ptrdiff_t, const void*, GLenum);
using PFNGLVERTEXATTRIBPOINTERPROC = void (*)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
using PFNGLENABLEVERTEXATTRIBARRAYPROC = void (*)(GLuint);
using PFNGLVIEWPORTPROC = void (*)(GLint, GLint, GLsizei, GLsizei);
using PFNGLCLEARCOLORPROC = void (*)(GLfloat, GLfloat, GLfloat, GLfloat);
using PFNGLCLEARPROC = void (*)(GLbitfield);
using PFNGLDRAWARRAYSPROC = void (*)(GLenum, GLint, GLsizei);
using PFNGLREADPIXELSPROC = void (*)(GLint, GLint, GLsizei, GLsizei, GLenum, GLenum, void*);
using PFNGLFINISHPROC = void (*)();
using PFNGLGETERRORPROC = GLenum (*)();
using PFNGLDELETEBUFFERSPROC = void (*)(GLsizei, const GLuint*);
using PFNGLDELETETEXTURESPROC = void (*)(GLsizei, const GLuint*);
using PFNGLEGLIMAGETARGETTEXTURE2DOESPROC = void (*)(GLenum, void*);

class Library {
   public:
    explicit Library(const char* path) : handle_(dlopen(path, RTLD_NOW | RTLD_LOCAL)) {}

    ~Library() {
        if (handle_) {
            dlclose(handle_);
        }
    }

    Library(const Library&) = delete;
    Library& operator=(const Library&) = delete;

    bool loaded() const {
        return handle_ != nullptr;
    }

    void* symbol(const char* name) const {
        return handle_ ? dlsym(handle_, name) : nullptr;
    }

   private:
    void* handle_ = nullptr;
};

struct ImportResult {
    std::array<uint8_t, 4> pixel = {0, 0, 0, 0};
    std::string egl_extensions;
    std::string gl_vendor;
    std::string gl_renderer;
    std::string gl_version;
    std::string gl_extensions;
};

class Runtime {
   public:
    Runtime() : egl_("/lib/aarch64-linux-gnu/libEGL.so.1"), gles_("/lib/aarch64-linux-gnu/libGLESv2.so.2") {
        if (!egl_.loaded() || !gles_.loaded()) {
            fail("failed to load libEGL.so.1 or libGLESv2.so.2");
        }

        eglGetDisplay = loadRequired<PFNEGLGETDISPLAYPROC>(egl_, "eglGetDisplay");
        eglInitialize = loadRequired<PFNEGLINITIALIZEPROC>(egl_, "eglInitialize");
        eglTerminate = loadRequired<PFNEGLTERMINATEPROC>(egl_, "eglTerminate");
        eglBindAPI = loadRequired<PFNEGLBINDAPIPROC>(egl_, "eglBindAPI");
        eglQueryString = loadRequired<PFNEGLQUERYSTRINGPROC>(egl_, "eglQueryString");
        eglChooseConfig = loadRequired<PFNEGLCHOOSECONFIGPROC>(egl_, "eglChooseConfig");
        eglCreatePbufferSurface = loadRequired<PFNEGLCREATEPBUFFERSURFACEPROC>(egl_, "eglCreatePbufferSurface");
        eglDestroySurface = loadRequired<PFNEGLDESTROYSURFACEPROC>(egl_, "eglDestroySurface");
        eglCreateContext = loadRequired<PFNEGLCREATECONTEXTPROC>(egl_, "eglCreateContext");
        eglDestroyContext = loadRequired<PFNEGLDESTROYCONTEXTPROC>(egl_, "eglDestroyContext");
        eglMakeCurrent = loadRequired<PFNEGLMAKECURRENTPROC>(egl_, "eglMakeCurrent");
        eglGetError = loadRequired<PFNEGLGETERRORPROC>(egl_, "eglGetError");
        eglGetProcAddress = loadRequired<PFNEGLGETPROCADDRESSPROC>(egl_, "eglGetProcAddress");
        eglGetPlatformDisplayEXT = loadOptional<PFNEGLGETPLATFORMDISPLAYEXTPROC>(egl_, "eglGetPlatformDisplayEXT");

        glGetString = loadRequired<PFNGLGETSTRINGPROC>(gles_, "glGetString");
        glGenTextures = loadRequired<PFNGLGENTEXTURESPROC>(gles_, "glGenTextures");
        glBindTexture = loadRequired<PFNGLBINDTEXTUREPROC>(gles_, "glBindTexture");
        glTexParameteri = loadRequired<PFNGLTEXPARAMETERIPROC>(gles_, "glTexParameteri");
        glCreateShader = loadRequired<PFNGLCREATESHADERPROC>(gles_, "glCreateShader");
        glShaderSource = loadRequired<PFNGLSHADERSOURCEPROC>(gles_, "glShaderSource");
        glCompileShader = loadRequired<PFNGLCOMPILESHADERPROC>(gles_, "glCompileShader");
        glGetShaderiv = loadRequired<PFNGLGETSHADERIVPROC>(gles_, "glGetShaderiv");
        glGetShaderInfoLog = loadRequired<PFNGLGETSHADERINFOLOGPROC>(gles_, "glGetShaderInfoLog");
        glDeleteShader = loadRequired<PFNGLDELETESHADERPROC>(gles_, "glDeleteShader");
        glCreateProgram = loadRequired<PFNGLCREATEPROGRAMPROC>(gles_, "glCreateProgram");
        glAttachShader = loadRequired<PFNGLATTACHSHADERPROC>(gles_, "glAttachShader");
        glBindAttribLocation = loadRequired<PFNGLBINDATTRIBLOCATIONPROC>(gles_, "glBindAttribLocation");
        glLinkProgram = loadRequired<PFNGLLINKPROGRAMPROC>(gles_, "glLinkProgram");
        glGetProgramiv = loadRequired<PFNGLGETPROGRAMIVPROC>(gles_, "glGetProgramiv");
        glGetProgramInfoLog = loadRequired<PFNGLGETPROGRAMINFOLOGPROC>(gles_, "glGetProgramInfoLog");
        glDeleteProgram = loadRequired<PFNGLDELETEPROGRAMPROC>(gles_, "glDeleteProgram");
        glUseProgram = loadRequired<PFNGLUSEPROGRAMPROC>(gles_, "glUseProgram");
        glActiveTexture = loadRequired<PFNGLACTIVETEXTUREPROC>(gles_, "glActiveTexture");
        glGetUniformLocation = loadRequired<PFNGLGETUNIFORMLOCATIONPROC>(gles_, "glGetUniformLocation");
        glUniform1i = loadRequired<PFNGLUNIFORM1IPROC>(gles_, "glUniform1i");
        glGenBuffers = loadRequired<PFNGLGENBUFFERSPROC>(gles_, "glGenBuffers");
        glBindBuffer = loadRequired<PFNGLBINDBUFFERPROC>(gles_, "glBindBuffer");
        glBufferData = loadRequired<PFNGLBUFFERDATAPROC>(gles_, "glBufferData");
        glVertexAttribPointer = loadRequired<PFNGLVERTEXATTRIBPOINTERPROC>(gles_, "glVertexAttribPointer");
        glEnableVertexAttribArray = loadRequired<PFNGLENABLEVERTEXATTRIBARRAYPROC>(gles_, "glEnableVertexAttribArray");
        glViewport = loadRequired<PFNGLVIEWPORTPROC>(gles_, "glViewport");
        glClearColor = loadRequired<PFNGLCLEARCOLORPROC>(gles_, "glClearColor");
        glClear = loadRequired<PFNGLCLEARPROC>(gles_, "glClear");
        glDrawArrays = loadRequired<PFNGLDRAWARRAYSPROC>(gles_, "glDrawArrays");
        glReadPixels = loadRequired<PFNGLREADPIXELSPROC>(gles_, "glReadPixels");
        glFinish = loadRequired<PFNGLFINISHPROC>(gles_, "glFinish");
        glGetError = loadRequired<PFNGLGETERRORPROC>(gles_, "glGetError");
        glDeleteBuffers = loadRequired<PFNGLDELETEBUFFERSPROC>(gles_, "glDeleteBuffers");
        glDeleteTextures = loadRequired<PFNGLDELETETEXTURESPROC>(gles_, "glDeleteTextures");
    }

    ImportResult importAndDraw(const Nv12Planes& planes) {
        std::cout << "[Panthor EGL Test] EGL Step: load default display\n";
        EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (display == EGL_NO_DISPLAY && eglGetPlatformDisplayEXT) {
            std::cout << "[Panthor EGL Test] EGL Step: fallback to surfaceless display\n";
            display = eglGetPlatformDisplayEXT(EGL_PLATFORM_SURFACELESS_MESA, nullptr, nullptr);
        }
        if (display == EGL_NO_DISPLAY) {
            fail(std::format("eglGetDisplay failed: 0x{:x}", eglGetError()));
        }

        if (eglInitialize(display, nullptr, nullptr) != EGL_TRUE) {
            fail(std::format("eglInitialize failed: 0x{:x}", eglGetError()));
        }
        auto cleanup_display = makeScopeExit([&] { eglTerminate(display); });

        ImportResult result;
        if (const char* egl_extensions = eglQueryString(display, EGL_EXTENSIONS)) {
            result.egl_extensions = egl_extensions;
        }
        if (!containsInsensitive(result.egl_extensions, "EGL_EXT_image_dma_buf_import")) {
            fail("EGL does not advertise EGL_EXT_image_dma_buf_import");
        }

        if (eglBindAPI(EGL_OPENGL_ES_API) != EGL_TRUE) {
            fail(std::format("eglBindAPI failed: 0x{:x}", eglGetError()));
        }

        const EGLint config_attribs[] = {
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
            EGL_RED_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_BLUE_SIZE, 8,
            EGL_ALPHA_SIZE, 8,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
            EGL_NONE,
        };
        EGLConfig config = nullptr;
        EGLint num_configs = 0;
        if (eglChooseConfig(display, config_attribs, &config, 1, &num_configs) != EGL_TRUE || num_configs != 1) {
            fail(std::format("eglChooseConfig failed: 0x{:x}", eglGetError()));
        }

        const EGLint pbuffer_attribs[] = {
            EGL_WIDTH, 16,
            EGL_HEIGHT, 16,
            EGL_NONE,
        };
        EGLSurface surface = eglCreatePbufferSurface(display, config, pbuffer_attribs);
        if (surface == EGL_NO_SURFACE) {
            fail(std::format("eglCreatePbufferSurface failed: 0x{:x}", eglGetError()));
        }
        auto cleanup_surface = makeScopeExit([&] { eglDestroySurface(display, surface); });

        const EGLint context_attribs[] = {
            EGL_CONTEXT_CLIENT_VERSION, 2,
            EGL_NONE,
        };
        EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, context_attribs);
        if (context == EGL_NO_CONTEXT) {
            fail(std::format("eglCreateContext failed: 0x{:x}", eglGetError()));
        }
        auto cleanup_context = makeScopeExit([&] { eglDestroyContext(display, context); });

        if (eglMakeCurrent(display, surface, surface, context) != EGL_TRUE) {
            fail(std::format("eglMakeCurrent failed: 0x{:x}", eglGetError()));
        }

        eglCreateImageKHR = loadRequiredProc<PFNEGLCREATEIMAGEKHRPROC>("eglCreateImageKHR");
        eglDestroyImageKHR = loadRequiredProc<PFNEGLDESTROYIMAGEKHRPROC>("eglDestroyImageKHR");
        glEGLImageTargetTexture2DOES = loadRequiredProc<PFNGLEGLIMAGETARGETTEXTURE2DOESPROC>("glEGLImageTargetTexture2DOES");

        result.gl_vendor = glString(GL_VENDOR);
        result.gl_renderer = glString(GL_RENDERER);
        result.gl_version = glString(GL_VERSION);
        result.gl_extensions = glString(GL_EXTENSIONS);
        if (!containsInsensitive(result.gl_extensions, "GL_OES_EGL_image")) {
            fail("GL does not advertise GL_OES_EGL_image");
        }

        const auto create_image = [&](const PlaneImport& plane) {
            const EGLint image_attribs[] = {
                EGL_WIDTH, static_cast<EGLint>(plane.width),
                EGL_HEIGHT, static_cast<EGLint>(plane.height),
                EGL_LINUX_DRM_FOURCC_EXT, static_cast<EGLint>(plane.drm_format),
                EGL_DMA_BUF_PLANE0_FD_EXT, plane.fd,
                EGL_DMA_BUF_PLANE0_OFFSET_EXT, static_cast<EGLint>(plane.offset),
                EGL_DMA_BUF_PLANE0_PITCH_EXT, static_cast<EGLint>(plane.pitch),
                EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT, static_cast<EGLint>(plane.modifier & 0xffffffffu),
                EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT, static_cast<EGLint>(plane.modifier >> 32),
                EGL_NONE,
            };
            return eglCreateImageKHR(display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, image_attribs);
        };

        std::cout << std::format("[Panthor EGL Test] EGL Step: create {} image {}x{} pitch={} fd={}\n",
                                 drmFormatName(planes.y.drm_format),
                                 planes.y.width,
                                 planes.y.height,
                                 planes.y.pitch,
                                 planes.y.fd);
        EGLImageKHR y_image = create_image(planes.y);
        if (y_image == EGL_NO_IMAGE_KHR) {
            fail(std::format("eglCreateImageKHR(Y) failed: 0x{:x}", eglGetError()));
        }

        std::cout << std::format("[Panthor EGL Test] EGL Step: create {} image {}x{} pitch={} fd={}\n",
                                 drmFormatName(planes.uv.drm_format),
                                 planes.uv.width,
                                 planes.uv.height,
                                 planes.uv.pitch,
                                 planes.uv.fd);
        EGLImageKHR uv_image = create_image(planes.uv);
        if (uv_image == EGL_NO_IMAGE_KHR) {
            eglDestroyImageKHR(display, y_image);
            fail(std::format("eglCreateImageKHR(UV) failed: 0x{:x}", eglGetError()));
        }
        auto cleanup_images = makeScopeExit([&] {
            eglDestroyImageKHR(display, uv_image);
            eglDestroyImageKHR(display, y_image);
        });

        GLuint textures[2] = {0, 0};
        glGenTextures(2, textures);
        auto cleanup_textures = makeScopeExit([&] { glDeleteTextures(2, textures); });

        glBindTexture(GL_TEXTURE_2D, textures[0]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, y_image);
        ensureNoGlError("glEGLImageTargetTexture2DOES(Y)");

        glBindTexture(GL_TEXTURE_2D, textures[1]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, uv_image);
        ensureNoGlError("glEGLImageTargetTexture2DOES(UV)");

        const GLuint program = createProgram();
        auto cleanup_program = makeScopeExit([&] {
            if (program != 0) {
                glDeleteProgram(program);
            }
        });

        const std::array<GLfloat, 16> vertices = {
            -1.0f, -1.0f, 0.0f, 0.0f,
             1.0f, -1.0f, 1.0f, 0.0f,
            -1.0f,  1.0f, 0.0f, 1.0f,
             1.0f,  1.0f, 1.0f, 1.0f,
        };
        GLuint vbo = 0;
        glGenBuffers(1, &vbo);
        auto cleanup_vbo = makeScopeExit([&] {
            if (vbo != 0) {
                glDeleteBuffers(1, &vbo);
            }
        });
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<std::ptrdiff_t>(vertices.size() * sizeof(GLfloat)), vertices.data(), GL_STATIC_DRAW);

        glViewport(0, 0, 16, 16);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(program);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, textures[0]);
        glUniform1i(glGetUniformLocation(program, "uYTexture"), 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, textures[1]);
        glUniform1i(glGetUniformLocation(program, "uUVTexture"), 1);
        glVertexAttribPointer(0, 2, GL_FLOAT, static_cast<GLboolean>(GL_FALSE), 4 * sizeof(GLfloat), reinterpret_cast<const void*>(0));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 2, GL_FLOAT, static_cast<GLboolean>(GL_FALSE), 4 * sizeof(GLfloat), reinterpret_cast<const void*>(2 * sizeof(GLfloat)));
        glEnableVertexAttribArray(1);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        ensureNoGlError("glDrawArrays");
        glFinish();
        glReadPixels(8, 8, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result.pixel.data());
        ensureNoGlError("glReadPixels");
        return result;
    }

   private:
    template <typename T>
    static T loadRequired(const Library& library, const char* name) {
        void* symbol = library.symbol(name);
        if (!symbol) {
            fail(std::format("missing required symbol {}", name));
        }
        return reinterpret_cast<T>(symbol);
    }

    template <typename T>
    static T loadOptional(const Library& library, const char* name) {
        return reinterpret_cast<T>(library.symbol(name));
    }

    template <typename T>
    T loadRequiredProc(const char* name) {
        void* symbol = eglGetProcAddress(name);
        if (!symbol) {
            symbol = gles_.symbol(name);
        }
        if (!symbol) {
            symbol = egl_.symbol(name);
        }
        if (!symbol) {
            fail(std::format("missing required proc {}", name));
        }
        return reinterpret_cast<T>(symbol);
    }

    std::string glString(GLenum name) const {
        const GLubyte* string = glGetString(name);
        return string ? reinterpret_cast<const char*>(string) : "";
    }

    void ensureNoGlError(const char* stage) const {
        const GLenum error = glGetError();
        if (error != 0) {
            fail(std::format("{} failed with GL error 0x{:x}", stage, error));
        }
    }

    GLuint compileShader(GLenum type, const char* source) {
        const GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);
        GLint compiled = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if (!compiled) {
            std::array<GLchar, 1024> log{};
            GLsizei length = 0;
            glGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), &length, log.data());
            fail(std::format("shader compilation failed: {}", std::string(log.data(), log.data() + length)));
        }
        return shader;
    }

    GLuint createProgram() {
        static constexpr const char* kVertexShader = R"(
attribute vec2 aPosition;
attribute vec2 aTexCoord;
varying vec2 vTexCoord;
void main() {
    gl_Position = vec4(aPosition, 0.0, 1.0);
    vTexCoord = aTexCoord;
}
)";
        static constexpr const char* kFragmentShader = R"(
precision mediump float;
varying vec2 vTexCoord;
uniform sampler2D uYTexture;
uniform sampler2D uUVTexture;
void main() {
    float y = texture2D(uYTexture, vTexCoord).r;
    vec2 uv = texture2D(uUVTexture, vTexCoord).rg - vec2(0.5, 0.5);
    float r = y + 1.402 * uv.y;
    float g = y - 0.344136 * uv.x - 0.714136 * uv.y;
    float b = y + 1.772 * uv.x;
    gl_FragColor = vec4(r, g, b, 1.0);
}
)";

        const GLuint vertex_shader = compileShader(GL_VERTEX_SHADER, kVertexShader);
        const GLuint fragment_shader = compileShader(GL_FRAGMENT_SHADER, kFragmentShader);
        auto cleanup_shaders = makeScopeExit([&] {
            glDeleteShader(vertex_shader);
            glDeleteShader(fragment_shader);
        });

        const GLuint program = glCreateProgram();
        glAttachShader(program, vertex_shader);
        glAttachShader(program, fragment_shader);
        glBindAttribLocation(program, 0, "aPosition");
        glBindAttribLocation(program, 1, "aTexCoord");
        glLinkProgram(program);
        GLint linked = 0;
        glGetProgramiv(program, GL_LINK_STATUS, &linked);
        if (!linked) {
            std::array<GLchar, 1024> log{};
            GLsizei length = 0;
            glGetProgramInfoLog(program, static_cast<GLsizei>(log.size()), &length, log.data());
            fail(std::format("program link failed: {}", std::string(log.data(), log.data() + length)));
        }
        return program;
    }

    Library egl_;
    Library gles_;
    PFNEGLGETDISPLAYPROC eglGetDisplay = nullptr;
    PFNEGLINITIALIZEPROC eglInitialize = nullptr;
    PFNEGLTERMINATEPROC eglTerminate = nullptr;
    PFNEGLBINDAPIPROC eglBindAPI = nullptr;
    PFNEGLQUERYSTRINGPROC eglQueryString = nullptr;
    PFNEGLCHOOSECONFIGPROC eglChooseConfig = nullptr;
    PFNEGLCREATEPBUFFERSURFACEPROC eglCreatePbufferSurface = nullptr;
    PFNEGLDESTROYSURFACEPROC eglDestroySurface = nullptr;
    PFNEGLCREATECONTEXTPROC eglCreateContext = nullptr;
    PFNEGLDESTROYCONTEXTPROC eglDestroyContext = nullptr;
    PFNEGLMAKECURRENTPROC eglMakeCurrent = nullptr;
    PFNEGLGETERRORPROC eglGetError = nullptr;
    PFNEGLGETPROCADDRESSPROC eglGetProcAddress = nullptr;
    PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR = nullptr;
    PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR = nullptr;
    PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT = nullptr;
    PFNGLGETSTRINGPROC glGetString = nullptr;
    PFNGLGENTEXTURESPROC glGenTextures = nullptr;
    PFNGLBINDTEXTUREPROC glBindTexture = nullptr;
    PFNGLTEXPARAMETERIPROC glTexParameteri = nullptr;
    PFNGLCREATESHADERPROC glCreateShader = nullptr;
    PFNGLSHADERSOURCEPROC glShaderSource = nullptr;
    PFNGLCOMPILESHADERPROC glCompileShader = nullptr;
    PFNGLGETSHADERIVPROC glGetShaderiv = nullptr;
    PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLog = nullptr;
    PFNGLDELETESHADERPROC glDeleteShader = nullptr;
    PFNGLCREATEPROGRAMPROC glCreateProgram = nullptr;
    PFNGLATTACHSHADERPROC glAttachShader = nullptr;
    PFNGLBINDATTRIBLOCATIONPROC glBindAttribLocation = nullptr;
    PFNGLLINKPROGRAMPROC glLinkProgram = nullptr;
    PFNGLGETPROGRAMIVPROC glGetProgramiv = nullptr;
    PFNGLGETPROGRAMINFOLOGPROC glGetProgramInfoLog = nullptr;
    PFNGLDELETEPROGRAMPROC glDeleteProgram = nullptr;
    PFNGLUSEPROGRAMPROC glUseProgram = nullptr;
    PFNGLACTIVETEXTUREPROC glActiveTexture = nullptr;
    PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation = nullptr;
    PFNGLUNIFORM1IPROC glUniform1i = nullptr;
    PFNGLGENBUFFERSPROC glGenBuffers = nullptr;
    PFNGLBINDBUFFERPROC glBindBuffer = nullptr;
    PFNGLBUFFERDATAPROC glBufferData = nullptr;
    PFNGLVERTEXATTRIBPOINTERPROC glVertexAttribPointer = nullptr;
    PFNGLENABLEVERTEXATTRIBARRAYPROC glEnableVertexAttribArray = nullptr;
    PFNGLVIEWPORTPROC glViewport = nullptr;
    PFNGLCLEARCOLORPROC glClearColor = nullptr;
    PFNGLCLEARPROC glClear = nullptr;
    PFNGLDRAWARRAYSPROC glDrawArrays = nullptr;
    PFNGLREADPIXELSPROC glReadPixels = nullptr;
    PFNGLFINISHPROC glFinish = nullptr;
    PFNGLGETERRORPROC glGetError = nullptr;
    PFNGLDELETEBUFFERSPROC glDeleteBuffers = nullptr;
    PFNGLDELETETEXTURESPROC glDeleteTextures = nullptr;
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES = nullptr;
};

}  // namespace egl_runtime

}  // namespace

int main(int argc, char** argv) {
    const Options options = parseArgs(argc, argv);
    DrmDevice device = resolvePanthorPrimaryDevice(options);

    std::cout << std::format("[Panthor EGL Test] Allocation Device: path={}, driver={}, fd={}\n",
                             device.path,
                             device.driver_name.empty() ? "unknown" : device.driver_name,
                             device.fd.get());

    DumbBuffer shared_buffer = createDumbBuffer(device.fd.get(), options.width, options.height + options.height / 2, 8, DRM_FORMAT_R8);
    auto cleanup_buffer = makeScopeExit([&] { destroyDumbBuffer(device.fd.get(), shared_buffer); });
    fillNv12LikeBuffer(shared_buffer, static_cast<uint32_t>(options.width), static_cast<uint32_t>(options.height));

    const Nv12Planes planes = buildPlanes(device, shared_buffer, static_cast<uint32_t>(options.width), static_cast<uint32_t>(options.height));
    std::cout << std::format("[Panthor EGL Test] DMA-BUF Object: size={}, pitch={}, fd={}\n",
                             shared_buffer.size,
                             shared_buffer.pitch,
                             planes.y.fd);
    std::cout << std::format("[Panthor EGL Test] Plane Y: format={}, {}x{}, pitch={}, offset={}, fd={}\n",
                             drmFormatName(planes.y.drm_format),
                             planes.y.width,
                             planes.y.height,
                             planes.y.pitch,
                             planes.y.offset,
                             planes.y.fd);
    std::cout << std::format("[Panthor EGL Test] Plane UV: format={}, {}x{}, pitch={}, offset={}, fd={}\n",
                             drmFormatName(planes.uv.drm_format),
                             planes.uv.width,
                             planes.uv.height,
                             planes.uv.pitch,
                             planes.uv.offset,
                             planes.uv.fd);
    printSyntheticMetadata(shared_buffer, planes);

    std::cout << "[Panthor EGL Test] EGL Import: running in isolated subprocess\n";
    const pid_t child = fork();
    if (child < 0) {
        fail("fork failed for EGL import probe");
    }
    if (child == 0) {
        egl_runtime::Runtime runtime;
        auto result = runtime.importAndDraw(planes);
        std::cout << std::format("[Panthor EGL Test] EGL Import: success, renderer={}, version={}\n",
                                 result.gl_renderer,
                                 result.gl_version);
        std::cout << std::format("[Panthor EGL Test] GLES Draw: sampled pixel=#{:02x}{:02x}{:02x}{:02x}\n",
                                 result.pixel[0],
                                 result.pixel[1],
                                 result.pixel[2],
                                 result.pixel[3]);
        std::fflush(stdout);
        std::_Exit(0);
    }

    int child_status = 0;
    if (waitpid(child, &child_status, 0) < 0) {
        fail("waitpid failed for EGL import probe");
    }

    if (WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0) {
        std::cout << "[Panthor EGL Test] Verdict: PASS\n";
        return 0;
    }
    if (WIFSIGNALED(child_status)) {
        std::cout << std::format("[Panthor EGL Test] Verdict: FAIL (child crashed with signal {})\n",
                                 WTERMSIG(child_status));
        return 2;
    }
    if (WIFEXITED(child_status)) {
        std::cout << std::format("[Panthor EGL Test] Verdict: FAIL (child exited with status {})\n",
                                 WEXITSTATUS(child_status));
        return 2;
    }

    std::cout << "[Panthor EGL Test] Verdict: FAIL (child ended unexpectedly)\n";
    return 2;
}