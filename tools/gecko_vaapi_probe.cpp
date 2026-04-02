extern "C" {
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>

#include <xf86drm.h>
#include <drm_fourcc.h>
}

#include <sys/mman.h>
#include <sys/wait.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <format>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

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

struct Options {
    std::string drm_device;
    std::string input_path;
    std::string export_type = "prime2";
    std::string format = "nv12";
    int width = 1920;
    int height = 1080;
    unsigned int requested_surface_id = 0;
    bool import_egl = false;
};

struct DrmDevice {
    UniqueFd fd;
    std::string path;
    std::string driver_name;
};

struct VaScope {
    VADisplay display = nullptr;
    VAConfigID config = VA_INVALID_ID;
    VAContextID context = VA_INVALID_ID;
    std::vector<VASurfaceID> surfaces;

    ~VaScope() {
        if (display) {
            if (context != VA_INVALID_ID) {
                vaDestroyContext(display, context);
            }
            if (!surfaces.empty()) {
                vaDestroySurfaces(display, surfaces.data(), static_cast<int>(surfaces.size()));
            }
            if (config != VA_INVALID_ID) {
                vaDestroyConfig(display, config);
            }
            vaTerminate(display);
        }
    }
};

[[noreturn]] void fail(std::string_view message);
[[noreturn]] void fail_va(std::string_view stage, VAStatus status);
bool containsInsensitive(std::string_view haystack, std::string_view needle);

struct InputStreamInfo {
    std::string codec;
    int width = 0;
    int height = 0;
};

class CommandPipe {
   public:
    CommandPipe() = default;
    explicit CommandPipe(const std::string& command) : pipe_(popen(command.c_str(), "r")) {}

    ~CommandPipe() {
        close();
    }

    CommandPipe(const CommandPipe&) = delete;
    CommandPipe& operator=(const CommandPipe&) = delete;

    CommandPipe(CommandPipe&& other) noexcept : pipe_(other.pipe_) {
        other.pipe_ = nullptr;
    }

    CommandPipe& operator=(CommandPipe&& other) noexcept {
        if (this != &other) {
            close();
            pipe_ = other.pipe_;
            other.pipe_ = nullptr;
        }
        return *this;
    }

    bool valid() const {
        return pipe_ != nullptr;
    }

    size_t read(void* buffer, size_t size) {
        return pipe_ ? std::fread(buffer, 1, size, pipe_) : 0;
    }

   private:
    void close() {
        if (pipe_) {
            pclose(pipe_);
            pipe_ = nullptr;
        }
    }

    FILE* pipe_ = nullptr;
};

uint32_t chooseExpectedLayer0Format(const Options& options);
uint32_t chooseExpectedLayer1Format(const Options& options);

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

    void release() {
        active_ = false;
    }

   private:
    F func_;
    bool active_ = true;
};

template <typename F>
ScopeExit<F> makeScopeExit(F&& func) {
    return ScopeExit<F>(std::forward<F>(func));
}

namespace egl_runtime {

[[noreturn]] inline void runtimeFail(std::string_view message) {
    fail(message);
}

[[noreturn]] inline void runtimeFailFormatted(const std::string& message) {
    fail(message);
}

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

constexpr EGLBoolean EGL_FALSE = 0;
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
constexpr EGLint EGL_DMA_BUF_PLANE1_FD_EXT = 0x3275;
constexpr EGLint EGL_DMA_BUF_PLANE1_OFFSET_EXT = 0x3276;
constexpr EGLint EGL_DMA_BUF_PLANE1_PITCH_EXT = 0x3277;
constexpr EGLint EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT = 0x3443;
constexpr EGLint EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT = 0x3444;
constexpr EGLint EGL_DMA_BUF_PLANE1_MODIFIER_LO_EXT = 0x3445;
constexpr EGLint EGL_DMA_BUF_PLANE1_MODIFIER_HI_EXT = 0x3446;
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
constexpr GLenum GL_TEXTURE_EXTERNAL_OES = 0x8D65;
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

struct ImportResult {
    std::array<uint8_t, 4> pixel = {0, 0, 0, 0};
    std::string egl_vendor;
    std::string egl_extensions;
    std::string gl_vendor;
    std::string gl_renderer;
    std::string gl_version;
    std::string gl_extensions;
};

class Library {
   public:
    Library() = default;

    explicit Library(const char* path) {
        handle_ = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    }

    ~Library() {
        if (handle_) {
            dlclose(handle_);
        }
    }

    Library(const Library&) = delete;
    Library& operator=(const Library&) = delete;

    Library(Library&& other) noexcept : handle_(other.handle_) {
        other.handle_ = nullptr;
    }

    Library& operator=(Library&& other) noexcept {
        if (this != &other) {
            if (handle_) {
                dlclose(handle_);
            }
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    bool loaded() const {
        return handle_ != nullptr;
    }

    void* symbol(const char* name) const {
        return handle_ ? dlsym(handle_, name) : nullptr;
    }

   private:
    void* handle_ = nullptr;
};

class Runtime {
   public:
    Runtime() : egl_("/lib/aarch64-linux-gnu/libEGL.so.1"), gles_("/lib/aarch64-linux-gnu/libGLESv2.so.2") {
        if (!egl_.loaded() || !gles_.loaded()) {
            runtimeFail("failed to load libEGL.so.1 or libGLESv2.so.2");
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

    ImportResult importAndDraw(const Options& options, const VADRMPRIMESurfaceDescriptor& desc) {
        std::cout << "[Gecko Probe] EGL Step: load default display\n";
        EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (display == EGL_NO_DISPLAY && eglGetPlatformDisplayEXT) {
            std::cout << "[Gecko Probe] EGL Step: fallback to surfaceless display\n";
            display = eglGetPlatformDisplayEXT(EGL_PLATFORM_SURFACELESS_MESA, nullptr, nullptr);
        }
        if (display == EGL_NO_DISPLAY) {
            runtimeFailFormatted(std::format("eglGetDisplay failed: 0x{:x}", eglGetError()));
        }

        std::cout << "[Gecko Probe] EGL Step: initialize display\n";
        EGLint major = 0;
        EGLint minor = 0;
        if (eglInitialize(display, &major, &minor) != EGL_TRUE) {
            runtimeFailFormatted(std::format("eglInitialize failed: 0x{:x}", eglGetError()));
        }

        auto cleanup_display = makeScopeExit([&] {
            eglTerminate(display);
        });

        ImportResult result;
        if (const char* egl_extensions = eglQueryString(display, EGL_EXTENSIONS)) {
            result.egl_extensions = egl_extensions;
        }
        std::cout << std::format("[Gecko Probe] EGL Step: got {} bytes of EGL extensions\n", result.egl_extensions.size());
        if (!containsInsensitive(result.egl_extensions, "EGL_EXT_image_dma_buf_import")) {
            runtimeFail("EGL does not advertise EGL_EXT_image_dma_buf_import");
        }

        std::cout << "[Gecko Probe] EGL Step: bind GLES API\n";
        if (eglBindAPI(EGL_OPENGL_ES_API) != EGL_TRUE) {
            runtimeFailFormatted(std::format("eglBindAPI failed: 0x{:x}", eglGetError()));
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
        std::cout << "[Gecko Probe] EGL Step: choose config\n";
        if (eglChooseConfig(display, config_attribs, &config, 1, &num_configs) != EGL_TRUE || num_configs != 1) {
            runtimeFailFormatted(std::format("eglChooseConfig failed: 0x{:x}", eglGetError()));
        }

        const EGLint pbuffer_attribs[] = {
            EGL_WIDTH, 16,
            EGL_HEIGHT, 16,
            EGL_NONE,
        };
        std::cout << "[Gecko Probe] EGL Step: create pbuffer\n";
        EGLSurface surface = eglCreatePbufferSurface(display, config, pbuffer_attribs);
        if (surface == EGL_NO_SURFACE) {
            runtimeFailFormatted(std::format("eglCreatePbufferSurface failed: 0x{:x}", eglGetError()));
        }
        auto cleanup_surface = makeScopeExit([&] {
            eglDestroySurface(display, surface);
        });

        const EGLint context_attribs[] = {
            EGL_CONTEXT_CLIENT_VERSION, 2,
            EGL_NONE,
        };
        std::cout << "[Gecko Probe] EGL Step: create context\n";
        EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, context_attribs);
        if (context == EGL_NO_CONTEXT) {
            runtimeFailFormatted(std::format("eglCreateContext failed: 0x{:x}", eglGetError()));
        }
        auto cleanup_context = makeScopeExit([&] {
            eglDestroyContext(display, context);
        });

        std::cout << "[Gecko Probe] EGL Step: make current\n";
        if (eglMakeCurrent(display, surface, surface, context) != EGL_TRUE) {
            runtimeFailFormatted(std::format("eglMakeCurrent failed: 0x{:x}", eglGetError()));
        }

        std::cout << "[Gecko Probe] EGL Step: resolve procs\n";
        eglCreateImageKHR = loadRequiredProc<PFNEGLCREATEIMAGEKHRPROC>("eglCreateImageKHR");
        eglDestroyImageKHR = loadRequiredProc<PFNEGLDESTROYIMAGEKHRPROC>("eglDestroyImageKHR");
        glEGLImageTargetTexture2DOES = loadRequiredProc<PFNGLEGLIMAGETARGETTEXTURE2DOESPROC>("glEGLImageTargetTexture2DOES");

        std::cout << "[Gecko Probe] EGL Step: query GL strings\n";
        result.egl_vendor = extensionString(display);
        result.gl_vendor = glString(GL_VENDOR);
        result.gl_renderer = glString(GL_RENDERER);
        result.gl_version = glString(GL_VERSION);
        result.gl_extensions = glString(GL_EXTENSIONS);

        if (!containsInsensitive(result.gl_extensions, "GL_OES_EGL_image")) {
            runtimeFail("GL does not advertise GL_OES_EGL_image");
        }

        const auto create_layer_image = [&](uint32_t layer_index, EGLint width, EGLint height) {
            const auto& layer = desc.layers[layer_index];
            const int object_index = layer.object_index[0];
            const uint64_t modifier = desc.objects[object_index].drm_format_modifier;
            const EGLint image_attribs[] = {
                EGL_WIDTH, width,
                EGL_HEIGHT, height,
                EGL_LINUX_DRM_FOURCC_EXT, static_cast<EGLint>(layer.drm_format),
                EGL_DMA_BUF_PLANE0_FD_EXT, desc.objects[object_index].fd,
                EGL_DMA_BUF_PLANE0_OFFSET_EXT, static_cast<EGLint>(layer.offset[0]),
                EGL_DMA_BUF_PLANE0_PITCH_EXT, static_cast<EGLint>(layer.pitch[0]),
                EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT, static_cast<EGLint>(modifier & 0xffffffffu),
                EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT, static_cast<EGLint>(modifier >> 32),
                EGL_NONE,
            };
            return eglCreateImageKHR(display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, image_attribs);
        };

        const auto create_combined_image = [&]() {
            const auto& layer = desc.layers[0];
            const uint64_t modifier0 = desc.objects[layer.object_index[0]].drm_format_modifier;
            const uint64_t modifier1 = desc.objects[layer.object_index[1]].drm_format_modifier;
            const EGLint image_attribs[] = {
                EGL_WIDTH, static_cast<EGLint>(desc.width),
                EGL_HEIGHT, static_cast<EGLint>(desc.height),
                EGL_LINUX_DRM_FOURCC_EXT, static_cast<EGLint>(layer.drm_format),
                EGL_DMA_BUF_PLANE0_FD_EXT, desc.objects[layer.object_index[0]].fd,
                EGL_DMA_BUF_PLANE0_OFFSET_EXT, static_cast<EGLint>(layer.offset[0]),
                EGL_DMA_BUF_PLANE0_PITCH_EXT, static_cast<EGLint>(layer.pitch[0]),
                EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT, static_cast<EGLint>(modifier0 & 0xffffffffu),
                EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT, static_cast<EGLint>(modifier0 >> 32),
                EGL_DMA_BUF_PLANE1_FD_EXT, desc.objects[layer.object_index[1]].fd,
                EGL_DMA_BUF_PLANE1_OFFSET_EXT, static_cast<EGLint>(layer.offset[1]),
                EGL_DMA_BUF_PLANE1_PITCH_EXT, static_cast<EGLint>(layer.pitch[1]),
                EGL_DMA_BUF_PLANE1_MODIFIER_LO_EXT, static_cast<EGLint>(modifier1 & 0xffffffffu),
                EGL_DMA_BUF_PLANE1_MODIFIER_HI_EXT, static_cast<EGLint>(modifier1 >> 32),
                EGL_NONE,
            };
            return eglCreateImageKHR(display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, image_attribs);
        };

        if (options.export_type != "prime2") {
            std::cout << "[Gecko Probe] EGL Step: create combined PRIME_1 EGLImage\n";
            EGLImageKHR combined_image = create_combined_image();
            if (combined_image == EGL_NO_IMAGE_KHR) {
                runtimeFailFormatted(std::format("eglCreateImageKHR(PRIME_1 combined) failed: 0x{:x}", eglGetError()));
            }
            eglDestroyImageKHR(display, combined_image);
            return result;
        }

        std::cout << "[Gecko Probe] EGL Step: create layer EGLImages\n";
        EGLImageKHR y_image = create_layer_image(0, static_cast<EGLint>(desc.width), static_cast<EGLint>(desc.height));
        if (y_image == EGL_NO_IMAGE_KHR) {
            runtimeFailFormatted(std::format("eglCreateImageKHR(Y) failed: 0x{:x}", eglGetError()));
        }
        EGLImageKHR uv_image = create_layer_image(1,
                                                  static_cast<EGLint>(std::max<uint32_t>(1, desc.width / 2)),
                                                  static_cast<EGLint>(std::max<uint32_t>(1, desc.height / 2)));
        if (uv_image == EGL_NO_IMAGE_KHR) {
            eglDestroyImageKHR(display, y_image);
            runtimeFailFormatted(std::format("eglCreateImageKHR(UV) failed: 0x{:x}", eglGetError()));
        }
        auto cleanup_images = makeScopeExit([&] {
            eglDestroyImageKHR(display, uv_image);
            eglDestroyImageKHR(display, y_image);
        });

        GLuint textures[2] = {0, 0};
        std::cout << "[Gecko Probe] EGL Step: bind layer textures\n";
        glGenTextures(2, textures);
        auto cleanup_textures = makeScopeExit([&] {
            glDeleteTextures(2, textures);
        });

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

        std::cout << "[Gecko Probe] EGL Step: create program\n";
        const GLuint program = createProgram();
        auto cleanup_program = makeScopeExit([&] {
            if (program != 0) glDeleteProgram(program);
        });

        const std::array<GLfloat, 16> vertices = {
            -1.0f, -1.0f, 0.0f, 0.0f,
             1.0f, -1.0f, 1.0f, 0.0f,
            -1.0f,  1.0f, 0.0f, 1.0f,
             1.0f,  1.0f, 1.0f, 1.0f,
        };
        GLuint vbo = 0;
        std::cout << "[Gecko Probe] EGL Step: draw\n";
        glGenBuffers(1, &vbo);
        auto cleanup_vbo = makeScopeExit([&] {
            if (vbo != 0) glDeleteBuffers(1, &vbo);
        });
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<std::ptrdiff_t>(vertices.size() * sizeof(GLfloat)), vertices.data(), GL_STATIC_DRAW);

        glViewport(0, 0, 16, 16);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(program);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, textures[0]);
        const GLint y_sampler_location = glGetUniformLocation(program, "uYTexture");
        glUniform1i(y_sampler_location, 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, textures[1]);
        const GLint uv_sampler_location = glGetUniformLocation(program, "uUVTexture");
        glUniform1i(uv_sampler_location, 1);
        glVertexAttribPointer(0, 2, GL_FLOAT, static_cast<GLboolean>(GL_FALSE), 4 * sizeof(GLfloat), reinterpret_cast<const void*>(0));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 2, GL_FLOAT, static_cast<GLboolean>(GL_FALSE), 4 * sizeof(GLfloat), reinterpret_cast<const void*>(2 * sizeof(GLfloat)));
        glEnableVertexAttribArray(1);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        ensureNoGlError("glDrawArrays");
        glFinish();
        std::cout << "[Gecko Probe] EGL Step: read pixels\n";
        glReadPixels(8, 8, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result.pixel.data());
        ensureNoGlError("glReadPixels");
        return result;
    }

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

   private:
    template <typename T>
    static T loadRequired(const Library& library, const char* name) {
        void* symbol = library.symbol(name);
        if (!symbol) {
            runtimeFailFormatted(std::format("missing required symbol {}", name));
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
            runtimeFailFormatted(std::format("missing required proc {}", name));
        }
        return reinterpret_cast<T>(symbol);
    }

    std::string extensionString(EGLDisplay display) const {
        const char* string = eglQueryString(display, EGL_EXTENSIONS);
        return string ? string : "";
    }

    std::string glString(GLenum name) const {
        const GLubyte* string = glGetString(name);
        return string ? reinterpret_cast<const char*>(string) : "";
    }

    void ensureNoGlError(const char* stage) const {
        const GLenum error = glGetError();
        if (error != 0) {
            runtimeFailFormatted(std::format("{} failed with GL error 0x{:x}", stage, error));
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
            runtimeFailFormatted(std::format("shader compilation failed: {}", std::string(log.data(), log.data() + length)));
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
            runtimeFailFormatted(std::format("program link failed: {}", std::string(log.data(), log.data() + length)));
        }
        return program;
    }

    Library egl_;
    Library gles_;
};

}  // namespace egl_runtime

[[noreturn]] void fail(std::string_view message) {
    std::cerr << std::format("[Gecko Probe] ERROR: {}\n", message);
    std::exit(1);
}

[[noreturn]] void fail_va(std::string_view stage, VAStatus status) {
    std::cerr << std::format("[Gecko Probe] ERROR: {} failed: {} ({})\n",
                             stage,
                             vaErrorStr(status),
                             status);
    std::exit(1);
}

bool containsInsensitive(std::string_view haystack, std::string_view needle) {
    auto to_lower = [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); };
    std::string lowered_haystack(haystack.begin(), haystack.end());
    std::string lowered_needle(needle.begin(), needle.end());
    std::ranges::transform(lowered_haystack, lowered_haystack.begin(), to_lower);
    std::ranges::transform(lowered_needle, lowered_needle.begin(), to_lower);
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

std::string vaFourccName(uint32_t fourcc) {
    switch (fourcc) {
        case VA_FOURCC_NV12:
            return "NV12";
        case VA_FOURCC_P010:
            return "P010";
        default:
            return fourccToString(fourcc);
    }
}

std::string drmFormatName(uint32_t format) {
    switch (format) {
        case DRM_FORMAT_R8:
            return "R8";
        case DRM_FORMAT_GR88:
            return "GR88";
        case DRM_FORMAT_R16:
            return "R16";
        case DRM_FORMAT_GR1616:
            return "GR1616";
        case DRM_FORMAT_NV12:
            return "NV12";
        case DRM_FORMAT_P010:
            return "P010";
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

std::string shellQuote(std::string_view input) {
    std::string quoted;
    quoted.reserve(input.size() + 2);
    quoted.push_back('\'');
    for (char ch : input) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted.push_back(ch);
        }
    }
    quoted.push_back('\'');
    return quoted;
}

InputStreamInfo probeInputStream(const std::string& input_path) {
    const std::string command = std::format(
        "ffprobe -v error -select_streams v:0 -show_entries stream=codec_name,width,height -of default=noprint_wrappers=1:nokey=1 {}",
        shellQuote(input_path));
    CommandPipe pipe(command);
    if (!pipe.valid()) {
        fail("failed to run ffprobe for input probing");
    }

    std::array<char, 256> buffer{};
    std::string output;
    while (const size_t bytes = pipe.read(buffer.data(), buffer.size())) {
        output.append(buffer.data(), bytes);
    }

    std::istringstream lines(output);
    InputStreamInfo info;
    if (!std::getline(lines, info.codec)) {
        fail("ffprobe did not return a codec name");
    }
    std::string width_text;
    std::string height_text;
    if (!std::getline(lines, width_text) || !std::getline(lines, height_text)) {
        fail("ffprobe did not return width and height");
    }

    try {
        info.width = std::stoi(width_text);
        info.height = std::stoi(height_text);
    } catch (...) {
        fail("failed to parse ffprobe dimensions");
    }

    if (info.width <= 0 || info.height <= 0) {
        fail("ffprobe returned invalid dimensions");
    }
    return info;
}

std::vector<uint8_t> readDecodeChunk(const std::string& input_path) {
    constexpr size_t kChunkSize = 1024 * 1024;
    const std::string command = std::format(
        "ffmpeg -hide_banner -loglevel error -i {} -vcodec copy -an -f h264 pipe:1 2>/dev/null",
        shellQuote(input_path));
    CommandPipe pipe(command);
    if (!pipe.valid()) {
        fail("failed to run ffmpeg for decode input");
    }

    std::vector<uint8_t> payload(kChunkSize);
    const size_t bytes = pipe.read(payload.data(), payload.size());
    if (bytes == 0) {
        fail("ffmpeg did not produce any H.264 bitstream bytes");
    }
    payload.resize(bytes);
    return payload;
}

void submitDecodeChunk(VADisplay display,
                       VAContextID context,
                       VASurfaceID surface,
                       const std::vector<uint8_t>& payload) {
    VABufferID buffer = VA_INVALID_ID;
    VAStatus status = vaCreateBuffer(display,
                                     context,
                                     VASliceDataBufferType,
                                     static_cast<unsigned int>(payload.size()),
                                     1,
                                     const_cast<uint8_t*>(payload.data()),
                                     &buffer);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaCreateBuffer", status);
    }

    auto cleanup = makeScopeExit([&] {
        if (buffer != VA_INVALID_ID) {
            vaDestroyBuffer(display, buffer);
        }
    });

    status = vaBeginPicture(display, context, surface);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaBeginPicture", status);
    }

    status = vaRenderPicture(display, context, &buffer, 1);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaRenderPicture", status);
    }

    status = vaEndPicture(display, context);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaEndPicture", status);
    }
}

VAStatus syncSurfaceWithRetry(VADisplay display, VASurfaceID surface) {
    constexpr int kMaxRetries = 50;
    for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
        const VAStatus status = vaSyncSurface(display, surface);
        if (status == VA_STATUS_SUCCESS) {
            return status;
        }
        if (status != VA_STATUS_ERROR_SURFACE_BUSY && status != VA_STATUS_ERROR_TIMEDOUT) {
            return status;
        }
        usleep(5000);
    }
    return VA_STATUS_ERROR_TIMEDOUT;
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

void printDescriptorMetadata(const VADRMPRIMESurfaceDescriptor& desc) {
    std::cout << "[Gecko Meta] begin descriptor\n";
    std::cout << std::format("[Gecko Meta] frame: width={} height={} fourcc={} objects={} layers={} unique_hint={}\n",
                             desc.width,
                             desc.height,
                             vaFourccName(desc.fourcc),
                             desc.num_objects,
                             desc.num_layers,
                             desc.num_layers == 0 ? 0u : desc.layers[0].object_index[0] == desc.layers[std::min<uint32_t>(1, desc.num_layers - 1)].object_index[0]);
    for (uint32_t object_index = 0; object_index < desc.num_objects; ++object_index) {
        const auto& object = desc.objects[object_index];
        std::cout << std::format("[Gecko Meta] object[{}]: fd={} size={} modifier=0x{:x}\n",
                                 object_index,
                                 object.fd,
                                 object.size,
                                 object.drm_format_modifier);
        printFdInfo("[Gecko Meta]", object.fd);
    }
    for (uint32_t layer_index = 0; layer_index < desc.num_layers; ++layer_index) {
        const auto& layer = desc.layers[layer_index];
        std::cout << std::format("[Gecko Meta] layer[{}]: drm_format={} num_planes={} pitch0={} offset0={} object0={} pitch1={} offset1={} object1={}\n",
                                 layer_index,
                                 drmFormatName(layer.drm_format),
                                 layer.num_planes,
                                 layer.pitch[0],
                                 layer.offset[0],
                                 layer.object_index[0],
                                 layer.pitch[1],
                                 layer.offset[1],
                                 layer.object_index[1]);
    }
    for (uint32_t layer_index = 0; layer_index < desc.num_layers; ++layer_index) {
        const auto& layer = desc.layers[layer_index];
        for (uint32_t plane_index = 0; plane_index < layer.num_planes; ++plane_index) {
            const uint32_t object_index = layer.object_index[plane_index];
            const int fd = object_index < desc.num_objects ? desc.objects[object_index].fd : -1;
            std::cout << std::format("[Gecko Meta] plane[{}:{}]: object={} fd={} offset={} pitch={} modifier=0x{:x}\n",
                                     layer_index,
                                     plane_index,
                                     object_index,
                                     fd,
                                     layer.offset[plane_index],
                                     layer.pitch[plane_index],
                                     object_index < desc.num_objects ? desc.objects[object_index].drm_format_modifier : 0ull);
        }
    }
    std::cout << "[Gecko Meta] end descriptor\n";
}

void printUsage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [options]\n"
              << "  --drm-device <path>      DRM render node to use\n"
              << "  --input <file>           Decode one real H.264 frame before export\n"
              << "  --width <pixels>         Surface width (default 1920)\n"
              << "  --height <pixels>        Surface height (default 1080)\n"
              << "  --format <nv12|p010>     Surface pixel format (default nv12)\n"
              << "  --export-type <prime1|prime2> Export memory type (default prime2)\n"
              << "  --surface-id <id>        Accepted for Gecko-like CLI parity; probe allocates its own surface\n"
              << "  --import-egl             Import exported DMA-BUF into EGLImage and draw with GLES\n"
              << "  --help                   Show this message\n";
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
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--drm-device") {
            options.drm_device = require_value(arg);
            continue;
        }
        if (arg == "--input") {
            options.input_path = require_value(arg);
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
        if (arg == "--format") {
            options.format = require_value(arg);
            continue;
        }
        if (arg == "--export-type") {
            options.export_type = require_value(arg);
            continue;
        }
        if (arg == "--surface-id") {
            options.requested_surface_id = static_cast<unsigned int>(std::stoul(require_value(arg)));
            continue;
        }
        if (arg == "--import-egl") {
            options.import_egl = true;
            continue;
        }

        fail(std::format("unknown argument {}", arg));
    }

    if (options.width <= 0 || options.height <= 0) {
        fail("width and height must be positive");
    }
    if (options.export_type != "prime2" && options.export_type != "prime1" && options.export_type != "prime") {
        fail("--export-type must be prime1 or prime2");
    }
    if (options.format != "nv12" && options.format != "p010") {
        fail("--format must be nv12 or p010");
    }
    if (!options.input_path.empty() && options.format != "nv12") {
        fail("--input currently supports only nv12 / H.264 decode validation");
    }
    return options;
}

uint32_t chooseRequestedMemType(const Options& options) {
    return options.export_type == "prime2" ? VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2
                                            : VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
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

DrmDevice resolveDrmDevice(const Options& options) {
    if (!options.drm_device.empty()) {
        auto device = openSpecificDevice(options.drm_device);
        if (!device) {
            fail(std::format("failed to open DRM device {}", options.drm_device));
        }
        return std::move(*device);
    }

    if (const char* env_path = std::getenv("LIBVA_DRM_DEVICE")) {
        auto device = openSpecificDevice(env_path);
        if (device) {
            return std::move(*device);
        }
    }

    drmDevicePtr raw_devices[64] = {};
    const int count = drmGetDevices2(0, raw_devices, static_cast<int>(std::size(raw_devices)));
    if (count < 0) {
        fail("drmGetDevices2 failed while resolving render node");
    }

    std::optional<DrmDevice> fallback;
    for (int index = 0; index < count; ++index) {
        drmDevicePtr info = raw_devices[index];
        if (!info) {
            continue;
        }
        if ((info->available_nodes & (1 << DRM_NODE_RENDER)) == 0 || !info->nodes[DRM_NODE_RENDER]) {
            continue;
        }

        auto opened = openSpecificDevice(info->nodes[DRM_NODE_RENDER]);
        if (!opened) {
            continue;
        }

        if (containsInsensitive(opened->driver_name, "rockchip") ||
            containsInsensitive(opened->driver_name, "panthor")) {
            drmFreeDevices(raw_devices, count);
            return std::move(*opened);
        }
        if (!fallback) {
            fallback = std::move(*opened);
        }
    }

    drmFreeDevices(raw_devices, count);
    if (fallback) {
        return std::move(*fallback);
    }
    fail("no DRM render node was found");
}

uint32_t chooseRtFormat(const Options& options) {
    return options.format == "p010" ? VA_RT_FORMAT_YUV420_10 : VA_RT_FORMAT_YUV420;
}

VAProfile chooseProfile(const Options& options) {
    return options.format == "p010" ? VAProfileHEVCMain10 : VAProfileH264High;
}

uint32_t chooseExpectedLayer0Format(const Options& options) {
    return options.format == "p010" ? DRM_FORMAT_R16 : DRM_FORMAT_R8;
}

uint32_t chooseExpectedLayer1Format(const Options& options) {
    return options.format == "p010" ? DRM_FORMAT_GR1616 : DRM_FORMAT_GR88;
}

bool queryPrimeSupport(VADisplay display, VAConfigID config, const Options& options) {
    unsigned int num_attribs = 0;
    VAStatus status = vaQuerySurfaceAttributes(display, config, nullptr, &num_attribs);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaQuerySurfaceAttributes(count)", status);
    }

    std::vector<VASurfaceAttrib> attribs(num_attribs);
    status = vaQuerySurfaceAttributes(display, config, attribs.data(), &num_attribs);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaQuerySurfaceAttributes(list)", status);
    }

    bool has_prime = false;
    bool has_prime2 = false;
    bool has_linear = false;
    for (const auto& attrib : attribs) {
        if (attrib.type == VASurfaceAttribMemoryType && attrib.value.type == VAGenericValueTypeInteger) {
            has_prime = (attrib.value.value.i & VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME) != 0;
            has_prime2 = (attrib.value.value.i & VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2) != 0;
        }
        if (attrib.type == VASurfaceAttribDRMFormatModifiers && attrib.value.type == VAGenericValueTypePointer && attrib.value.value.p) {
            auto* list = static_cast<VADRMFormatModifierList*>(attrib.value.value.p);
            for (uint32_t index = 0; index < list->num_modifiers; ++index) {
                if (list->modifiers[index] == DRM_FORMAT_MOD_LINEAR) {
                    has_linear = true;
                    break;
                }
            }
        }
    }

    std::cout << std::format("[Gecko Probe] Surface Attributes: DRM_PRIME={}, DRM_PRIME_2={}, LINEAR modifier={}\n",
                             has_prime ? "yes" : "no",
                             has_prime2 ? "yes" : "no",
                             has_linear ? "yes" : "no");
    return (chooseRequestedMemType(options) == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME ? has_prime : has_prime2) && has_linear;
}

void closeExportedDescriptorObjects(VADRMPRIMESurfaceDescriptor& desc) {
    for (uint32_t index = 0; index < desc.num_objects; ++index) {
        if (desc.objects[index].fd >= 0) {
            close(desc.objects[index].fd);
            desc.objects[index].fd = -1;
        }
    }
}

void validateDescriptor(const Options& options, const VADRMPRIMESurfaceDescriptor& desc) {
    const uint32_t expected_fourcc = options.format == "p010" ? VA_FOURCC_P010 : VA_FOURCC_NV12;
    if (desc.fourcc != expected_fourcc) {
        fail(std::format("unexpected exported fourcc {}, expected {}",
                         vaFourccName(desc.fourcc),
                         vaFourccName(expected_fourcc)));
    }
    if (desc.num_objects < 1 || desc.objects[0].fd < 0) {
        fail("export descriptor did not contain a valid object fd");
    }
    if (desc.objects[0].drm_format_modifier != DRM_FORMAT_MOD_LINEAR) {
        fail(std::format("unexpected modifier 0x{:x}, expected LINEAR", desc.objects[0].drm_format_modifier));
    }
    if (options.export_type == "prime2") {
        if (desc.num_layers != 2) {
            fail(std::format("unexpected layer count {}, expected 2", desc.num_layers));
        }
        if (desc.layers[0].num_planes != 1 || desc.layers[1].num_planes != 1) {
            fail("expected two separate single-plane layers");
        }
        if (desc.layers[0].drm_format != chooseExpectedLayer0Format(options)) {
            fail(std::format("unexpected layer 0 format {}",
                             drmFormatName(desc.layers[0].drm_format)));
        }
        if (desc.layers[1].drm_format != chooseExpectedLayer1Format(options)) {
            fail(std::format("unexpected layer 1 format {}",
                             drmFormatName(desc.layers[1].drm_format)));
        }
        if (desc.layers[0].pitch[0] == 0 || desc.layers[1].pitch[0] == 0) {
            fail("exported layers have zero pitch");
        }
        return;
    }

    if (desc.num_layers != 1) {
        fail(std::format("unexpected layer count {}, expected 1 for prime1", desc.num_layers));
    }
    if (desc.layers[0].num_planes != 2) {
        fail(std::format("unexpected plane count {}, expected 2 for prime1", desc.layers[0].num_planes));
    }
    const uint32_t expected_format = options.format == "p010" ? DRM_FORMAT_P010 : DRM_FORMAT_NV12;
    if (desc.layers[0].drm_format != expected_format) {
        fail(std::format("unexpected prime1 layer format {}",
                         drmFormatName(desc.layers[0].drm_format)));
    }
    if (desc.layers[0].pitch[0] == 0 || desc.layers[0].pitch[1] == 0) {
        fail("prime1 exported planes have zero pitch");
    }
}

void probeZeroCopyMap(const VADRMPRIMESurfaceDescriptor& desc) {
    void* mapping = mmap(nullptr, desc.objects[0].size, PROT_READ, MAP_SHARED, desc.objects[0].fd, 0);
    if (mapping == MAP_FAILED) {
        fail("failed to mmap exported dma-buf object");
    }
    munmap(mapping, desc.objects[0].size);
    std::cout << "[Gecko Probe] Zero-copy validation: exported DMA-BUF mapped directly, no memcpy involved\n";
}

void printCompatibilityReport(const Options& options,
                              const DrmDevice& drm_device,
                              const VADRMPRIMESurfaceDescriptor& desc,
                              bool egl_success,
                              std::string_view egl_failure) {
    std::cout << "[Gecko Report] Driver Compatibility Report\n";
    std::cout << std::format("[Gecko Report] DRM device: {} ({})\n",
                             drm_device.path,
                             drm_device.driver_name.empty() ? "unknown" : drm_device.driver_name);
    std::cout << "[Gecko Report] vaGetDisplayDRM: PASS\n";
    std::cout << "[Gecko Report] vaInitialize: PASS\n";
    std::cout << std::format("[Gecko Report] Surface memory type: PASS ({})\n",
                             options.export_type == "prime2" ? "DRM_PRIME_2" : "DRM_PRIME");
    std::cout << std::format("[Gecko Report] DRM modifier: PASS (0x{:x} = LINEAR)\n",
                             desc.objects[0].drm_format_modifier);
    std::cout << std::format("[Gecko Report] Surface fourcc: PASS ({})\n",
                             vaFourccName(desc.fourcc));
    std::cout << std::format("[Gecko Report] Layer layout: PASS ({} layers)\n", desc.num_layers);
    std::cout << std::format("[Gecko Report] Layer 0: format={}, offset={}, pitch={}\n",
                             drmFormatName(desc.layers[0].drm_format),
                             desc.layers[0].offset[0],
                             desc.layers[0].pitch[0]);
    if (desc.num_layers > 1) {
        std::cout << std::format("[Gecko Report] Layer 1: format={}, offset={}, pitch={}\n",
                                 drmFormatName(desc.layers[1].drm_format),
                                 desc.layers[1].offset[0],
                                 desc.layers[1].pitch[0]);
    } else if (desc.layers[0].num_planes > 1) {
        std::cout << std::format("[Gecko Report] Plane 1: offset={}, pitch={}\n",
                                 desc.layers[0].offset[1],
                                 desc.layers[0].pitch[1]);
    }
    std::cout << "[Gecko Report] Zero-copy CPU path: PASS (direct mmap of exported DMA-BUF)\n";
    if (options.import_egl) {
        if (egl_success) {
            std::cout << "[Gecko Report] EGL/GLES import: PASS (see child probe logs above)\n";
        } else {
            std::cout << std::format("[Gecko Report] EGL/GLES import: FAIL ({})\n", egl_failure);
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    const Options options = parseArgs(argc, argv);
    InputStreamInfo input_info;
    std::vector<uint8_t> decode_payload;
    int surface_width = options.width;
    int surface_height = options.height;
    if (!options.input_path.empty()) {
        input_info = probeInputStream(options.input_path);
        if (!containsInsensitive(input_info.codec, "h264") && !containsInsensitive(input_info.codec, "avc")) {
            fail(std::format("--input requires an H.264 stream, got {}", input_info.codec));
        }
        surface_width = input_info.width;
        surface_height = input_info.height;
        decode_payload = readDecodeChunk(options.input_path);
    }

    DrmDevice drm_device = resolveDrmDevice(options);

    std::cout << std::format("[Gecko Probe] DRM Device: path={}, driver={}, fd={}\n",
                             drm_device.path,
                             drm_device.driver_name.empty() ? "unknown" : drm_device.driver_name,
                             drm_device.fd.get());

    VADisplay display = vaGetDisplayDRM(drm_device.fd.get());
    if (!display) {
        fail("vaGetDisplayDRM returned null");
    }

    VaScope va;
    va.display = display;

    int major = 0;
    int minor = 0;
    VAStatus status = vaInitialize(display, &major, &minor);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaInitialize", status);
    }

    const char* vendor = vaQueryVendorString(display);
    std::cout << std::format("[Gecko Probe] VA Display: version={}.{}, vendor={}\n",
                             major,
                             minor,
                             vendor ? vendor : "unknown");

    VAConfigAttrib attrib{};
    attrib.type = VAConfigAttribRTFormat;
    attrib.value = chooseRtFormat(options);
    status = vaCreateConfig(display,
                            chooseProfile(options),
                            VAEntrypointVLD,
                            &attrib,
                            1,
                            &va.config);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaCreateConfig", status);
    }

    if (!queryPrimeSupport(display, va.config, options)) {
        fail(std::format("surface attributes do not advertise requested export type {} + LINEAR support", options.export_type));
    }

    va.surfaces.resize(1, VA_INVALID_ID);
    status = vaCreateSurfaces(display,
                              chooseRtFormat(options),
                              surface_width,
                              surface_height,
                              va.surfaces.data(),
                              static_cast<unsigned int>(va.surfaces.size()),
                              nullptr,
                              0);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaCreateSurfaces", status);
    }

    status = vaCreateContext(display,
                             va.config,
                             surface_width,
                             surface_height,
                             VA_PROGRESSIVE,
                             va.surfaces.data(),
                             static_cast<int>(va.surfaces.size()),
                             &va.context);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaCreateContext", status);
    }

    if (options.requested_surface_id != 0 && options.requested_surface_id != va.surfaces[0]) {
        std::cout << std::format("[Gecko Probe] Requested surface-id={} but VA allocated surface={}\n",
                                 options.requested_surface_id,
                                 va.surfaces[0]);
    }

    if (!options.input_path.empty()) {
        std::cout << std::format("[Gecko Probe] Decode Input: path={} codec={} width={} height={} bytes={}\n",
                                 options.input_path,
                                 input_info.codec,
                                 surface_width,
                                 surface_height,
                                 decode_payload.size());
        submitDecodeChunk(display, va.context, va.surfaces[0], decode_payload);
    } else {
        std::cout << "[Gecko Probe] No input provided; export will reflect blank-surface behavior\n";
    }

    status = syncSurfaceWithRetry(display, va.surfaces[0]);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaSyncSurface", status);
    }

    VADRMPRIMESurfaceDescriptor desc{};
    const uint32_t export_flags = options.export_type == "prime2"
                                      ? VA_EXPORT_SURFACE_READ_ONLY | VA_EXPORT_SURFACE_SEPARATE_LAYERS
                                      : VA_EXPORT_SURFACE_READ_ONLY | VA_EXPORT_SURFACE_COMPOSED_LAYERS;
    status = vaExportSurfaceHandle(display,
                                   va.surfaces[0],
                                   chooseRequestedMemType(options),
                                   export_flags,
                                   &desc);
    if (status != VA_STATUS_SUCCESS) {
        fail_va("vaExportSurfaceHandle", status);
    }

    validateDescriptor(options, desc);

    std::cout << std::format("[Gecko Probe] Surface Exported: Surface={}, FD={}, Modifier=0x{:x} (LINEAR), Format={}, ExportType={}\n",
                             va.surfaces[0],
                             desc.objects[0].fd,
                             desc.objects[0].drm_format_modifier,
                             vaFourccName(desc.fourcc),
                             options.export_type);
    std::cout << std::format("[Gecko Probe] Layer 0: offset={}, pitch={}, format={}\n",
                             desc.layers[0].offset[0],
                             desc.layers[0].pitch[0],
                             drmFormatName(desc.layers[0].drm_format));
    if (desc.num_layers > 1) {
        std::cout << std::format("[Gecko Probe] Layer 1: offset={}, pitch={}, format={}\n",
                                 desc.layers[1].offset[0],
                                 desc.layers[1].pitch[0],
                                 drmFormatName(desc.layers[1].drm_format));
    } else if (desc.layers[0].num_planes > 1) {
        std::cout << std::format("[Gecko Probe] Plane 1: offset={}, pitch={}\n",
                                 desc.layers[0].offset[1],
                                 desc.layers[0].pitch[1]);
    }
    printDescriptorMetadata(desc);

    probeZeroCopyMap(desc);

    bool egl_import_success = false;
    std::string egl_import_failure;
    if (options.import_egl) {
        std::cout << "[Gecko Probe] EGL Import: running in isolated subprocess\n";
        const pid_t child = fork();
        if (child < 0) {
            fail("fork failed for EGL import probe");
        }
        if (child == 0) {
            egl_runtime::Runtime runtime;
            auto egl_result = runtime.importAndDraw(options, desc);
            std::cout << std::format("[Gecko Probe] EGL Import: success, renderer={}, version={}\n",
                                     egl_result.gl_renderer,
                                     egl_result.gl_version);
            std::cout << std::format("[Gecko Probe] GLES Draw: success, sampled pixel=#{:02x}{:02x}{:02x}{:02x}\n",
                                     egl_result.pixel[0],
                                     egl_result.pixel[1],
                                     egl_result.pixel[2],
                                     egl_result.pixel[3]);
            std::fflush(stdout);
            std::_Exit(0);
        }

        int child_status = 0;
        if (waitpid(child, &child_status, 0) < 0) {
            fail("waitpid failed for EGL import probe");
        }
        if (WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0) {
            egl_import_success = true;
        } else if (WIFSIGNALED(child_status)) {
            egl_import_failure = std::format("child crashed with signal {}", WTERMSIG(child_status));
        } else if (WIFEXITED(child_status)) {
            egl_import_failure = std::format("child exited with status {}", WEXITSTATUS(child_status));
        } else {
            egl_import_failure = "child ended unexpectedly";
        }
        if (!egl_import_success) {
            std::cout << std::format("[Gecko Probe] EGL Import: failure, {}\n", egl_import_failure);
        }
    }

    printCompatibilityReport(options, drm_device, desc, egl_import_success, egl_import_failure);

    std::cout << std::format("[Gecko Probe] Verdict: requested {} export path validated through {}\n",
                             options.export_type,
                             options.input_path.empty() ? "surface sync + descriptor export" : "real H.264 decode + surface sync + descriptor export");
    closeExportedDescriptorObjects(desc);
    return options.import_egl && !egl_import_success ? 2 : 0;
}