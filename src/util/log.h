#pragma once

#include <cstdio>
#include <format>
#include <mutex>
#include <string_view>

namespace util {

struct LogSink {
    FILE* file;
};

static inline LogSink stdout_sink{stdout};
static inline LogSink stderr_sink{stderr};

enum class LogLevel {
    Info,
    Warn,
    Error,
};

/// Log a formatted message to the given sink.
///
/// @param sink Output sink (stdout/stderr, or a file pointer).
/// @param fmt  Format string (std::format style).
/// @param args Format arguments.
template <typename... Args>
void log(const LogSink& sink, LogLevel level, std::string_view fmt, Args&&... args) {
    static std::mutex mutex;
    const char* prefix = "";
    switch (level) {
        case LogLevel::Info: prefix = "[rockchip-vaapi] "; break;
        case LogLevel::Warn: prefix = "[rockchip-vaapi][WARN] "; break;
        case LogLevel::Error: prefix = "[rockchip-vaapi][ERROR] "; break;
    }

    auto msg = std::vformat(fmt, std::make_format_args(std::forward<Args>(args)...));
    std::lock_guard lock(mutex);
    std::fputs(prefix, sink.file);
    std::fputs(msg.c_str(), sink.file);
    std::fputc('\n', sink.file);
}

} // namespace util
