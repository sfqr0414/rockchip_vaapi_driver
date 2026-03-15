#pragma once

#include <string_view>
#include <format>
#include <iostream>

namespace util {

enum class LogLevel {
    Debug,
    Info,
    Warn,
    Error
};

struct LogSink {
    std::ostream& os;
};

static inline LogSink stdout_sink{std::cout};
static inline LogSink stderr_sink{std::cerr};

template<typename... Args>
void log(const LogSink& sink, LogLevel level, std::string_view fmt, Args&&... args) {
    const char* prefix = "";
    switch (level) {
        case LogLevel::Debug: prefix = "[DEBUG] "; break;
        case LogLevel::Info:  prefix = "[INFO ] "; break;
        case LogLevel::Warn:  prefix = "[WARN ] "; break;
        case LogLevel::Error: prefix = "[ERROR] "; break;
    }
    
    try {
        auto msg = std::vformat(fmt, std::make_format_args(args...));
        sink.os << "[rockchip-vaapi] " << prefix << msg << std::endl;
    } catch (...) {
        sink.os << "[rockchip-vaapi] [ERR  ] Failed to format log message" << std::endl;
    }
}

// Specialization for no arguments to avoid make_format_args issues.
inline void log(const LogSink& sink, LogLevel level, std::string_view msg) {
    const char* prefix = "";
    switch (level) {
        case LogLevel::Debug: prefix = "[DEBUG] "; break;
        case LogLevel::Info:  prefix = "[INFO ] "; break;
        case LogLevel::Warn:  prefix = "[WARN ] "; break;
        case LogLevel::Error: prefix = "[ERROR] "; break;
    }
    sink.os << "[rockchip-vaapi] " << prefix << msg << std::endl;
}

} // namespace util
