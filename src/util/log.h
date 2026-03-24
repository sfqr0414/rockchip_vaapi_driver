#pragma once

extern "C" {
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
}

#include <format>
#include <iostream>
#include <string_view>
#include <sstream>
#include <vector>

namespace util {

class SubProcess {
   public:
    enum Mode { READ,
                WRITE };

    explicit SubProcess(const std::string& cmd, Mode mode) {
        pipe_ = popen(cmd.c_str(), mode == READ ? "r" : "w");
        if (!pipe_) {
            throw std::runtime_error("Failed to launch sub-process: " + cmd);
        }
    }

    ~SubProcess() {
        if (pipe_) pclose(pipe_);
    }

    SubProcess(const SubProcess&) = delete;
    SubProcess& operator=(const SubProcess&) = delete;

    size_t read(void* buffer, size_t size) {
        return std::fread(buffer, 1, size, pipe_);
    }

    size_t read(std::vector<uint8_t>& buffer) {
        return std::fread(buffer.data(), 1, buffer.size(), pipe_);
    }

    void write(const void* data, size_t size) {
        std::fwrite(data, 1, size, pipe_);
    }

    FILE* get_pipe(){
        return pipe_;
    }

   private:
    FILE* pipe_ = nullptr;
};

template <class T>
struct AsStream {
    const T& v;
};

template <class T>
decltype(auto) wrap(const T& arg) {
    if constexpr (requires { std::formatter<std::remove_cvref_t<T>, char>(); })
        return arg;
    else
        return AsStream<T>{arg};
}

template <class... Args>
using mapped_string = std::basic_format_string<char, decltype(wrap(std::declval<Args>()))...>;

template <class... Args>
auto format_to(mapped_string<Args...> fmt, Args&&... args) {
    return std::format(fmt, wrap(args)...);
}

class AnsiColorGuard {
   private:
    std::ostream& os;
    bool need_reset = false;
    constexpr static const char *RED = "\033[91m", *YELLOW = "\033[93m", *RESET = "\033[0m";

   public:
    explicit AnsiColorGuard(std::ostream& o) : os(o) {
        if (&os == &std::cerr) {
            std::cout << RED << std::flush;
            need_reset = true;
        } else if (&os == &std::cout) {
            std::cout << YELLOW << std::flush;
            need_reset = true;
        }
    }
    ~AnsiColorGuard() noexcept {
        if (need_reset) std::cout << RESET << std::flush;
    }
};

template <typename... Args>
auto console(std::ostream& os, mapped_string<Args...> fmt, Args&&... args) {
    AnsiColorGuard g{os};
    os << std::format(fmt, wrap(args)...);
}

template <class... Args>
void console(mapped_string<Args...> fmt, Args&&... args) {
    console(std::cout, fmt, std::forward<Args>(args)...);
}

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

template <typename... Args>
void log(const LogSink& sink, LogLevel level, mapped_string<Args...> fmt, Args&&... args) {
    const char* prefix = "";
    switch (level) {
        case LogLevel::Debug:
            prefix = "[DEBUG] ";
            break;
        case LogLevel::Info:
            prefix = "[INFO ] ";
            break;
        case LogLevel::Warn:
            prefix = "[WARN ] ";
            break;
        case LogLevel::Error:
            prefix = "[ERROR] ";
            break;
    }

    try {
        auto msg = format_to(fmt, std::forward<Args>(args)...);
        sink.os << "[rockchip-vaapi] " << prefix << msg << std::endl;
    } catch (...) {
        sink.os << "[rockchip-vaapi] [ERR  ] Failed to format log message" << std::endl;
    }
}

} // namespace util


namespace std {
template <class T>
struct formatter<util::AsStream<T>, char> {
    constexpr auto parse(auto& ctx) { return ctx.begin(); }
    auto format(const util::AsStream<T>& s, auto& ctx) const {
        std::ostringstream os;
        os << s.v;
        auto str = os.str();
        return std::ranges::copy(str, ctx.out()).out;
    }
};

} // namespace std