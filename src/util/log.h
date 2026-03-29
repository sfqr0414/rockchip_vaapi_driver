#pragma once

extern "C" {
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
}

#include <atomic>
#include <format>
#include <iostream>
#include <sstream>
#include <string_view>
#include <vector>
#include <source_location>
#include <type_traits>
#include <ranges>
#include <thread>
#include <optional>
#include <chrono>

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

    FILE* get_pipe() {
        return pipe_;
    }

   private:
    FILE* pipe_ = nullptr;
};

inline std::string timestamp_prefix() {
    auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    std::chrono::zoned_time zt{std::chrono::current_zone(), now};
    return std::format("[{:%Y-%m-%d %H:%M:%S}] ", zt);
}

template <typename T>
class lock_free_stack {
   private:
    struct node {
        node(T const& data_) : data(data_), next(nullptr) {}
        node(T&& data_) : data(std::move(data_)), next(nullptr) {}

        T data;
        node* next;
    };

    std::atomic<unsigned> threads_in_pop;
    std::atomic<node*> to_be_deleted;

    static void delete_nodes(node* nodes) {
        while (nodes) {
            node* next = nodes->next;
            delete nodes;
            nodes = next;
        }
    }

    void chain_pending_nodes(node* nodes) {
        node* last = nodes;
        while (node* const next = last->next) {
            last = next;
        }
        chain_pending_nodes(nodes, last);
    }

    void chain_pending_nodes(node* first, node* last) {
        last->next = to_be_deleted.load(std::memory_order_relaxed);
        while (!to_be_deleted.compare_exchange_weak(last->next, first, std::memory_order_release, std::memory_order_relaxed));
    }

    void chain_pending_node(node* n) { chain_pending_nodes(n, n); }

    void try_reclaim(node* old_head) {
        if (threads_in_pop == 1) {
            node* nodes_to_delete = to_be_deleted.exchange(nullptr, std::memory_order_acquire);
            if (!--threads_in_pop) {
                delete_nodes(nodes_to_delete);
            } else if (nodes_to_delete) {
                chain_pending_nodes(nodes_to_delete);
            }
            delete old_head;
        } else {
            chain_pending_node(old_head);
            --threads_in_pop;
        }
    }

    std::atomic<node*> head;
    std::atomic<std::size_t> m_size;

   public:
    lock_free_stack() : head(nullptr), m_size(0), threads_in_pop(0), to_be_deleted(nullptr) {}
    ~lock_free_stack() {
        while (pop());
        delete_nodes(to_be_deleted.load(std::memory_order_acquire));
    }

    std::size_t size() { return m_size.load(std::memory_order_relaxed); }
    bool empty() { return head.load(std::memory_order_relaxed) == nullptr; }

    void push(T const& data) {
        node* const new_node = new node(data);
        new_node->next = head.load(std::memory_order_relaxed);
        while (!head.compare_exchange_weak(new_node->next, new_node, std::memory_order_release, std::memory_order_relaxed));
        m_size.fetch_add(1, std::memory_order_relaxed);
    }

    void push(T&& data) {
        node* const new_node = new node(std::move(data));
        new_node->next = head.load(std::memory_order_relaxed);
        while (!head.compare_exchange_weak(new_node->next, new_node, std::memory_order_release, std::memory_order_relaxed));
        m_size.fetch_add(1, std::memory_order_relaxed);
    }

    std::optional<T> pop() {
        ++threads_in_pop;
        node* old_head = head.load(std::memory_order_relaxed);
        while (old_head && !head.compare_exchange_weak(old_head, old_head->next, std::memory_order_acquire, std::memory_order_relaxed));
        std::optional<T> res(std::nullopt);
        if (old_head) {
            m_size.fetch_sub(1, std::memory_order_relaxed);
            res = std::move(old_head->data);
            try_reclaim(old_head);
        } else {
            --threads_in_pop;
        }
        return res;
    }

    void pop_all(std::vector<T>& batch) {
        node* current = head.exchange(nullptr, std::memory_order_acquire);
        if (!current) return;

        std::size_t count = 0;
        while (current) {
            batch.push_back(std::move(current->data));
            node* next = current->next;
            delete current;
            current = next;
            count++;
        }
        m_size.fetch_sub(count, std::memory_order_relaxed);
    }
};

class async_print {
   public:
    using PrefixFunc = std::string (*)();
    using PostPrintFunc = void (*)();

    struct info {
        std::ostream* os;
        std::source_location loc;
        std::string prefix;
        std::string message;
    };

    static async_print& get() {
        static async_print instance;
        return instance;
    }

    void set_cout_color(std::string_view color) { cout_color.store(color.empty() ? "" : color.data(), std::memory_order_release); }
    void set_cerr_color(std::string_view color) { cerr_color.store(color.empty() ? "" : color.data(), std::memory_order_release); }
    void set_static_prefix(std::string_view prefix) { static_prefix.store(prefix.empty() ? "" : prefix.data(), std::memory_order_release); }
    void set_prefix_callback(PrefixFunc func) { prefix_func.store(func, std::memory_order_release); }
    void set_print_location(bool enable) { print_source_location.store(enable, std::memory_order_release); }
    void set_post_print_callback(PostPrintFunc func) { post_print_func.store(func, std::memory_order_release); }

    void enqueue(std::ostream* os, std::source_location loc, std::string message) {
        std::string pfx;
        if (auto func = prefix_func.load(std::memory_order_acquire)) {
            pfx = func();
        } else {
            const char* sp = static_prefix.load(std::memory_order_acquire);
            if (sp != nullptr && sp[0] != '\0') pfx = sp;
        }

        message_queue.push(info{os, loc, std::move(pfx), std::move(message)});

        int expected = 0;
        if (work_available.compare_exchange_strong(expected, 1, std::memory_order_release, std::memory_order_relaxed)) {
            work_available.notify_one();
        }
    }

   private:
    async_print() : print_thread([this](std::stop_token st) { loop(st); }) {
        batch.reserve(2048);
    }

    ~async_print() {
        print_thread.request_stop();
        work_available.store(1, std::memory_order_release);
        work_available.notify_all();

        if (print_thread.joinable()) {
            print_thread.join();
        }
        drain_and_print();
    }

    void loop(std::stop_token st) {
        while (!st.stop_requested()) {
            work_available.wait(0, std::memory_order_acquire);
            if (st.stop_requested()) break;

            if (work_available.exchange(0, std::memory_order_acquire) != 0) {
                drain_and_print();
            }
        }
        drain_and_print();
    }

    void drain_and_print() {
        message_queue.pop_all(batch);
        if (batch.empty()) return;

        bool flush_cout = false;
        bool flush_cerr = false;
        const char* reset_color = "\033[0m";

        for (const auto& item : batch | std::views::reverse) {
            if (!item.os) continue;

            bool p_loc = print_source_location.load(std::memory_order_relaxed);
            PostPrintFunc post_print = post_print_func.load(std::memory_order_relaxed);

            if (item.os == &std::cout) {
                const char* color = cout_color.load(std::memory_order_relaxed);
                bool use_color = (color != nullptr && color[0] != '\0');

                if (use_color) std::cout << color;
                if (!item.prefix.empty()) std::cout << item.prefix;
                if (p_loc) std::cout << "[" << item.loc.file_name() << ":" << item.loc.line() << ":" << item.loc.column() << " " << item.loc.function_name() << "] ";

                std::cout << item.message;

                if (use_color) std::cout << reset_color;
                flush_cout = true;
            } else if (item.os == &std::cerr) {
                const char* color = cerr_color.load(std::memory_order_relaxed);
                bool use_color = (color != nullptr && color[0] != '\0');

                if (use_color) std::cerr << color;
                if (!item.prefix.empty()) std::cerr << item.prefix;
                if (p_loc) std::cerr << "[" << item.loc.file_name() << ":" << item.loc.line() << ":" << item.loc.column() << " " << item.loc.function_name() << "] ";

                std::cerr << item.message;

                if (use_color) std::cerr << reset_color;
                flush_cerr = true;
            } else {
                if (!item.prefix.empty()) *(item.os) << item.prefix;
                if (p_loc) *(item.os) << "[" << item.loc.file_name() << ":" << item.loc.line() << ":" << item.loc.column() << " " << item.loc.function_name() << "] ";

                *(item.os) << item.message;
                item.os->flush();
            }

            if (post_print) {
                post_print();
            }
        }

        if (flush_cout) std::cout.flush();
        if (flush_cerr) std::cerr.flush();

        batch.clear();
    }

    util::lock_free_stack<info> message_queue;
    std::atomic<int> work_available{0};
    std::jthread print_thread;
    std::vector<info> batch;

    std::atomic<const char*> cout_color{"\033[93m"};
    std::atomic<const char*> cerr_color{"\033[91m"};
    std::atomic<const char*> static_prefix{""};
    std::atomic<PrefixFunc> prefix_func{nullptr};
    std::atomic<bool> print_source_location{false};
    std::atomic<PostPrintFunc> post_print_func{nullptr};
};

inline void set_log_prefix(std::string_view prefix) { async_print::get().set_static_prefix(prefix); }
inline void set_log_prefix(std::string (*func)()) { async_print::get().set_prefix_callback(func); }
inline void set_log_color_cout(std::string_view color) { async_print::get().set_cout_color(color); }
inline void set_log_color_cerr(std::string_view color) { async_print::get().set_cerr_color(color); }
inline void set_log_source_location(bool enable) { async_print::get().set_print_location(enable); }
inline void set_log_post_print_callback(void (*func)()) { async_print::get().set_post_print_callback(func); }

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

struct log_format {
    std::string_view fmt;
    std::source_location loc;

    template <class String>
        requires std::constructible_from<std::string_view, const String&>
    consteval log_format(const String& s, std::source_location loc = std::source_location::current())
        : fmt(s), loc(loc) {
    }
};

template <class... Args>
auto format_to(mapped_string<Args...> fmt, Args&&... args) {
    return std::format(fmt, wrap(args)...);
}

template <class... Args>
auto vformat_to(std::string_view fmt, Args&&... args) {
    auto wrapped = std::forward_as_tuple(wrap(args)...);
    return std::apply(
        [&](auto&&... wargs) {
            return std::vformat(fmt, std::make_format_args(wargs...));
        },
        wrapped);
}

template <typename... Args>
void console(std::ostream& os, log_format fmt_loc, Args&&... args) {
    async_print::get().enqueue(&os, fmt_loc.loc, vformat_to(fmt_loc.fmt, std::forward<Args>(args)...));
}

template <class... Args>
void console(log_format fmt_loc, Args&&... args) {
    async_print::get().enqueue(&std::cout, fmt_loc.loc, vformat_to(fmt_loc.fmt, std::forward<Args>(args)...));
}

enum class LogLevel {
    Debug,
    Info,
    Warn,
    Error
};

inline LogLevel current_log_level() {
    static const LogLevel level = [] {
        const char* env = std::getenv("ROCKCHIP_VAAPI_LOG_LEVEL");
        if (!env) return LogLevel::Warn;
        std::string_view value{env};
        if (value == "debug") return LogLevel::Debug;
        if (value == "info") return LogLevel::Info;
        if (value == "warn") return LogLevel::Warn;
        if (value == "error") return LogLevel::Error;
        return LogLevel::Warn;
    }();
    return level;
}

inline bool should_log(LogLevel level) {
    return static_cast<int>(level) >= static_cast<int>(current_log_level());
}

struct LogSink {
    std::ostream& os;
};

static inline LogSink stdout_sink{std::cout};
static inline LogSink stderr_sink{std::cerr};

template <typename... Args>
void log(const LogSink& sink, LogLevel level, log_format fmt_loc, Args&&... args) {
    //if (!should_log(level)) return;

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
        auto msg = vformat_to(fmt_loc.fmt, std::forward<Args>(args)...);
        fmt_loc.fmt = "\n  [rockchip-vaapi] {}{}\n";
        console(sink.os, fmt_loc, prefix, msg);
    } catch (...) {
        fmt_loc.fmt = "\n  [rockchip-vaapi] {}{}\n";
        console(sink.os, fmt_loc, "[ERR  ] ", "Failed to format log message");
    }
}

}  // namespace util

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

}  // namespace std