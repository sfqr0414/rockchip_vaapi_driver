#include <atomic>
#include <cstddef>
#include <memory>
#include <new>
#include <optional>
#include <thread>
#include <vector>
#include <latch>
#include <chrono>
#include <iostream>
#include <cassert>
#include <concepts>
#include <random>

namespace util {

    template <size_t N>
    concept ValidCapacity = N > 0 && N <= 0x3FFFFFFF;

    template <typename T, size_t Capacity> requires ValidCapacity<Capacity>
    class AtomicSyncQueue {
        static_assert(std::is_nothrow_move_constructible_v<T> || std::is_nothrow_copy_constructible_v<T>,
            "Type T must be noexcept constructible for lock-free safety");
        static_assert(std::is_nothrow_destructible_v<T>,
            "Type T must be noexcept destructible");

#if defined(__cpp_lib_hardware_interference_size) && __cpp_lib_hardware_interference_size >= 64
        static constexpr size_t CacheLineSize = std::hardware_destructive_interference_size;
#else
        static constexpr size_t CacheLineSize = 64;
#endif

        struct alignas(CacheLineSize) Slot {
            std::atomic<uint32_t> turn;
            alignas(T) unsigned char data[sizeof(T)];
        };

    public:
        AtomicSyncQueue() {
            for (size_t i = 0; i < Capacity; ++i) {
                buffer_[i].turn.store(0, std::memory_order_relaxed);
            }
        }

        ~AtomicSyncQueue() {
            // Wake any waiters to help ensure they can exit quickly.
            shutdown();

            uint64_t h = head_.load(std::memory_order_acquire);
            uint64_t t = tail_.load(std::memory_order_acquire);

            while (h != t) {
                uint32_t lap = uint32_t(h >> 32);
                uint32_t idx = uint32_t(h);
                Slot& slot = buffer_[idx];

                if (slot.turn.load(std::memory_order_acquire) == lap * 2 + 1) {
                    T* ptr = std::launder(reinterpret_cast<T*>(slot.data));
                    std::destroy_at(ptr);
                }

                if (++idx == Capacity) {
                    idx = 0;
                    ++lap;
                }
                h = (static_cast<uint64_t>(lap) << 32) | idx;
            }
        }

        AtomicSyncQueue(const AtomicSyncQueue&) = delete;
        AtomicSyncQueue& operator=(const AtomicSyncQueue&) = delete;

        void shutdown() noexcept {
            bool expected = false;
            if (shutdown_.compare_exchange_strong(expected, true, std::memory_order_release, std::memory_order_relaxed)) {
                for (size_t i = 0; i < Capacity; ++i) {
                    // Change the slot value so any waiters wake even if notify was missed.
                    buffer_[i].turn.fetch_add(1, std::memory_order_release);
                    buffer_[i].turn.notify_all();
                }
            }
        }

        template <bool blocking = true>
        bool push(auto&& item) {
            uint64_t curr = tail_.load(std::memory_order_relaxed);
            while (true) {
                if (shutdown_.load(std::memory_order_acquire))
                    return false;

                uint32_t lap = uint32_t(curr >> 32);
                uint32_t idx = uint32_t(curr);
                Slot& slot = buffer_[idx];

                uint32_t turn = slot.turn.load(std::memory_order_acquire);
                uint32_t expected_turn = lap * 2;

                if (turn == expected_turn) {
                    uint32_t next_idx = idx + 1;
                    uint32_t next_lap = lap;
                    if (next_idx == Capacity) {
                        next_idx = 0;
                        next_lap++;
                    }
                    uint64_t next_val = (static_cast<uint64_t>(next_lap) << 32) | next_idx;

                    if (tail_.compare_exchange_weak(curr, next_val, std::memory_order_relaxed)) {
                        T* ptr = std::launder(reinterpret_cast<T*>(slot.data));
                        new (ptr) T(std::forward<decltype(item)>(item));

                        slot.turn.store(expected_turn + 1, std::memory_order_release);
                        slot.turn.notify_all();
                        return true;
                    }
                }
                else if (static_cast<int32_t>(turn - expected_turn) < 0) {
                    if constexpr (blocking) {
                        slot.turn.wait(turn, std::memory_order_relaxed);
                        curr = tail_.load(std::memory_order_relaxed);
                    }
                    else {
                        return false;
                    }
                }
                else {
                    curr = tail_.load(std::memory_order_relaxed);
                }
            }
        }

        template <bool blocking = true>
        std::optional<T> pop() {
            uint64_t curr = head_.load(std::memory_order_relaxed);
            while (true) {
                if (shutdown_.load(std::memory_order_acquire))
                    return std::nullopt;

                uint32_t lap = uint32_t(curr >> 32);
                uint32_t idx = uint32_t(curr);
                Slot& slot = buffer_[idx];

                uint32_t turn = slot.turn.load(std::memory_order_acquire);
                uint32_t expected_turn = lap * 2 + 1;

                if (turn == expected_turn) {
                    uint32_t next_idx = idx + 1;
                    uint32_t next_lap = lap;
                    if (next_idx == Capacity) {
                        next_idx = 0;
                        next_lap++;
                    }
                    uint64_t next_val = (static_cast<uint64_t>(next_lap) << 32) | next_idx;

                    if (head_.compare_exchange_weak(curr, next_val, std::memory_order_relaxed)) {
                        T* ptr = std::launder(reinterpret_cast<T*>(slot.data));
                        T result = std::move(*ptr);
                        std::destroy_at(ptr);

                        slot.turn.store(expected_turn + 1, std::memory_order_release);
                        slot.turn.notify_all();
                        return result;
                    }
                }
                else if (static_cast<int32_t>(turn - expected_turn) < 0) {
                    if constexpr (blocking) {
                        slot.turn.wait(turn, std::memory_order_relaxed);
                        curr = head_.load(std::memory_order_relaxed);
                    }
                    else {
                        return std::nullopt;
                    }
                }
                else {
                    curr = head_.load(std::memory_order_relaxed);
                }
            }
        }

        bool empty() const noexcept {
            return head_.load(std::memory_order_relaxed) == tail_.load(std::memory_order_relaxed);
        }

    private:
        // Padding to avoid false sharing between head/tail and slot array.
        alignas(CacheLineSize) std::atomic<uint64_t> head_{ 0 };
        alignas(CacheLineSize) std::atomic<uint64_t> tail_{ 0 };
        std::atomic<bool> shutdown_{ false };
        alignas(CacheLineSize) Slot buffer_[Capacity];
    };

}