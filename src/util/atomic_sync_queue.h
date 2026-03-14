#pragma once

#include <atomic>
#include <cstddef>
#include <optional>
#include <semaphore>
#include <type_traits>

namespace util {

/// A simple lock-free producer/consumer queue using atomic wait/notify and semaphores.
///
/// The implementation avoids std::mutex and std::condition_variable, and uses
/// std::counting_semaphore for capacity control.
template <typename T, size_t Capacity>
class AtomicSyncQueue {
    static_assert(Capacity > 0, "Capacity must be > 0");

public:
    AtomicSyncQueue() = default;
    ~AtomicSyncQueue() = default;

    AtomicSyncQueue(const AtomicSyncQueue&) = delete;
    AtomicSyncQueue& operator=(const AtomicSyncQueue&) = delete;

    /// Push an item into the queue.
    /// Blocks if the queue is full.
    void push(T item) {
        free_slots_.acquire();
        const size_t idx = tail_.fetch_add(1, std::memory_order_acq_rel) % Capacity;
        buffer_[idx] = std::move(item);
        const size_t prev = count_.fetch_add(1, std::memory_order_acq_rel);
        used_slots_.release();
        // Notify any waiters that new data is available.
        if (prev == 0) {
            count_.notify_all();
        }
    }

    /// Pop an item from the queue.
    /// Returns std::nullopt if the queue is shutting down.
    std::optional<T> pop() {
        // Wait until there is at least one item available.
        while (true) {
            size_t current = count_.load(std::memory_order_acquire);
            if (current > 0) {
                break;
            }
            if (shutdown_.load(std::memory_order_acquire)) {
                return std::nullopt;
            }
            count_.wait(current);
        }

        used_slots_.acquire();
        const size_t idx = head_.fetch_add(1, std::memory_order_acq_rel) % Capacity;
        T item = std::move(buffer_[idx]);
        const size_t prev = count_.fetch_sub(1, std::memory_order_acq_rel);

        free_slots_.release();
        if (prev == 1) {
            // queue is now empty; wake any waiters that might be waiting for emptiness
            count_.notify_all();
        }
        return item;
    }

    /// Request the queue to shutdown. After calling this, pop() will return std::nullopt
    /// once all pending items are drained.
    void shutdown() {
        shutdown_.store(true, std::memory_order_release);
        count_.notify_all();
        used_slots_.release(Capacity);
        free_slots_.release(Capacity);
    }

private:
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
    std::atomic<size_t> count_{0};
    std::atomic<bool> shutdown_{false};

    std::counting_semaphore<Capacity> free_slots_{Capacity};
    std::counting_semaphore<Capacity> used_slots_{0};

    T buffer_[Capacity];
};

} // namespace util
