// Copyright (C) 2026, Chad Attermann
//
// Platform abstraction for cross-target builds. Provides:
//   - FreeRTOS xQueue-compatible shim on native Linux (std::deque + mutex)
//   - WDT / reboot wrappers that no-op on hosts without those primitives
//
// Existing ESP32/nRF52 code paths use FreeRTOS APIs directly; we re-implement
// the small subset actually used (xQueueCreate / xQueueSendFromISR /
// xQueueReceive / pdPASS / pdTRUE) for MCU_NATIVE so call sites don't need
// per-platform forks.

#ifndef PLATFORM_H
#define PLATFORM_H

#include "Boards.h"

#if MCU_VARIANT == MCU_NATIVE

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <condition_variable>

namespace mr_platform {

// Fixed-size, single-producer / single-consumer queue of byte blobs.
// The xQueue API stores by value; we mirror that by memcpy'ing item_size
// bytes per push/pop. In practice the firmware enqueues pointers
// (sizeof(modem_packet_t*) == 8), so each "item" is a pointer-sized blob.
class Queue {
public:
    Queue(std::size_t capacity, std::size_t item_size)
        : capacity_(capacity), item_size_(item_size) {}

    bool send(const void* item) {
        std::lock_guard<std::mutex> lk(m_);
        if (q_.size() >= capacity_) return false;
        std::vector<uint8_t> blob(item_size_);
        std::memcpy(blob.data(), item, item_size_);
        q_.push_back(std::move(blob));
        cv_.notify_one();
        return true;
    }

    bool recv(void* item) {
        std::lock_guard<std::mutex> lk(m_);
        if (q_.empty()) return false;
        std::memcpy(item, q_.front().data(), item_size_);
        q_.pop_front();
        return true;
    }

private:
    std::size_t capacity_;
    std::size_t item_size_;
    std::deque<std::vector<uint8_t>> q_;
    std::mutex m_;
    std::condition_variable cv_;
};

} // namespace mr_platform

using xQueueHandle = mr_platform::Queue*;

inline xQueueHandle xQueueCreate(std::size_t count, std::size_t item_size) {
    return new mr_platform::Queue(count, item_size);
}

#define pdPASS  1
#define pdTRUE  1
#define pdFALSE 0

inline int xQueueSendFromISR(xQueueHandle q, const void* item, void* /*unused*/) {
    return (q && q->send(item)) ? pdPASS : 0;
}

inline int xQueueReceive(xQueueHandle q, void* item, uint32_t /*ticks*/) {
    return (q && q->recv(item)) ? pdTRUE : pdFALSE;
}

// Watchdog & critical-section shims: native has no FreeRTOS scheduler
// and systemd Restart=on-failure provides equivalent supervision.
inline void esp_task_wdt_reset() {}
#define portENTER_CRITICAL() do { } while (0)
#define portEXIT_CRITICAL()  do { } while (0)

#endif // MCU_VARIANT == MCU_NATIVE

#endif // PLATFORM_H
