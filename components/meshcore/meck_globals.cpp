/*
 * meck_globals.cpp — instantiations and method implementations for the
 * Meck adapter layer.
 *
 * Notes:
 * - SoftRtcClock returns _epoch_offset + (millis()/1000). It returns 0
 *   if the user hasn't called setCurrentTime(), which MeshCore reads as
 *   "time unknown". Replacing this with PCF8563 reads is a future step;
 *   the constraint is that getCurrentTime() must be cheap and non-blocking
 *   because it gets called on the radio receive hot path.
 * - StaticPoolPacketManager is constructed with 16 packet slots. MeshCore
 *   uses these for both inbound queue and outbound retransmits; 16 was the
 *   T-Deck Pro Meck baseline.
 * - SimpleMeshTables uses default sizes from MeshCore upstream (it manages
 *   its own seen-packet ring buffer internally).
 */

#include "meck_globals.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace meck {

// ---- Clock ----

unsigned long FreeRTOSClock::getMillis() {
    // esp_timer_get_time returns microseconds since boot, monotonic.
    // Wrap-around at 2^32 ms is ~49 days; MeshCore handles this internally
    // since the upstream Arduino millis() has the same property.
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

// ---- RNG ----

void HwRng::random(uint8_t* dest, size_t sz) {
    // esp_random fills 32 bits at a time, hardware-backed TRNG.
    // We pull 4 bytes per call and copy out the requested count.
    while (sz >= 4) {
        uint32_t r = esp_random();
        dest[0] = (uint8_t)(r);
        dest[1] = (uint8_t)(r >> 8);
        dest[2] = (uint8_t)(r >> 16);
        dest[3] = (uint8_t)(r >> 24);
        dest += 4; sz -= 4;
    }
    if (sz > 0) {
        uint32_t r = esp_random();
        for (size_t i = 0; i < sz; i++) {
            dest[i] = (uint8_t)(r >> (8 * i));
        }
    }
}

// ---- RTC (soft, until PCF8563 integration) ----

uint32_t SoftRtcClock::getCurrentTime() {
    // Return 0 if no epoch has been set yet. MeshCore treats 0 as
    // "time unknown" and relies on advert timestamps from peers instead.
    if (_epoch_offset == 0) return 0;
    uint64_t boot_secs = esp_timer_get_time() / 1000000ULL;
    return _epoch_offset + (uint32_t)boot_secs;
}

void SoftRtcClock::setCurrentTime(uint32_t time) {
    uint64_t boot_secs = esp_timer_get_time() / 1000000ULL;
    _epoch_offset = time - (uint32_t)boot_secs;
}

// ---- Singleton instances ----

FreeRTOSClock      clock;
HwRng              rng;
SoftRtcClock       rtc;
StaticPoolPacketManager packets(16 /*slot count*/);
SimpleMeshTables   tables;

}  // namespace meck
