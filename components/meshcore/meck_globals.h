/*
 * meck_globals.h — flat-globals adapter layer for MeshCore on T-Display P4.
 *
 * mesh::Mesh and BaseChatMesh need five collaborators besides the radio:
 *   MillisecondClock, RNG, RTCClock, PacketManager, MeshTables.
 *
 * This header declares one instance of each as a Meck-flavoured global,
 * implemented in meck_globals.cpp.
 *
 * Choice of namespace `meck::` (not `Meck::`) keeps these out of the global
 * scope without dragging in a class hierarchy. Add new collaborators here
 * as the integration grows (e.g. meck::storage when SD/NVS comes online).
 */

#pragma once

#include "Mesh.h"
#include "Utils.h"
#include "MeshCore.h"
#include "Dispatcher.h"
#include "StaticPoolPacketManager.h"
#include "SimpleMeshTables.h"

namespace meck {

// ---- Concrete adapter classes (defined in meck_globals.cpp) ----

class FreeRTOSClock : public mesh::MillisecondClock {
public:
    unsigned long getMillis() override;
};

class HwRng : public mesh::RNG {
public:
    void random(uint8_t* dest, size_t sz) override;
};

// PCF8563 wiring deferred — for now this is a soft RTC counting from boot.
// MeshCore handles "time unknown" (returning 0) gracefully.
class SoftRtcClock : public mesh::RTCClock {
    uint32_t _epoch_offset;  // seconds added to monotonic clock
public:
    SoftRtcClock() : _epoch_offset(0) {}
    uint32_t getCurrentTime() override;
    void setCurrentTime(uint32_t time) override;
};

// ---- Singleton instances (defined in meck_globals.cpp) ----
extern FreeRTOSClock      clock;
extern HwRng              rng;
extern SoftRtcClock       rtc;
extern StaticPoolPacketManager packets;
extern SimpleMeshTables   tables;

}  // namespace meck
