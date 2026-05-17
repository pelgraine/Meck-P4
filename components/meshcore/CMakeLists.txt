# MeshCore protocol stack + radio adapter + Meck application + LVGL UI
#
# Phase B.2 — home screen ported. Compiles:
#   - MeshCore protocol stack (Mesh, Dispatcher, Packet, Identity, etc.)
#   - ed25519 crypto
#   - arduino_compat shim
#   - target.cpp        (radio attach: meck_radio_attach())
#   - meck_globals.cpp  (legacy globals, currently unused but harmless)
#   - meck_app.cpp      (Meck instance, NVS init, meck_task)
#   - MeckUI.cpp        (LVGL home screen + 7-tile structure + refresh timer)
#
# Headers (no .cpp):
#   - MeckMesh.h        (BaseChatMesh subclass, ported from old repo's P4Mesh.h)
#   - MeckDataStore.h   (NVS + SD backed storage)
#   - MeckSDCard.h      (read-only SD helpers; LilyGo handles the actual mount)
#   - MeckUI.h          (UI public API)

file(GLOB MESHCORE_SRCS
    "meshcore_src/*.cpp"
    "meshcore_src/helpers/*.cpp"
)

# Prune Arduino-FS-dependent files
list(FILTER MESHCORE_SRCS EXCLUDE REGEX ".*IdentityStore\\.cpp$")
list(FILTER MESHCORE_SRCS EXCLUDE REGEX ".*TransportKeyStore\\.cpp$")

file(GLOB ED25519_SRCS
    "meshcore_src/ed25519/*.c"
)

idf_component_register(
    SRCS
        ${MESHCORE_SRCS}
        ${ED25519_SRCS}
        "arduino_compat.cpp"
        "target.cpp"
        "meck_globals.cpp"
        "meck_app.cpp"
        "MeckUI.cpp"
        "MeckAudio.cpp"
        "MeckAudioUI.cpp"
        "es8311.cpp" 
        "meck_montserrat_14.c"
        "meck_montserrat_16.c"
        "meck_montserrat_18.c"
        "meck_montserrat_22.c"
        "meck_montserrat_24.c"
        "meck_montserrat_28.c"
    INCLUDE_DIRS
        "."
        "meshcore_src"
        "meshcore_src/helpers"
        "meshcore_src/ed25519"
    REQUIRES
        arduino_cpp_bus_driver
        cpp_bus_driver
        private_library
        lvgl__lvgl
        mbedtls
        nvs_flash
        esp_timer
        esp_system
        esp_hw_support
        freertos
        vfs
        fatfs
        chmorgan__esp-audio-player
        driver
        espressif__cjson
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=reorder
    -Wno-error=unused-variable
    -Wno-error=unused-function
    -Wno-error=class-memaccess
    -Wno-error=format=
    -Wno-error=sign-compare
    -Wno-error=narrowing
    -Wno-error=format-truncation
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    -DMAX_GROUP_CHANNELS=8
    -DMAX_CONTACTS=2000
)

# Capture the actual UTC build epoch and inject it into the firmware so
# P4RTCClock's fallback boot time is a true UTC value, not the build
# machine's local time.
#
# Why: __DATE__ / __TIME__ are LOCAL time on the build machine. If we
# parse those and treat the result as UTC, the UI subsequently adds the
# user's UTC offset on top, displaying pre-sync timestamps off by the
# build machine's timezone offset (e.g. AEST/+10 → "12:01 build" shown
# as "22:01 = 10:01 PM"). string(TIMESTAMP ... %s UTC) is built into
# CMake itself (no shell or `date` dependency) and re-evaluates every
# configure step, so each clean build bakes a fresh epoch.
#
# PRIVATE scope: only P4RTCClock in MeckMesh.h consumes the macro; no
# need to leak it to other components.
string(TIMESTAMP MECK_BUILD_EPOCH "%s" UTC)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    MECK_BUILD_EPOCH=${MECK_BUILD_EPOCH}
)