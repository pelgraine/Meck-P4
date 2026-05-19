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
        "meck_montserrat_30.c"
        "meck_montserrat_32.c"
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
# CMake itself (no shell or `date` dependency) and gives a true UTC
# epoch.
#
# The catch: string(TIMESTAMP) only runs at CMake CONFIGURE time, not
# at every BUILD. Without help, MECK_BUILD_EPOCH stays frozen at the
# value from your last `idf.py reconfigure` (or first build after a
# clean). After hours/days without a reconfigure the baked epoch goes
# stale, and a fresh-flashed device boots with an out-of-date clock
# that the mesh's "first sync wins" logic then locks in forever.
#
# Fix: declare a sentinel file in the build directory as a configure
# dependency, and touch that sentinel on every build via an ALL custom
# target. Result: every `idf.py build` touches the sentinel, which
# makes the NEXT `idf.py build` re-run configure, which re-bakes
# MECK_BUILD_EPOCH. The first build after a fullclean configures
# normally (fresh epoch), and every build thereafter also reconfigures
# (also fresh epoch).
#
# PRIVATE scope: only P4RTCClock in MeckMesh.h consumes the macro; no
# need to leak it to other components.
set(MECK_EPOCH_SENTINEL "${CMAKE_CURRENT_BINARY_DIR}/.meck_epoch_sentinel")

# Make sure the sentinel exists on a fresh configure so
# CMAKE_CONFIGURE_DEPENDS has something to stat. file(TOUCH) creates
# the file if missing and bumps its mtime if present; harmless either
# way during configure (the touch-during-build is what actually drives
# the re-bake on subsequent builds).
file(TOUCH "${MECK_EPOCH_SENTINEL}")

# Register the sentinel as a configure-time dependency. When the
# sentinel's mtime is newer than the last configure, CMake will
# re-run configure on the next build, which re-evaluates the
# string(TIMESTAMP) below.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${MECK_EPOCH_SENTINEL}"
)

# Always-run custom target that touches the sentinel during the build.
# ALL ensures it's part of the default build; add_dependencies wires
# it ahead of the component library so the sentinel is fresh before
# anything that might use the epoch starts compiling. (The actual
# re-bake happens on the NEXT build via the configure dependency
# above; the touch on THIS build is what triggers that.)
add_custom_target(meck_touch_epoch_sentinel ALL
    COMMAND ${CMAKE_COMMAND} -E touch "${MECK_EPOCH_SENTINEL}"
    COMMENT "Marking MECK_BUILD_EPOCH stale for next configure"
    VERBATIM
)
add_dependencies(${COMPONENT_LIB} meck_touch_epoch_sentinel)

string(TIMESTAMP MECK_BUILD_EPOCH "%s" UTC)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    MECK_BUILD_EPOCH=${MECK_BUILD_EPOCH}
)