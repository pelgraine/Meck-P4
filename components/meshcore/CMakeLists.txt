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
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=reorder
    -Wno-error=unused-variable
    -Wno-error=unused-function
    -Wno-error=class-memaccess
    -Wno-error=format=
    -Wno-error=sign-compare
    -Wno-error=narrowing
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    -DMAX_GROUP_CHANNELS=8
)
