# MeshCore protocol stack — passive component (compile-only, not linked yet)
#
# Excluded from this phase:
#   helpers/IdentityStore.cpp  — uses Arduino LittleFS/SD API (FILESYSTEM, File)
#   meshcore_src/TransportKeyStore.cpp — depends on IdentityStore.h
# Both will be replaced with NVS-backed equivalents in step C.6 (P4DataStore).

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
    INCLUDE_DIRS
        "."
        "meshcore_src"
        "meshcore_src/helpers"
        "meshcore_src/ed25519"
    REQUIRES
        arduino_cpp_bus_driver
        mbedtls
        nvs_flash
        esp_timer
        esp_system
        esp_hw_support
        freertos
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
