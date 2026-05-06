/*
 * MeckSDCard.h — SD card helpers for Meck on T-Display P4
 *
 * Trimmed from P4SDCard.h: LilyGo's app_main mounts the SD card at /sdcard
 * via SDMMC before our code runs, so we don't need init/unmount/host config.
 * We just need read-only helpers so MeckDataStore can test for files.
 *
 * Mount status is checked dynamically via stat() rather than a static flag,
 * which removes any need to coordinate with LilyGo's mount sequence.
 */

#pragma once

#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <sys/unistd.h>

static inline bool p4_sdcard_is_mounted() {
    struct stat st;
    if (stat("/sdcard", &st) != 0) return false;
    return S_ISDIR(st.st_mode);
}

static inline bool p4_sdcard_file_exists(const char* path) {
    struct stat st;
    return (stat(path, &st) == 0);
}

static inline size_t p4_sdcard_file_size(const char* path) {
    struct stat st;
    if (stat(path, &st) != 0) return 0;
    return (size_t)st.st_size;
}
