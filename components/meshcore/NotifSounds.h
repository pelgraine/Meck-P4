#pragma once

// =============================================================================
// NotifSounds.h -- Per-channel notification sound configuration (P4 adaptation)
//
// Stores a custom sound filename per channel for notification tones.
// Config persisted to /sdcard/meshcore/notif_sounds.cfg on SD card.
// Sound files are MP3s in /sdcard/audio/tones/ (bundled defaults +
// user-added files).
//
// P4 adaptation: uses ESP-IDF VFS (fopen/fwrite/opendir/readdir) instead
// of Arduino SD. No Arduino dependencies.
// =============================================================================

#ifndef P4_NOTIF_SOUNDS_H
#define P4_NOTIF_SOUNDS_H

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>
#include <vector>
#include <string>

#include "BundledSounds.h"  // for MECK_TONES_DIR

#ifndef MAX_GROUP_CHANNELS
#define MAX_GROUP_CHANNELS 20
#endif

#define NOTIF_SOUND_NAME_MAX   32
#define NOTIF_SOUND_SLOTS      (MAX_GROUP_CHANNELS + 1)  // +1 for DMs
#define NOTIF_SOUND_CONFIG_PATH "/sdcard/meshcore/notif_sounds.cfg"
#define NOTIF_SOUND_MAGIC      0x4E534E44  // "NSND"
#define NOTIF_SOUND_VERSION    1

struct __attribute__((packed)) NotifSoundCfgHeader {
  uint32_t magic;
  uint8_t  version;
  uint8_t  count;      // Number of slots stored
  uint8_t  reserved[2];
  // Followed by count * NOTIF_SOUND_NAME_MAX bytes
};

class NotifSounds {
public:
  NotifSounds() {
    memset(_sounds, 0, sizeof(_sounds));
    _pendingPlay = false;
    _pendingFile[0] = '\0';
  }

  void begin() {
    loadConfig();
    printf("NotifSounds: Config loaded\n");
  }

  // --- Config accessors ---

  const char* getSoundForChannel(uint8_t channel_idx) const {
    int slot = (channel_idx == 0xFF) ? MAX_GROUP_CHANNELS : (int)channel_idx;
    if (slot < 0 || slot >= NOTIF_SOUND_SLOTS) return "";
    return _sounds[slot];
  }

  bool hasSoundForChannel(uint8_t channel_idx) const {
    const char* s = getSoundForChannel(channel_idx);
    return s && s[0] != '\0';
  }

  void setSoundForChannel(uint8_t channel_idx, const char* filename) {
    int slot = (channel_idx == 0xFF) ? MAX_GROUP_CHANNELS : (int)channel_idx;
    if (slot < 0 || slot >= NOTIF_SOUND_SLOTS) return;
    if (filename) {
      strncpy(_sounds[slot], filename, NOTIF_SOUND_NAME_MAX - 1);
      _sounds[slot][NOTIF_SOUND_NAME_MAX - 1] = '\0';
    } else {
      _sounds[slot][0] = '\0';
    }
    saveConfig();
    printf("NotifSounds: Channel %d -> '%s'\n", (int)channel_idx,
           _sounds[slot][0] ? _sounds[slot] : "(none)");
  }

  void clearSoundForChannel(uint8_t channel_idx) {
    setSoundForChannel(channel_idx, nullptr);
  }

  // --- Sound file scanning ---

  void scanSoundFiles() {
    _soundFiles.clear();

    struct stat st;
    if (stat(MECK_TONES_DIR, &st) != 0) {
      mkdir("/sdcard/audio", 0755);
      mkdir(MECK_TONES_DIR, 0755);
    }

    DIR* dir = opendir(MECK_TONES_DIR);
    if (!dir) {
      printf("NotifSounds: Failed to open %s\n", MECK_TONES_DIR);
      return;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
      if (entry->d_type == DT_DIR) continue;
      const char* name = entry->d_name;
      if (!name || name[0] == '.') continue;

      // Check for .mp3 extension (case-insensitive)
      size_t len = strlen(name);
      if (len > 4) {
        const char* ext = name + len - 4;
        if (strcasecmp(ext, ".mp3") == 0) {
          _soundFiles.push_back(std::string(name));
        }
      }
    }
    closedir(dir);

    std::sort(_soundFiles.begin(), _soundFiles.end());
    printf("NotifSounds: Found %d sound files\n", (int)_soundFiles.size());
  }

  int getSoundFileCount() const { return (int)_soundFiles.size(); }
  const std::string& getSoundFile(int idx) const { return _soundFiles[idx]; }
  const std::vector<std::string>& getSoundFiles() const { return _soundFiles; }

  // --- Pending playback request ---

  void requestPlay(const char* fullPath) {
    strncpy(_pendingFile, fullPath, sizeof(_pendingFile) - 1);
    _pendingFile[sizeof(_pendingFile) - 1] = '\0';
    _pendingPlay = true;
  }

  bool hasPendingPlay() const { return _pendingPlay; }
  const char* getPendingFile() const { return _pendingFile; }

  void clearPending() {
    _pendingPlay = false;
    _pendingFile[0] = '\0';
  }

  // Build the full path for a tone filename
  static void buildTonePath(char* dest, size_t destSize, const char* filename) {
    snprintf(dest, destSize, "%s/%s", MECK_TONES_DIR, filename);
  }

private:
  char _sounds[NOTIF_SOUND_SLOTS][NOTIF_SOUND_NAME_MAX];
  std::vector<std::string> _soundFiles;
  bool _pendingPlay;
  char _pendingFile[64];

  void loadConfig() {
    memset(_sounds, 0, sizeof(_sounds));

    FILE* f = fopen(NOTIF_SOUND_CONFIG_PATH, "rb");
    if (!f) {
      printf("NotifSounds: No config file, using defaults\n");
      return;
    }

    NotifSoundCfgHeader hdr;
    if (fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr) ||
        hdr.magic != NOTIF_SOUND_MAGIC || hdr.version != NOTIF_SOUND_VERSION) {
      printf("NotifSounds: Config invalid or wrong version\n");
      fclose(f);
      return;
    }

    int slotsToRead = hdr.count;
    if (slotsToRead > NOTIF_SOUND_SLOTS) slotsToRead = NOTIF_SOUND_SLOTS;

    for (int i = 0; i < slotsToRead; i++) {
      if (fread(_sounds[i], 1, NOTIF_SOUND_NAME_MAX, f) != NOTIF_SOUND_NAME_MAX) {
        break;
      }
      _sounds[i][NOTIF_SOUND_NAME_MAX - 1] = '\0';
    }

    fclose(f);
    printf("NotifSounds: Loaded %d slots from config\n", slotsToRead);
  }

  void saveConfig() {
    struct stat st;
    if (stat("/sdcard/meshcore", &st) != 0) {
      mkdir("/sdcard/meshcore", 0755);
    }

    FILE* f = fopen(NOTIF_SOUND_CONFIG_PATH, "wb");
    if (!f) {
      printf("NotifSounds: Failed to save config\n");
      return;
    }

    NotifSoundCfgHeader hdr;
    hdr.magic = NOTIF_SOUND_MAGIC;
    hdr.version = NOTIF_SOUND_VERSION;
    hdr.count = NOTIF_SOUND_SLOTS;
    hdr.reserved[0] = 0;
    hdr.reserved[1] = 0;
    fwrite(&hdr, 1, sizeof(hdr), f);

    for (int i = 0; i < NOTIF_SOUND_SLOTS; i++) {
      fwrite(_sounds[i], 1, NOTIF_SOUND_NAME_MAX, f);
    }

    fclose(f);
  }
};

// Global singleton (defined in meck_app.cpp)
extern NotifSounds g_notif_sounds;

#endif // P4_NOTIF_SOUNDS_H
