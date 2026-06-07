#pragma once
// =============================================================================
// EpubZipReader.h - Minimal ZIP reader for EPUB files (Meck-P4 port)
//
// Parses ZIP archives from the SD card using POSIX stdio (fopen/fread), so it
// runs on the P4's mounted /sdcard without any Arduino FS dependency. DEFLATE
// is handled by zlib (espressif/zlib managed component) using raw inflate.
//
// Supports:
//   - STORED   (method 0) entries - direct copy
//   - DEFLATED (method 8) entries - zlib raw inflate
//   - ZIP64 is NOT supported (EPUBs don't need it)
//
// Memory: extraction buffers and the entry table prefer PSRAM on device
// (heap_caps SPIRAM), falling back to the regular heap. On a host build
// (ESP_PLATFORM undefined) everything uses malloc, so this header compiles and
// can be unit-tested off-target.
// =============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>

#include <zlib.h>

#if defined(ESP_PLATFORM)
  #include "esp_heap_caps.h"
#endif

// Logging: routed to the same console printf the rest of Meck-P4 uses. Quiet to
// override (define MECK_EPUB_LOG to nothing) if it ever gets noisy.
#ifndef MECK_EPUB_LOG
  #define MECK_EPUB_LOG(...) printf(__VA_ARGS__)
#endif

// ---- ZIP format constants ----
#define ZIP_LOCAL_FILE_HEADER_SIG   0x04034b50
#define ZIP_CENTRAL_DIR_SIG         0x02014b50
#define ZIP_END_OF_CENTRAL_DIR_SIG  0x06054b50

#define ZIP_METHOD_STORED   0
#define ZIP_METHOD_DEFLATED 8

// Maximum files we track in a ZIP (EPUBs typically have 20-100 files)
#define ZIP_MAX_ENTRIES 128

// Maximum filename length within the ZIP
#define ZIP_MAX_FILENAME 128

// ---- Data structures ----

struct ZipEntry {
  char     filename[ZIP_MAX_FILENAME];
  uint16_t compressionMethod;   // 0=STORED, 8=DEFLATED
  uint32_t compressedSize;
  uint32_t uncompressedSize;
  uint32_t localHeaderOffset;   // Offset to local file header in ZIP
  uint32_t crc32;
};

// ---- Helper: read little-endian values from a byte buffer ----

static inline uint16_t zipRead16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t zipRead32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// =============================================================================
// EpubZipReader class
// =============================================================================

class EpubZipReader {
public:
  EpubZipReader() : _file(nullptr), _entries(nullptr), _entryCount(0),
                    _isOpen(false), _cdOffset(0), _cdSize(0), _totalEntries(0) {
    _entries = (ZipEntry*)_allocBuffer(ZIP_MAX_ENTRIES * sizeof(ZipEntry));
    if (!_entries) {
      MECK_EPUB_LOG("ZipReader: FATAL - failed to allocate entry table\n");
    }
  }

  ~EpubZipReader() {
    close();
    if (_entries) {
      free(_entries);
      _entries = nullptr;
    }
  }

  // ----------------------------------------------------------
  // Open a ZIP file by path and parse its central directory.
  // Owns the FILE* until close() / destruction.
  // ----------------------------------------------------------
  bool open(const char* path) {
    close();           // drop any previously open file
    _isOpen = false;
    _entryCount = 0;

    if (!_entries) {
      MECK_EPUB_LOG("ZipReader: entry table not allocated\n");
      return false;
    }
    if (!path) return false;

    _file = fopen(path, "rb");
    if (!_file) {
      MECK_EPUB_LOG("ZipReader: cannot open %s\n", path);
      return false;
    }

    fseek(_file, 0, SEEK_END);
    long sz = ftell(_file);
    fseek(_file, 0, SEEK_SET);
    if (sz < 22) {
      MECK_EPUB_LOG("ZipReader: file too small for ZIP\n");
      return false;
    }
    uint32_t fileSize = (uint32_t)sz;

    // ---- Step 1: Find the End of Central Directory record ----
    // EOCD is at least 22 bytes, at end of file. EPUBs have no trailing
    // comment, so the last 1KB is plenty to find the signature.
    uint32_t searchStart = (fileSize > 65557) ? (fileSize - 65557) : 0;
    uint32_t searchLen = fileSize - searchStart;
    if (searchLen > 1024) {
      searchStart = fileSize - 1024;
      searchLen = 1024;
    }

    uint8_t* searchBuf = (uint8_t*)_allocBuffer(searchLen);
    if (!searchBuf) {
      MECK_EPUB_LOG("ZipReader: failed to alloc search buffer\n");
      return false;
    }

    if (!_readAt(searchStart, searchBuf, searchLen)) {
      free(searchBuf);
      MECK_EPUB_LOG("ZipReader: failed to read EOCD area\n");
      return false;
    }

    bool foundEocd = false;
    for (int i = (int)searchLen - 22; i >= 0; i--) {
      if (zipRead32(&searchBuf[i]) == ZIP_END_OF_CENTRAL_DIR_SIG) {
        _totalEntries = zipRead16(&searchBuf[i + 10]);
        _cdSize       = zipRead32(&searchBuf[i + 12]);
        _cdOffset     = zipRead32(&searchBuf[i + 16]);
        foundEocd = true;
        break;
      }
    }
    free(searchBuf);

    if (!foundEocd) {
      MECK_EPUB_LOG("ZipReader: EOCD not found - not a valid ZIP\n");
      return false;
    }

    MECK_EPUB_LOG("ZipReader: EOCD %u entries, CD at %u (%u bytes)\n",
                  (unsigned)_totalEntries, (unsigned)_cdOffset, (unsigned)_cdSize);

    // ---- Step 2: Parse Central Directory entries ----
    if (_cdSize == 0 || _cdSize > 512 * 1024) {
      MECK_EPUB_LOG("ZipReader: central directory size unreasonable\n");
      return false;
    }

    uint8_t* cdBuf = (uint8_t*)_allocBuffer(_cdSize);
    if (!cdBuf) {
      MECK_EPUB_LOG("ZipReader: failed to alloc %u bytes for CD\n", (unsigned)_cdSize);
      return false;
    }

    if (!_readAt(_cdOffset, cdBuf, _cdSize)) {
      free(cdBuf);
      MECK_EPUB_LOG("ZipReader: failed to read central directory\n");
      return false;
    }

    uint32_t pos = 0;
    _entryCount = 0;
    while (pos + 46 <= _cdSize && _entryCount < ZIP_MAX_ENTRIES) {
      if (zipRead32(&cdBuf[pos]) != ZIP_CENTRAL_DIR_SIG) break;

      uint16_t method      = zipRead16(&cdBuf[pos + 10]);
      uint32_t crc         = zipRead32(&cdBuf[pos + 16]);
      uint32_t compSize    = zipRead32(&cdBuf[pos + 20]);
      uint32_t uncompSize  = zipRead32(&cdBuf[pos + 24]);
      uint16_t fnLen       = zipRead16(&cdBuf[pos + 28]);
      uint16_t extraLen    = zipRead16(&cdBuf[pos + 30]);
      uint16_t commentLen  = zipRead16(&cdBuf[pos + 32]);
      uint32_t localOffset = zipRead32(&cdBuf[pos + 42]);

      int copyLen = (fnLen < ZIP_MAX_FILENAME - 1) ? fnLen : ZIP_MAX_FILENAME - 1;
      memcpy(_entries[_entryCount].filename, &cdBuf[pos + 46], copyLen);
      _entries[_entryCount].filename[copyLen] = '\0';

      _entries[_entryCount].compressionMethod = method;
      _entries[_entryCount].compressedSize    = compSize;
      _entries[_entryCount].uncompressedSize  = uncompSize;
      _entries[_entryCount].localHeaderOffset = localOffset;
      _entries[_entryCount].crc32             = crc;

      // Skip directory entries (names ending in '/').
      if (copyLen > 0 && _entries[_entryCount].filename[copyLen - 1] != '/') {
        _entryCount++;
      }

      pos += 46 + fnLen + extraLen + commentLen;
    }
    free(cdBuf);

    MECK_EPUB_LOG("ZipReader: parsed %d file entries\n", _entryCount);
    _isOpen = true;
    return true;
  }

  // Close the file and reset. Safe to call repeatedly.
  void close() {
    if (_file) { fclose(_file); _file = nullptr; }
    _isOpen = false;
    _entryCount = 0;
  }

  int getEntryCount() const { return _entryCount; }

  const ZipEntry* getEntry(int index) const {
    if (index < 0 || index >= _entryCount) return nullptr;
    return &_entries[index];
  }

  // Find an entry by exact filename (case-sensitive). Returns index or -1.
  int findEntry(const char* filename) const {
    for (int i = 0; i < _entryCount; i++)
      if (strcmp(_entries[i].filename, filename) == 0) return i;
    return -1;
  }

  // Find first entry whose name ends with suffix (case-insensitive). -1 if none.
  int findEntryBySuffix(const char* suffix) const {
    int suffixLen = (int)strlen(suffix);
    for (int i = 0; i < _entryCount; i++) {
      int fnLen = (int)strlen(_entries[i].filename);
      if (fnLen >= suffixLen &&
          strcasecmp(&_entries[i].filename[fnLen - suffixLen], suffix) == 0)
        return i;
    }
    return -1;
  }

  // Fill matchIndices[] with entries whose name starts with prefix. Returns count.
  int findEntriesByPrefix(const char* prefix, int* matchIndices, int maxMatches) const {
    int count = 0;
    int prefixLen = (int)strlen(prefix);
    for (int i = 0; i < _entryCount && count < maxMatches; i++)
      if (strncmp(_entries[i].filename, prefix, prefixLen) == 0)
        matchIndices[count++] = i;
    return count;
  }

  // Extract an entry to a freshly allocated, NUL-terminated buffer (caller
  // frees). Sets *outSize to the uncompressed length. nullptr on failure.
  uint8_t* extractEntry(int index, uint32_t* outSize) {
    if (!_isOpen || !_file || index < 0 || index >= _entryCount) return nullptr;

    const ZipEntry& entry = _entries[index];

    // Local header: 30 fixed bytes + variable filename + extra field. The
    // local header's own filename/extra lengths give the true data offset.
    uint8_t localHeader[30];
    if (!_readAt(entry.localHeaderOffset, localHeader, 30)) {
      MECK_EPUB_LOG("ZipReader: failed to read local header\n");
      return nullptr;
    }
    if (zipRead32(localHeader) != ZIP_LOCAL_FILE_HEADER_SIG) {
      MECK_EPUB_LOG("ZipReader: bad local header signature\n");
      return nullptr;
    }
    uint16_t localFnLen    = zipRead16(&localHeader[26]);
    uint16_t localExtraLen = zipRead16(&localHeader[28]);
    uint32_t dataOffset = entry.localHeaderOffset + 30 + localFnLen + localExtraLen;

    if (entry.compressionMethod == ZIP_METHOD_STORED) {
      return _extractStored(dataOffset, entry.uncompressedSize, outSize);
    } else if (entry.compressionMethod == ZIP_METHOD_DEFLATED) {
      return _extractDeflated(dataOffset, entry.compressedSize,
                              entry.uncompressedSize, outSize);
    } else {
      MECK_EPUB_LOG("ZipReader: unsupported method %d for %s\n",
                    entry.compressionMethod, entry.filename);
      return nullptr;
    }
  }

  uint8_t* extractByName(const char* filename, uint32_t* outSize) {
    int idx = findEntry(filename);
    if (idx < 0) return nullptr;
    return extractEntry(idx, outSize);
  }

  bool isOpen() const { return _isOpen; }

  void printEntries() const {
    MECK_EPUB_LOG("ZIP contains %d files:\n", _entryCount);
    for (int i = 0; i < _entryCount; i++) {
      const ZipEntry& e = _entries[i];
      MECK_EPUB_LOG("  [%d] %s (%s, %u -> %u bytes)\n", i, e.filename,
                    e.compressionMethod == 0 ? "STORED" : "DEFLATED",
                    (unsigned)e.compressedSize, (unsigned)e.uncompressedSize);
    }
  }

private:
  FILE*     _file;
  ZipEntry* _entries;
  int       _entryCount;
  bool      _isOpen;
  uint32_t  _cdOffset;
  uint32_t  _cdSize;
  uint16_t  _totalEntries;

  // Buffers live in PSRAM (the entry table, decompressor, and large output
  // buffers need the room, and none of these are DMA targets - they are filled
  // by CPU memcpy / inflate). SD reads never land here directly; _readAt stages
  // them through an internal-RAM bounce buffer, so DMA never touches PSRAM.
  void* _allocBuffer(size_t size) {
#if defined(ESP_PLATFORM)
    void* buf = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!buf) buf = malloc(size);
    return buf;
#else
    return malloc(size);
#endif
  }

  // Read exactly len bytes at absolute offset. Returns false on short read.
  bool _readAt(uint32_t offset, uint8_t* dst, uint32_t len) {
    if (!_file) return false;
    if (fseek(_file, (long)offset, SEEK_SET) != 0) return false;
    return fread(dst, 1, len, _file) == len;
  }

  uint8_t* _extractStored(uint32_t dataOffset, uint32_t size, uint32_t* outSize) {
    uint8_t* buf = (uint8_t*)_allocBuffer(size + 1);
    if (!buf) {
      MECK_EPUB_LOG("ZipReader: failed to alloc %u bytes (stored)\n", (unsigned)size);
      return nullptr;
    }
    if (!_readAt(dataOffset, buf, size)) {
      MECK_EPUB_LOG("ZipReader: short read on stored entry\n");
      free(buf);
      return nullptr;
    }
    buf[size] = '\0';
    *outSize = size;
    return buf;
  }

  uint8_t* _extractDeflated(uint32_t dataOffset, uint32_t compSize,
                            uint32_t uncompSize, uint32_t* outSize) {
    uint8_t* compBuf = (uint8_t*)_allocBuffer(compSize);
    if (!compBuf) {
      MECK_EPUB_LOG("ZipReader: failed to alloc %u bytes (compressed)\n", (unsigned)compSize);
      return nullptr;
    }
    uint8_t* outBuf = (uint8_t*)_allocBuffer(uncompSize + 1);
    if (!outBuf) {
      MECK_EPUB_LOG("ZipReader: failed to alloc %u bytes (output)\n", (unsigned)uncompSize);
      free(compBuf);
      return nullptr;
    }

    if (!_readAt(dataOffset, compBuf, compSize)) {
      MECK_EPUB_LOG("ZipReader: failed to read compressed data\n");
      free(compBuf); free(outBuf);
      return nullptr;
    }

    // ZIP entries store raw DEFLATE (no zlib/gzip header). Decompress with
    // zlib in raw mode (negative windowBits). zlib owns its own state and
    // sliding window, so there is no scratch struct for us to allocate.
    z_stream strm;
    memset(&strm, 0, sizeof(strm));
    strm.next_in   = (Bytef*)compBuf;
    strm.avail_in  = compSize;
    strm.next_out  = (Bytef*)outBuf;
    strm.avail_out = uncompSize;
    int zret = inflateInit2(&strm, -15);   // -15 = raw deflate, 32 KB window
    if (zret != Z_OK) {
      MECK_EPUB_LOG("ZipReader: inflateInit2 failed (%d)\n", zret);
      free(compBuf); free(outBuf);
      return nullptr;
    }
    zret = inflate(&strm, Z_FINISH);
    uint32_t outBytes = (uint32_t)strm.total_out;
    inflateEnd(&strm);

    free(compBuf);

    if (zret != Z_STREAM_END) {
      MECK_EPUB_LOG("ZipReader: DEFLATE failed (zlib %d)\n", zret);
      free(outBuf);
      return nullptr;
    }

    outBuf[outBytes] = '\0';
    *outSize = outBytes;
    if (outBytes != uncompSize) {
      MECK_EPUB_LOG("ZipReader: decompressed %u, expected %u\n",
                    (unsigned)outBytes, (unsigned)uncompSize);
    }
    return outBuf;
  }
};