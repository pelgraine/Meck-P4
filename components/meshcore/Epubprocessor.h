#pragma once
// =============================================================================
// Epubprocessor.h - Convert EPUB files to plain text (Meck-P4 port)
//
// Pipeline: EPUB (ZIP) -> container.xml -> OPF spine -> extract chapters ->
//           strip XHTML tags -> concatenated plain text -> cached .txt on SD
//
// The resulting .txt is paged by MeckReader exactly like any other text file.
//
// Port notes (vs the original ESP32-S3 / Arduino version):
//   - POSIX stdio (fopen/fwrite/stat/mkdir) instead of Arduino SD.h/File.
//   - UTF-8 output. The P4 fonts are UTF-8 native, so there is no CP437
//     conversion: accented characters and entities are emitted as UTF-8 and
//     preserved. Typographic punctuation (smart quotes, en/em dashes,
//     ellipsis, guillemets, non-breaking space) is still normalised to ASCII,
//     matching the original reader's intent.
//   - Allocations prefer PSRAM (heap_caps SPIRAM) with a heap fallback.
//
// Dependencies: EpubZipReader.h (ZIP extraction + the MECK_EPUB_LOG macro and,
// on device, esp_heap_caps).
// =============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <sys/stat.h>

#include "EpubZipReader.h"

// Maximum chapters in spine (most novels have 20-80)
#define EPUB_MAX_CHAPTERS 200

// Maximum manifest items we track
#define EPUB_MAX_MANIFEST 256

// Buffer size for OPF/container path strings (these paths are short)
#define EPUB_XML_BUF_SIZE 64

class EpubProcessor {
public:

  // ----------------------------------------------------------
  // Process an EPUB file: extract text and write to the cache .txt.
  //
  // epubPath:  source, e.g. "/sdcard/books/The Iliad.epub"
  // txtPath:   output, e.g. "/sdcard/books/.epub_cache/The Iliad.txt"
  //
  // Returns true if the .txt was written (or already existed).
  // ----------------------------------------------------------
  static bool processToText(const char* epubPath, const char* txtPath) {
    struct stat st;
    if (stat(txtPath, &st) == 0) {
      MECK_EPUB_LOG("EpubProc: '%s' already cached\n", txtPath);
      return true;
    }

    MECK_EPUB_LOG("EpubProc: processing '%s'\n", epubPath);

    EpubZipReader* zip = new EpubZipReader();
    if (!zip) {
      MECK_EPUB_LOG("EpubProc: cannot allocate ZipReader\n");
      return false;
    }
    if (!zip->open(epubPath)) {
      delete zip;
      MECK_EPUB_LOG("EpubProc: cannot parse ZIP structure\n");
      return false;
    }

    // Step 1: find the OPF path from container.xml
    char opfPath[EPUB_XML_BUF_SIZE];
    opfPath[0] = '\0';
    if (!_findOpfPath(zip, opfPath, sizeof(opfPath))) {
      delete zip;
      MECK_EPUB_LOG("EpubProc: cannot find OPF path\n");
      return false;
    }
    MECK_EPUB_LOG("EpubProc: OPF at '%s'\n", opfPath);

    // Content base directory (e.g. "OEBPS/")
    char baseDir[EPUB_XML_BUF_SIZE];
    _getDirectory(opfPath, baseDir, sizeof(baseDir));

    // Step 2: parse OPF for title and spine chapter order
    char title[128];
    title[0] = '\0';
    char** chapterPaths = nullptr;
    int chapterCount = 0;

    if (!_parseOpf(zip, opfPath, baseDir, title, sizeof(title),
                   &chapterPaths, &chapterCount)) {
      delete zip;
      MECK_EPUB_LOG("EpubProc: cannot parse OPF\n");
      return false;
    }
    MECK_EPUB_LOG("EpubProc: title='%s', %d chapters\n", title, chapterCount);

    // Step 3: extract each chapter, strip XHTML, write the output .txt
    FILE* out = fopen(txtPath, "wb");
    if (!out) {
      _freeChapterPaths(chapterPaths, chapterCount);
      delete zip;
      MECK_EPUB_LOG("EpubProc: cannot create '%s'\n", txtPath);
      return false;
    }

    if (title[0]) {
      fputs(title, out);
      fputc('\n', out);
      fputc('\n', out);
    }

    int chaptersWritten = 0;
    uint32_t totalBytes = 0;

    for (int i = 0; i < chapterCount; i++) {
      int entryIdx = zip->findEntry(chapterPaths[i]);
      if (entryIdx < 0) {
        MECK_EPUB_LOG("EpubProc: chapter not found: '%s'\n", chapterPaths[i]);
        continue;
      }

      uint32_t rawSize = 0;
      uint8_t* rawData = zip->extractEntry(entryIdx, &rawSize);
      if (!rawData || rawSize == 0) {
        MECK_EPUB_LOG("EpubProc: failed to extract chapter %d\n", i);
        if (rawData) free(rawData);
        continue;
      }

      uint32_t textLen = 0;
      uint8_t* plainText = _stripXhtml(rawData, rawSize, &textLen);
      free(rawData);

      if (plainText && textLen > 0) {
        fwrite(plainText, 1, textLen, out);
        fputs("\n\n", out);              // chapter separator
        totalBytes += textLen + 2;
        chaptersWritten++;
      }
      if (plainText) free(plainText);
    }

    fflush(out);
    fclose(out);

    _freeChapterPaths(chapterPaths, chapterCount);
    delete zip;

    MECK_EPUB_LOG("EpubProc: done - %d chapters, %u bytes -> '%s'\n",
                  chaptersWritten, (unsigned)totalBytes, txtPath);

    return chaptersWritten > 0;
  }

  // ----------------------------------------------------------
  // Extract just the title from an EPUB (for display in a file list).
  // ----------------------------------------------------------
  static bool getTitle(const char* epubPath, char* titleBuf, int titleBufSize) {
    EpubZipReader* zip = new EpubZipReader();
    if (!zip) return false;
    if (!zip->open(epubPath)) { delete zip; return false; }

    char opfPath[EPUB_XML_BUF_SIZE];
    if (!_findOpfPath(zip, opfPath, sizeof(opfPath))) { delete zip; return false; }

    int opfIdx = zip->findEntry(opfPath);
    if (opfIdx < 0) { delete zip; return false; }

    uint32_t opfSize = 0;
    uint8_t* opfData = zip->extractEntry(opfIdx, &opfSize);
    delete zip;
    if (!opfData) return false;

    bool found = _extractTagContent((const char*)opfData, opfSize,
                                    "dc:title", titleBuf, titleBufSize);
    free(opfData);
    return found;
  }

  // ----------------------------------------------------------
  // Build a cache .txt path from an .epub path.
  // e.g. "/sdcard/books/mybook.epub" -> "/sdcard/books/.epub_cache/mybook.txt"
  // Creates the cache directory if needed.
  // ----------------------------------------------------------
  static void buildCachePath(const char* epubPath, char* cachePath, int cachePathSize) {
    const char* lastSlash = strrchr(epubPath, '/');
    const char* filename = lastSlash ? lastSlash + 1 : epubPath;

    char dir[128];
    if (lastSlash) {
      int dirLen = (int)(lastSlash - epubPath);
      if (dirLen >= (int)sizeof(dir)) dirLen = sizeof(dir) - 1;
      strncpy(dir, epubPath, dirLen);
      dir[dirLen] = '\0';
    } else {
      strcpy(dir, "/sdcard/books");
    }

    char cacheDir[160];
    snprintf(cacheDir, sizeof(cacheDir), "%s/.epub_cache", dir);
    struct stat st;
    if (stat(cacheDir, &st) != 0) {
      mkdir(cacheDir, 0775);
    }

    char baseName[128];
    strncpy(baseName, filename, sizeof(baseName) - 1);
    baseName[sizeof(baseName) - 1] = '\0';
    char* dot = strrchr(baseName, '.');
    if (dot) *dot = '\0';

    snprintf(cachePath, cachePathSize, "%s/%s.txt", cacheDir, baseName);
  }

private:

  // Prefer PSRAM on device; plain heap otherwise (and on host builds).
  static void* _palloc(size_t size) {
#if defined(ESP_PLATFORM)
    void* p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!p) p = malloc(size);
    return p;
#else
    return malloc(size);
#endif
  }

  // ----------------------------------------------------------
  // Parse container.xml to find the OPF file path.
  // ----------------------------------------------------------
  static bool _findOpfPath(EpubZipReader* zip, char* opfPath, int opfPathSize) {
    int idx = zip->findEntry("META-INF/container.xml");
    if (idx < 0) {
      // Fallback: any .opf directly
      idx = zip->findEntryBySuffix(".opf");
      if (idx >= 0) {
        const ZipEntry* e = zip->getEntry(idx);
        strncpy(opfPath, e->filename, opfPathSize - 1);
        opfPath[opfPathSize - 1] = '\0';
        return true;
      }
      return false;
    }

    uint32_t size = 0;
    uint8_t* data = zip->extractEntry(idx, &size);
    if (!data) return false;

    bool found = _extractAttribute((const char*)data, size,
                                   "full-path", opfPath, opfPathSize);
    free(data);
    return found;
  }

  // ----------------------------------------------------------
  // Parse OPF: title + manifest + spine -> chapter paths in reading order.
  // Caller frees chapterPaths with _freeChapterPaths().
  // ----------------------------------------------------------
  static bool _parseOpf(EpubZipReader* zip, const char* opfPath,
                        const char* baseDir, char* title, int titleSize,
                        char*** outChapterPaths, int* outChapterCount) {
    int opfIdx = zip->findEntry(opfPath);
    if (opfIdx < 0) return false;

    uint32_t opfSize = 0;
    uint8_t* opfData = zip->extractEntry(opfIdx, &opfSize);
    if (!opfData) return false;

    const char* xml = (const char*)opfData;

    _extractTagContent(xml, opfSize, "dc:title", title, titleSize);

    struct ManifestItem {
      char id[64];
      char href[128];
      bool isContent;   // media-type contains "html" or "xml"
    };

    ManifestItem* manifest = (ManifestItem*)_palloc(EPUB_MAX_MANIFEST * sizeof(ManifestItem));
    if (!manifest) { free(opfData); return false; }
    int manifestCount = 0;

    const char* manifestStart = _findTag(xml, opfSize, "<manifest");
    const char* manifestEnd = manifestStart ?
        _findTag(manifestStart, opfSize - (manifestStart - xml), "</manifest") : nullptr;
    if (!manifestEnd) manifestEnd = xml + opfSize;

    if (manifestStart) {
      const char* pos = manifestStart;
      while (pos < manifestEnd && manifestCount < EPUB_MAX_MANIFEST) {
        pos = _findTag(pos, manifestEnd - pos, "<item");
        if (!pos || pos >= manifestEnd) break;

        const char* tagEnd = (const char*)memchr(pos, '>', manifestEnd - pos);
        if (!tagEnd) break;
        tagEnd++;

        ManifestItem& item = manifest[manifestCount];
        item.id[0] = '\0';
        item.href[0] = '\0';
        item.isContent = false;

        _extractAttributeFromTag(pos, tagEnd - pos, "id",   item.id,   sizeof(item.id));
        _extractAttributeFromTag(pos, tagEnd - pos, "href", item.href, sizeof(item.href));

        char mediaType[64];
        mediaType[0] = '\0';
        _extractAttributeFromTag(pos, tagEnd - pos, "media-type",
                                 mediaType, sizeof(mediaType));
        item.isContent = (strstr(mediaType, "html") != nullptr ||
                          strstr(mediaType, "xml")  != nullptr);

        if (item.id[0] && item.href[0]) manifestCount++;

        pos = tagEnd;
      }
    }

    MECK_EPUB_LOG("EpubProc: manifest has %d items\n", manifestCount);

    const char* spineStart = _findTag(xml, opfSize, "<spine");
    const char* spineEnd = spineStart ?
        _findTag(spineStart, opfSize - (spineStart - xml), "</spine") : nullptr;
    if (!spineEnd) spineEnd = xml + opfSize;

    char** chapterPaths = (char**)_palloc(EPUB_MAX_CHAPTERS * sizeof(char*));
    if (!chapterPaths) { free(manifest); free(opfData); return false; }
    int chapterCount = 0;

    if (spineStart) {
      const char* pos = spineStart;
      while (pos < spineEnd && chapterCount < EPUB_MAX_CHAPTERS) {
        pos = _findTag(pos, spineEnd - pos, "<itemref");
        if (!pos || pos >= spineEnd) break;

        const char* tagEnd = (const char*)memchr(pos, '>', spineEnd - pos);
        if (!tagEnd) break;
        tagEnd++;

        char idref[64];
        idref[0] = '\0';
        _extractAttributeFromTag(pos, tagEnd - pos, "idref", idref, sizeof(idref));

        if (idref[0]) {
          for (int m = 0; m < manifestCount; m++) {
            if (strcmp(manifest[m].id, idref) == 0 && manifest[m].isContent) {
              int pathLen = (int)(strlen(baseDir) + strlen(manifest[m].href) + 1);
              char* fullPath = (char*)malloc(pathLen);
              if (fullPath) {
                snprintf(fullPath, pathLen, "%s%s", baseDir, manifest[m].href);
                chapterPaths[chapterCount++] = fullPath;
              }
              break;
            }
          }
        }

        pos = tagEnd;
      }
    }

    free(manifest);
    free(opfData);

    *outChapterPaths = chapterPaths;
    *outChapterCount = chapterCount;
    return chapterCount > 0;
  }

  // ----------------------------------------------------------
  // Emit one Unicode codepoint to output, applying typographic normalisation
  // (smart punctuation -> ASCII) and otherwise encoding as UTF-8. Performs the
  // same whitespace collapsing as the main loop. outPos is advanced; the
  // last-was-newline / last-was-space flags are updated.
  //
  // output is sized to inputLen+1; every transform here is non-expanding
  // (entities shrink, raw UTF-8 is copied 1:1), so no bound check is needed.
  // ----------------------------------------------------------
  static void _emitCodepoint(uint8_t* output, uint32_t& outPos, uint32_t cp,
                             bool& lastWasNewline, bool& lastWasSpace) {
    // Typographic punctuation -> ASCII (matches the original reader's intent).
    switch (cp) {
      case 0x2018: case 0x2019: case 0x2032: case 0x2039: case 0x203A:
        cp = '\''; break;                                   // single quotes / primes
      case 0x201C: case 0x201D: case 0x2033: case 0x00AB: case 0x00BB:
        cp = '"';  break;                                   // double quotes / guillemets
      case 0x2010: case 0x2011: case 0x2012: case 0x2013: case 0x2014: case 0x2015:
        cp = '-';  break;                                   // hyphens / dashes
      case 0x2026: cp = '.'; break;                          // ellipsis
      case 0x2022: cp = '*'; break;                          // bullet
      case 0x00A0: cp = ' '; break;                          // non-breaking space
      default: break;
    }

    if (cp == '\n' || cp == '\r') {
      if (!lastWasNewline && outPos > 0) {
        output[outPos++] = '\n';
        lastWasNewline = true;
        lastWasSpace = false;
      }
      return;
    }
    if (cp == ' ' || cp == '\t') {
      if (!lastWasSpace && !lastWasNewline && outPos > 0) {
        output[outPos++] = ' ';
        lastWasSpace = true;
      }
      return;
    }

    if (cp < 0x80) {
      output[outPos++] = (uint8_t)cp;
    } else if (cp <= 0x7FF) {
      output[outPos++] = (uint8_t)(0xC0 | (cp >> 6));
      output[outPos++] = (uint8_t)(0x80 | (cp & 0x3F));
    } else if (cp <= 0xFFFF) {
      output[outPos++] = (uint8_t)(0xE0 | (cp >> 12));
      output[outPos++] = (uint8_t)(0x80 | ((cp >> 6) & 0x3F));
      output[outPos++] = (uint8_t)(0x80 | (cp & 0x3F));
    } else if (cp <= 0x10FFFF) {
      output[outPos++] = (uint8_t)(0xF0 | (cp >> 18));
      output[outPos++] = (uint8_t)(0x80 | ((cp >> 12) & 0x3F));
      output[outPos++] = (uint8_t)(0x80 | ((cp >> 6) & 0x3F));
      output[outPos++] = (uint8_t)(0x80 | (cp & 0x3F));
    } else {
      return;  // out of range, drop
    }
    lastWasNewline = false;
    lastWasSpace = false;
  }

  // ----------------------------------------------------------
  // Strip XHTML/HTML tags from raw content, producing plain UTF-8 text.
  //
  //   - tag removal; block elements (<p>,<br>,<div>,<h1-6>,<li>,<tr>,
  //     <blockquote>,<hr>) become newlines
  //   - <head>/<style>/<script> content skipped
  //   - HTML entities decoded (named, accented-named, numeric dec/hex)
  //   - whitespace/newlines collapsed
  //   - UTF-8 preserved; smart punctuation normalised to ASCII
  //
  // Returns a heap-allocated buffer (caller frees).
  // ----------------------------------------------------------
  static uint8_t* _stripXhtml(const uint8_t* input, uint32_t inputLen, uint32_t* outLen) {
    uint8_t* output = (uint8_t*)_palloc(inputLen + 1);
    if (!output) { *outLen = 0; return nullptr; }

    uint32_t outPos = 0;
    bool inTag = false;
    bool skipContent = false;     // inside <head>/<style>/<script>
    char tagName[32];
    int  tagNamePos = 0;
    bool tagNameDone = false;
    bool isClosingTag = false;
    bool lastWasNewline = false;
    bool lastWasSpace = false;

    const uint8_t* inputEnd = input + inputLen;

    // Skip to <body> if present.
    const uint8_t* start = input;
    const char* bodyStart = _findTagCI((const char*)input, inputLen, "<body");
    if (bodyStart) {
      const char* bodyTagEnd = (const char*)memchr(bodyStart, '>',
          inputEnd - (const uint8_t*)bodyStart);
      if (bodyTagEnd) start = (const uint8_t*)(bodyTagEnd + 1);
    }
    const uint8_t* end = inputEnd;

    for (const uint8_t* p = start; p < end; p++) {
      char c = (char)*p;

      if (inTag) {
        if (!tagNameDone) {
          if (tagNamePos == 0 && c == '/') { isClosingTag = true; continue; }
          if (c == '>' || c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '/') {
            tagName[tagNamePos] = '\0';
            tagNameDone = true;
          } else if (tagNamePos < (int)sizeof(tagName) - 1) {
            tagName[tagNamePos++] = (c >= 'A' && c <= 'Z') ? (c + 32) : c;
          }
        }

        if (c == '>') {
          inTag = false;

          if (!isClosingTag) {
            if (strcmp(tagName, "head")  == 0 ||
                strcmp(tagName, "style") == 0 ||
                strcmp(tagName, "script") == 0) {
              skipContent = true;
            }
          } else {
            if (strcmp(tagName, "head")  == 0 ||
                strcmp(tagName, "style") == 0 ||
                strcmp(tagName, "script") == 0) {
              skipContent = false;
            }
          }

          if (!skipContent) {
            if (strcmp(tagName, "p") == 0 || strcmp(tagName, "div") == 0 ||
                strcmp(tagName, "br") == 0 || strcmp(tagName, "h1") == 0 ||
                strcmp(tagName, "h2") == 0 || strcmp(tagName, "h3") == 0 ||
                strcmp(tagName, "h4") == 0 || strcmp(tagName, "h5") == 0 ||
                strcmp(tagName, "h6") == 0 || strcmp(tagName, "li") == 0 ||
                strcmp(tagName, "tr") == 0 || strcmp(tagName, "blockquote") == 0 ||
                strcmp(tagName, "hr") == 0) {
              if (outPos > 0 && !lastWasNewline) {
                output[outPos++] = '\n';
                lastWasNewline = true;
                lastWasSpace = false;
              }
            }
          }
          continue;
        }
        continue;
      }

      // Not in a tag
      if (c == '<') {
        inTag = true;
        tagNamePos = 0;
        tagNameDone = false;
        isClosingTag = false;
        continue;
      }

      if (skipContent) continue;

      // HTML entity -> codepoint
      if (c == '&') {
        const uint8_t* np = p;
        uint32_t cp = _decodeEntity(p, end, &np);
        p = np;
        _emitCodepoint(output, outPos, cp, lastWasNewline, lastWasSpace);
        continue;
      }

      // Raw UTF-8 multi-byte -> codepoint (preserved)
      if ((uint8_t)c >= 0xC0) {
        uint32_t cp = 0;
        int extra = 0;
        if (((uint8_t)c & 0xE0) == 0xC0)      { cp = (uint8_t)c & 0x1F; extra = 1; }
        else if (((uint8_t)c & 0xF0) == 0xE0) { cp = (uint8_t)c & 0x0F; extra = 2; }
        else if (((uint8_t)c & 0xF8) == 0xF0) { cp = (uint8_t)c & 0x07; extra = 3; }

        bool valid = (extra > 0);
        for (int b = 0; b < extra; b++) {
          if (p + 1 + b >= end) { valid = false; break; }
          uint8_t cb = *(p + 1 + b);
          if ((cb & 0xC0) != 0x80) { valid = false; break; }
          cp = (cp << 6) | (cb & 0x3F);
        }
        if (valid) {
          p += extra;
          _emitCodepoint(output, outPos, cp, lastWasNewline, lastWasSpace);
        }
        continue;   // malformed: skip the lead byte
      } else if ((uint8_t)c >= 0x80) {
        continue;   // stray continuation byte
      }

      // Plain ASCII
      _emitCodepoint(output, outPos, (uint32_t)(uint8_t)c, lastWasNewline, lastWasSpace);
    }

    // Trim trailing whitespace
    while (outPos > 0 && (output[outPos - 1] == '\n' || output[outPos - 1] == ' ')) {
      outPos--;
    }

    output[outPos] = '\0';
    *outLen = outPos;
    return output;
  }

  // ----------------------------------------------------------
  // Decode an HTML entity starting at '&'. Sets *outPos to the last byte
  // consumed (the ';', or the '&' itself if it isn't an entity). Returns a
  // Unicode codepoint; the caller emits it as UTF-8 (or normalises it).
  // ----------------------------------------------------------
  static uint32_t _decodeEntity(const uint8_t* p, const uint8_t* end,
                                const uint8_t** outPos) {
    const uint8_t* semi = p + 1;
    const int maxLen = 12;
    while (semi < end && semi < p + maxLen && *semi != ';') semi++;

    if (semi >= end || *semi != ';') {
      *outPos = p;          // not an entity; emit a literal '&'
      return '&';
    }

    int entityLen = (int)(semi - p - 1);   // chars between & and ;
    const char* e = (const char*)(p + 1);
    *outPos = semi;                         // consume through ';'

    // Core named entities
    if (entityLen == 3 && strncmp(e, "amp",  3) == 0) return '&';
    if (entityLen == 2 && strncmp(e, "lt",   2) == 0) return '<';
    if (entityLen == 2 && strncmp(e, "gt",   2) == 0) return '>';
    if (entityLen == 4 && strncmp(e, "quot", 4) == 0) return '"';
    if (entityLen == 4 && strncmp(e, "apos", 4) == 0) return '\'';
    if (entityLen == 4 && strncmp(e, "nbsp", 4) == 0) return 0x00A0;  // -> ' '
    if (entityLen == 5 && strncmp(e, "mdash", 5) == 0) return 0x2014;
    if (entityLen == 5 && strncmp(e, "ndash", 5) == 0) return 0x2013;
    if (entityLen == 6 && strncmp(e, "hellip",6) == 0) return 0x2026;
    if (entityLen == 5 && strncmp(e, "lsquo", 5) == 0) return 0x2018;
    if (entityLen == 5 && strncmp(e, "rsquo", 5) == 0) return 0x2019;
    if (entityLen == 5 && strncmp(e, "ldquo", 5) == 0) return 0x201C;
    if (entityLen == 5 && strncmp(e, "rdquo", 5) == 0) return 0x201D;

    // Accented named entities -> Unicode codepoints (emitted as UTF-8)
    if (entityLen == 6 && strncmp(e, "eacute", 6) == 0) return 0x00E9;
    if (entityLen == 6 && strncmp(e, "egrave", 6) == 0) return 0x00E8;
    if (entityLen == 5 && strncmp(e, "ecirc",  5) == 0) return 0x00EA;
    if (entityLen == 4 && strncmp(e, "euml",   4) == 0) return 0x00EB;
    if (entityLen == 6 && strncmp(e, "agrave", 6) == 0) return 0x00E0;
    if (entityLen == 6 && strncmp(e, "aacute", 6) == 0) return 0x00E1;
    if (entityLen == 5 && strncmp(e, "acirc",  5) == 0) return 0x00E2;
    if (entityLen == 4 && strncmp(e, "auml",   4) == 0) return 0x00E4;
    if (entityLen == 6 && strncmp(e, "ccedil", 6) == 0) return 0x00E7;
    if (entityLen == 6 && strncmp(e, "iacute", 6) == 0) return 0x00ED;
    if (entityLen == 5 && strncmp(e, "icirc",  5) == 0) return 0x00EE;
    if (entityLen == 4 && strncmp(e, "iuml",   4) == 0) return 0x00EF;
    if (entityLen == 6 && strncmp(e, "igrave", 6) == 0) return 0x00EC;
    if (entityLen == 6 && strncmp(e, "oacute", 6) == 0) return 0x00F3;
    if (entityLen == 5 && strncmp(e, "ocirc",  5) == 0) return 0x00F4;
    if (entityLen == 4 && strncmp(e, "ouml",   4) == 0) return 0x00F6;
    if (entityLen == 6 && strncmp(e, "ograve", 6) == 0) return 0x00F2;
    if (entityLen == 6 && strncmp(e, "uacute", 6) == 0) return 0x00FA;
    if (entityLen == 5 && strncmp(e, "ucirc",  5) == 0) return 0x00FB;
    if (entityLen == 4 && strncmp(e, "uuml",   4) == 0) return 0x00FC;
    if (entityLen == 6 && strncmp(e, "ugrave", 6) == 0) return 0x00F9;
    if (entityLen == 6 && strncmp(e, "ntilde", 6) == 0) return 0x00F1;
    if (entityLen == 6 && strncmp(e, "Eacute", 6) == 0) return 0x00C9;
    if (entityLen == 6 && strncmp(e, "Ccedil", 6) == 0) return 0x00C7;
    if (entityLen == 6 && strncmp(e, "Ntilde", 6) == 0) return 0x00D1;
    if (entityLen == 4 && strncmp(e, "Auml",   4) == 0) return 0x00C4;
    if (entityLen == 4 && strncmp(e, "Ouml",   4) == 0) return 0x00D6;
    if (entityLen == 4 && strncmp(e, "Uuml",   4) == 0) return 0x00DC;
    if (entityLen == 5 && strncmp(e, "szlig",  5) == 0) return 0x00DF;

    // Numeric: &#NNN; or &#xHH;
    if (entityLen >= 2 && e[0] == '#') {
      uint32_t cp = 0;
      if (e[1] == 'x' || e[1] == 'X') {
        for (int i = 2; i < entityLen; i++) {
          char ch = e[i];
          if      (ch >= '0' && ch <= '9') cp = cp * 16 + (ch - '0');
          else if (ch >= 'a' && ch <= 'f') cp = cp * 16 + (ch - 'a' + 10);
          else if (ch >= 'A' && ch <= 'F') cp = cp * 16 + (ch - 'A' + 10);
        }
      } else {
        for (int i = 1; i < entityLen; i++) {
          char ch = e[i];
          if (ch >= '0' && ch <= '9') cp = cp * 10 + (ch - '0');
        }
      }
      if (cp == 0) return ' ';
      return cp;   // emitted as UTF-8 (or normalised) by the caller
    }

    return ' ';    // unknown entity -> space
  }

  // ----------------------------------------------------------
  // Find a tag (case-sensitive, e.g. "<manifest"). Pointer to '<' or nullptr.
  // ----------------------------------------------------------
  static const char* _findTag(const char* data, int dataLen, const char* tag) {
    int tagLen = (int)strlen(tag);
    const char* end = data + dataLen - tagLen;
    for (const char* p = data; p <= end; p++)
      if (memcmp(p, tag, tagLen) == 0) return p;
    return nullptr;
  }

  // Case-insensitive tag find (for <body>/<BODY>).
  static const char* _findTagCI(const char* data, int dataLen, const char* tag) {
    int tagLen = (int)strlen(tag);
    const char* end = data + dataLen - tagLen;
    for (const char* p = data; p <= end; p++)
      if (strncasecmp(p, tag, tagLen) == 0) return p;
    return nullptr;
  }

  // Extract attr="value" (or attr='value') from a region of XML.
  static bool _extractAttribute(const char* data, int dataLen,
                                const char* attrName, char* outBuf, int outBufSize) {
    int nameLen = (int)strlen(attrName);
    const char* end = data + dataLen;
    for (const char* p = data; p < end - nameLen - 2; p++) {
      if (strncmp(p, attrName, nameLen) == 0 && p[nameLen] == '=') {
        p += nameLen + 1;
        char quote = *p;
        if (quote != '"' && quote != '\'') continue;
        p++;
        const char* valEnd = (const char*)memchr(p, quote, end - p);
        if (!valEnd) continue;
        int valLen = (int)(valEnd - p);
        if (valLen >= outBufSize) valLen = outBufSize - 1;
        memcpy(outBuf, p, valLen);
        outBuf[valLen] = '\0';
        return true;
      }
    }
    return false;
  }

  static bool _extractAttributeFromTag(const char* tag, int tagLen,
                                       const char* attrName,
                                       char* outBuf, int outBufSize) {
    return _extractAttribute(tag, tagLen, attrName, outBuf, outBufSize);
  }

  // Extract text between <tagName>...</tagName> (simple cases like <dc:title>).
  static bool _extractTagContent(const char* data, int dataLen,
                                 const char* tagName, char* outBuf, int outBufSize) {
    char openTag[64];
    snprintf(openTag, sizeof(openTag), "<%s", tagName);

    const char* start = _findTag(data, dataLen, openTag);
    if (!start) return false;

    const char* end = data + dataLen;
    const char* contentStart = (const char*)memchr(start, '>', end - start);
    if (!contentStart) return false;
    contentStart++;

    char closeTag[64];
    snprintf(closeTag, sizeof(closeTag), "</%s>", tagName);
    const char* contentEnd = _findTag(contentStart, end - contentStart, closeTag);
    if (!contentEnd) return false;

    int len = (int)(contentEnd - contentStart);
    if (len >= outBufSize) len = outBufSize - 1;
    memcpy(outBuf, contentStart, len);
    outBuf[len] = '\0';
    return true;
  }

  // Directory portion of a path: "OEBPS/content.opf" -> "OEBPS/".
  static void _getDirectory(const char* path, char* dirBuf, int dirBufSize) {
    const char* lastSlash = strrchr(path, '/');
    if (lastSlash) {
      int len = (int)(lastSlash - path + 1);
      if (len >= dirBufSize) len = dirBufSize - 1;
      memcpy(dirBuf, path, len);
      dirBuf[len] = '\0';
    } else {
      dirBuf[0] = '\0';
    }
  }

  static void _freeChapterPaths(char** paths, int count) {
    if (paths) {
      for (int i = 0; i < count; i++)
        if (paths[i]) free(paths[i]);
      free(paths);
    }
  }
};
