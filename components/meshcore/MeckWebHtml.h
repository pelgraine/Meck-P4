#ifndef MECK_WEB_HTML_H
#define MECK_WEB_HTML_H

// HTML reader-mode parser, lifted verbatim from upstream Meck Webreaderscreen.h
// (the parseHtml cluster: structs, tag tables, helpers, and parseHtml itself).
// Converts an HTML body into readable text in a single pass, extracting links
// and forms as byproducts. Framework-independent: plain char buffers, no
// Arduino / WiFi / LVGL / display dependencies.

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cctype>

#define WEB_MAX_URL_LEN     256   // from upstream Webreaderscreen.h

// --- Meck-P4 adaptation -----------------------------------------------------
// Upstream decodeHtmlEntity maps numeric character references >= U+0080 to a
// single CP437 byte via unicodeToCP437(), a helper from the T-Deck e-ink font
// layer that does not exist on the P4 (LVGL uses UTF-8 fonts). Per the agreed
// minimal interim this returns 0, so the lifted decoder falls back to '?' for
// those references. Proper UTF-8 handling is a later scoped edit.
static inline uint8_t unicodeToCP437(uint32_t cp) {
  (void)cp;
  return 0;
}
// ----------------------------------------------------------------------------

// ============================================================================
// Link structure - stores extracted hyperlinks
// ============================================================================
struct WebLink {
  char url[WEB_MAX_URL_LEN];
  char text[48];   // Display text for the link (truncated)
};

// ============================================================================
// Form structures - stores parsed HTML forms for user interaction
// ============================================================================
#define WEB_MAX_FORMS        4
#define WEB_MAX_FORM_FIELDS  16
#define WEB_MAX_FIELD_VALUE  128

struct WebFormField {
  char name[64];                    // name= attribute
  char value[WEB_MAX_FIELD_VALUE];  // Current/default value
  char label[48];                   // Display label (from <label> or placeholder)
  char type;                        // 't'=text, 'p'=password, 'h'=hidden, 's'=submit, 'c'=checkbox
};

struct WebForm {
  char action[WEB_MAX_URL_LEN];    // Form action URL
  bool isPost;                      // true=POST, false=GET
  WebFormField fields[WEB_MAX_FORM_FIELDS];
  int fieldCount;
  int textFieldCount;               // Visible (non-hidden) field count
  int formMarker;                   // Index in text where form marker was placed
};

// ============================================================================
// HTML Parser - minimal tag-stripping reader-mode extractor
// ============================================================================

// Tags whose content should be completely removed (not just the tag itself)
// Note: form/input/button/label are NOT skipped — they're parsed for form support.
// header is NOT skipped — it contains login/navigation links on most sites.
// nav IS skipped — its links are redundant with header and add clutter.
static const char* HTML_SKIP_TAGS[] = {
  "script", "style", "nav", "footer", "aside",
  "iframe", "noscript", "svg", "select", "textarea", nullptr
};

// Tags that produce a paragraph break
static const char* HTML_BLOCK_TAGS[] = {
  "div", "br", "tr", "blockquote", "article", "section", "figcaption",
  "ul", "ol", "dl",
  nullptr
};

// Tags that get paragraph-style double breaks
static const char* HTML_PARA_TAGS[] = {
  "p", nullptr
};

inline bool tagNameEquals(const char* tag, int tagLen, const char* name) {
  int nameLen = strlen(name);
  if (tagLen != nameLen) return false;
  for (int i = 0; i < nameLen; i++) {
    char c = tag[i];
    if (c >= 'A' && c <= 'Z') c += 32; // tolower
    if (c != name[i]) return false;
  }
  return true;
}

inline bool isSkipTag(const char* tag, int tagLen) {
  for (int i = 0; HTML_SKIP_TAGS[i]; i++) {
    if (tagNameEquals(tag, tagLen, HTML_SKIP_TAGS[i])) return true;
  }
  return false;
}

inline bool isBlockTag(const char* tag, int tagLen) {
  for (int i = 0; HTML_BLOCK_TAGS[i]; i++) {
    if (tagNameEquals(tag, tagLen, HTML_BLOCK_TAGS[i])) return true;
  }
  return false;
}

// Decode common HTML entities: &amp; &lt; &gt; &quot; &apos; &nbsp; &#NNN; &#xHH;
inline int decodeHtmlEntity(const char* src, int srcLen, int pos, char* outChar) {
  if (pos >= srcLen || src[pos] != '&') return 0;

  // Find the semicolon
  int end = pos + 1;
  int maxSearch = pos + 10; // entities are short
  if (maxSearch > srcLen) maxSearch = srcLen;
  while (end < maxSearch && src[end] != ';' && src[end] != '&' && src[end] != '<') end++;
  if (end >= maxSearch || src[end] != ';') return 0;

  int entLen = end - pos - 1; // length between & and ;
  const char* ent = src + pos + 1;

  if (entLen == 3 && memcmp(ent, "amp", 3) == 0) { *outChar = '&'; return end - pos + 1; }
  if (entLen == 2 && memcmp(ent, "lt", 2) == 0) { *outChar = '<'; return end - pos + 1; }
  if (entLen == 2 && memcmp(ent, "gt", 2) == 0) { *outChar = '>'; return end - pos + 1; }
  if (entLen == 4 && memcmp(ent, "quot", 4) == 0) { *outChar = '"'; return end - pos + 1; }
  if (entLen == 4 && memcmp(ent, "apos", 4) == 0) { *outChar = '\''; return end - pos + 1; }
  if (entLen == 4 && memcmp(ent, "nbsp", 4) == 0) { *outChar = ' '; return end - pos + 1; }
  if (entLen == 5 && memcmp(ent, "mdash", 5) == 0) { *outChar = '-'; return end - pos + 1; }
  if (entLen == 5 && memcmp(ent, "ndash", 5) == 0) { *outChar = '-'; return end - pos + 1; }
  if (entLen == 5 && memcmp(ent, "lsquo", 5) == 0) { *outChar = '\''; return end - pos + 1; }
  if (entLen == 5 && memcmp(ent, "rsquo", 5) == 0) { *outChar = '\''; return end - pos + 1; }
  if (entLen == 5 && memcmp(ent, "ldquo", 5) == 0) { *outChar = '"'; return end - pos + 1; }
  if (entLen == 5 && memcmp(ent, "rdquo", 5) == 0) { *outChar = '"'; return end - pos + 1; }
  if (entLen == 5 && memcmp(ent, "laquo", 5) == 0) { *outChar = '<'; return end - pos + 1; }
  if (entLen == 5 && memcmp(ent, "raquo", 5) == 0) { *outChar = '>'; return end - pos + 1; }
  if (entLen == 5 && memcmp(ent, "trade", 5) == 0) { *outChar = ' '; return end - pos + 1; }
  if (entLen == 4 && memcmp(ent, "copy", 4) == 0) { *outChar = 'c'; return end - pos + 1; }
  if (entLen == 4 && memcmp(ent, "bull", 4) == 0) { *outChar = '*'; return end - pos + 1; }
  // hellip handled specially in caller (outputs "..." multi-char)

  // Numeric: &#NNN; or &#xHH;
  if (entLen >= 2 && ent[0] == '#') {
    uint32_t cp = 0;
    if (ent[1] == 'x' || ent[1] == 'X') {
      for (int i = 2; i < entLen; i++) {
        char c = ent[i];
        if (c >= '0' && c <= '9') cp = cp * 16 + (c - '0');
        else if (c >= 'a' && c <= 'f') cp = cp * 16 + (c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') cp = cp * 16 + (c - 'A' + 10);
        else break;
      }
    } else {
      for (int i = 1; i < entLen; i++) {
        if (ent[i] >= '0' && ent[i] <= '9') cp = cp * 10 + (ent[i] - '0');
        else break;
      }
    }
    if (cp < 128) {
      *outChar = (char)cp;
    } else {
      // Try CP437 mapping for common chars
      uint8_t glyph = unicodeToCP437(cp);
      *outChar = glyph ? (char)glyph : '?';
    }
    return end - pos + 1;
  }

  return 0; // Unknown entity
}

// Extract the tag name from inside a < > bracket.
// Returns length of tag name, and sets isClosing if it starts with /
inline int extractTagName(const char* inside, int insideLen, bool& isClosing) {
  int i = 0;
  isClosing = false;
  while (i < insideLen && (inside[i] == ' ' || inside[i] == '\t')) i++;
  if (i < insideLen && inside[i] == '/') { isClosing = true; i++; }
  int start = i;
  while (i < insideLen && inside[i] != ' ' && inside[i] != '/' &&
         inside[i] != '>' && inside[i] != '\t' && inside[i] != '\n') i++;
  return i - start; // tagName starts at inside+start, length is return value
}

// Extract href attribute value from inside an <a ...> tag
inline bool extractHref(const char* tagContent, int tagLen, char* hrefOut, int hrefMax) {
  // Search for href= (case insensitive)
  for (int i = 0; i < tagLen - 5; i++) {
    char c0 = tagContent[i]; if (c0 >= 'A' && c0 <= 'Z') c0 += 32;
    char c1 = tagContent[i+1]; if (c1 >= 'A' && c1 <= 'Z') c1 += 32;
    char c2 = tagContent[i+2]; if (c2 >= 'A' && c2 <= 'Z') c2 += 32;
    char c3 = tagContent[i+3]; if (c3 >= 'A' && c3 <= 'Z') c3 += 32;

    if (c0 == 'h' && c1 == 'r' && c2 == 'e' && c3 == 'f') {
      int j = i + 4;
      while (j < tagLen && tagContent[j] == ' ') j++;
      if (j < tagLen && tagContent[j] == '=') {
        j++;
        while (j < tagLen && tagContent[j] == ' ') j++;
        char quote = 0;
        if (j < tagLen && (tagContent[j] == '"' || tagContent[j] == '\'')) {
          quote = tagContent[j]; j++;
        }
        int start = j;
        if (quote) {
          while (j < tagLen && tagContent[j] != quote) j++;
        } else {
          while (j < tagLen && tagContent[j] != ' ' && tagContent[j] != '>') j++;
        }
        int len = j - start;
        if (len >= hrefMax) len = hrefMax - 1;
        memcpy(hrefOut, tagContent + start, len);
        hrefOut[len] = '\0';
        return len > 0;
      }
    }
  }
  return false;
}

// Encode spaces in URL as %20 (in-place, buffer must have room)
inline void encodeUrlSpaces(char* url, int maxLen) {
  int len = strlen(url);
  // Count spaces to check if result fits
  int spaces = 0;
  for (int i = 0; i < len; i++) {
    if (url[i] == ' ') spaces++;
  }
  int newLen = len + spaces * 2;  // Each space becomes 3 chars (%20) instead of 1
  if (newLen >= maxLen) return;    // Won't fit, leave as-is

  // Work backwards to encode in-place
  url[newLen] = '\0';
  int dst = newLen - 1;
  for (int src = len - 1; src >= 0; src--) {
    if (url[src] == ' ') {
      url[dst--] = '0';
      url[dst--] = '2';
      url[dst--] = '%';
    } else {
      url[dst--] = url[src];
    }
  }
}

// Resolve a relative URL against a base URL
inline void resolveUrl(const char* base, const char* relative, char* out, int outMax) {
  if (!relative || !relative[0]) {
    strncpy(out, base, outMax - 1);
    out[outMax - 1] = '\0';
    return;
  }

  // Already absolute
  if (strncmp(relative, "http://", 7) == 0 || strncmp(relative, "https://", 8) == 0) {
    strncpy(out, relative, outMax - 1);
    out[outMax - 1] = '\0';
    return;
  }

  // Protocol-relative //example.com/...
  if (relative[0] == '/' && relative[1] == '/') {
    snprintf(out, outMax, "https:%s", relative);
    return;
  }

  // Find scheme + host from base
  const char* schemeEnd = strstr(base, "://");
  if (!schemeEnd) {
    strncpy(out, relative, outMax - 1);
    out[outMax - 1] = '\0';
    return;
  }
  const char* hostStart = schemeEnd + 3;
  const char* pathStart = strchr(hostStart, '/');

  if (relative[0] == '/') {
    // Absolute path
    int hostLen = pathStart ? (pathStart - base) : strlen(base);
    snprintf(out, outMax, "%.*s%s", hostLen, base, relative);
  } else {
    // Relative path - append to base directory
    if (pathStart) {
      const char* lastSlash = strrchr(pathStart, '/');
      int baseLen = lastSlash ? (lastSlash - base + 1) : strlen(base);
      snprintf(out, outMax, "%.*s%s", baseLen, base, relative);
    } else {
      snprintf(out, outMax, "%s/%s", base, relative);
    }
  }
}


// Extract a named attribute value from inside a tag
inline bool extractAttr(const char* tag, int tagLen, const char* attrName,
                        char* out, int outMax) {
  int nameLen = strlen(attrName);
  for (int i = 0; i < tagLen - nameLen; i++) {
    bool match = true;
    for (int j = 0; j < nameLen && match; j++) {
      char c = tag[i + j];
      if (c >= 'A' && c <= 'Z') c += 32;
      if (c != attrName[j]) match = false;
    }
    if (!match) continue;
    int j = i + nameLen;
    while (j < tagLen && tag[j] == ' ') j++;
    if (j >= tagLen || tag[j] != '=') continue;
    j++;
    while (j < tagLen && tag[j] == ' ') j++;
    char quote = 0;
    if (j < tagLen && (tag[j] == '"' || tag[j] == '\'')) { quote = tag[j]; j++; }
    int start = j;
    if (quote) { while (j < tagLen && tag[j] != quote) j++; }
    else { while (j < tagLen && tag[j] != ' ' && tag[j] != '>' && tag[j] != '/') j++; }
    int len = j - start;
    if (len >= outMax) len = outMax - 1;
    memcpy(out, tag + start, len);
    out[len] = '\0';
    return len > 0;
  }
  return false;
}

// ============================================================================
// Main HTML-to-text parser
//
// Strips HTML tags, extracts text content, collects links and forms.
// Outputs clean text with paragraph breaks as double newlines.
// Links are inserted as [N] markers in the text flow.
// Forms are inserted as {FN} markers with visible fields.
// ============================================================================

struct ParseResult {
  int textLen;
  int linkCount;
  int formCount;
  bool truncated;   // set by the fetch layer: capture hit the byte cap (page too large)
};

inline ParseResult parseHtml(const char* html, int htmlLen,
                             char* textOut, int textMax,
                             WebLink* links, int maxLinks,
                             WebForm* forms, int maxForms,
                             const char* baseUrl) {
  ParseResult result = {0, 0, 0, false};
  int ti = 0;       // text output index
  int hi = 0;       // html input index
  int skipDepth = 0; // depth inside skip tags
  bool inTag = false;
  bool inAnchor = false;
  int anchorTextStart = 0;
  char currentHref[WEB_MAX_URL_LEN] = {0};
  bool lastWasBreak = true; // Track if we just emitted a paragraph break (avoid doubles)
  bool lastWasSpace = false;

  // Form parsing state
  bool inForm = false;
  int currentForm = -1;
  char pendingLabel[48] = {0};
  bool inLabel = false;
  int labelTextStart = 0;

  // Find <body> tag to skip <head> section
  for (int i = 0; i < htmlLen - 6; i++) {
    char c = html[i];
    if (c == '<') {
      // Check for <body
      char b1 = html[i+1]; if (b1 >= 'A' && b1 <= 'Z') b1 += 32;
      char b2 = html[i+2]; if (b2 >= 'A' && b2 <= 'Z') b2 += 32;
      char b3 = html[i+3]; if (b3 >= 'A' && b3 <= 'Z') b3 += 32;
      char b4 = html[i+4]; if (b4 >= 'A' && b4 <= 'Z') b4 += 32;
      if (b1 == 'b' && b2 == 'o' && b3 == 'd' && b4 == 'y') {
        // Skip to after the >
        while (i < htmlLen && html[i] != '>') i++;
        hi = i + 1;
        break;
      }
    }
  }

  while (hi < htmlLen && ti < textMax - 4) {
    char c = html[hi];

    if (c == '<') {
      // Start of a tag
      int tagStart = hi + 1;
      int tagEnd = tagStart;
      // Find closing >
      while (tagEnd < htmlLen && html[tagEnd] != '>') tagEnd++;
      if (tagEnd >= htmlLen) break;

      int insideLen = tagEnd - tagStart;
      const char* inside = html + tagStart;

      // Extract tag name
      bool isClosing = false;
      int nameStart = 0;
      while (nameStart < insideLen && (inside[nameStart] == ' ' || inside[nameStart] == '\t'))
        nameStart++;
      if (nameStart < insideLen && inside[nameStart] == '/') {
        isClosing = true;
        nameStart++;
      }
      int nameEnd = nameStart;
      while (nameEnd < insideLen && inside[nameEnd] != ' ' && inside[nameEnd] != '/' &&
             inside[nameEnd] != '>' && inside[nameEnd] != '\t' && inside[nameEnd] != '\n')
        nameEnd++;

      const char* tagName = inside + nameStart;
      int tagNameLen = nameEnd - nameStart;

      // Check for skip tags (script, style, nav, etc.)
      if (isSkipTag(tagName, tagNameLen)) {
        if (isClosing) {
          if (skipDepth > 0) skipDepth--;
        } else {
          // Check if self-closing
          bool selfClose = (insideLen > 0 && inside[insideLen - 1] == '/');
          if (!selfClose) skipDepth++;
        }
        hi = tagEnd + 1;
        continue;
      }

      if (skipDepth > 0) {
        hi = tagEnd + 1;
        continue;
      }

      // Handle paragraph tags - emit double break
      bool isPara = false;
      for (int pt = 0; HTML_PARA_TAGS[pt]; pt++) {
        if (tagNameEquals(tagName, tagNameLen, HTML_PARA_TAGS[pt])) { isPara = true; break; }
      }
      if (isPara) {
        if (!lastWasBreak && ti > 0) {
          textOut[ti++] = '\n';
          if (ti < textMax - 2) textOut[ti++] = '\n';
          lastWasBreak = true;
          lastWasSpace = false;
        }
      }

      // Handle block tags - emit single break
      if (!isPara && isBlockTag(tagName, tagNameLen)) {
        if (!lastWasBreak && ti > 0) {
          textOut[ti++] = '\n';
          lastWasBreak = true;
          lastWasSpace = false;
        }
      }

      // Handle <h1>-<h6> opening: ensure line break before heading
      if (!isClosing && tagNameLen == 2 && tagName[0] == 'h' &&
          tagName[1] >= '1' && tagName[1] <= '6') {
        if (ti < textMax - 12) {
          if (!lastWasBreak && ti > 0) {
            textOut[ti++] = '\n';
          }
          // Separator line before h4 headings (work titles on listing pages)
          if (tagName[1] == '4' && ti > 1) {
            const char* sep = "--------";
            for (int s = 0; sep[s]; s++) textOut[ti++] = sep[s];
            textOut[ti++] = '\n';
          }
          // Double break before h1/h2 for visual separation
          if (tagName[1] <= '2' && ti > 0 && ti < textMax - 1) {
            textOut[ti++] = '\n';
          }
          // Wrap h1-h4 headings with * markers to make them stand out
          if (tagName[1] <= '4' && ti < textMax - 2) {
            textOut[ti++] = '*';
            textOut[ti++] = ' ';
          }
          lastWasBreak = false;
          lastWasSpace = true;
        }
      }

      // Handle closing </h1>-</h6>: closing marker + line break
      if (isClosing && tagNameLen == 2 && tagName[0] == 'h' &&
          tagName[1] >= '1' && tagName[1] <= '6') {
        if (ti < textMax - 2) {
          // Trim trailing space before closing marker
          if (ti > 0 && textOut[ti-1] == ' ') ti--;
          if (tagName[1] <= '4') {
            textOut[ti++] = ' ';
            textOut[ti++] = '*';
          }
          textOut[ti++] = '\n';
          lastWasBreak = true;
          lastWasSpace = false;
        }
      }

      // Handle <a href="..."> - collect link
      if (!isClosing && tagNameLen == 1 && (tagName[0] == 'a' || tagName[0] == 'A')) {
        char href[WEB_MAX_URL_LEN] = {0};
        if (extractHref(inside, insideLen, href, WEB_MAX_URL_LEN)) {
          // Skip javascript:, mailto:, and # fragment-only links
          if (strncmp(href, "javascript:", 11) != 0 &&
              strncmp(href, "mailto:", 7) != 0 &&
              href[0] != '#') {
            resolveUrl(baseUrl, href, currentHref, WEB_MAX_URL_LEN);
            inAnchor = true;
            anchorTextStart = ti;
          }
        }
      }

      // Handle </a> - finalize link
      if (isClosing && tagNameLen == 1 && (tagName[0] == 'a' || tagName[0] == 'A')) {
        if (inAnchor && currentHref[0] && result.linkCount < maxLinks) {
          WebLink& link = links[result.linkCount];
          strncpy(link.url, currentHref, WEB_MAX_URL_LEN - 1);
          link.url[WEB_MAX_URL_LEN - 1] = '\0';

          // Extract link display text from what was accumulated
          int linkTextLen = ti - anchorTextStart;
          if (linkTextLen > (int)sizeof(link.text) - 1)
            linkTextLen = sizeof(link.text) - 1;
          if (linkTextLen > 0) {
            memcpy(link.text, textOut + anchorTextStart, linkTextLen);
          }
          link.text[linkTextLen] = '\0';

          // Append link number marker: [N]
          result.linkCount++;
          if (ti < textMax - 8) {
            int n = result.linkCount;
            textOut[ti++] = '[';
            if (n >= 100) textOut[ti++] = '0' + (n / 100);
            if (n >= 10) textOut[ti++] = '0' + ((n / 10) % 10);
            textOut[ti++] = '0' + (n % 10);
            textOut[ti++] = ']';
            lastWasSpace = false;
            lastWasBreak = false;
          }
        }
        inAnchor = false;
        currentHref[0] = '\0';
      }

      // Handle <li> - always comma-separated inline flow
      if (!isClosing && tagNameLen == 2 && tagName[0] == 'l' && tagName[1] == 'i') {
        if (ti < textMax - 3) {
          while (ti > 0 && textOut[ti-1] == ' ') ti--;
          if (ti > 0 && textOut[ti-1] != '\n' && textOut[ti-1] != ',') {
            textOut[ti++] = ',';
            textOut[ti++] = ' ';
            lastWasSpace = true;
          } else if (ti > 0 && textOut[ti-1] == ',') {
            textOut[ti++] = ' ';
            lastWasSpace = true;
          }
          lastWasBreak = (ti == 0 || textOut[ti-1] == '\n');
        }
      }

      // Handle <dt> - inline flow with space separator (matches browser's inline stats)
      if (!isClosing && tagNameLen == 2 && tagName[0] == 'd' && tagName[1] == 't') {
        if (ti > 0 && !lastWasSpace && !lastWasBreak) {
          textOut[ti++] = ' ';
          lastWasSpace = true;
        }
      }

      // Handle <dd> - just a space after the term (keeps "Label: Value" on one line)
      if (!isClosing && tagNameLen == 2 && tagName[0] == 'd' && tagName[1] == 'd') {
        if (ti > 0 && !lastWasSpace && !lastWasBreak) {
          textOut[ti++] = ' ';
          lastWasSpace = true;
        }
        lastWasBreak = false;
      }

      // ---- Form handling ----

      // <form action="..." method="...">
      if (!isClosing && tagNameLen == 4 &&
          tagName[0] == 'f' && tagName[1] == 'o' && tagName[2] == 'r' && tagName[3] == 'm') {
        if (result.formCount < maxForms) {
          currentForm = result.formCount;
          WebForm& f = forms[currentForm];
          memset(&f, 0, sizeof(WebForm));
          char actionBuf[WEB_MAX_URL_LEN] = {0};
          extractAttr(inside, insideLen, "action", actionBuf, WEB_MAX_URL_LEN);
          if (actionBuf[0]) {
            resolveUrl(baseUrl, actionBuf, f.action, WEB_MAX_URL_LEN);
          } else {
            strncpy(f.action, baseUrl, WEB_MAX_URL_LEN - 1);
          }
          char methodBuf[8] = {0};
          extractAttr(inside, insideLen, "method", methodBuf, sizeof(methodBuf));
          // tolower
          for (int m = 0; methodBuf[m]; m++) {
            if (methodBuf[m] >= 'A' && methodBuf[m] <= 'Z') methodBuf[m] += 32;
          }
          f.isPost = (strcmp(methodBuf, "post") == 0);
          inForm = true;
          // Emit form marker in text
          if (!lastWasBreak && ti > 0) { textOut[ti++] = '\n'; textOut[ti++] = '\n'; }
          f.formMarker = ti;
          if (ti < textMax - 12) {
            textOut[ti++] = '-'; textOut[ti++] = '-';
            textOut[ti++] = ' '; textOut[ti++] = 'F';
            textOut[ti++] = 'o'; textOut[ti++] = 'r';
            textOut[ti++] = 'm'; textOut[ti++] = ' ';
            textOut[ti++] = '{'; textOut[ti++] = 'F';
            textOut[ti++] = '0' + (result.formCount + 1);
            textOut[ti++] = '}';
            textOut[ti++] = ' '; textOut[ti++] = '-'; textOut[ti++] = '-';
            textOut[ti++] = '\n';
          }
          lastWasBreak = false; lastWasSpace = false;
        }
      }

      // </form>
      if (isClosing && tagNameLen == 4 &&
          tagName[0] == 'f' && tagName[1] == 'o' && tagName[2] == 'r' && tagName[3] == 'm') {
        if (inForm && currentForm >= 0) {
          // Emit submit hint if form has visible fields
          WebForm& f = forms[currentForm];
          if (f.textFieldCount > 0 && ti < textMax - 20) {
            textOut[ti++] = '\n';
            textOut[ti++] = '['; textOut[ti++] = 'f';
            textOut[ti++] = ':'; textOut[ti++] = ' ';
            textOut[ti++] = 'F'; textOut[ti++] = 'i';
            textOut[ti++] = 'l'; textOut[ti++] = 'l';
            textOut[ti++] = ' '; textOut[ti++] = 'f';
            textOut[ti++] = 'o'; textOut[ti++] = 'r';
            textOut[ti++] = 'm'; textOut[ti++] = ']';
            textOut[ti++] = '\n'; textOut[ti++] = '\n';
          }
          result.formCount++;
          lastWasBreak = true;
        }
        inForm = false;
        currentForm = -1;
      }

      // <input type="..." name="..." value="...">
      if (!isClosing && tagNameLen == 5 &&
          tagName[0] == 'i' && tagName[1] == 'n' && tagName[2] == 'p' &&
          tagName[3] == 'u' && tagName[4] == 't') {
        if (inForm && currentForm >= 0) {
          WebForm& f = forms[currentForm];
          if (f.fieldCount < WEB_MAX_FORM_FIELDS) {
            WebFormField& fld = f.fields[f.fieldCount];
            memset(&fld, 0, sizeof(WebFormField));
            char typeBuf[16] = "text";
            extractAttr(inside, insideLen, "type", typeBuf, sizeof(typeBuf));
            for (int m = 0; typeBuf[m]; m++) {
              if (typeBuf[m] >= 'A' && typeBuf[m] <= 'Z') typeBuf[m] += 32;
            }
            extractAttr(inside, insideLen, "name", fld.name, sizeof(fld.name));
            extractAttr(inside, insideLen, "value", fld.value, sizeof(fld.value));

            if (strcmp(typeBuf, "hidden") == 0) {
              fld.type = 'h';
              // type 'h' marks it as hidden — no display needed
            } else if (strcmp(typeBuf, "password") == 0) {
              fld.type = 'p';
              // Use pending label or placeholder
              if (pendingLabel[0]) { strncpy(fld.label, pendingLabel, sizeof(fld.label)-1); pendingLabel[0] = 0; }
              else extractAttr(inside, insideLen, "placeholder", fld.label, sizeof(fld.label));
              if (!fld.label[0]) strncpy(fld.label, "Password", sizeof(fld.label)-1);
              f.textFieldCount++;
              // Emit field display
              if (ti < textMax - 40) {
                int w = snprintf(textOut + ti, textMax - ti, "%s: [****]\n", fld.label);
                if (w > 0) ti += w;
              }
              lastWasBreak = false; lastWasSpace = false;
            } else if (strcmp(typeBuf, "submit") == 0) {
              fld.type = 's';
              if (!fld.value[0]) strncpy(fld.value, "Submit", sizeof(fld.value)-1);
              strncpy(fld.label, fld.value, sizeof(fld.label)-1);
            } else if (strcmp(typeBuf, "checkbox") == 0) {
              fld.type = 'c';
              if (pendingLabel[0]) { strncpy(fld.label, pendingLabel, sizeof(fld.label)-1); pendingLabel[0] = 0; }
            } else {
              // text, email, search, etc — treat as text input
              fld.type = 't';
              if (pendingLabel[0]) { strncpy(fld.label, pendingLabel, sizeof(fld.label)-1); pendingLabel[0] = 0; }
              else extractAttr(inside, insideLen, "placeholder", fld.label, sizeof(fld.label));
              if (!fld.label[0]) {
                // Use name as fallback label
                strncpy(fld.label, fld.name, sizeof(fld.label)-1);
                // Search-named fields display as "Search" (matches the fill modal)
                if (!strcmp(fld.name, "q") || !strcmp(fld.name, "s") ||
                    !strcmp(fld.name, "query") || !strcmp(fld.name, "search")) {
                  strncpy(fld.label, "Search", sizeof(fld.label)-1);
                }
              }
              f.textFieldCount++;
              // Emit field display
              if (ti < textMax - 40) {
                int w = snprintf(textOut + ti, textMax - ti, "%s: [___]\n", fld.label);
                if (w > 0) ti += w;
              }
              lastWasBreak = false; lastWasSpace = false;
            }
            f.fieldCount++;
          }
        }
      }

      // <label> / </label> - capture text for next input
      if (tagNameLen == 5 &&
          tagName[0] == 'l' && tagName[1] == 'a' && tagName[2] == 'b' &&
          tagName[3] == 'e' && tagName[4] == 'l') {
        if (!isClosing) {
          inLabel = true;
          labelTextStart = ti;
        } else if (inLabel) {
          // Capture label text
          int labelLen = ti - labelTextStart;
          if (labelLen > (int)sizeof(pendingLabel) - 1)
            labelLen = sizeof(pendingLabel) - 1;
          if (labelLen > 0)
            memcpy(pendingLabel, textOut + labelTextStart, labelLen);
          pendingLabel[labelLen] = '\0';
          // Remove trailing colon/space
          while (labelLen > 0 && (pendingLabel[labelLen-1] == ':' || pendingLabel[labelLen-1] == ' '))
            pendingLabel[--labelLen] = '\0';
          // Rewind text output — label text is used for form field display only,
          // not shown in reading view (avoids duplicate: label text + field marker)
          ti = labelTextStart;
          lastWasBreak = (ti == 0 || textOut[ti-1] == '\n');
          lastWasSpace = (ti > 0 && textOut[ti-1] == ' ');
          inLabel = false;
        }
      }

      // <button> - render content inline (don't skip, as buttons may wrap links)
      // Form submit buttons are handled separately in form rendering.

      hi = tagEnd + 1;
      continue;
    }

    // Skip content inside skip tags
    if (skipDepth > 0) {
      hi++;
      continue;
    }

    // HTML entity
    if (c == '&') {
      // Special multi-char entity: &hellip; → ...
      if (hi + 7 <= htmlLen && memcmp(html + hi, "&hellip;", 8) == 0) {
        if (ti < textMax - 3) {
          textOut[ti++] = '.';
          textOut[ti++] = '.';
          textOut[ti++] = '.';
          lastWasSpace = false;
          lastWasBreak = false;
        }
        hi += 8;
        continue;
      }
      char decoded;
      int consumed = decodeHtmlEntity(html, htmlLen, hi, &decoded);
      if (consumed > 0) {
        if (decoded == ' ') {
          if (!lastWasSpace && !lastWasBreak) {
            textOut[ti++] = ' ';
            lastWasSpace = true;
          }
        } else {
          textOut[ti++] = decoded;
          lastWasSpace = false;
          lastWasBreak = false;
        }
        hi += consumed;
        continue;
      }
    }

    // Whitespace collapsing
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
      if (!lastWasSpace && !lastWasBreak && ti > 0) {
        textOut[ti++] = ' ';
        lastWasSpace = true;
      }
      hi++;
      continue;
    }

    // Regular character
    textOut[ti++] = c;
    lastWasSpace = false;
    lastWasBreak = false;
    hi++;
  }

  textOut[ti] = '\0';
  
  // Post-processing: clean up stray commas from empty list items
  // (e.g. <li> containing only images produce ", " with no content)
  int wi = 0;
  for (int ri = 0; ri < ti; ri++) {
    // Skip ", " that follows a newline (first empty items)
    if (textOut[ri] == ',' && ri + 1 < ti && textOut[ri+1] == ' ') {
      // Check if preceded by newline or start of string
      if (wi == 0 || textOut[wi-1] == '\n') {
        ri++; // skip the space too
        continue;
      }
      // Check if followed by another comma (consecutive empty items)
      if (ri + 2 < ti && textOut[ri+2] == ',') {
        ri++; // skip ", " — next iteration handles the next comma
        continue;
      }
    }
    textOut[wi++] = textOut[ri];
  }
  textOut[wi] = '\0';
  ti = wi;
  
  // Also collapse multiple consecutive newlines (max 2)
  wi = 0;
  int nlCount = 0;
  for (int ri = 0; ri < ti; ri++) {
    if (textOut[ri] == '\n') {
      nlCount++;
      if (nlCount <= 2) textOut[wi++] = textOut[ri];
    } else {
      nlCount = 0;
      textOut[wi++] = textOut[ri];
    }
  }
  textOut[wi] = '\0';
  ti = wi;

  result.textLen = ti;
  return result;
}

#endif // MECK_WEB_HTML_H