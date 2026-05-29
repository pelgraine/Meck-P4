// MeckReader.cpp
//
// See MeckReader.h. Plain-text reader paging backend.

#include "MeckReader.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

// ---------------------------------------------------------------------------
// Reader state
// ---------------------------------------------------------------------------
static FILE *g_file = NULL;
static long  g_file_size = 0;
static char  g_path[256] = {0};

// Page-start byte offsets visited this session. g_pages[0] is where the
// session began (the resume point, or 0). g_idx is the current page.
static std::vector<long> g_pages;
static int  g_idx = 0;

// Byte offset just past the current page (start of the next page). Set by
// meck_reader_show_page. Used to push the next page and to compute progress.
static long g_cur_next = 0;

// One page's worth of file bytes is read into this window for measurement.
// A page of text is well under this even at the smallest font.
#define READER_WINDOW 8192
static char g_win[READER_WINDOW + 1];

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

// Byte offset of the Nth (0-based) UTF-8 character in a NUL-terminated string.
static long utf8_byte_of_char(const char *s, uint32_t char_idx) {
    long b = 0;
    uint32_t c = 0;
    while (s[b] && c < char_idx) {
        unsigned char ch = (unsigned char)s[b];
        if      (ch < 0x80)         b += 1;
        else if ((ch >> 5) == 0x6)  b += 2;
        else if ((ch >> 4) == 0xE)  b += 3;
        else if ((ch >> 3) == 0x1E) b += 4;
        else                        b += 1;  // invalid lead byte, step one
        c++;
    }
    return b;
}

// Resume sidecar path for a book: "<path>.pos" (never matched by the .txt
// scan, so it won't show up as its own entry).
static void resume_path_for(const char *book, char *out, size_t out_sz) {
    snprintf(out, out_sz, "%s.pos", book);
}

static long load_resume(const char *book) {
    char rp[256 + 8];
    resume_path_for(book, rp, sizeof(rp));
    FILE *f = fopen(rp, "r");
    if (!f) return 0;
    long off = 0;
    if (fscanf(f, "%ld", &off) != 1) off = 0;
    fclose(f);
    if (off < 0) off = 0;
    return off;
}

static void save_resume(const char *book, long off) {
    char rp[256 + 8];
    resume_path_for(book, rp, sizeof(rp));
    FILE *f = fopen(rp, "w");
    if (!f) return;
    fprintf(f, "%ld", off);
    fclose(f);
}

// ---------------------------------------------------------------------------
// Open / close
// ---------------------------------------------------------------------------
bool meck_reader_open(const char *path) {
    meck_reader_close();
    if (!path) return false;

    g_file = fopen(path, "rb");
    if (!g_file) return false;

    fseek(g_file, 0, SEEK_END);
    g_file_size = ftell(g_file);
    fseek(g_file, 0, SEEK_SET);

    strncpy(g_path, path, sizeof(g_path) - 1);
    g_path[sizeof(g_path) - 1] = '\0';

    long resume = load_resume(path);
    if (resume >= g_file_size) resume = 0;   // stale sidecar; start over

    g_pages.clear();
    g_pages.push_back(resume);
    g_idx = 0;
    g_cur_next = resume;
    return true;
}

void meck_reader_close(void) {
    if (g_file) {
        if (g_path[0]) save_resume(g_path, g_pages.empty() ? 0 : g_pages[g_idx]);
        fclose(g_file);
        g_file = NULL;
    }
    g_path[0] = '\0';
    g_file_size = 0;
    g_pages.clear();
    g_idx = 0;
    g_cur_next = 0;
}

bool meck_reader_is_open(void) { return g_file != NULL; }

bool meck_reader_has_resume(const char *path) {
    return path && load_resume(path) > 0;
}

// ---------------------------------------------------------------------------
// Paging
// ---------------------------------------------------------------------------

// Read up to READER_WINDOW bytes at `off` into g_win (NUL-terminated). Returns
// the number of bytes read.
static size_t read_window(long off) {
    if (!g_file) return 0;
    fseek(g_file, off, SEEK_SET);
    size_t n = fread(g_win, 1, READER_WINDOW, g_file);
    g_win[n] = '\0';
    return n;
}

void meck_reader_show_page(lv_obj_t *label, int32_t page_w, int32_t page_h) {
    if (!g_file || !label || g_pages.empty()) return;

    long off = g_pages[g_idx];
    size_t n = read_window(off);

    // Lay the whole window out at the page width and let LVGL wrap it.
    lv_obj_set_width(label, page_w);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(label, g_win);
    lv_obj_update_layout(label);

    long cut;
    if (lv_obj_get_height(label) <= page_h) {
        // The whole window fits the page (near end-of-file, or short text).
        cut = (long)n;
    } else {
        // Find the first character that falls past the visible height. At x=0
        // that lands on a line start, so the cut is a clean line boundary.
        lv_point_t p; p.x = 0; p.y = page_h;
        uint32_t ci = lv_label_get_letter_on(label, &p, false);
        cut = utf8_byte_of_char(g_win, ci);
        if (cut <= 0 || cut > (long)n) cut = (long)n;
    }

    // Show only this page's text.
    g_win[cut] = '\0';
    lv_label_set_text(label, g_win);
    lv_obj_update_layout(label);

    g_cur_next = off + cut;
    if (g_cur_next > g_file_size) g_cur_next = g_file_size;
}

void meck_reader_next(lv_obj_t *label, int32_t page_w, int32_t page_h) {
    if (!g_file || meck_reader_at_end()) return;

    if (g_idx == (int)g_pages.size() - 1) {
        // Frontier: the next page starts where the current one ended.
        g_pages.push_back(g_cur_next);
    }
    g_idx++;
    meck_reader_show_page(label, page_w, page_h);
}

void meck_reader_prev(lv_obj_t *label, int32_t page_w, int32_t page_h) {
    if (!g_file || g_idx <= 0) return;
    g_idx--;
    meck_reader_show_page(label, page_w, page_h);
}

bool meck_reader_at_end(void)   { return g_cur_next >= g_file_size; }
bool meck_reader_at_start(void) { return g_idx <= 0; }

int meck_reader_progress_pct(void) {
    if (g_file_size <= 0) return 0;
    long p = (g_cur_next * 100) / g_file_size;
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    return (int)p;
}