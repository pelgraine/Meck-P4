// MeckReader.h
//
// Text reader paging backend for Meck-P4. Plain-text only (.txt); EPUB
// import is a later stage that produces a .txt this reader then opens.
//
// The reader never holds the whole book. It keeps a byte offset into the
// file, reads a window from there into an lv_label, and uses LVGL's own
// layout to find where the current page ends (the byte offset of the first
// character that would fall past the visible text area). Memory and layout
// cost stay fixed to one screen regardless of file size.
//
// Page-back is supported through the pages visited this session (a history
// of page-start offsets). Resume restores the last page-start offset, so
// reading continues forward from where you left off across reboots.
//
// Directory browsing lives in MeckReaderUI; this module is the paging and
// resume core, operating on a caller-supplied label plus SD file access.

#ifndef MECK_READER_H
#define MECK_READER_H

#include "lvgl.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Open a book and restore its saved resume position. Returns false if the
// file cannot be opened. Closes any previously open book first.
bool meck_reader_open(const char *path);

// Save the current page position to the book's sidecar and close the file.
void meck_reader_close(void);

bool meck_reader_is_open(void);

// True if the book at `path` has a saved (non-zero) reading position.
bool meck_reader_has_resume(const char *path);

// Render the current page into `label`. `page_w`/`page_h` are the pixel
// width and height of the text area; the label is width-constrained to
// page_w and wrapped, and only the text that fits page_h is shown.
void meck_reader_show_page(lv_obj_t *label, int32_t page_w, int32_t page_h);

// Advance / go back one page and re-render into `label`. No-op at the ends.
void meck_reader_next(lv_obj_t *label, int32_t page_w, int32_t page_h);
void meck_reader_prev(lv_obj_t *label, int32_t page_w, int32_t page_h);

bool meck_reader_at_end(void);     // current page is the last
bool meck_reader_at_start(void);   // current page is the first of this session
int  meck_reader_progress_pct(void); // 0..100 by byte position through the file

#ifdef __cplusplus
}
#endif

#endif // MECK_READER_H