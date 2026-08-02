// MeckNotes.h
//
// Notes backend for Meck-P4.
//
// Notes are plain UTF-8 .md (markdown) and .txt files stored under
// /sdcard/notes, so they live on
// the removable card, survive a firmware reflash, and can be read or edited on
// a computer. Reading reuses the MeckReader paging backend; create / edit /
// save / delete / rename land in later stages.
//
// Stage 1 scope: ensure the notes directory exists on the SD card.

#ifndef MECK_NOTES_H
#define MECK_NOTES_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Absolute path of the notes folder on the mounted SD card.
#define MECK_NOTES_DIR "/sdcard/notes"

// Create MECK_NOTES_DIR if it does not already exist. Safe to call repeatedly;
// a no-op when the folder is present. Called on UI init so a freshly flashed
// device gets the folder without the user having to create it by hand.
void meck_notes_ensure_dir(void);

// Build a path for a NEW note (does not create the file). New notes are
// markdown: a timestamped name (note_YYYYMMDD_HHMM.md) when the clock is
// synced, falling back to a sequential name (note_NNN.md) otherwise. Writes the full path into out.
// Returns false only on bad arguments.
bool meck_notes_new_path(char* out, size_t out_sz);

// Read a note into buf (NUL-terminated, truncated to buf_sz - 1). Returns the
// number of bytes read, or -1 if the file cannot be opened.
int meck_notes_load(const char* path, char* buf, size_t buf_sz);

// Overwrite a note with text (UTF-8, NUL-terminated). Returns true on success.
bool meck_notes_save(const char* path, const char* text);

// Delete a note and its resume sidecar (<path>.pos), if present. Returns true
// if the note file itself was removed.
bool meck_notes_delete(const char* path);

// Rename a note from old_path to new_path, moving its resume sidecar too if it
// exists. Refuses to overwrite an existing new_path. Returns true on success.
bool meck_notes_rename(const char* old_path, const char* new_path);

#ifdef __cplusplus
}
#endif

#endif // MECK_NOTES_H