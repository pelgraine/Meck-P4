// MeckNotes.cpp
//
// See MeckNotes.h. Stage 1: notes directory bootstrap.

#include "MeckNotes.h"

#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <dirent.h>
#include <unistd.h>

// Local epoch (mesh RTC UTC + UTC offset), or 0 if the clock isn't synced.
// Defined in MeckUI.cpp; declared here so the notes backend can timestamp new
// note filenames without depending on the mesh headers.
extern "C" uint32_t meck_ui_local_epoch(void);

void meck_notes_ensure_dir(void) {
    struct stat st;
    if (stat(MECK_NOTES_DIR, &st) == 0) {
        // Already present (a folder, or — unexpectedly — a file; either way we
        // leave it alone rather than clobber whatever is there).
        return;
    }
    if (mkdir(MECK_NOTES_DIR, 0775) == 0) {
        printf("MeckNotes: created %s\n", MECK_NOTES_DIR);
    } else {
        printf("MeckNotes: could not create %s\n", MECK_NOTES_DIR);
    }
}

bool meck_notes_new_path(char* out, size_t out_sz) {
    if (!out || out_sz == 0) return false;

    uint32_t local = meck_ui_local_epoch();
    if (local != 0) {
        // Clock is synced: name by local date/time to the minute. On the rare
        // same-minute collision, fall through to second precision.
        time_t t = (time_t)local;
        struct tm tm;
        gmtime_r(&t, &tm);   // local already has the offset folded in
        snprintf(out, out_sz, "%s/note_%04d%02d%02d_%02d%02d.txt",
                 MECK_NOTES_DIR, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                 tm.tm_hour, tm.tm_min);
        struct stat st;
        if (stat(out, &st) != 0) return true;   // no file with that name yet
        snprintf(out, out_sz, "%s/note_%04d%02d%02d_%02d%02d%02d.txt",
                 MECK_NOTES_DIR, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                 tm.tm_hour, tm.tm_min, tm.tm_sec);
        return true;
    }

    // Clock not synced: sequential note_NNN.txt, one past the highest existing.
    int maxnum = 0;
    DIR* d = opendir(MECK_NOTES_DIR);
    if (d) {
        struct dirent* de;
        while ((de = readdir(d)) != NULL) {
            int n = 0;
            if (sscanf(de->d_name, "note_%d.txt", &n) == 1 && n > maxnum)
                maxnum = n;
        }
        closedir(d);
    }
    snprintf(out, out_sz, "%s/note_%03d.txt", MECK_NOTES_DIR, maxnum + 1);
    return true;
}

int meck_notes_load(const char* path, char* buf, size_t buf_sz) {
    if (!path || !buf || buf_sz == 0) return -1;
    FILE* f = fopen(path, "rb");
    if (!f) return -1;
    size_t n = fread(buf, 1, buf_sz - 1, f);
    buf[n] = '\0';
    fclose(f);
    return (int)n;
}

bool meck_notes_save(const char* path, const char* text) {
    if (!path || !text) return false;
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    size_t len = strlen(text);
    size_t w = fwrite(text, 1, len, f);
    fclose(f);
    return w == len;
}

bool meck_notes_delete(const char* path) {
    if (!path) return false;
    // Resume sidecar is "<path>.pos" (same convention as MeckReader). Remove it
    // best-effort; absence is fine.
    char pos[300];
    snprintf(pos, sizeof(pos), "%s.pos", path);
    unlink(pos);
    return unlink(path) == 0;
}

bool meck_notes_rename(const char* old_path, const char* new_path) {
    if (!old_path || !new_path) return false;
    struct stat st;
    if (stat(new_path, &st) == 0) return false;   // don't clobber an existing note
    if (rename(old_path, new_path) != 0) return false;
    // Move the resume sidecar alongside, if present. The saved byte offset is
    // still valid since the content is unchanged. Best-effort.
    char old_pos[300], new_pos[300];
    snprintf(old_pos, sizeof(old_pos), "%s.pos", old_path);
    snprintf(new_pos, sizeof(new_pos), "%s.pos", new_path);
    if (stat(old_pos, &st) == 0) rename(old_pos, new_pos);
    return true;
}