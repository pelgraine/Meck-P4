// MeckEmojiPicker.h
//
// Modal emoji picker for Meck-P4. Opened from the emoji key on the two
// message-composer keyboards; presents a scrollable grid of the renderable
// emoji and inserts the chosen one (as UTF-8) into the target textarea.
//
// The grid cells render the emoji through the meck_montserrat_* imgfont
// fallback set up by meck_emoji_init(), so this requires the emoji render
// path (CONFIG_LV_USE_IMGFONT=y) to be active.

#ifndef MECK_EMOJI_PICKER_H
#define MECK_EMOJI_PICKER_H

#include "lvgl.h"

// The label used for the emoji key in the composer keyboard maps. It is a
// single emoji codepoint (U+1F601), which also renders in colour on the key
// via the imgfont fallback. The composer keyboards compare a pressed key's
// text against this to decide whether to open the picker.
#define MECK_EMOJI_KEY "\xF0\x9F\x98\x81"

#ifdef __cplusplus
extern "C" {
#endif

// Open the picker as a modal over the active screen. The chosen emoji is
// inserted into `ta` as UTF-8. `kb` (the keyboard that opened it) is disabled
// while the picker is up and re-enabled on close. `dark` selects the theme.
void meck_emoji_picker_open(lv_obj_t *kb, lv_obj_t *ta, bool dark);

// Tear the picker down (also called internally on selection or tap-outside).
void meck_emoji_picker_close(void);

bool meck_emoji_picker_is_open(void);

#ifdef __cplusplus
}
#endif

#endif // MECK_EMOJI_PICKER_H
