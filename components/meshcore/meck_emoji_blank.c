/* Generated: 1x1 transparent spacer for VS-16 / trailing regional indicator. */

#include "lvgl.h"


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_EMOJI_BLANK
#define LV_ATTRIBUTE_EMOJI_BLANK
#endif

static const
LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_EMOJI_BLANK
uint8_t emoji_blank_map[] = {

    0x00,0x00,
    0x00,

};

const lv_image_dsc_t emoji_blank = {
  .header.magic = LV_IMAGE_HEADER_MAGIC,
  .header.cf = LV_COLOR_FORMAT_RGB565A8,
  .header.flags = 0,
  .header.w = 1,
  .header.h = 1,
  .header.stride = 2,
  .data_size = sizeof(emoji_blank_map),
  .data = emoji_blank_map,
};

