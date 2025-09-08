/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-08-09 11:13:53
 * @LastEditTime: 2025-09-05 17:38:52
 * @License: GPL 3.0
 */
#pragma once
#include "sdkconfig.h"

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
#include "rfal_rfst25r3916.h"

void St25r3916_Init(void);
void St25r3916_Loop(void);

extern RfalRfST25R3916Class rfst25r3916;
#endif
