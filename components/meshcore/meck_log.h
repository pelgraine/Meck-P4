/*
 * meck_log.h — printf redirection for Meck debug logging
 *
 * When Settings > Debug Logs > Start is active, printf calls in
 * translation units that include this header are routed to a sync
 * write into the per-session log file on SD (under a recursive
 * FreeRTOS mutex) instead of going to UART. When inactive, the
 * wrapper falls through to vprintf so printf behaves normally.
 *
 * The implementation lives in MeckUI.cpp (meck_debug_log_printf).
 *
 * Macro propagation:
 *   This header defines a preprocessor macro that rewrites printf
 *   into meck_debug_log_printf. Any file that includes this header
 *   gets the rewrite for the rest of its translation unit. When this
 *   header is pulled in by a .h file (e.g. MeckMesh.h, MeckDataStore.h,
 *   P4SX1262Radio.h), the rewrite also applies to any .cpp file that
 *   includes that header. That's intentional — it spreads the
 *   redirection across the Meck code without having to touch every
 *   site.
 *
 * The defining translation unit (MeckUI.cpp) must NOT include this
 * header — meck_debug_log_printf's body calls vprintf directly, and
 * we don't want printf calls elsewhere in MeckUI.cpp to recurse
 * through the wrapper at the point of definition. (The wrapper's
 * body uses vprintf/vfprintf, not printf, so even if MeckUI.cpp did
 * include this header the wrapper itself wouldn't self-recurse —
 * but keeping the definition file macro-free is the simpler rule.)
 */

#pragma once

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

int meck_debug_log_printf(const char* fmt, ...)
    __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

#define printf meck_debug_log_printf
