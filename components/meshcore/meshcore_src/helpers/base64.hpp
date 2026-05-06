/*
 * base64.hpp — Minimal Base64 decode for MeshCore channel PSK handling
 * Only decode is needed (addChannel parses base64 PSK strings)
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

static inline int base64_decode(const char* input, size_t in_len, uint8_t* output, size_t out_max) {
    static const uint8_t d[] = {
        66,66,66,66,66,66,66,66,66,66,64,66,66,66,66,66,66,66,66,66,66,66,66,66,66,
        66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,62,66,66,66,63,52,53,
        54,55,56,57,58,59,60,61,66,66,66,65,66,66,66, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
        10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,66,66,66,66,66,66,26,27,28,
        29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,66,66,
        66,66,66
    };
    size_t out_len = 0;
    uint32_t buf = 0;
    int bits = 0;
    for (size_t i = 0; i < in_len && out_len < out_max; i++) {
        uint8_t c = (uint8_t)input[i];
        if (c >= 128 || d[c] >= 64) { if (d[c] == 65) break; continue; }
        buf = (buf << 6) | d[c];
        bits += 6;
        if (bits >= 8) { bits -= 8; output[out_len++] = (buf >> bits) & 0xFF; }
    }
    return (int)out_len;
}
    static inline int decode_base64(const unsigned char* input, size_t in_len, uint8_t* output) {
    return base64_decode((const char*)input, in_len, output, 256);
}