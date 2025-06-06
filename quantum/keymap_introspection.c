// Copyright 2022 Nick Brassel (@tzarc)
// SPDX-License-Identifier: GPL-2.0-or-later

// Pull the actual keymap code so that we can inspect stuff from it
#include KEYMAP_C

// Allow for keymap or userspace rules.mk to specify an alternate location for the keymap array
#ifdef INTROSPECTION_KEYMAP_C
#    include INTROSPECTION_KEYMAP_C
#endif // INTROSPECTION_KEYMAP_C

#include "keymap_introspection.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Key mapping

#define NUM_KEYMAP_LAYERS_RAW ((uint8_t)(sizeof(keymaps) / ((MATRIX_ROWS) * (MATRIX_COLS) * sizeof(uint16_t))))

uint8_t keymap_layer_count_raw(void) {
    return NUM_KEYMAP_LAYERS_RAW;
}

__attribute__((weak)) uint8_t keymap_layer_count(void) {
    return keymap_layer_count_raw();
}

#ifdef DYNAMIC_KEYMAP_ENABLE
_Static_assert(NUM_KEYMAP_LAYERS_RAW <= MAX_LAYER, "Number of keymap layers exceeds maximum set by DYNAMIC_KEYMAP_LAYER_COUNT");
#else
_Static_assert(NUM_KEYMAP_LAYERS_RAW <= MAX_LAYER, "Number of keymap layers exceeds maximum set by LAYER_STATE_(8|16|32)BIT");
#endif

uint16_t keycode_at_keymap_location_raw(uint8_t layer_num, uint8_t row, uint8_t column) {
    if (layer_num < NUM_KEYMAP_LAYERS_RAW && row < MATRIX_ROWS && column < MATRIX_COLS) {
        return pgm_read_word(&keymaps[layer_num][row][column]);
    }
    return KC_TRNS;
}

__attribute__((weak)) uint16_t keycode_at_keymap_location(uint8_t layer_num, uint8_t row, uint8_t column) {
    return keycode_at_keymap_location_raw(layer_num, row, column);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Encoder mapping

#if defined(ENCODER_ENABLE) && defined(ENCODER_MAP_ENABLE)

#    define NUM_ENCODERMAP_LAYERS_RAW ((uint8_t)(sizeof(encoder_map) / ((NUM_ENCODERS) * (NUM_DIRECTIONS) * sizeof(uint16_t))))

uint8_t encodermap_layer_count_raw(void) {
    return NUM_ENCODERMAP_LAYERS_RAW;
}

__attribute__((weak)) uint8_t encodermap_layer_count(void) {
    return encodermap_layer_count_raw();
}

_Static_assert(NUM_KEYMAP_LAYERS_RAW == NUM_ENCODERMAP_LAYERS_RAW, "Number of encoder_map layers doesn't match the number of keymap layers");

uint16_t keycode_at_encodermap_location_raw(uint8_t layer_num, uint8_t encoder_idx, bool clockwise) {
    if (layer_num < NUM_ENCODERMAP_LAYERS_RAW && encoder_idx < NUM_ENCODERS) {
        return pgm_read_word(&encoder_map[layer_num][encoder_idx][clockwise ? 0 : 1]);
    }
    return KC_TRNS;
}

__attribute__((weak)) uint16_t keycode_at_encodermap_location(uint8_t layer_num, uint8_t encoder_idx, bool clockwise) {
    return keycode_at_encodermap_location_raw(layer_num, encoder_idx, clockwise);
}

#endif // defined(ENCODER_ENABLE) && defined(ENCODER_MAP_ENABLE)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Combos

#if defined(COMBO_ENABLE)

uint16_t combo_count_raw(void) {
    return sizeof(key_combos) / sizeof(combo_t);
}
__attribute__((weak)) uint16_t combo_count(void) {
    return combo_count_raw();
}

combo_t* combo_get_raw(uint16_t combo_idx) {
    return &key_combos[combo_idx];
}
__attribute__((weak)) combo_t* combo_get(uint16_t combo_idx) {
    return combo_get_raw(combo_idx);
}

#endif // defined(COMBO_ENABLE)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pointing mode mapping

#if defined(POINTING_MODE_MAP_ENABLE)

#    define POINTING_MODE_MAP_COUNT_RAW (uint8_t)(sizeof(pointing_mode_maps) / ((POINTING_NUM_DIRECTIONS) * sizeof(uint16_t)))

uint8_t pointing_mode_map_count_raw(void) {
    return POINTING_MODE_MAP_COUNT_RAW;
}

__attribute__((weak)) uint8_t pointing_mode_map_count(void) {
    return pointing_mode_map_count_raw();
}

/*
 * @brief Retrieve keycode from pointing mode map
 *
 * Returns keycode from pointing mode map based on 8 bit index location
 * breakdown of location:
 * mode id | direction
 * XXXX XX | XX
 *
 * NOTE: silently fails and returns KC_NO if mode map out of range
 *
 * @param[in] map_loc uint8_t
 *
 * @return uint16_t keycode at pointing mode map location
 */
uint16_t keycode_at_pointing_mode_map_location_raw(uint8_t map_loc) {
    uint8_t map_id = map_loc >> 2;
    uint8_t dir    = map_loc & 0x03;

    if (map_id < pointing_mode_map_count()) {
        return pgm_read_word(&pointing_mode_maps[map_id][dir]);
    }
    return KC_NO;
}

/*
 * @brief Weakly defined function for retreiving keycode from pointing mode map
 *
 * Defaults to passing map_loc to raw function, would allow interception of
 * keycode retrieval process for pointing mode maps
 *
 * @param[in] map_loc uint8_t
 *
 * @return uint16_t keycode at pointing mode map location
 */
__attribute__((weak)) uint16_t keycode_at_pointing_mode_map_location(uint8_t map_loc) {
    return keycode_at_pointing_mode_map_location_raw(map_loc);
}

#endif // defined(POINTING_MODE_MAP_ENABLE)
