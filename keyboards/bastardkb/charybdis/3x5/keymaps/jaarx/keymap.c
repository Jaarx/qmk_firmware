/**
 * Copyright 2021 Charly Delay <charly@codesink.dev> (@0xcharly)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include QMK_KEYBOARD_H
#include "features/achordion.h"

enum charybdis_keymap_layers {
    LAYER_BASE = 0,
    LAYER_POINTER,
    LAYER_RSYMBOL,
    LAYER_RNUM,
    LAYER_NAV,
    LAYER_LFUNC,
    LAYER_LSYMBOL
};

/** Thumb-layer switchers */
#define BSPC_RSYMBOL LT(LAYER_RSYMBOL, KC_BSPC)
#define TAB_RNUM LT(LAYER_RNUM, KC_TAB)
#define ENT_LFUNC LT(LAYER_LFUNC, KC_ENT)
#define SPC_LSYMBOL LT(LAYER_LSYMBOL, KC_SPC)


#ifndef POINTING_DEVICE_ENABLE
#    define DRGSCRL KC_NO
#    define DPI_MOD KC_NO
#    define S_D_MOD KC_NO
#    define SNIPING KC_NO
#endif // !POINTING_DEVICE_ENABLE

// clang-format off
/** \brief QWERTY layout (3 rows, 10 columns). */
#define LAYOUT_LAYER_BASE                                                                     \
       KC_Q,    KC_W,    KC_F,    KC_P,    KC_V,    KC_J,    KC_L,    KC_U,    KC_Y,    KC_SCLN, \
       KC_A,    KC_R,    KC_S,    KC_T,    LT(4,KC_G),    KC_M,    KC_N,    KC_E,    KC_I, KC_O, \
       LT(0,KC_Z),    LT(0,KC_X),    LT(0,KC_C),    LT(0,KC_D),    KC_B,    KC_K,    KC_H, KC_COMM,  KC_DOT, KC_SLSH, \
                      MO(1), BSPC_RSYMBOL, TAB_RNUM, ENT_LFUNC, SPC_LSYMBOL

/** Convenience row shorthands. */
#define _______________DEAD_HALF_ROW_______________ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
#define ______________HOME_ROW_GACS_L______________ KC_LGUI, KC_LALT, KC_LCTL, KC_LSFT, XXXXXXX
#define ______________HOME_ROW_GACS_R______________ XXXXXXX, KC_LSFT, KC_LCTL, KC_LALT, KC_LGUI

#define SNIPREV POINTER_SNIPING_DPI_REVERSE
#define SNIPFOR POINTER_SNIPING_DPI_FORWARD
#define DPI_REV POINTER_DEFAULT_DPI_REVERSE
#define DPI_FOR POINTER_DEFAULT_DPI_FORWARD

#define CRTSCRL PM_MO(PM_CARET)

/*
 * Layers used on the Charybdis Nano.
 *
 * These layers started off heavily inspired by the Miryoku layout, but trimmed
 * down and tailored for a stock experience that is meant to be fundation for
 * further personalization.
 *
 * See https://github.com/manna-harbour/miryoku for the original layout.
 */

#define LAYOUT_LAYER_POINTER                                                                  \
    QK_BOOT, SNIPREV, SNIPFOR, DPI_REV, DPI_FOR,  EE_CLR,   KC_F7,   KC_F8,   KC_F9,  KC_F12, \
    ______________HOME_ROW_GACS_L______________, XXXXXXX, KC_LEFT,   KC_UP, KC_DOWN, KC_RGHT, \
    XXXXXXX, DRGSCRL, SNIPING, CRTSCRL, XXXXXXX, KC_BTN4, KC_BTN1, KC_BTN2, KC_BTN3, KC_BTN5, \
                      _______, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX

#define LAYOUT_LAYER_RSYMBOL                                                                  \
    _______________DEAD_HALF_ROW_______________, XXXXXXX, KC_PLUS, KC_MINS, KC_ASTR, KC_QUOT, \
    ______________HOME_ROW_GACS_L______________, KC_GRV,   KC_EQL, KC_UNDS,   KC_LT,   KC_GT, \
    _______________DEAD_HALF_ROW_______________, XXXXXXX, KC_AMPR, KC_PIPE, KC_EXLM, KC_BSLS, \
                      XXXXXXX, _______, XXXXXXX, XXXXXXX, XXXXXXX

#define LAYOUT_LAYER_RNUM                                                                     \
    _______________DEAD_HALF_ROW_______________, XXXXXXX,    KC_7,    KC_8,    KC_9, KC_BSPC, \
    ______________HOME_ROW_GACS_L______________, XXXXXXX,    KC_4,    KC_5,    KC_6,    KC_0, \
    _______________DEAD_HALF_ROW_______________, XXXXXXX,    KC_1,    KC_2,    KC_3, XXXXXXX, \
                      XXXXXXX, XXXXXXX, _______, XXXXXXX, XXXXXXX

#define LAYOUT_LAYER_NAV                                                                  \
    _______________DEAD_HALF_ROW_______________, KC_VOLU,   KC_MRWD,   KC_MPLY,   KC_MFFD,LSG(KC_5), \
    XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, _______, KC_VOLD,   KC_LEFT,     KC_UP,   KC_DOWN,  KC_RGHT, \
    _______________DEAD_HALF_ROW_______________, KC_MUTE,   XXXXXXX,   XXXXXXX,   XXXXXXX,  XXXXXXX, \
                      XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX

#define LAYOUT_LAYER_LFUNC                                                                   \
     KC_F12,   KC_F7,   KC_F8,   KC_F9, XXXXXXX, RGB_MOD, XXXXXXX, RGB_HUI, RGB_SAI, RGB_VAI, \
     KC_F11,   KC_F4,   KC_F5,   KC_F6, XXXXXXX, ______________HOME_ROW_GACS_R______________, \
     KC_F10,   KC_F1,   KC_F2,   KC_F3, XXXXXXX, RGB_M_P, RGB_M_B, XXXXXXX, XXXXXXX, RGB_SPI, \
                      XXXXXXX, XXXXXXX, XXXXXXX, _______, XXXXXXX

#define LAYOUT_LAYER_LSYMBOL                                                                  \
    KC_TILD,   KC_AT, KC_LCBR, KC_RCBR, XXXXXXX, _______________DEAD_HALF_ROW_______________, \
    KC_HASH,  KC_DLR, KC_LPRN, KC_RPRN, XXXXXXX, ______________HOME_ROW_GACS_R______________, \
    KC_PERC, KC_CIRC, KC_LBRC, KC_RBRC, XXXXXXX, _______________DEAD_HALF_ROW_______________, \
                      XXXXXXX,  KC_DEL, XXXXXXX, XXXXXXX, XXXXXXX

/**
 * \brief Add Home Row mod to a layout.
 *
 * Expects a 10-key per row layout.  Adds support for GACS (Gui, Alt, Ctl, Shift)
 * home row.  The layout passed in parameter must contain at least 20 keycodes.
 *
 * This is meant to be used with `LAYER_ALPHAS_QWERTY` defined above, eg.:
 *
 *     HOME_ROW_MOD_GACS(LAYER_ALPHAS_QWERTY)
 */
#define _HOME_ROW_MOD_GACS(                                            \
    L00, L01, L02, L03, L04, R05, R06, R07, R08, R09,                  \
    L10, L11, L12, L13, L14, R15, R16, R17, R18, R19,                  \
    ...)                                                               \
             L00,         L01,         L02,         L03,         L04,  \
             R05,         R06,         R07,         R08,         R09,  \
      LGUI_T(L10), LALT_T(L11), LCTL_T(L12), LSFT_T(L13),        L14,  \
             R15,  RSFT_T(R16), RCTL_T(R17), LALT_T(R18), RGUI_T(R19), \
      __VA_ARGS__
#define HOME_ROW_MOD_GACS(...) _HOME_ROW_MOD_GACS(__VA_ARGS__)

#define _NAV_LAYER_LAG(                                            \
    L00, L01, L02, L03, L04, R05, R06, R07, R08, R09,                  \
    L10, L11, L12, L13, L14, R15, R16, R17, R18, R19,                  \
    ...)                                                               \
             L00,         L01,         L02,         L03,         L04,  \
             R05,         R06,         R07,         R08,         R09,  \
             L10,         L11,         L12,         L13,         L14,  \
             R15,     LAG(R16),   LAG(R17),    LAG(R18),    LAG(R19), \
      __VA_ARGS__
#define NAV_LAYER_LAG(...) _NAV_LAYER_LAG(__VA_ARGS__)

#define LAYOUT_wrapper(...) LAYOUT(__VA_ARGS__)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [LAYER_BASE] = LAYOUT_wrapper(
    HOME_ROW_MOD_GACS(LAYOUT_LAYER_BASE)
  ),
  [LAYER_POINTER] = LAYOUT_wrapper(LAYOUT_LAYER_POINTER),
  [LAYER_RSYMBOL] = LAYOUT_wrapper(LAYOUT_LAYER_RSYMBOL),
  [LAYER_RNUM] = LAYOUT_wrapper(LAYOUT_LAYER_RNUM),
  [LAYER_NAV] = LAYOUT_wrapper(NAV_LAYER_LAG(LAYOUT_LAYER_NAV)),
  [LAYER_LFUNC] = LAYOUT_wrapper(LAYOUT_LAYER_LFUNC),
  [LAYER_LSYMBOL] = LAYOUT_wrapper(LAYOUT_LAYER_LSYMBOL),
};
// clang-format on

#    ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    if (abs(mouse_report.x) > CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD || abs(mouse_report.y) > CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD) {
        if (auto_pointer_layer_timer == 0) {
            layer_on(LAYER_POINTER);
#        ifdef RGB_MATRIX_ENABLE
            rgb_matrix_mode_noeeprom(RGB_MATRIX_NONE);
            rgb_matrix_sethsv_noeeprom(HSV_GREEN);
#        endif // RGB_MATRIX_ENABLE
        }
        auto_pointer_layer_timer = timer_read();
    }
    return mouse_report;
}

void matrix_scan_user(void) {
    if (auto_pointer_layer_timer != 0 && TIMER_DIFF_16(timer_read(), auto_pointer_layer_timer) >= CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS) {
        auto_pointer_layer_timer = 0;
        layer_off(LAYER_POINTER);
#        ifdef RGB_MATRIX_ENABLE
        rgb_matrix_mode_noeeprom(RGB_MATRIX_DEFAULT_MODE);
#        endif // RGB_MATRIX_ENABLE
    }
}
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

#ifdef RGB_MATRIX_ENABLE
// Forward-declare this helper function since it is defined in
// rgb_matrix.c.
void rgb_matrix_update_pwm_buffers(void);
#endif

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
    switch(keycode) {
        case LT(0,KC_Z):
        if (!record->tap.count && record->event.pressed) {
            tap_code16(G(KC_Z)); // Intercept hold function to send Ctrl-Z
            return false;
        }
        break;
        case LT(0,KC_X):
        if (!record->tap.count && record->event.pressed) {
            tap_code16(G(KC_X)); // Intercept hold function to send Ctrl-X
            return false;
        }
        break;
        case LT(0,KC_C):
        if (!record->tap.count && record->event.pressed) {
            tap_code16(G(KC_C)); // Intercept hold function to send Ctrl-C
            return false;
        }
        break;
        case LT(0,KC_D):
        if (!record->tap.count && record->event.pressed) {
            tap_code16(G(KC_V)); // Intercept hold function to send Ctrl-V
            return false;
        }
        break;
        case LT(2,KC_QUOT):
        if (!record->tap.count && record->event.pressed) {
            tap_code16(KC_GRV); // Intercept hold function to send Grave
            return false;
        }
        break;
    }

  if (!process_achordion(keycode, record)) { return false; }

  return true;
}

void matrix_scan_user(void) {
  achordion_task();
}

enum combos {
  VJ_ESC,
  HCOMMA_BTN1,
  COMMADOT_BTN2,
  DOTSLASH_BTN3,
  HCOMMADOT_DRG_TOG
};

const uint16_t PROGMEM vj_combo[] = {KC_V, KC_J, COMBO_END};
const uint16_t PROGMEM hcomma_combo[] = {KC_H, KC_COMM, COMBO_END};
const uint16_t PROGMEM commadot_combo[] = {KC_COMM, KC_DOT, COMBO_END};
const uint16_t PROGMEM dotslash_combo[] = {KC_DOT, KC_SLSH, COMBO_END};
const uint16_t PROGMEM hcommadot_combo[] = {KC_H, KC_COMM, KC_DOT, COMBO_END};

combo_t key_combos[] = {
    [VJ_ESC] = COMBO(vj_combo, KC_ESCAPE),
    [HCOMMA_BTN1] = COMBO(hcomma_combo, KC_BTN1),
    [COMMADOT_BTN2] = COMBO(commadot_combo, KC_BTN2),
    [DOTSLASH_BTN3] = COMBO(dotslash_combo, KC_BTN3),
    [HCOMMADOT_DRG_TOG] = COMBO(hcommadot_combo, DRG_TOG)
};
