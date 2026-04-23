/*
 * Action codes for the &tp_tune behavior (zmk,behavior-tp-tune).
 *
 * Usage from keymap:
 *     &tp_tune TP_SENS_UP   0
 *     &tp_tune TP_SENS_DN   0
 *     &tp_tune TP_SENS_SET  150
 *     &tp_tune TP_INV_X     0
 *     &tp_tune TP_INV_Y     0
 *     &tp_tune TP_SWAP      0
 *     &tp_tune TP_RESET     0
 */

#pragma once

#define TP_SENS_UP   0
#define TP_SENS_DN   1
#define TP_SENS_SET  2
#define TP_INV_X     3
#define TP_INV_Y     4
#define TP_SWAP      5
#define TP_RESET     6
