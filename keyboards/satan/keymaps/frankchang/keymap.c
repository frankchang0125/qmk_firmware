#include "satan.h"

/*
 * Switching and toggling layers
 *   MO(layer)     - Momentary switch to layer
 *   OSL(layer)    - Momentary switch to layer, as a one-shot operation
 *   LT(layer, kc) - Momentary switch to layer when held, and kc when tapped
 *   TG(layer)     - Toggles a layer on or off
 *   TO(layer)     - Goes to a layer, go either up or down the stack
 *   TT(layer)     - Layer becomes active when hold down, deactivates when let go and toggles on when tap
 *
 * Fun with modifier keys
 *   OSM(mod)      - This is a one shot modifier
 *   LSFT(kc)      - Applies left Shift to kc (keycode) - S(kc) is an alias
 *   RSFT(kc)      - Applies right Shift to kc
 *   LCTL(kc)      - Applies left Control to kc
 *   RCTL(kc)      - Applies right Control to kc
 *   LALT(kc)      - Applies left Alt to kc
 *   RALT(kc)      - Applies right Alt to kc
 *   LGUI(kc)      - Applies left GUI (command/win) to kc
 *   RGUI(kc)      - Applies right GUI (command/win) to kc
 *   HYPR(kc)      - Applies Hyper (all modifiers) to kc
 *   MEH(kc)       - Applies Meh (all modifiers except Win/Cmd) to kc
 *   LCAG(kc)      - Applies CtrlAltGui to kc
 *   LALT(LCTL(KC_DEL)) - This makes a key that sends Alt, Control, and Delete in a single keypress
 *
 * Mod-Tap alias
 *   MT(mod, kc)   - Applies modifier key when held, and kc when tapped, and there are some alias below
 *   CTL_T(kc)     - Applies LCTL when held and kc when tapped
 *   SFT_T(kc)     - Applies LSFT when held and kc when tapped
 *   ALT_T(kc)     - Applies LALT when held and kc when tapped
 *   GUI_T(kc)     - Applies LGUI when held and kc when tapped
 *   ALL_T(kc)     - Applies Hyper (all mods) when held and kc when tapped
 *   LCAG_T(kc)    - Applies CtrlAltGui when held and kc when tapped
 *   MEH_T(kc)     - Applies like Hyper, but does not include the Cmd/Win key, so just sends Alt+Ctrl+Shift
 *
 * Built-in actions note
 *   ACTION_LAYER_MODS(layer, mod) - Momentary switch to layer with modifier
 */

// Increase readability
#define ________  KC_TRNS
#define XXXXXXXX  KC_NO

// Alias
#define C(kc) LCTL(kc)

// Layer name definitions
#define _DEF   0  // Default layer
#define _ESC   1  // Tricky ESC layer
#define _SFT   2  // Smart SFT layer, mix with shift modified arrow keys
#define _CTL   3  // Smart CTL layer, mix with arrow keys
#define _WIN   4  // Windows management hotkeys
#define _LFN   5  // FN layer on the left side
#define _RFN   6  // FN layer on the right side
#define _ARW   7  // Arrow keys layer
#define _MOU   8  // Mouse keys layer
#define _SUP   9  // Supplement layer for smart CTL
#define _CLN  10  // Clean layer, provide the ISO keyboard layout with F1~F12
#define _BLK  11  // Blank layer, just do nothing
#define _KEY  12  // Keybaord settings layer

// Tricky ESC
#define GRV_LCTL   F(0)
#define GRV_LALT   F(1)
#define GRV_LGUI   F(2)
#define GRV_LSFT   F(3)

// Layer Control
#define L_DEF      F(4)
#define L_ORW      F(5)
#define L_ARW      TG(_ARW)
#define L_MOU      TG(_MOU)
#define L_BLK      MO(_BLK)
#define L_KEY      MO(_KEY)

// Smart keys
#define SMT_LCTL   MO(_CTL)
#define SMT_RSFT   MO(_SFT)

// Dual keys
#define TAB_DUAL   LT(_WIN, KC_TAB)
#define ESC_DUAL   LT(_LFN, KC_ESC)
#define BSLS_DUAL  LT(_RFN, KC_BSLS)
#define GUI_DUAL   MT(MOD_RGUI, KC_F16)              /* F16                 = List application windows                 BTT */
#define CLN_DUAL   LT(_CLN, KC_RGHT)

// Shortcuts
// NOTE: Don't use LCTL with F14, F15, seems they are built-in brightness control on macOS
#define PREV_DP    LCTL(KC_LEFT)                     /* LCTL + LEFT         = Switch to previous display             macOS */
#define NEXT_DP    LCTL(KC_RGHT)                     /* LCTL + RGHT         = Switch to next display                 macOS */
#define OP_ALFED   LCTL(LALT(KC_SPC))                /* LCTL + LALT + SPAC  = Open Alfred                           Alfred */
#define OP_DICT    LALT(KC_D)                        /* LALT + D            = Dictionary                               BTT */
#define OP_AIRML   LCTL(LSFT(LALT(LGUI(KC_M))))      /* HYPER + M           = Open Airmail                         Airmail */
#define OP_TASKS   LCTL(LSFT(LALT(LGUI(KC_T))))      /* HYPER + T           = Show/Hide Todoist                    Todoist */
#define OP_SLACK   LCTL(LSFT(LALT(LGUI(KC_S))))      /* HYPER + S           = Open Slack                               BTT */
#define OP_CLPIK   LCTL(LSFT(LALT(LGUI(KC_C))))      /* HYPER + C           = Color Picker                             BTT */
#define OP_MENUC   LCTL(LSFT(LGUI(KC_SLSH)))         /* MEH + SLSH          = Open menubar in context menu             BTT */
#define SW_FOCUS   KC_F13                            /* F13                 = Move focus to active or next window    macOS */
#define BRIT_DN    KC_F14                            /* F14                 = Brightness decrease                    macOS */
#define BRIT_UP    KC_F15                            /* F15                 = Brightness increase                    macOS */
#define CONTEXTS   KC_F18                            /* F18                 = Contexts, search mode hotkey        Contexts */
#define IME_SWIC   KC_F19                            /* F19                 = Switch IME                             macOS */
#define GOOGL_IT   LCTL(KC_F16)                      /* LCTL + F16          = Google selected text                     BTT */
#define EMP_TRSH   LCTL(KC_F17)                      /* LCTL + F17          = Empty trash                              BTT */
#define OP_NOTIF   LCTL(KC_F18)                      /* LCTL + F18          = Show Notifications Center              macOS */
#define FUL_SCRN   LCTL(KC_F19)                      /* LCTL + F19          = Full screen window                       BTT */
#define LOCK_SCR   LCTL(LSFT(KC_POWER))              /* LCTL + SFT + POWER  = Lock screen                            macOS */

// Not using
#define LS_WINS    LCTL(KC_DOWN)                     /* LCTL + DOWN         = List application windows               macOS */
#define OP_FINDR   LALT(KC_E)                        /* LALT + E            = Open Finder                              BTT */

// Windows management hotkeys, provide by macOS & Moom
#define W_SIZE1    LCTL(KC_1)
#define W_SIZE2    LCTL(KC_2)
#define W_SIZE3    LCTL(KC_3)
#define W_SIZE4    LCTL(KC_4)
#define W_SIZE5    LCTL(KC_5)
#define W_MOVEU    LALT(KC_UP)
#define W_MOVER    LALT(KC_RGHT)
#define W_MOVED    LALT(KC_DOWN)
#define W_MOVEL    LALT(KC_LEFT)
#define W_MOVEC    LALT(KC_TAB)
#define W_MOVEDP   LCTL(KC_GRV)
#define W_PUTU     LCTL(LGUI(KC_UP))
#define W_PUTR     LCTL(LGUI(KC_RGHT))
#define W_PUTD     LCTL(LGUI(KC_DOWN))
#define W_PUTL     LCTL(LGUI(KC_LEFT))
#define W_PREVSP   LCTL(LALT(LGUI(KC_LEFT)))
#define W_NEXTSP   LCTL(LALT(LGUI(KC_RGHT)))
#define W_MIN      LGUI(KC_M)
#define W_REVERT   LALT(KC_BSPC)

/*
 * Other shortcuts note
 *   LGUI + F18 = 1Password, show dialog hotkey
 */

// Used for custom macros
enum keyboard_macros {
  MACRO_BREATH_TOGGLE,
  MACRO_BREATH_SPEED_INC,
  MACRO_BREATH_SPEED_DEC,
  MACRO_BREATH_DEFAULT
};

// Custom macros
#define M_BRTOG    M(MACRO_BREATH_TOGGLE)
#define M_BSPDU    M(MACRO_BREATH_SPEED_INC)
#define M_BSPDD    M(MACRO_BREATH_SPEED_DEC)
#define M_BDFLT    M(MACRO_BREATH_DEFAULT)

// Used for custom action functions
#define MODS_CTL_MASK  (MOD_BIT(KC_LCTL) | MOD_BIT(KC_RCTL))

// Keymap definitions
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

[_DEF] = KEYMAP_ANSI(
   ESC_DUAL, KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINS,  KC_EQL,   KC_BSPC,  \
   TAB_DUAL, KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRC,  KC_RBRC,  BSLS_DUAL,\
   SMT_LCTL, KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,  KC_QUOT,            KC_ENT,   \
   GRV_LSFT,           KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,   KC_SLSH,            SMT_RSFT, \
   KC_LCTL,  GRV_LALT, GRV_LGUI,                               KC_SPC,                                 KC_LEFT,  KC_UP,    KC_DOWN,  KC_RGHT   ),


[_ESC] = KEYMAP_ANSI(
   KC_GRV,   ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, \
   ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, \
   ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________,           ________, \
   ________,           ________, ________, ________, ________, ________, ________, ________, ________, ________, ________,           ________, \
   ________, ________, ________,                               ________,                               KC_LEFT,  KC_DOWN,  KC_UP,    KC_RGHT   ),

[_SFT] = KEYMAP_ANSI(
   KC_GRV,   KC_F1,    KC_F2,    KC_F3,    KC_F4,     KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,   KC_F12,   ________, \
   ________, ________, KC_INSERT, KC_HOME, KC_PGUP,   ________, ________, ________, ________, ________, ________, ________, ________, ________, \
   KC_CAPS,  ________, KC_DELETE, KC_END,  KC_PGDOWN, ________, ________, ________, ________, ________, ________, ________,           ________, \
   GRV_LSFT,           ________, ________, ________,  ________, ________, ________, ________, ________, ________, ________,           ________, \
   ________, ________, ________,                                ________,                               ________, ________, ________, ________  ),

[_CTL] = KEYMAP_ANSI(
   C(KC_GRV), C(KC_1),    C(KC_2),    C(KC_3),    C(KC_4),  C(KC_5),  C(KC_6),  C(KC_7),  C(KC_8),  C(KC_9),    C(KC_0),    C(KC_MINS), C(KC_EQL),  C(KC_BSPC), \
   L_ORW,     C(KC_Q),    C(KC_W),    C(KC_E),    C(KC_R),  C(KC_T),  KC_HOME,  KC_PGUP,  KC_PGDN,  KC_END,     C(KC_P),    C(KC_LBRC), C(KC_RBRC), C(KC_BSLS), \
   ________,  C(KC_A),    C(KC_S),    C(KC_D),    C(KC_F),  C(KC_G),  KC_LEFT,  KC_DOWN,  KC_UP,    KC_RGHT,    C(KC_SCLN), C(KC_QUOT),             ________,   \
   ________,              C(KC_Z),    C(KC_X),    C(KC_C),  C(KC_V),  C(KC_B),  C(KC_N),  C(KC_M),  C(KC_COMM), C(KC_DOT),  C(KC_SLSH),             KC_RSFT,    \
   ________,  ________,   ________,                                   IME_SWIC,                                 C(KC_LEFT), C(KC_DOWN), C(KC_UP),   C(KC_RGHT)  ),

[_WIN] = KEYMAP_ANSI(
   W_MOVEDP, W_SIZE1,  W_SIZE2,  W_SIZE3,  W_SIZE4,  W_SIZE5,  ________, ________, ________, ________, ________, ________, ________, W_REVERT, \
   ________, ________, W_MOVEC,  ________, ________, ________, W_PUTL,   W_PUTD,   W_PUTU,   W_PUTR,   ________, PREV_DP,  NEXT_DP,  CONTEXTS, \
   KC_LCTL,  ________, ________, ________, ________, ________, W_MOVEL,  W_MOVED,  W_MOVEU,  W_MOVER,  ________, ________,           W_MOVEC,  \
   ________,           ________, ________, ________, ________, ________, ________, ________, ________, ________, ________,           KC_RSFT,  \
   ________, ________, ________,                               OP_ALFED,                               GUI_DUAL, W_MIN,    W_PREVSP, W_NEXTSP  ),

[_LFN] = KEYMAP_ANSI(
   ________, KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,   KC_F12,   KC_DEL,   \
   KC_TAB,   BRIT_DN,  BRIT_UP,  EMP_TRSH, ________, OP_TASKS, ________, ________, ________, ________, KC_MPLY,  ________, ________, KC_BSLS,  \
   KC_CAPS,  ________, OP_SLACK, OP_DICT,  FUL_SCRN, GOOGL_IT, KC_MRWD,  KC_VOLD,  KC_VOLU,  KC_MFFD,  ________, ________,           LOCK_SCR, \
   ________,           ________, ________, OP_CLPIK, ________, ________, OP_NOTIF, KC_MUTE,  ________, ________, OP_MENUC,           KC_RSFT,  \
   ________, ________, ________,                               L_ARW,                                  KC_MPLY,  KC_MUTE,  KC_MRWD,  KC_MFFD   ),

[_RFN] = KEYMAP_ANSI(
   W_MOVEDP, W_SIZE1,  W_SIZE2,  W_SIZE3,  W_SIZE4,  W_SIZE5,  ________, ________, W_SIZE5,  W_SIZE4,  W_SIZE3,  W_SIZE2,  W_SIZE1,  W_MOVEDP, \
   CONTEXTS, ________, W_MOVEC,  EMP_TRSH, ________, OP_TASKS, ________, ________, ________, ________, ________, BRIT_DN,  BRIT_UP,  ________, \
   KC_CAPS,  ________, OP_SLACK, OP_DICT,  FUL_SCRN, GOOGL_IT, ________, ________, ________, ________, ________, ________,           LOCK_SCR, \
   ________,           ________, ________, OP_CLPIK, ________, ________, OP_NOTIF, OP_AIRML, ________, ________, ________,           KC_RSFT,  \
   ________, ________, ________,                               L_MOU,                                  KC_MPLY,  KC_MUTE,  KC_MRWD,  KC_MFFD   ),

[_ARW] = KEYMAP_ANSI(
   ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, \
   ________, KC_7,     KC_8,     KC_9,     ________, ________, KC_HOME,  KC_PGUP,  KC_PGDN,  KC_END,   ________, ________, ________, ________, \
   ________, KC_4,     KC_5,     KC_6,     ________, ________, KC_LEFT,  KC_DOWN,  KC_UP,    KC_RGHT,  ________, ________,           ________, \
   ________,           KC_1,     KC_2,     KC_3,     ________, ________, ________, ________, ________, ________, ________,           ________, \
   ________, ________, KC_0,                                   L_DEF,                                  KC_LEFT,  KC_DOWN,  KC_UP,    KC_RGHT   ),

[_MOU] = KEYMAP_ANSI(
   ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, ________, \
   ________, KC_ACL2,  KC_ACL0,  ________, ________, ________, KC_WH_L,  KC_WH_D,  KC_WH_U,  KC_WH_R,  ________, ________, ________, ________, \
   ________, ________, ________, ________, ________, ________, KC_MS_L,  KC_MS_D,  KC_MS_U,  KC_MS_R,  ________, ________,           ________, \
   ________,           ________, ________, ________, ________, KC_BTN1,  KC_BTN2,  KC_BTN3,  ________, ________, ________,           ________, \
   ________, ________, ________,                               L_DEF,                                  KC_MS_L,  KC_MS_D,  KC_MS_U,  KC_MS_R   ),

[_SUP] = KEYMAP_ANSI(
   KC_ESC,   KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,   KC_F12,   KC_DEL,   \
   ________, KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRC,  KC_RBRC,  KC_BSLS,  \
   ________, KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,  KC_QUOT,            KC_ENT,   \
   ________,           KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,   KC_SLSH,            KC_RSFT,  \
   ________, ________, ________,                               KC_SPC,                                 KC_LEFT,  KC_DOWN,  KC_UP,    KC_RGHT   ),

[_CLN] = KEYMAP_ANSI(
   KC_GRV,   KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINS,  KC_EQL,   KC_BSPC,  \
   KC_TAB,   KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRC,  KC_RBRC,  KC_BSLS,  \
   KC_LCTL,  KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,  KC_QUOT,            KC_ENT,   \
   KC_LSFT,            KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,   KC_SLSH,            KC_RSFT,  \
   XXXXXXXX, KC_LALT,  KC_LGUI,                                KC_SPC,                                 KC_RGUI,  KC_RALT,  KC_RCTL,  ________  ),

[_BLK] = KEYMAP_ANSI(
   XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, \
   XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, \
   XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,           XXXXXXXX, \
   XXXXXXXX,           XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,           XXXXXXXX, \
   ________, XXXXXXXX, XXXXXXXX,                               XXXXXXXX,                               XXXXXXXX, XXXXXXXX, XXXXXXXX, L_KEY     ),

[_KEY] = KEYMAP_ANSI(
   XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, \
   XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, \
   XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,           RESET,    \
   XXXXXXXX,           BL_DEC,   BL_INC,   BL_TOGG,  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, M_BRTOG,  M_BSPDD,  M_BSPDU,            XXXXXXXX, \
   ________, XXXXXXXX, XXXXXXXX,                               XXXXXXXX,                               XXXXXXXX, XXXXXXXX, XXXXXXXX, ________  ),

};

enum function_id {
  TRICKY_ESC,
};

// FN actions definitions
const uint16_t PROGMEM fn_actions[] = {
  [0] = ACTION_LAYER_MODS(_ESC, MOD_LCTL),
  [1] = ACTION_LAYER_MODS(_ESC, MOD_LALT),
  [2] = ACTION_LAYER_MODS(_ESC, MOD_LGUI),
  [3] = ACTION_LAYER_MODS(_ESC, MOD_LSFT),
  [4] = ACTION_LAYER_SET_CLEAR(_DEF),
  [5] = ACTION_LAYER_MODS(_SUP, MOD_LCTL),
//[0] = ACTION_FUNCTION(TRICKY_ESC)
};

void action_function(keyrecord_t *record, uint8_t id, uint8_t opt) {
  static uint8_t ctrl_mod_mask;
  switch (id) {
    case TRICKY_ESC:
      ctrl_mod_mask = get_mods() & MODS_CTL_MASK;
      if (record->event.pressed) {
        if (ctrl_mod_mask) {
          add_key(KC_GRV);
        } else {
          add_key(KC_ESC);
        }
      } else {
        if (ctrl_mod_mask) {
          del_key(KC_GRV);
        } else {
          del_key(KC_ESC);
        }
      }
      send_keyboard_report();
      break;
  }
}

// Macros definitions
const macro_t *action_get_macro(keyrecord_t *record, uint8_t id, uint8_t opt) {
  switch (id) {
    case MACRO_BREATH_TOGGLE:
      if (record->event.pressed) {
        breathing_toggle();
      }
      break;

    case MACRO_BREATH_SPEED_INC:
      if (record->event.pressed) {
        breathing_speed_inc(1);
      }
      break;

    case MACRO_BREATH_SPEED_DEC:
      if (record->event.pressed) {
        breathing_speed_dec(1);
      }
      break;

    case MACRO_BREATH_DEFAULT:
      if (record->event.pressed) {
        breathing_defaults();
      }
      break;

    default:
      break;
  }
  return MACRO_NONE;
}

/*
 * LED related code
 *  according to the information from tmk project
 *  GH60_REV_CHN has only LED on
 *    CapsLock - PB2
 *    PWM      - PB6
 *  Other GH60 has
 *    LED Row pin configuration
 *    row: 0   1   2   3   4   5
 *    pin: B6  F5  F6  F7  F4  B2
 */
void led_matrix_row_on(uint8_t row)
{
  switch (row) {
    case 0:
      DDRB  |= (1<<PB6); PORTB &= ~(1<<PB6);
      break;
    case 1:
      DDRF  |= (1<<PF5); PORTF &= ~(1<<PF5);
      break;
    case 2:
      DDRF  |= (1<<PF6); PORTF &= ~(1<<PF6);
      break;
    case 3:
      DDRF  |= (1<<PF7); PORTF &= ~(1<<PF7);
      break;
    case 4:
      DDRF  |= (1<<PF4); PORTF &= ~(1<<PF4);
      break;
    case 5:
      DDRB  |= (1<<PB2); PORTB &= ~(1<<PB2);
  }
}

void led_matrix_row_off(uint8_t row)
{
  switch (row) {
    case 0:
      DDRB  &= ~(1<<PB6); PORTB &= ~(1<<PB6);
      break;
    case 1:
      DDRF  &= ~(1<<PF5); PORTF &= ~(1<<PF5);
      break;
    case 2:
      DDRF  &= ~(1<<PF6); PORTF &= ~(1<<PF6);
      break;
    case 3:
      DDRF  &= ~(1<<PF7); PORTF &= ~(1<<PF7);
      break;
    case 4:
      DDRF  &= ~(1<<PF4); PORTF &= ~(1<<PF4);
      break;
    case 5:
      DDRB  &= ~(1<<PB2); PORTB &= ~(1<<PB2);
  }
}

static void indicate_using_led(const uint8_t row, const bool enabled) {
  if (enabled) {
    led_matrix_row_on(row);
  } else {
    led_matrix_row_off(row);
  }
}

static inline void indicate_smart_key_pressed(const bool enabled) {
  indicate_using_led(5, enabled);
}

static inline void indicate_layer_changed(const bool enabled) {
  indicate_using_led(0, enabled);
}

// Runs constantly in the background, in a loop.
void matrix_scan_user(void) {
  // GH60 Santa only support PWM or CapsLock LED switch
  // we use CapsLock LED to indicate fn layer, give up capital status
  indicate_smart_key_pressed(
    IS_LAYER_ON(_CTL) ||
    IS_LAYER_ON(_SFT) ||
    IS_LAYER_ON(_WIN) ||
    IS_LAYER_ON(_LFN) ||
    IS_LAYER_ON(_RFN)
  );
  indicate_layer_changed(
    IS_LAYER_ON(_ARW) ||
    IS_LAYER_ON(_MOU) ||
    IS_LAYER_ON(_SUP) ||
    IS_LAYER_ON(_CLN) ||
    IS_LAYER_ON(_KEY)
  );
};

void matrix_init_user(void) {
  // breathing_enable();
  breathing_speed_set(4);
}
