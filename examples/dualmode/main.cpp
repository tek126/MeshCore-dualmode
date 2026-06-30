// Dual-mode launcher for the Seeed T1000-E.
//
// Co-compiles the stock simple_repeater and companion_radio apps into one
// firmware and chooses which one to run at boot based on a flag persisted in
// the internal filesystem. Pressing the user button five times (a press count
// not used by either app) toggles the flag and reboots into the other mode.
//
// The two apps' entry points are renamed to rpt_*/cmp_* when built with
// -D DUALMODE (see each app's main.cpp), so this file owns the real Arduino
// setup()/loop().

#include <Arduino.h>
#include <InternalFileSystem.h>
#include "Button.h"   // from companion_radio/ui-orig (on the include path)

using namespace Adafruit_LittleFS_Namespace;

#ifndef PIN_USER_BTN
  #error "dualmode requires PIN_USER_BTN"
#endif
#ifndef USER_BTN_PRESSED
  #define USER_BTN_PRESSED HIGH
#endif

// Per-app entry points (defined in each app's main.cpp under -D DUALMODE).
void rpt_setup();
void rpt_loop();
void cmp_setup();
void cmp_loop();

enum AppMode { MODE_COMPANION = 0, MODE_REPEATER = 1 };

static const char* MODE_PATH = "/dualmode";
static AppMode g_mode = MODE_COMPANION;

static AppMode readMode() {
  InternalFS.begin();   // idempotent; the chosen app calls begin() again later
  AppMode m = MODE_COMPANION;
  File f(InternalFS);
  if (f.open(MODE_PATH, FILE_O_READ)) {
    int c = f.read();
    if (c == 'R') m = MODE_REPEATER;
    f.close();
  }
  return m;
}

static void writeMode(AppMode m) {
  InternalFS.begin();
  InternalFS.remove(MODE_PATH);
  File f(InternalFS);
  if (f.open(MODE_PATH, FILE_O_WRITE)) {
    f.write((uint8_t)(m == MODE_REPEATER ? 'R' : 'C'));
    f.close();
  }
}

static Button mode_btn(PIN_USER_BTN, USER_BTN_PRESSED);

// --- Button activity window (keeps the repeater awake during a press burst) ---
// A GPIO interrupt on the button wakes the MCU from power-saving sleep on the
// first edge; this timestamp then holds sleep off for long enough to clock in
// the whole 5x sequence and its ~500ms dispatch tail. Set from the ISR (on every
// edge) and from onAnyPress as a backstop.
static const uint32_t BTN_ACTIVE_WINDOW_MS = 1500;
static volatile uint32_t g_last_btn_ms = 0;

static void onButtonEdgeISR() {
  g_last_btn_ms = millis();
}

// Consulted by the repeater loop (declared extern there) to gate power-saving
// sleep. True while the button is held or recently active.
bool dualmode_button_active() {
  return mode_btn.isPressed() ||
         (uint32_t)(millis() - g_last_btn_ms) < BTN_ACTIVE_WINDOW_MS;
}

// Drive the buzzer with a manual square wave (this core has no tone(); the
// stock buzzer uses RTTTL, but a self-contained beep is simplest here since we
// reset immediately after). Also blink the status LED. The beep count tells you
// the target mode without a screen: 2 beeps = repeater, 1 beep = companion.
static void confirmFeedback(uint8_t beeps) {
#ifdef PIN_STATUS_LED
  pinMode(PIN_STATUS_LED, OUTPUT);
#endif
#ifdef PIN_BUZZER
  #ifdef PIN_BUZZER_EN
    pinMode(PIN_BUZZER_EN, OUTPUT);
    digitalWrite(PIN_BUZZER_EN, HIGH);   // enable buzzer amp
  #endif
  pinMode(PIN_BUZZER, OUTPUT);
#endif

  for (uint8_t b = 0; b < beeps; b++) {
#ifdef PIN_STATUS_LED
    digitalWrite(PIN_STATUS_LED, HIGH);
#endif
#ifdef PIN_BUZZER
    // ~2.2 kHz for ~150 ms: half-period ~227 us.
    for (uint16_t i = 0; i < 330; i++) {
      digitalWrite(PIN_BUZZER, HIGH);
      delayMicroseconds(227);
      digitalWrite(PIN_BUZZER, LOW);
      delayMicroseconds(227);
    }
#else
    delay(150);
#endif
#ifdef PIN_STATUS_LED
    digitalWrite(PIN_STATUS_LED, LOW);
#endif
    delay(120);   // gap between beeps
  }

#ifdef PIN_BUZZER
  digitalWrite(PIN_BUZZER, LOW);          // leave low to avoid power draw
  #ifdef PIN_BUZZER_EN
    digitalWrite(PIN_BUZZER_EN, LOW);
  #endif
#endif
}

static void onModeSwitch() {
  // Persist the opposite of whatever we booted as, then reboot into it.
  AppMode target = (g_mode == MODE_REPEATER) ? MODE_COMPANION : MODE_REPEATER;
  writeMode(target);
  confirmFeedback(target == MODE_REPEATER ? 2 : 1);
  delay(150);            // let the flash write settle
  NVIC_SystemReset();    // does not return
}

void setup() {
  g_mode = readMode();

  if (g_mode == MODE_REPEATER) {
    rpt_setup();
  } else {
    cmp_setup();
  }

  // The chosen app's setup() has initialised the board; now arm the
  // mode-switch button. Active-HIGH button idles low, so use a pulldown.
  pinMode(PIN_USER_BTN, (USER_BTN_PRESSED == HIGH) ? INPUT_PULLDOWN : INPUT_PULLUP);
  mode_btn.begin();
  mode_btn.onQuintuplePress(onModeSwitch);
  mode_btn.onAnyPress([]() { g_last_btn_ms = millis(); });

  // Arm a GPIO interrupt so a button edge wakes the MCU out of the repeater's
  // power-saving sleep (sd_app_evt_wait/WFE wakes on any event). CHANGE catches
  // both press and release so the active window keeps extending through a burst.
  attachInterrupt(digitalPinToInterrupt(PIN_USER_BTN), onButtonEdgeISR, CHANGE);
}

void loop() {
  mode_btn.update();

  if (g_mode == MODE_REPEATER) {
    rpt_loop();
  } else {
    cmp_loop();
  }
}
