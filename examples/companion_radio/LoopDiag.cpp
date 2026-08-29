#if defined(ESP32)
#include "LoopDiag.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_attr.h>
#include <esp_system.h>

// RTC-noinit survives every reset except full power removal, which is exactly
// the lifetime we want: the phase written just before a watchdog reset is still
// readable after the reboot.
RTC_NOINIT_ATTR static uint32_t diag_magic;
RTC_NOINIT_ATTR static uint8_t  diag_cur_phase;
RTC_NOINIT_ATTR static uint8_t  diag_hung_phase;
RTC_NOINIT_ATTR static uint16_t diag_rescues;

static esp_reset_reason_t boot_reason;
static const uint32_t DIAG_MAGIC = 0x4C504447;

static const char* phase_name(uint8_t p) {
  switch (p) {
    case loop_diag::IDLE:       return "idle";
    case loop_diag::MESH:       return "mesh";
    case loop_diag::PRESENCE:   return "pres";
    case loop_diag::INTERFACES: return "intf";
    case loop_diag::SENSORS:    return "sens";
    case loop_diag::UI:         return "ui";
    case loop_diag::CLOCK:      return "clk";
  }
  return "unk";
}

static const char* reason_name(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "pwron";
    case ESP_RST_SW:        return "sw";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "intwdt";
    case ESP_RST_TASK_WDT:  return "taskwdt";
    case ESP_RST_WDT:       return "wdt";
    case ESP_RST_DEEPSLEEP: return "sleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    default:                return "other";
  }
}

void loop_diag::begin() {
  boot_reason = esp_reset_reason();
  if (diag_magic != DIAG_MAGIC) {   // cold power-on: RTC memory is garbage
    diag_magic = DIAG_MAGIC;
    diag_cur_phase = IDLE;
    diag_hung_phase = 0xFF;
    diag_rescues = 0;
  } else if (boot_reason == ESP_RST_TASK_WDT || boot_reason == ESP_RST_INT_WDT
          || boot_reason == ESP_RST_WDT || boot_reason == ESP_RST_PANIC) {
    diag_hung_phase = diag_cur_phase;
    diag_rescues++;
  }
  Serial.printf("loop_diag: %s\n", summary());
  esp_task_wdt_init(60, true);   // panic (-> reboot) if the loop stalls 60s
  esp_task_wdt_add(NULL);        // watch the task running setup()/loop()
}

void loop_diag::mark(Phase p) { diag_cur_phase = p; }
void loop_diag::feed() { esp_task_wdt_reset(); }

const char* loop_diag::summary() {
  static char buf[24];
  snprintf(buf, sizeof(buf), "%s.%s.%u", reason_name(boot_reason),
           diag_hung_phase == 0xFF ? "none" : phase_name(diag_hung_phase),
           (unsigned)diag_rescues);
  return buf;
}
#endif
