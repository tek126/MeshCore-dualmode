#pragma once
#include <stdint.h>

// ESP32 loop-hang diagnostics: a 60s task watchdog on the loop task (a wedged
// loop reboots itself instead of needing a power cycle) plus a phase breadcrumb
// kept in RTC-noinit memory, so after a watchdog rescue the phone app can ask
// which subsystem the loop was stuck in (read-only custom var "sys.diag").
// nRF52 boards already have a 90s hardware watchdog; the macros no-op there.
#if defined(ESP32)
namespace loop_diag {
  enum Phase : uint8_t { IDLE = 0, MESH, PRESENCE, INTERFACES, SENSORS, UI, CLOCK };
  void begin();              // classify the reset + arm the watchdog (call from setup)
  void mark(Phase p);        // record the subsystem about to run (cheap store)
  void feed();               // pet the watchdog, once per healthy loop pass
  const char* summary();     // "<reset>.<hung-phase>.<rescues>", e.g. "taskwdt.pres.2"
}
  #define LOOP_DIAG_BEGIN()  loop_diag::begin()
  #define LOOP_DIAG_MARK(p)  loop_diag::mark(loop_diag::p)
  #define LOOP_DIAG_FEED()   loop_diag::feed()
#else
  #define LOOP_DIAG_BEGIN()
  #define LOOP_DIAG_MARK(p)
  #define LOOP_DIAG_FEED()
#endif
