#pragma once

#include <stdint.h>

/*
 * Compile-time plausibility window for wall-clock time.
 *
 * Several clock sources have proven capable of delivering wildly wrong
 * times: cold-starting GNSS receivers emit valid-flagged sentences with
 * garbage dates (correct time-of-day, date decades off), and the ESP32's
 * RTC-domain time restore can land years away after the RTC memory is
 * corrupted by a brownout.  Because every legitimate clock-set path in
 * MeshCore is forward-only, a single far-future timestamp sticks forever
 * and propagates (replay checks, contact records, clock sync).
 *
 * The window: nothing can legitimately claim a time more than ~5 years
 * after this firmware was compiled.  Anything beyond that is treated as a
 * corrupt source.  The trade-off is that a node still running this build
 * more than 5 years from now would start rejecting genuine times; a
 * reflash (or any newer build) moves the window forward.
 */
namespace time_sanity {

// __DATE__ is "Mmm dd yyyy" (e.g. "Aug 21 2026"); all expressions are
// single-return so this stays C++11-constexpr for every toolchain we build.
constexpr int _build_year() {
  return (__DATE__[7]-'0')*1000 + (__DATE__[8]-'0')*100 + (__DATE__[9]-'0')*10 + (__DATE__[10]-'0');
}
constexpr int _build_month() {
  return __DATE__[0]=='J' ? (__DATE__[1]=='a' ? 1 : (__DATE__[2]=='n' ? 6 : 7))
       : __DATE__[0]=='F' ? 2
       : __DATE__[0]=='M' ? (__DATE__[2]=='r' ? 3 : 5)
       : __DATE__[0]=='A' ? (__DATE__[1]=='p' ? 4 : 8)
       : __DATE__[0]=='S' ? 9
       : __DATE__[0]=='O' ? 10
       : __DATE__[0]=='N' ? 11 : 12;
}
constexpr int _build_day() {
  return (__DATE__[4]==' ' ? 0 : (__DATE__[4]-'0')*10) + (__DATE__[5]-'0');
}
constexpr long _days_before_month(int m) {   // in a non-leap year
  return m==1?0:m==2?31:m==3?59:m==4?90:m==5?120:m==6?151:m==7?181:m==8?212:m==9?243:m==10?273:m==11?304:334;
}
// days since 1970-01-01; valid 2001..2099 (no century leap exception in range)
constexpr long _days_since_epoch(int y, int m, int d) {
  return (long)(y-1970)*365 + (y-1969)/4 + _days_before_month(m) + ((m > 2 && y % 4 == 0) ? 1 : 0) + (d - 1);
}

constexpr uint32_t BUILD_EPOCH =
    (uint32_t)_days_since_epoch(_build_year(), _build_month(), _build_day()) * 86400UL;

// build date + ~5 years
constexpr uint32_t MAX_PLAUSIBLE_EPOCH = BUILD_EPOCH + 5UL*365UL*86400UL;

static_assert(BUILD_EPOCH > 1735689600UL, "TimeSanity: build epoch parsed before 2025?");   // 2025-01-01
static_assert(BUILD_EPOCH < 4102444800UL, "TimeSanity: build epoch parsed past 2100?");     // 2100-01-01

// true if 't' could be a real current time as far as this build can know
constexpr bool plausible(uint32_t t) {
  return t <= MAX_PLAUSIBLE_EPOCH;
}

}  // namespace time_sanity
