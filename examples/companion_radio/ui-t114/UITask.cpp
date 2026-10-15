#include "UITask.h"
#include <helpers/TxtDataHelpers.h>
#include <RTClib.h>
#include "../MyMesh.h"
#include "target.h"
#include "../ui-new/icons.h"

#ifndef AUTO_OFF_MILLIS
  #define AUTO_OFF_MILLIS     15000   // 15 seconds
#endif
#define BOOT_SCREEN_MILLIS   3000     // 3 seconds

#define UI_RECENT_LIST_SIZE  6

// Layout (240x135): an 18 px status bar, six 18 px content rows, page dots.
#define SCREEN_W      ST7789ColorDisplay::W
#define STATUS_H      18
#define ROW_Y(i)      (21 + (i) * 18)
#define DOTS_Y        131

using namespace TftColor;

static inline ST7789ColorDisplay& tft(DisplayDriver& d) { return static_cast<ST7789ColorDisplay&>(d); }

static void fmtAge(char* buf, int secs) {
  if (secs < 0) secs = 0;
  if (secs < 10)            strcpy(buf, "now");
  else if (secs < 60)       sprintf(buf, "%ds", secs);
  else if (secs < 3600)     sprintf(buf, "%dm", secs / 60);
  else if (secs < 86400)    sprintf(buf, "%dh", secs / 3600);
  else                      sprintf(buf, "%dd", secs / 86400);
}

static int battPercent(uint16_t mv) {
#ifndef BATT_MIN_MILLIVOLTS
  #define BATT_MIN_MILLIVOLTS 3000
#endif
#ifndef BATT_MAX_MILLIVOLTS
  #define BATT_MAX_MILLIVOLTS 4200
#endif
  int pct = ((int)mv - BATT_MIN_MILLIVOLTS) * 100 / (BATT_MAX_MILLIVOLTS - BATT_MIN_MILLIVOLTS);
  return pct < 0 ? 0 : (pct > 100 ? 100 : pct);
}

// a clock still sitting at its boot default has not been set
static bool clockIsSet(uint32_t t) { return t > 1735689600UL; }   // 2025-01-01

// ---------------- geo ----------------

static const double DEG = 0.017453292519943295;

static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  double dlat = (lat2 - lat1) * DEG, dlon = (lon2 - lon1) * DEG;
  double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1 * DEG) * cos(lat2 * DEG) * sin(dlon / 2) * sin(dlon / 2);
  return 6371000.0 * 2 * atan2(sqrt(a), sqrt(1 - a));
}

// initial bearing from point 1 to point 2, degrees clockwise from north
static double bearingDegrees(double lat1, double lon1, double lat2, double lon2) {
  double y = sin((lon2 - lon1) * DEG) * cos(lat2 * DEG);
  double x = cos(lat1 * DEG) * sin(lat2 * DEG) - sin(lat1 * DEG) * cos(lat2 * DEG) * cos((lon2 - lon1) * DEG);
  double b = atan2(y, x) / DEG;
  return b < 0 ? b + 360 : b;
}

static void fmtDistance(char* buf, double m) {
  if (m < 1000)        sprintf(buf, "%d m", (int)(m + 0.5));
  else if (m < 10000)  sprintf(buf, "%.1f km", m / 1000);
  else                 sprintf(buf, "%d km", (int)(m / 1000 + 0.5));
}

static const char* compassPoint(double b) {
  static const char* P[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
  return P[(int)((b + 22.5) / 45) % 8];
}

// filled arrow centred on (cx, cy), pointing at bearing (0 = up/north)
static void drawArrow(Adafruit_GFX& g, int cx, int cy, int r, double bearing, uint16_t c) {
  double a = bearing * DEG;
  int tx = cx + (int)(r * sin(a)),             ty = cy - (int)(r * cos(a));
  int lx = cx + (int)(r * 0.6 * sin(a + 2.5)), ly = cy - (int)(r * 0.6 * cos(a + 2.5));
  int rx = cx + (int)(r * 0.6 * sin(a - 2.5)), ry = cy - (int)(r * 0.6 * cos(a - 2.5));
  int nx = cx - (int)(r * 0.25 * sin(a)),      ny = cy + (int)(r * 0.25 * cos(a));   // notch
  g.fillTriangle(tx, ty, lx, ly, nx, ny, c);
  g.fillTriangle(tx, ty, rx, ry, nx, ny, c);
}

// ---------------- message store ----------------

void UIMessageStore::add(uint32_t ts, uint8_t hops, bool channel, const char* from, const char* text) {
  _head = (_head + 1) % UI_MSG_HISTORY;
  if (_count < UI_MSG_HISTORY) _count++;
  UIMessage* m = &_msgs[_head];
  m->timestamp = ts;
  m->hops = hops;
  m->channel = channel;
  m->unread = true;
  StrHelper::strncpy(m->from, from, sizeof(m->from));
  StrHelper::strncpy(m->text, text, sizeof(m->text));
}

int UIMessageStore::unread() const {
  int n = 0;
  for (int i = 0; i < _count; i++) {
    if (_msgs[(_head - i + UI_MSG_HISTORY) % UI_MSG_HISTORY].unread) n++;
  }
  return n;
}

UIMessage* UIMessageStore::get(int i) {
  if (i < 0 || i >= _count) return NULL;
  return &_msgs[(_head - i + UI_MSG_HISTORY) % UI_MSG_HISTORY];
}

void UIMessageStore::markAllRead() {
  for (int i = 0; i < _count; i++) get(i)->unread = false;
}

// ---------------- shared drawing ----------------

static void drawBluetoothGlyph(Adafruit_GFX& g, int x, int y, uint16_t c) {
  g.drawLine(x + 3, y, x + 3, y + 11, c);
  g.drawLine(x + 3, y, x + 6, y + 3, c);
  g.drawLine(x + 6, y + 3, x, y + 8, c);
  g.drawLine(x + 3, y + 11, x + 6, y + 8, c);
  g.drawLine(x + 6, y + 8, x, y + 3, c);
}

static void drawEnvelope(Adafruit_GFX& g, int x, int y, uint16_t c) {
  g.drawRect(x, y, 13, 9, c);
  g.drawLine(x, y, x + 6, y + 5, c);
  g.drawLine(x + 12, y, x + 6, y + 5, c);
}

// Status bar: title on the left; unread, GPS, Bluetooth, clock, battery on the right.
static void drawStatusBar(DisplayDriver& display, UITask* task, mesh::RTCClock* rtc, const char* title) {
  ST7789ColorDisplay& d = tft(display);
  Adafruit_GFX& g = d.gfx();
  d.setColor(UIColor::title_bkg);
  d.fillRect(0, 0, SCREEN_W, STATUS_H);

  // battery, right edge
  int pct = battPercent(task->getBattMilliVolts());
  int bx = SCREEN_W - 27, by = 4;
  g.drawRect(bx, by, 22, 11, WHITE);
  g.fillRect(bx + 22, by + 3, 2, 5, WHITE);
  uint16_t bc = pct > 50 ? GREEN : (pct > 20 ? YELLOW : RED);
  g.fillRect(bx + 2, by + 2, (pct * 18) / 100, 7, bc);
  int x = bx - 4;
  if (board.isExternalPowered()) {
    d.setColor(YELLOW);
    d.drawXbm(x - 8, by + 2, pct >= 95 ? plug_icon : charging_icon, 8, 8);
    x -= 12;
  }

  // clock
  char tmp[16];
  uint32_t now = rtc->getCurrentTime();
  if (clockIsSet(now)) {
    DateTime dt(now + (int32_t)task->getNodePrefs()->tz_offset * 3600);
    sprintf(tmp, "%02d:%02d", dt.hour(), dt.minute());
  } else {
    strcpy(tmp, "--:--");
  }
  d.setTextSize(1);
  d.setColor(clockIsSet(now) ? WHITE : GREY);
  int tw = d.getTextWidth(tmp);
  x -= tw;
  d.setCursor(x, 1);
  d.print(tmp);
  x -= 12;

  // Bluetooth: green = phone connected, blue = waiting for phone, red = off
  uint16_t btc = !task->isBluetoothEnabled() ? RED : (task->hasConnection() ? GREEN : BLUE);
  drawBluetoothGlyph(g, x, 3, btc);
  x -= 13;

#if ENV_INCLUDE_GPS == 1
  // GPS: green = fix, grey = searching, nothing = off
  if (task->getGPSState()) {
    LocationProvider* nmea = sensors.getLocationProvider();
    bool fix = nmea != NULL && nmea->isValid();
    g.drawCircle(x + 4, 9, 4, fix ? GREEN : GREY);
    if (fix) g.fillCircle(x + 4, 9, 2, GREEN);
    x -= 13;
  }
#endif

  // unread messages
  int unread = task->messages.unread();
  if (unread > 0) {
    sprintf(tmp, "%d", unread);
    d.setColor(YELLOW);
    int cw = d.getTextWidth(tmp);
    x -= cw;
    d.setCursor(x, 1);
    d.print(tmp);
    x -= 16;
    drawEnvelope(g, x, 4, YELLOW);
    x -= 6;
  }

  // title, clipped to what is left
  d.setTextSize(2);
  d.setColor(UIColor::title_txt);
  d.drawClipped(4, 1, x - 6, title);
}

static void drawPageDots(DisplayDriver& display, int page, int count) {
  ST7789ColorDisplay& d = tft(display);
  int x = SCREEN_W / 2 - 4 * (count - 1);
  for (int i = 0; i < count; i++, x += 8) {
    if (i == page) d.gfx().fillRect(x - 1, DOTS_Y - 1, 4, 4, WHITE);
    else           d.gfx().fillRect(x, DOTS_Y, 2, 2, GREY);
  }
}

// label left (grey), value right (white)
static void drawKV(DisplayDriver& display, int row, const char* label, const char* value, ColorVal vc = WHITE) {
  ST7789ColorDisplay& d = tft(display);
  d.setTextSize(1);
  d.setColor(LIGHT_GREY);
  d.setCursor(6, ROW_Y(row));
  d.print(label);
  d.setColor(vc);
  d.drawTextRightAlign(SCREEN_W - 6, ROW_Y(row), value);
}

static void drawHint(DisplayDriver& display, int y, const char* text) {
  ST7789ColorDisplay& d = tft(display);
  d.setTextSize(1);
  d.setColor(GREY);
  d.drawTextCentered(SCREEN_W / 2, y, text);
}

// ---------------- splash ----------------

class SplashScreen : public UIScreen {
  UITask* _task;
  unsigned long dismiss_after;
  char _version_info[24];

public:
  SplashScreen(UITask* task) : _task(task) {
    // keep "v1.18.0" or "v1.18.0+carnode-abc123"; drop any "-<hash>" suffix
    const char *ver = FIRMWARE_VERSION;
    const char *dash = strchr(ver, '-');
    const char *plus = strchr(ver, '+');
    int len = (dash && !plus) ? dash - ver : strlen(ver);
    if (len >= (int)sizeof(_version_info)) len = sizeof(_version_info) - 1;
    memcpy(_version_info, ver, len);
    _version_info[len] = 0;
    dismiss_after = millis() + BOOT_SCREEN_MILLIS;
  }

  int render(DisplayDriver& display) override {
    ST7789ColorDisplay& d = tft(display);
    d.setColor(BLUE);
    d.drawXbm((SCREEN_W - 128) / 2, 22, meshcore_logo, 128, 13);
    d.setTextSize(1);
    d.setColor(LIGHT_GREY);
    d.drawTextCentered(SCREEN_W / 2, 48, "meshcore.io");
    d.setTextSize(2);
    d.setColor(WHITE);
    d.drawTextCentered(SCREEN_W / 2, 74, _version_info);
    d.setTextSize(1);
    d.setColor(GREY);
    d.drawTextCentered(SCREEN_W / 2, 98, FIRMWARE_BUILD_DATE);
    return 1000;
  }

  void poll() override {
    if (millis() >= dismiss_after) _task->gotoHomeScreen();
  }
};

// ---------------- home pages ----------------

class HomeScreen : public UIScreen {
public:
  enum HomePage {
    FIRST,
    MESSAGES,
    CAR,
    NODES,
#ifdef WITH_MT_PRESENCE
    PRESENCE,
#endif
    RADIO,
    BLUETOOTH,
    ADVERT,
#if ENV_INCLUDE_GPS == 1
    GPS,
#endif
#if UI_DISCOVER_SCREEN
    DISCOVERY,
#endif
#ifndef UI_NO_HIBERNATE
    SHUTDOWN,
#endif
    Count    // keep as last
  };

private:
  UITask* _task;
  mesh::RTCClock* _rtc;
  NodePrefs* _node_prefs;
  uint8_t _page;
  uint8_t _car = 0;      // selected car on the CAR page
  bool _shutdown_init;
  AdvertPath recent[UI_RECENT_LIST_SIZE];
#if UI_DISCOVER_SCREEN
  DiscoveredNode discovered[DISCOVERED_NODES_TABLE_SIZE];
  uint32_t disc_node_req_tag = 0;
  uint32_t disc_nodes_count = 0;
  uint32_t discovery_req_time = 0;
#endif

  const char* title() {
    switch (_page) {
      case FIRST:     return _node_prefs->node_name;
      case MESSAGES:  return "Messages";
      case CAR:       return _task->num_cars > 1 ? "Cars" : "Car";
      case NODES:     return "Nearby";
#ifdef WITH_MT_PRESENCE
      case PRESENCE:  return "Meshtastic";
#endif
      case RADIO:     return "Radio";
      case BLUETOOTH: return "Bluetooth";
      case ADVERT:    return "Advert";
#if ENV_INCLUDE_GPS == 1
      case GPS:       return "GPS";
#endif
#if UI_DISCOVER_SCREEN
      case DISCOVERY: return "Repeaters";
#endif
#ifndef UI_NO_HIBERNATE
      case SHUTDOWN:  return "Power";
#endif
    }
    return "";
  }

  void renderFirst(ST7789ColorDisplay& d) {
    char tmp[40];
    uint32_t now = _rtc->getCurrentTime();
    if (clockIsSet(now)) {
      DateTime dt(now + (int32_t)_node_prefs->tz_offset * 3600);
      static const char* DOW[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
      static const char* MON[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
      d.setTextSize(3);
      d.setColor(WHITE);
      sprintf(tmp, "%02d:%02d", dt.hour(), dt.minute());
      d.drawTextCentered(SCREEN_W / 2, 22, tmp);
      d.setTextSize(1);
      d.setColor(LIGHT_GREY);
      sprintf(tmp, "%s %d %s %d", DOW[dt.dayOfTheWeek() % 7], dt.day(), MON[(dt.month() + 11) % 12], dt.year());
      d.drawTextCentered(SCREEN_W / 2, 58, tmp);
    } else {
      d.setTextSize(2);
      d.setColor(GREY);
      d.drawTextCentered(SCREEN_W / 2, 30, "Clock not set");
      d.setTextSize(1);
      d.drawTextCentered(SCREEN_W / 2, 52, "connect the app to set it");
    }

    d.setTextSize(1);
    int unread = _task->messages.unread();
    if (unread > 0) {
      d.setColor(YELLOW);
      sprintf(tmp, unread == 1 ? "1 new message" : "%d new messages", unread);
      d.drawTextCentered(SCREEN_W / 2, 80, tmp);
    } else if (_task->getMsgCount() > 0) {
      d.setColor(GREY);
      sprintf(tmp, "%d waiting for the app", _task->getMsgCount());
      d.drawTextCentered(SCREEN_W / 2, 80, tmp);
    }

    if (!_task->isBluetoothEnabled()) {
      d.setColor(RED);
      d.drawTextCentered(SCREEN_W / 2, 104, "Bluetooth off");
    } else if (_task->hasConnection()) {
      d.setColor(GREEN);
      d.drawTextCentered(SCREEN_W / 2, 104, "App connected");
    } else if (the_mesh.getBLEPin() != 0) {
      d.setColor(ORANGE);
      sprintf(tmp, "PIN %06lu", (unsigned long)the_mesh.getBLEPin());
      d.drawTextCentered(SCREEN_W / 2, 104, tmp);
    } else {
      d.setColor(GREY);
      d.drawTextCentered(SCREEN_W / 2, 104, "Waiting for the app");
    }
  }

  void renderMessages(ST7789ColorDisplay& d) {
    int n = _task->messages.count();
    if (n == 0) {
      drawHint(d, 50, "No messages yet");
      return;
    }
    uint32_t now = _rtc->getCurrentTime();
    d.setTextSize(1);
    for (int i = 0; i < n && i < 6; i++) {
      UIMessage* m = _task->messages.get(i);
      int y = ROW_Y(i);
      if (m->unread) d.gfx().fillRect(0, y + 2, 3, 13, YELLOW);
      char age[8];
      fmtAge(age, (int)(now - m->timestamp));
      d.setColor(GREY);
      int aw = d.getTextWidth(age);
      d.drawTextRightAlign(SCREEN_W - 4, y, age);
      d.setColor(m->channel ? CYAN : GREEN);
      d.drawClipped(7, y, 78, m->from);
      int fx = 7 + (int)d.getTextWidth(m->from);
      if (fx > 7 + 78) fx = 7 + 78;
      d.setColor(m->unread ? WHITE : LIGHT_GREY);
      d.drawClipped(fx + 6, y, SCREEN_W - 4 - aw - 6 - (fx + 6), m->text);
    }
  }

  // Recently heard nodes; distance and direction when both ends have a position.
  void renderNodes(ST7789ColorDisplay& d) {
    the_mesh.getRecentlyHeard(recent, UI_RECENT_LIST_SIZE);
    uint32_t now = _rtc->getCurrentTime();
    double mylat, mylon;
    bool me = _task->getOwnPosition(mylat, mylon);
    d.setTextSize(1);
    int row = 0;
    for (int i = 0; i < UI_RECENT_LIST_SIZE; i++) {
      AdvertPath* a = &recent[i];
      if (a->name[0] == 0) continue;
      int y = ROW_Y(row++);
      char age[8], info[16];
      fmtAge(age, (int)(now - a->recv_timestamp));
      d.setColor(GREY);
      int aw = d.getTextWidth(age);
      d.drawTextRightAlign(SCREEN_W - 4, y, age);
      int right = SCREEN_W - 4 - aw - 8;

      ContactInfo* c = the_mesh.lookupContactByPubKey(a->pubkey_prefix, sizeof(a->pubkey_prefix));
      if (me && c != NULL && (c->gps_lat != 0 || c->gps_lon != 0)) {
        double lat = c->gps_lat / 1000000.0, lon = c->gps_lon / 1000000.0;
        fmtDistance(info, distanceMeters(mylat, mylon, lat, lon));
        d.setColor(WHITE);
        int iw = d.getTextWidth(info);
        d.drawTextRightAlign(right, y, info);
        drawArrow(d.gfx(), right - iw - 9, y + 8, 7, bearingDegrees(mylat, mylon, lat, lon), CYAN);
        right -= iw + 20;
      } else {
        int h = a->path_len & 63;
        if (h == 0) strcpy(info, "direct"); else sprintf(info, "%d hop%s", h, h == 1 ? "" : "s");
        d.setColor(h == 0 ? GREEN : LIGHT_GREY);
        int iw = d.getTextWidth(info);
        d.drawTextRightAlign(right, y, info);
        right -= iw + 8;
      }
      d.setColor(WHITE);
      d.drawClipped(6, y, right - 6, a->name);
    }
    if (row == 0) drawHint(d, 50, "Nothing heard yet");
  }

  // Find my car: direction and distance to the selected car node's parked spot.
  void renderCar(ST7789ColorDisplay& d) {
    if (_task->num_cars == 0) {
      drawHint(d, 36, "No car node heard yet");
      drawHint(d, 62, "Its next advert will show here");
      return;
    }
    if (_car >= _task->num_cars) _car = 0;
    UICar* car = &_task->cars[_car];
    Adafruit_GFX& g = d.gfx();
    const int cx = 50, cy = 76, r = 34;

    char tmp[40];
    double mylat, mylon;
    bool me = _task->getOwnPosition(mylat, mylon);
    g.drawCircle(cx, cy, r, DARK_GREY);
    d.setTextSize(0);
    d.setColor(GREY);
    d.setCursor(cx - 2, cy - r + 3);
    d.print("N");
    double dist = 0, brg = 0;
    if (me && car->has_loc) {
      dist = distanceMeters(mylat, mylon, car->lat / 1000000.0, car->lon / 1000000.0);
      brg = bearingDegrees(mylat, mylon, car->lat / 1000000.0, car->lon / 1000000.0);
      if (dist < 25) {
        g.fillCircle(cx, cy, 10, GREEN);              // you're there
      } else {
        drawArrow(g, cx, cy, r - 8, brg, ORANGE);
      }
    } else {
      d.setTextSize(3);
      d.setColor(GREY);
      d.drawTextCentered(cx, cy - 17, "?");
    }

    const int tx = 98, tw = SCREEN_W - tx - 4;
    d.setTextSize(2);
    d.setColor(WHITE);
    d.drawClipped(tx, 22, tw, car->name);

    if (!car->has_loc) {
      d.setTextSize(1);
      d.setColor(GREY);
      d.setCursor(tx, 48);
      d.print("no location sent");
    } else if (!me) {
      d.setTextSize(1);
      d.setColor(GREY);
      d.drawClipped(tx, 44, tw, "Where am I?");
      d.drawClipped(tx, 62, tw, "turn on GPS");
    } else {
      d.setTextSize(3);
      d.setColor(WHITE);
      fmtDistance(tmp, dist);
      if (dist < 25) strcpy(tmp, "here");
      d.drawClipped(tx, 42, tw, tmp);
      if (dist >= 25) {
        d.setTextSize(1);
        d.setColor(LIGHT_GREY);
        d.setCursor(tx, 76);
        d.print(compassPoint(brg));
        sprintf(tmp, "%d deg", (int)(brg + 0.5));
        d.drawTextRightAlign(SCREEN_W - 4, 76, tmp);
      }
    }

    char age[16];
    fmtAge(age, (int)(_rtc->getCurrentTime() - car->heard_at));
    if (strcmp(age, "now") == 0) strcpy(age, "just now"); else strcat(age, " ago");
    d.setTextSize(1);
    if (car->parked) {
      d.setColor(GREEN);
      sprintf(tmp, "parked, %s", age);
    } else {
      d.setColor(YELLOW);
      sprintf(tmp, "moving, %s", age);
    }
    d.drawClipped(tx, 98, tw, tmp);
    if (_task->num_cars > 1) {
      d.setTextSize(0);
      d.setColor(GREY);
      sprintf(tmp, "%d/%d  long press: next car", _car + 1, _task->num_cars);
      d.drawTextRightAlign(SCREEN_W - 4, 120, tmp);
    }
  }

#ifdef WITH_MT_PRESENCE
  // Meshtastic presence status; long press toggles it.
  void renderPresence(ST7789ColorDisplay& d) {
    const MtBeaconControl& b = the_mesh.getPresence();
    char v[32];
    drawKV(d, 0, "Presence", b.enabled() ? "on" : "off", b.enabled() ? GREEN : GREY);
    long s = b.secsToNext();
    if (s < 0) strcpy(v, "-");
    else if (s < 60) sprintf(v, "in %lds", s);
    else sprintf(v, "in %ldm %02lds", s / 60, s % 60);
    drawKV(d, 1, "Next", v);
    if (!b.positionOn() || b.precision() == 0) strcpy(v, "off");
    else if (b.precision() >= 32) strcpy(v, "exact");
    else {
      double m = 23905787.9 / (double)(1UL << (b.precision() > 31 ? 31 : b.precision()));
      char dist[16];
      fmtDistance(dist, m);
      sprintf(v, "within %s", dist);
    }
    drawKV(d, 2, "Position", v);
    sprintf(v, "%s %s", b.regionName(), b.presetName());
    drawKV(d, 3, "Channel", v);
    const meshtastic::TxStats& tx = b.txStats();
    uint32_t failed = tx.attempts - tx.delivered;
    sprintf(v, "%lu ok  %lu failed", (unsigned long)tx.delivered, (unsigned long)failed);
    drawKV(d, 4, "Sent", v, failed > 0 ? ORANGE : WHITE);
    drawHint(d, 111, b.enabled() ? "long press: turn off" : "long press: turn on");
  }
#endif

  void renderRadio(ST7789ColorDisplay& d) {
    char v[32];
    sprintf(v, "%.3f MHz", _node_prefs->freq);
    drawKV(d, 0, "Frequency", v);
    sprintf(v, "SF%d  BW%.1f  CR%d", _node_prefs->sf, _node_prefs->bw, _node_prefs->cr);
    drawKV(d, 1, "Modem", v);
    sprintf(v, "%d dBm", _node_prefs->tx_power_dbm);
    drawKV(d, 2, "TX power", v);
    sprintf(v, "%d dBm", radio_driver.getNoiseFloor());
    drawKV(d, 3, "Noise floor", v);
    sprintf(v, "%.0f dBm  %.1f dB", radio_driver.getLastRSSI(), radio_driver.getLastSNR());
    drawKV(d, 4, "Last packet", v);
  }

  void renderIconPage(ST7789ColorDisplay& d, const uint8_t* icon, ColorVal ic, const char* state, ColorVal sc, const char* hint) {
    d.setColor(ic);
    d.drawXbm((SCREEN_W - 32) / 2, 24, icon, 32, 32);
    if (state) {
      d.setTextSize(2);
      d.setColor(sc);
      d.drawTextCentered(SCREEN_W / 2, 66, state);
    }
    drawHint(d, 100, hint);
  }

#if ENV_INCLUDE_GPS == 1
  void renderGPS(ST7789ColorDisplay& d) {
    LocationProvider* nmea = sensors.getLocationProvider();
    char v[32];
    bool on = _task->getGPSState();
    if (nmea == NULL) {
      drawKV(d, 0, "GPS", "not available", GREY);
      return;
    }
    bool fix = nmea->isValid();
    drawKV(d, 0, "GPS", !on ? "off" : (fix ? "fix" : "searching"), !on ? GREY : (fix ? GREEN : YELLOW));
    sprintf(v, "%d", (int)nmea->satellitesCount());
    drawKV(d, 1, "Satellites", v);
    sprintf(v, "%.5f", nmea->getLatitude() / 1000000.0);
    drawKV(d, 2, "Latitude", v);
    sprintf(v, "%.5f", nmea->getLongitude() / 1000000.0);
    drawKV(d, 3, "Longitude", v);
    sprintf(v, "%.0f m", nmea->getAltitude() / 1000.0);
    drawKV(d, 4, "Altitude", v);
  }
#endif

#if UI_DISCOVER_SCREEN
  void renderDiscovery(ST7789ColorDisplay& d) {
    d.setTextSize(1);
    for (uint32_t i = 0; i < disc_nodes_count && i < 6; i++) {
      DiscoveredNode* a = &discovered[i];
      char name[32], snr[24];
      if (a->name[0] == 0) mesh::Utils::toHex(name, a->pubkey_prefix, 4);
      else StrHelper::strncpy(name, a->name, sizeof(name));
      sprintf(snr, "%.1f / %.1f dB", a->snr_out, a->snr_in);
      int y = ROW_Y(i);
      float worst = a->snr_in < a->snr_out ? a->snr_in : a->snr_out;
      d.setColor(worst > 5 ? GREEN : (worst > -5 ? YELLOW : RED));
      int sw = d.getTextWidth(snr);
      d.drawTextRightAlign(SCREEN_W - 4, y, snr);
      d.setColor(WHITE);
      d.drawClipped(6, y, SCREEN_W - 4 - sw - 8 - 6, name);
    }
    if (millis() < discovery_req_time + 5000) {
      if (disc_nodes_count == 0) drawHint(d, 50, "Scanning...");
    } else if (disc_nodes_count == 0) {
      drawHint(d, 40, "Repeaters within one hop");
      drawHint(d, 70, "long press: scan");
    }
  }
#endif

public:
  HomeScreen(UITask* task, mesh::RTCClock* rtc, NodePrefs* node_prefs)
     : _task(task), _rtc(rtc), _node_prefs(node_prefs), _page(0), _shutdown_init(false) { }

  void setPage(uint8_t p) { _page = p; }

#if UI_DISCOVER_SCREEN
  bool sendDiscoverRequest() {
    uint8_t cmd_bytes[6];
    cmd_bytes[0] = CTL_TYPE_NODE_DISCOVER_REQ | 1; // DISCOVER_REQ | prefix only
    cmd_bytes[1] = 0xFF;     // Repeaters
    the_mesh.getRNG()->random((uint8_t *) &disc_node_req_tag, 4);
    memcpy(&cmd_bytes[2], &disc_node_req_tag, 4);
    disc_nodes_count = 0;
    mesh::Packet* req = the_mesh.createControlData(cmd_bytes, sizeof(cmd_bytes));
    if (req) {
      the_mesh.sendZeroHop(req);
      discovery_req_time = millis();
      return true;
    }
    return false;
  }

  void handleDiscoverResponse(const mesh::Packet* packet) {
    if (disc_nodes_count < DISCOVERED_NODES_TABLE_SIZE && memcmp(&packet->payload[2], &disc_node_req_tag, 4) == 0) {
      auto d = &discovered[disc_nodes_count++];
      memcpy(d->pubkey_prefix, &packet->payload[6], 8);
      d->type = packet->payload[0] & 0xF;
      d->snr_out = ((int8_t)packet->payload[1]) / 4.0;
      d->snr_in = radio_driver.getLastSNR();
      ContactInfo* c = the_mesh.lookupContactByPubKey(&packet->payload[6], 8);
      if (c != NULL) StrHelper::strncpy(d->name, c->name, sizeof(d->name));
      else d->name[0] = 0;
    }
  }
#endif

  void poll() override {
    if (_shutdown_init && !_task->isButtonPressed()) {  // wait for the button to be released
      _task->shutdown();
    }
  }

  int render(DisplayDriver& display) override {
    ST7789ColorDisplay& d = tft(display);
    drawStatusBar(display, _task, _rtc, title());

    switch (_page) {
      case FIRST:    renderFirst(d); break;
      case MESSAGES: renderMessages(d); break;
      case CAR:      renderCar(d); break;
      case NODES:    renderNodes(d); break;
#ifdef WITH_MT_PRESENCE
      case PRESENCE: renderPresence(d); break;
#endif
      case RADIO:    renderRadio(d); break;
      case BLUETOOTH: {
        bool on = _task->isBluetoothEnabled();
        renderIconPage(d, on ? bluetooth_on : bluetooth_off, on ? BLUE : RED,
                       on ? (_task->hasConnection() ? "Connected" : "On") : "Off",
                       on ? WHITE : RED, on ? "long press: turn off" : "long press: turn on");
        break;
      }
      case ADVERT:
        renderIconPage(d, advert_icon, BLUE, NULL, WHITE, "long press: send advert");
        break;
#if ENV_INCLUDE_GPS == 1
      case GPS:      renderGPS(d); break;
#endif
#if UI_DISCOVER_SCREEN
      case DISCOVERY: renderDiscovery(d); break;
#endif
#ifndef UI_NO_HIBERNATE
      case SHUTDOWN:
        if (_shutdown_init) {
          d.setTextSize(2);
          d.setColor(ORANGE);
          d.drawTextCentered(SCREEN_W / 2, 60, "Hibernating...");
        } else {
          renderIconPage(d, power_icon, GREY, NULL, WHITE, "long press: hibernate");
        }
        break;
#endif
    }
    drawPageDots(display, _page, HomePage::Count);

#if UI_DISCOVER_SCREEN
    if (_page == DISCOVERY && millis() < discovery_req_time + 5000) return 500;
#endif
    return _page == FIRST || _page == MESSAGES ? 1000 : 2000;
  }

  bool handleInput(char c) override {
    if (c == KEY_LEFT || c == KEY_PREV) {
      _page = (_page + HomePage::Count - 1) % HomePage::Count;
      return true;
    }
    if (c == KEY_NEXT || c == KEY_RIGHT) {
      _page = (_page + 1) % HomePage::Count;
      return true;
    }
    if (c != KEY_ENTER) return false;
    switch (_page) {
      case FIRST:
      case MESSAGES:
        if (_task->messages.count() > 0) _task->openReader(0);
        else _task->showAlert("No messages yet", 800);
        return true;
      case CAR:
        if (_task->num_cars > 1) _car = (_car + 1) % _task->num_cars;
        return true;
#ifdef WITH_MT_PRESENCE
      case PRESENCE: {
        bool on = !the_mesh.getPresence().enabled();
        bool ok = the_mesh.setPresenceVar("mt.presence", on ? "1" : "0");
        _task->showAlert(!ok ? "Failed" : (on ? "Presence on" : "Presence off"), 800);
        return true;
      }
#endif
      case BLUETOOTH:
        if (_task->isBluetoothEnabled()) _task->disableBluetooth();
        else _task->enableBluetooth();
        return true;
      case ADVERT:
        _task->notify(UIEventType::ack);
        _task->showAlert(the_mesh.advert() ? "Advert sent" : "Advert failed", 1000);
        return true;
#if ENV_INCLUDE_GPS == 1
      case GPS:
        _task->toggleGPS();
        return true;
#endif
#if UI_DISCOVER_SCREEN
      case DISCOVERY:
        if (millis() > discovery_req_time + 5000) sendDiscoverRequest();   // rate limit
        return true;
#endif
#ifndef UI_NO_HIBERNATE
      case SHUTDOWN:
        _shutdown_init = true;   // shuts down once the button is released
        return true;
#endif
    }
    return false;
  }
};

// ---------------- message reader ----------------

#define READER_LINES  5

class ReaderScreen : public UIScreen {
  UITask* _task;
  mesh::RTCClock* _rtc;
  int _index = 0;    // 0 = newest
  int _scroll = 0;   // first wrapped line shown
  int _lines = 0;    // wrapped line count of the current message

public:
  ReaderScreen(UITask* task, mesh::RTCClock* rtc) : _task(task), _rtc(rtc) { }

  void open(int index) { _index = index; _scroll = 0; }

  int render(DisplayDriver& display) override {
    ST7789ColorDisplay& d = tft(display);
    UIMessage* m = _task->messages.get(_index);
    if (m == NULL) {
      _task->gotoHomeScreen();
      return 100;
    }
    m->unread = false;

    char title[24];
    sprintf(title, "%d of %d", _index + 1, _task->messages.count());
    drawStatusBar(display, _task, _rtc, title);

    // sender, hops, age
    char meta[24], age[8];
    fmtAge(age, (int)(_rtc->getCurrentTime() - m->timestamp));
    if (m->hops == 0xFF) sprintf(meta, "direct  %s", age);
    else sprintf(meta, "%d hop%s  %s", m->hops, m->hops == 1 ? "" : "s", age);
    d.setTextSize(1);
    d.setColor(GREY);
    int mw = d.getTextWidth(meta);
    d.drawTextRightAlign(SCREEN_W - 4, 21, meta);
    d.setTextSize(2);
    d.setColor(m->channel ? CYAN : GREEN);
    d.drawClipped(4, 21, SCREEN_W - 4 - mw - 8 - 4, m->from);
    d.gfx().drawFastHLine(0, 40, SCREEN_W, DARK_GREY);

    // body
    d.setTextSize(1);
    d.setColor(WHITE);
    _lines = d.drawWrapped(4, 43, SCREEN_W - 12, READER_LINES, _scroll, m->text);

    // scroll bar when the text does not fit
    if (_lines > READER_LINES) {
      int track = 135 - 43;
      int bar = track * READER_LINES / _lines;
      int pos = track * _scroll / _lines;
      d.gfx().fillRect(SCREEN_W - 4, 43 + pos, 3, bar, GREY);
    }
    return 1000;
  }

  // click: scroll down, then older message; double click: newer message;
  // long press: back to the Messages page
  bool handleInput(char c) override {
    if (c == KEY_NEXT || c == KEY_RIGHT) {
      if (_scroll + READER_LINES < _lines) {
        _scroll += READER_LINES - 1;
      } else if (_index + 1 < _task->messages.count()) {
        open(_index + 1);
      } else {
        _task->gotoHomeScreen();
      }
      return true;
    }
    if (c == KEY_PREV || c == KEY_LEFT) {
      if (_scroll > 0) {
        _scroll = 0;
      } else if (_index > 0) {
        open(_index - 1);
      }
      return true;
    }
    if (c == KEY_ENTER) {
      _task->gotoHomeScreen();
      return true;
    }
    return false;
  }
};

// ---------------- UITask ----------------

void UITask::begin(DisplayDriver* display, SensorManager* sensors, NodePrefs* node_prefs) {
  _display = display;
  _sensors = sensors;
  _auto_off = millis() + AUTO_OFF_MILLIS;

#if defined(PIN_USER_BTN)
  user_btn.begin();
#endif

  _node_prefs = node_prefs;

  if (_display != NULL) {
    _display->turnOn();
  }

#ifdef PIN_BUZZER
  buzzer.begin();
  buzzer.quiet(_node_prefs->buzzer_quiet);
  buzzer.startup();
#endif

#ifdef PIN_VIBRATION
  vibration.begin();
#endif

  ui_started_at = millis();
  _alert_expiry = 0;

  splash = new SplashScreen(this);
  home = new HomeScreen(this, &rtc_clock, node_prefs);
  reader = new ReaderScreen(this, &rtc_clock);
  setCurrScreen(splash);
}

void UITask::gotoHomeScreen() {
  if (curr == reader) ((HomeScreen *) home)->setPage(HomeScreen::MESSAGES);
  setCurrScreen(home);
}

void UITask::openReader(int index) {
  ((ReaderScreen *) reader)->open(index);
  setCurrScreen(reader);
}

void UITask::showAlert(const char* text, int duration_millis) {
  StrHelper::strncpy(_alert, text, sizeof(_alert));
  _alert_expiry = millis() + duration_millis;
}

void UITask::notify(UIEventType t) {
#if defined(PIN_BUZZER)
  switch (t) {
    case UIEventType::contactMessage:
      buzzer.play("MsgRcv3:d=4,o=6,b=200:32e,32g,32b,16c7");
      break;
    case UIEventType::channelMessage:
      buzzer.play("kerplop:d=16,o=6,b=120:32g#,32c#");
      break;
    case UIEventType::ack:
      buzzer.play("ack:d=32,o=8,b=120:c");
      break;
    default:
      break;
  }
#endif

#ifdef PIN_VIBRATION
  if (t != UIEventType::none) {
    vibration.trigger();
  }
#endif
}

// A new message is shown full-screen, as ui-new's preview does.
void UITask::onNewMessage() {
  openReader(0);
  if (_display != NULL) {
    if (!_display->isOn() && !hasConnection()) {
      _display->turnOn();
    }
    if (_display->isOn()) {
      _auto_off = millis() + AUTO_OFF_MILLIS;
      _next_refresh = 100;
    }
  }
}

void UITask::onMessageRecv(mesh::Packet *pkt, const ContactInfo &from, uint8_t txt_type, uint32_t sender_timestamp, const char* text) {
  // only text messages, not CLI data
  if (!(txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_SIGNED_PLAIN)) return;

  uint8_t hops = pkt->isRouteFlood() ? pkt->getPathHashCount() : 0xFF;
  messages.add(rtc_clock.getCurrentTime(), hops, false, from.name, text);
  onNewMessage();
  if (!hasConnection()) notify(UIEventType::contactMessage);
}

void UITask::onChannelMessageRecv(mesh::Packet *pkt, ChannelDetails& channel_details, const char* text) {
  uint8_t hops = pkt->isRouteFlood() ? pkt->getPathHashCount() : 0xFF;
  // channel text arrives as "sender: message"; show the channel as the origin
  messages.add(rtc_clock.getCurrentTime(), hops, true, channel_details.name, text);
  onNewMessage();
  if (!hasConnection()) notify(UIEventType::channelMessage);
}

void UITask::onQueueSizeChanged(int msgcount) {
  _msgcount = msgcount;
}

void UITask::onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t* path) {
  if (!hasConnection()) {
    notify(UIEventType::newContactMessage);
  }
}

void UITask::onControlDataRecv(const mesh::Packet* packet) {
#if UI_DISCOVER_SCREEN
  if (packet->payload_len >= 14 && (packet->payload[0] & 0xF0) == CTL_TYPE_NODE_DISCOVER_RESP) {
    ((HomeScreen *) home)->handleDiscoverResponse(packet);
  }
#endif
}

void UITask::onCarNodeAdvert(const uint8_t* pub_key, const char* name, bool has_loc,
                             int32_t lat_e6, int32_t lon_e6, bool parked, uint8_t hops) {
  int i = 0;
  while (i < num_cars && memcmp(cars[i].key, pub_key, sizeof(cars[i].key)) != 0) i++;
  if (i == num_cars) {
    if (num_cars < UI_MAX_CARS) {
      num_cars++;
    } else {   // full: replace the one heard longest ago
      i = 0;
      for (int j = 1; j < num_cars; j++) if (cars[j].heard_at < cars[i].heard_at) i = j;
    }
    memcpy(cars[i].key, pub_key, sizeof(cars[i].key));
  }
  UICar* c = &cars[i];
  StrHelper::strncpy(c->name, name, sizeof(c->name));
  c->has_loc = has_loc && (lat_e6 != 0 || lon_e6 != 0);
  c->lat = lat_e6;
  c->lon = lon_e6;
  c->parked = parked;
  c->hops = hops;
  c->heard_at = rtc_clock.getCurrentTime();
  _next_refresh = 100;
}

bool UITask::getOwnPosition(double& lat, double& lon) {
#if ENV_INCLUDE_GPS == 1
  LocationProvider* nmea = sensors.getLocationProvider();
  if (nmea != NULL && getGPSState() && nmea->isValid()) {
    lat = nmea->getLatitude() / 1000000.0;
    lon = nmea->getLongitude() / 1000000.0;
    return true;
  }
#endif
  if (sensors.node_lat != 0 || sensors.node_lon != 0) {   // set in the app
    lat = sensors.node_lat;
    lon = sensors.node_lon;
    return true;
  }
  return false;
}

void UITask::userLedHandler() {
}

void UITask::setCurrScreen(UIScreen* c) {
  curr = c;
  _next_refresh = 100;
}

void UITask::shutdown(bool restart) {
#ifdef PIN_BUZZER
  buzzer.shutdown();
  uint32_t buzzer_timer = millis();
  while (buzzer.isPlaying() && (millis() - 2500) < buzzer_timer)
    buzzer.loop();
#endif

  if (restart) {
    _board->reboot();
  } else {
    display.clear();
    display.endFrame();
    _board->powerOff();
  }
}

bool UITask::isButtonPressed() const {
#ifdef PIN_USER_BTN
  return user_btn.isPressed();
#else
  return false;
#endif
}

void UITask::loop() {
  char c = 0;
#if defined(PIN_USER_BTN)
  int ev = user_btn.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_NEXT);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_ENTER);
  } else if (ev == BUTTON_EVENT_DOUBLE_CLICK) {
    c = handleDoubleClick(KEY_PREV);
  } else if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
    c = handleTripleClick(KEY_SELECT);
  }
#endif

  if (c != 0 && curr) {
    curr->handleInput(c);
    _auto_off = millis() + AUTO_OFF_MILLIS;
    _next_refresh = 100;
  }

  userLedHandler();

#ifdef PIN_BUZZER
  if (buzzer.isPlaying()) buzzer.loop();
#endif

  if (curr) curr->poll();

  if (_display != NULL && _display->isOn()) {
    if (millis() >= _next_refresh && curr) {
      _display->startFrame();
      int delay_millis = curr->render(*_display);
      if (millis() < _alert_expiry) {   // alert popup
        ST7789ColorDisplay& d = tft(*_display);
        d.gfx().fillRoundRect(24, 44, SCREEN_W - 48, 44, 6, DARK_GREY);
        d.gfx().drawRoundRect(24, 44, SCREEN_W - 48, 44, 6, LIGHT_GREY);
        d.setTextSize(2);
        d.setColor(WHITE);
        d.drawTextCentered(SCREEN_W / 2, 57, _alert);
        _next_refresh = _alert_expiry;
      } else {
        _next_refresh = millis() + delay_millis;
      }
      _display->endFrame();
    }
#if AUTO_OFF_MILLIS > 0
    if (millis() > _auto_off) {
      _display->turnOff();
      // wake up on the home page
      if (curr != splash) {
        ((HomeScreen *) home)->setPage(HomeScreen::FIRST);
        setCurrScreen(home);
      }
    }
#endif
  }

#ifdef PIN_VIBRATION
  vibration.loop();
#endif

#ifdef AUTO_SHUTDOWN_MILLIVOLTS
  if (millis() > next_batt_chck) {
    uint16_t milliVolts = getBattMilliVolts();
    if (milliVolts > 0 && milliVolts < AUTO_SHUTDOWN_MILLIVOLTS) {
      if (!board.isExternalPowered()) {
        if (_display != NULL) {
          _display->startFrame();
          _display->setTextSize(2);
          _display->setColor(UIColor::warning_txt);
          _display->drawTextCentered(_display->width() / 2, 40, "Low battery");
          _display->drawTextCentered(_display->width() / 2, 70, "Shutting down");
          _display->endFrame();
          delay(3000);
        }
        shutdown();
      }
    }
    next_batt_chck = millis() + 8000;
  }
#endif
}

char UITask::checkDisplayOn(char c) {
  if (_display != NULL) {
    if (!_display->isOn()) {
      _display->turnOn();   // turn the display on and consume the event
      c = 0;
    }
    _auto_off = millis() + AUTO_OFF_MILLIS;
    _next_refresh = 0;
  }
  return c;
}

char UITask::handleLongPress(char c) {
  if (millis() - ui_started_at < 8000) {   // long press in the first 8 s -> CLI rescue
    the_mesh.enterCLIRescue();
    c = 0;
  }
  return checkDisplayOn(c);
}

char UITask::handleDoubleClick(char c) {
  return checkDisplayOn(c);
}

char UITask::handleTripleClick(char c) {
  checkDisplayOn(c);
  toggleBuzzer();
  return 0;
}

bool UITask::getGPSState() {
  if (_sensors != NULL) {
    int num = _sensors->getNumSettings();
    for (int i = 0; i < num; i++) {
      if (strcmp(_sensors->getSettingName(i), "gps") == 0) {
        return !strcmp(_sensors->getSettingValue(i), "1");
      }
    }
  }
  return false;
}

void UITask::toggleGPS() {
  if (_sensors != NULL) {
    int num = _sensors->getNumSettings();
    for (int i = 0; i < num; i++) {
      if (strcmp(_sensors->getSettingName(i), "gps") == 0) {
        if (strcmp(_sensors->getSettingValue(i), "1") == 0) {
          _sensors->setSettingValue("gps", "0");
          _node_prefs->gps_enabled = 0;
        } else {
          _sensors->setSettingValue("gps", "1");
          _node_prefs->gps_enabled = 1;
        }
        notify(UIEventType::ack);
        the_mesh.savePrefs();
        showAlert(_node_prefs->gps_enabled ? "GPS on" : "GPS off", 800);
        _next_refresh = 0;
        break;
      }
    }
  }
}

void UITask::toggleBuzzer() {
#ifdef PIN_BUZZER
  if (buzzer.isQuiet()) {
    buzzer.quiet(false);
    notify(UIEventType::ack);
  } else {
    buzzer.quiet(true);
  }
  _node_prefs->buzzer_quiet = buzzer.isQuiet();
  the_mesh.savePrefs();
  showAlert(buzzer.isQuiet() ? "Buzzer off" : "Buzzer on", 800);
  _next_refresh = 0;
#endif
}
