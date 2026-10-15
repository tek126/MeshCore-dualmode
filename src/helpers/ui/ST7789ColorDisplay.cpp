#ifdef ST7789_COLOR

#include "ST7789ColorDisplay.h"
#include <Arduino.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>

// Where the 240x135 panel sits in the controller's 320x240 RAM, for the
// landscape orientation used here (MADCTL MV|MX). The stock driver uses
// (40, 52) plus a 1-row nudge to avoid a cut-off top row, i.e. 53.
#ifndef TFT_X_OFFSET
  #define TFT_X_OFFSET 40
#endif
#ifndef TFT_Y_OFFSET
  #define TFT_Y_OFFSET 53
#endif

#define ST_SWRESET 0x01
#define ST_SLPOUT  0x11
#define ST_NORON   0x13
#define ST_INVON   0x21
#define ST_DISPON  0x29
#define ST_CASET   0x2A
#define ST_RASET   0x2B
#define ST_RAMWR   0x2C
#define ST_MADCTL  0x36
#define ST_COLMOD  0x3A
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20

// RGB565, indexed by TftColor
static const uint16_t PALETTE[TftColor::COUNT] = {
  0x0000,  // BLACK
  0xFFFF,  // WHITE
  0xC618,  // LIGHT_GREY
  0x8410,  // GREY
  0x39E7,  // DARK_GREY
  0x10A6,  // NAVY
  0x2BDC,  // BLUE      (MeshCore blue, #2A7AE2)
  0x07FF,  // CYAN
  0x3E8E,  // GREEN     (#3CD070)
  0x0320,  // DARK_GREEN
  0xFFE0,  // YELLOW
  0xFD20,  // ORANGE
  0xF208,  // RED       (#F04040)
  0xB31E,  // PURPLE    (#B060F0)
  0x19CB,  // HILITE    (selected row, #1E3A5F)
  0x1108,  // DIM_BLUE  (title bar, #102040)
};

ColorVal UIColor::window_bkg    = TftColor::BLACK;
ColorVal UIColor::title_bkg     = TftColor::DIM_BLUE;
ColorVal UIColor::title_txt     = TftColor::WHITE;
ColorVal UIColor::primary_txt   = TftColor::WHITE;
ColorVal UIColor::secondary_txt = TftColor::LIGHT_GREY;
ColorVal UIColor::warning_txt   = TftColor::ORANGE;
ColorVal UIColor::popup_bkg     = TftColor::DARK_GREY;
ColorVal UIColor::popup_txt     = TftColor::WHITE;
ColorVal UIColor::corp_blue     = TftColor::BLUE;

// ---- Canvas4: Adafruit_GFX into the 4-bit framebuffer ----

void ST7789ColorDisplay::Canvas4::drawPixel(int16_t x, int16_t y, uint16_t c) {
  if (x < 0 || y < 0 || x >= W || y >= H) return;
  uint8_t* p = &_buf[(y * W + x) >> 1];
  if (x & 1) *p = (*p & 0xF0) | (c & 0x0F);
  else       *p = (*p & 0x0F) | (uint8_t)(c << 4);
}

void ST7789ColorDisplay::Canvas4::writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) {
  if (y < 0 || y >= H || w <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (x + w > W) w = W - x;
  if (w <= 0) return;
  if (x & 1) { drawPixel(x, y, c); x++; w--; }
  uint8_t* p = &_buf[(y * W + x) >> 1];
  uint8_t both = (uint8_t)((c << 4) | (c & 0x0F));
  memset(p, both, w >> 1);
  if (w & 1) drawPixel(x + w - 1, y, c);
}

void ST7789ColorDisplay::Canvas4::writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) {
  if (y < 0) { h += y; y = 0; }
  if (y + h > H) h = H - y;
  for (int16_t i = 0; i < h; i++) writeFastHLine(x, y + i, w, c);
}

void ST7789ColorDisplay::Canvas4::fillScreen(uint16_t c) {
  memset(_buf, (uint8_t)((c << 4) | (c & 0x0F)), W * H / 2);
}

// ---- panel I/O ----

ST7789ColorDisplay::ST7789ColorDisplay()
  : DisplayDriver(W, H), _gfx(_fb), _spi(&SPI1),
    _spi_settings(32000000, MSBFIRST, SPI_MODE0), _isOn(false),
    _color(TftColor::WHITE), _font(1), _ascent(0), _line_h(17) {
  memset(_fb, 0, sizeof(_fb));
  memset(_row_hash, 0, sizeof(_row_hash));
}

void ST7789ColorDisplay::writeCmd(uint8_t c) {
  digitalWrite(PIN_TFT_DC, LOW);
  digitalWrite(PIN_TFT_CS, LOW);
  _spi->beginTransaction(_spi_settings);
  _spi->transfer(c);
  _spi->endTransaction();
  digitalWrite(PIN_TFT_CS, HIGH);
  digitalWrite(PIN_TFT_DC, HIGH);
}

void ST7789ColorDisplay::writeData(const uint8_t* d, size_t n) {
  digitalWrite(PIN_TFT_DC, HIGH);
  digitalWrite(PIN_TFT_CS, LOW);
  _spi->beginTransaction(_spi_settings);
  for (size_t i = 0; i < n; i++) _spi->transfer(d[i]);
  _spi->endTransaction();
  digitalWrite(PIN_TFT_CS, HIGH);
}

void ST7789ColorDisplay::setAddrWindow(int x, int y, int w, int h) {
  uint16_t x1 = x + w - 1, y1 = y + h - 1;
  uint8_t ca[4] = { (uint8_t)(x >> 8), (uint8_t)x, (uint8_t)(x1 >> 8), (uint8_t)x1 };
  uint8_t ra[4] = { (uint8_t)(y >> 8), (uint8_t)y, (uint8_t)(y1 >> 8), (uint8_t)y1 };
  writeCmd(ST_CASET); writeData(ca, 4);
  writeCmd(ST_RASET); writeData(ra, 4);
  writeCmd(ST_RAMWR);
}

void ST7789ColorDisplay::panelInit() {
  pinMode(PIN_TFT_CS, OUTPUT);
  pinMode(PIN_TFT_DC, OUTPUT);
  pinMode(PIN_TFT_RST, OUTPUT);
  digitalWrite(PIN_TFT_CS, HIGH);
  _spi->begin();

  digitalWrite(PIN_TFT_RST, HIGH); delay(1);
  digitalWrite(PIN_TFT_RST, LOW);  delay(10);
  digitalWrite(PIN_TFT_RST, HIGH); delay(10);

  writeCmd(ST_SWRESET); delay(150);
  writeCmd(ST_SLPOUT);  delay(120);
  uint8_t colmod = 0x55;                       // 16 bits/pixel
  writeCmd(ST_COLMOD); writeData(&colmod, 1);
#ifdef DISPLAY_FLIP_VERTICALLY
  uint8_t madctl = MADCTL_MV | MADCTL_MY;
#else
  uint8_t madctl = MADCTL_MV | MADCTL_MX;      // landscape, same as the stock driver
#endif
  writeCmd(ST_MADCTL); writeData(&madctl, 1);
  writeCmd(ST_INVON);                          // IPS panel: inverted is normal
  writeCmd(ST_NORON);  delay(10);
  writeCmd(ST_DISPON); delay(10);
}

// Blank the whole 320x240 controller RAM, so rows outside our window (the
// offsets are a best guess for this panel) never show stale garbage.
void ST7789ColorDisplay::panelClearAll() {
  setAddrWindow(0, 0, 320, 240);
  digitalWrite(PIN_TFT_DC, HIGH);
  digitalWrite(PIN_TFT_CS, LOW);
  _spi->beginTransaction(_spi_settings);
  uint8_t zeros[64];
  memset(zeros, 0, sizeof(zeros));
  for (uint32_t n = 0; n < 320UL * 240 * 2; n += sizeof(zeros)) {
    _spi->transfer(zeros, NULL, sizeof(zeros));
  }
  _spi->endTransaction();
  digitalWrite(PIN_TFT_CS, HIGH);
}

void ST7789ColorDisplay::pushRows(int y0, int y1) {
  setAddrWindow(TFT_X_OFFSET, TFT_Y_OFFSET + y0, W, y1 - y0 + 1);
  digitalWrite(PIN_TFT_DC, HIGH);
  digitalWrite(PIN_TFT_CS, LOW);
  _spi->beginTransaction(_spi_settings);
  uint8_t line[W * 2];
  for (int y = y0; y <= y1; y++) {
    const uint8_t* src = &_fb[y * W / 2];
    for (int i = 0; i < W / 2; i++) {
      uint16_t a = PALETTE[src[i] >> 4], b = PALETTE[src[i] & 0x0F];
      line[i * 4]     = a >> 8; line[i * 4 + 1] = a;
      line[i * 4 + 2] = b >> 8; line[i * 4 + 3] = b;
    }
    _spi->transfer(line, NULL, sizeof(line));
  }
  _spi->endTransaction();
  digitalWrite(PIN_TFT_CS, HIGH);
}

static void backlight(bool on) {
#ifdef PIN_TFT_LEDA_CTL_ACTIVE
  digitalWrite(PIN_TFT_LEDA_CTL, on ? PIN_TFT_LEDA_CTL_ACTIVE : !PIN_TFT_LEDA_CTL_ACTIVE);
#else
  digitalWrite(PIN_TFT_LEDA_CTL, on ? LOW : HIGH);
#endif
}

bool ST7789ColorDisplay::begin() {
  if (!_isOn) {
    pinMode(PIN_TFT_VDD_CTL, OUTPUT);
    pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
    turnOn();
  }
  return true;
}

void ST7789ColorDisplay::turnOn() {
  if (_isOn) return;
  backlight(false);
  digitalWrite(PIN_TFT_VDD_CTL, LOW);    // panel power on
  delay(5);
  panelInit();
  panelClearAll();
  // the panel lost its contents: force every row out on the next frame
  for (int y = 0; y < H; y++) _row_hash[y] = ~_row_hash[y];
  backlight(true);
  _isOn = true;
}

void ST7789ColorDisplay::turnOff() {
  backlight(false);
  digitalWrite(PIN_TFT_VDD_CTL, HIGH);   // panel power off
  digitalWrite(PIN_TFT_RST, LOW);
  _isOn = false;
}

// ---- drawing ----

void ST7789ColorDisplay::clear() {
  _gfx.fillScreen(TftColor::BLACK);
}

void ST7789ColorDisplay::startFrame(ColorVal bkg) {
  _gfx.fillScreen(bkg);
  setColor(UIColor::primary_txt);
  setTextSize(1);
}

void ST7789ColorDisplay::setTextSize(int sz) {
  _font = sz;
  _gfx.setTextSize(1);
  switch (sz) {
    case 0:  _gfx.setFont(NULL);                 _ascent = 0;  _line_h = 10; break;
    case 2:  _gfx.setFont(&FreeSansBold9pt7b);   _ascent = 13; _line_h = 18; break;
    case 3:  _gfx.setFont(&FreeSansBold18pt7b);  _ascent = 25; _line_h = 34; break;
    default: _gfx.setFont(&FreeSans9pt7b);       _ascent = 13; _line_h = 18; _font = 1; break;
  }
}

void ST7789ColorDisplay::setColor(ColorVal c) {
  _color = c & 0x0F;
  _gfx.setTextColor(_color);
}

void ST7789ColorDisplay::setCursor(int x, int y) {
  _gfx.setCursor(x, y + _ascent);   // GFX fonts position on the baseline
}

int ST7789ColorDisplay::unknownWidth() const {
  return _font == 0 ? 6 : (_font == 3 ? 18 : 9);
}

int ST7789ColorDisplay::charWidth(uint8_t c) {
  if (_font == 0) return 6;
  const GFXfont* f = _font == 2 ? &FreeSansBold9pt7b : (_font == 3 ? &FreeSansBold18pt7b : &FreeSans9pt7b);
  if (c < f->first || c > f->last) return unknownWidth();
  return pgm_read_byte(&f->glyph[c - f->first].xAdvance);
}

// A non-ASCII character (emoji, accented letter...) the font can't draw:
// a small hollow box, so names stay readable and aligned.
void ST7789ColorDisplay::drawGlyphBox(int x, int y_top) {
  int w = unknownWidth() - 2, h = _font == 0 ? 7 : (_font == 3 ? 24 : 12);
  _gfx.drawRect(x + 1, y_top + (_font == 0 ? 0 : _ascent - h), w, h, _color);
}

void ST7789ColorDisplay::print(const char* str) {
  const uint8_t* s = (const uint8_t*)str;
  while (*s) {
    uint8_t c = *s++;
    if (c >= 0x20 && c < 0x7F) {
      _gfx.write(c);
    } else if (c >= 0x80) {
      // 0xDB is translateUTF8ToBlocks()' block; otherwise a UTF-8 lead byte
      int x = _gfx.getCursorX();
      drawGlyphBox(x, _gfx.getCursorY() - _ascent);
      _gfx.setCursor(x + unknownWidth(), _gfx.getCursorY());
      while ((*s & 0xC0) == 0x80) s++;   // skip continuation bytes
    }
  }
}

uint16_t ST7789ColorDisplay::getTextWidth(const char* str) {
  int w = 0;
  const uint8_t* s = (const uint8_t*)str;
  while (*s) {
    uint8_t c = *s++;
    if (c >= 0x20 && c < 0x7F) {
      w += charWidth(c);
    } else if (c >= 0x80) {
      w += unknownWidth();
      while ((*s & 0xC0) == 0x80) s++;
    }
  }
  return w;
}

void ST7789ColorDisplay::fillRect(int x, int y, int w, int h) {
  _gfx.fillRect(x, y, w, h, _color);
}

void ST7789ColorDisplay::drawRect(int x, int y, int w, int h) {
  _gfx.drawRect(x, y, w, h, _color);
}

void ST7789ColorDisplay::drawXbm(int x, int y, const uint8_t* bits, int w, int h) {
  // MSB-first rows, like the icons in ui-new/icons.h (Adafruit drawBitmap order)
  _gfx.drawBitmap(x, y, bits, w, h, _color);
}

void ST7789ColorDisplay::drawClipped(int x, int y, int max_w, const char* str) {
  if ((int)getTextWidth(str) <= max_w) {
    setCursor(x, y);
    print(str);
    return;
  }
  int ell = getTextWidth("...");
  char tmp[160];
  int n = 0, w = 0;
  const uint8_t* s = (const uint8_t*)str;
  while (*s && n < (int)sizeof(tmp) - 8) {
    const uint8_t* start = s;
    uint8_t c = *s++;
    int cw = (c >= 0x80) ? unknownWidth() : charWidth(c);
    if (c >= 0x80) while ((*s & 0xC0) == 0x80) s++;
    if (w + cw > max_w - ell) break;
    while (start < s && n < (int)sizeof(tmp) - 8) tmp[n++] = *start++;
    w += cw;
  }
  strcpy(&tmp[n], "...");
  setCursor(x, y);
  print(tmp);
}

// Greedy word wrap. Long words break mid-word. Returns the total line count.
int ST7789ColorDisplay::drawWrapped(int x, int y, int w, int max_lines, int skip_lines, const char* str) {
  const uint8_t* s = (const uint8_t*)str;
  int line = 0;
  while (*s) {
    while (*s == ' ') s++;
    if (!*s) break;
    // find the longest prefix that fits, preferring to break at a space
    const uint8_t* p = s;
    const uint8_t* last_space = NULL;
    int lw = 0;
    while (*p && *p != '\n') {
      uint8_t c = *p;
      int cw = (c >= 0x80) ? unknownWidth() : (c < 0x20 ? 0 : charWidth(c));
      const uint8_t* q = p + 1;
      if (c >= 0x80) while ((*q & 0xC0) == 0x80) q++;
      if (lw + cw > w && p > s) break;
      if (c == ' ') last_space = p;
      lw += cw;
      p = q;
    }
    const uint8_t* end = p;
    if (*p && *p != '\n' && last_space != NULL) end = last_space;
    if (line >= skip_lines && line < skip_lines + max_lines) {
      char tmp[200];
      int n = end - s;
      if (n > (int)sizeof(tmp) - 1) n = sizeof(tmp) - 1;
      memcpy(tmp, s, n);
      tmp[n] = 0;
      setCursor(x, y + (line - skip_lines) * _line_h);
      print(tmp);
    }
    line++;
    s = end;
    if (*s == '\n') s++;
  }
  return line;
}

void ST7789ColorDisplay::printWordWrap(const char* str, int max_width) {
  int x = _gfx.getCursorX(), y = _gfx.getCursorY() - _ascent;
  int lines_left = (H - y) / _line_h;
  if (lines_left < 1) lines_left = 1;
  drawWrapped(x, y, max_width - x, lines_left, 0, str);
}

static inline uint32_t fnv1a(const uint8_t* p, int n) {
  uint32_t h = 2166136261u;
  while (n--) { h ^= *p++; h *= 16777619u; }
  return h;
}

void ST7789ColorDisplay::endFrame() {
  if (!_isOn) return;
  int run_start = -1;
  for (int y = 0; y <= H; y++) {
    bool dirty = false;
    if (y < H) {
      uint32_t h = fnv1a(&_fb[y * W / 2], W / 2);
      dirty = (h != _row_hash[y]);
      _row_hash[y] = h;
    }
    if (dirty && run_start < 0) run_start = y;
    if (!dirty && run_start >= 0) {
      pushRows(run_start, y - 1);
      run_start = -1;
    }
  }
}

#endif
