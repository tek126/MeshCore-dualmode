#pragma once

// Native-resolution colour driver for the Heltec T114's 240x135 ST7789 panel.
//
// The stock ST7789Display draws on a 128x64 logical canvas, scaled ~1.9x and
// painted in one colour per frame. This driver exposes the real 240x135 grid
// and a 16-colour palette: a 4-bit framebuffer (~16 KB) holds the frame, and
// endFrame() streams only the rows that changed since the last frame.
//
// ColorVal values are palette indices (see TftColor). setTextSize() picks a
// font: 0 = tiny 6x8, 1 = body (FreeSans 9pt), 2 = bold (FreeSans Bold 9pt),
// 3 = large (FreeSans Bold 18pt). setCursor() takes the TOP of the text line,
// like the other MeshCore drivers.

#include "DisplayDriver.h"
#include <SPI.h>
#include <Adafruit_GFX.h>

namespace TftColor {
  enum : ColorVal {
    BLACK = 0, WHITE, LIGHT_GREY, GREY, DARK_GREY, NAVY, BLUE, CYAN,
    GREEN, DARK_GREEN, YELLOW, ORANGE, RED, PURPLE, HILITE, DIM_BLUE,
    COUNT
  };
}

class ST7789ColorDisplay : public DisplayDriver {
public:
  static const int W = 240;
  static const int H = 135;

private:
  // Adafruit_GFX drawing into the 4-bit framebuffer
  class Canvas4 : public Adafruit_GFX {
    uint8_t* _buf;
  public:
    Canvas4(uint8_t* buf) : Adafruit_GFX(W, H), _buf(buf) { }
    void drawPixel(int16_t x, int16_t y, uint16_t c) override;
    void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) override;
    void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) override { writeFastHLine(x, y, w, c); }
    void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) override;
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) override { writeFillRect(x, y, w, h, c); }
    void fillScreen(uint16_t c) override;
  };

  uint8_t _fb[W * H / 2];        // 2 pixels per byte, high nibble = even x
  uint32_t _row_hash[H];         // per-row hash of what the panel currently shows
  Canvas4 _gfx;
  SPIClass* _spi;
  SPISettings _spi_settings;
  bool _isOn;
  ColorVal _color;
  int _font;
  int _ascent;                   // pixels from line top to baseline, current font
  int _line_h;                   // line pitch, current font

  void writeCmd(uint8_t c);
  void writeData(const uint8_t* d, size_t n);
  void setAddrWindow(int x, int y, int w, int h);
  void panelInit();
  void panelClearAll();
  void pushRows(int y0, int y1);
  void drawGlyphBox(int x, int y_top);
  int charWidth(uint8_t c);
  int unknownWidth() const;

public:
  ST7789ColorDisplay();
  bool begin();

  bool isOn() override { return _isOn; }
  void turnOn() override;
  void turnOff() override;
  void clear() override;
  void startFrame(ColorVal bkg = UIColor::window_bkg) override;
  void setTextSize(int sz) override;
  void setColor(ColorVal c) override;
  void setCursor(int x, int y) override;
  void print(const char* str) override;
  void printWordWrap(const char* str, int max_width) override;
  void fillRect(int x, int y, int w, int h) override;
  void drawRect(int x, int y, int w, int h) override;
  void drawXbm(int x, int y, const uint8_t* bits, int w, int h) override;
  uint16_t getTextWidth(const char* str) override;
  void endFrame() override;

  // --- extras for the T114 UI (not part of DisplayDriver) ---
  Adafruit_GFX& gfx() { return _gfx; }
  ColorVal color() const { return _color; }
  int lineHeight() const { return _line_h; }
  // Word-wrap str into the box (x, y, w); draws at most max_lines lines after
  // skipping skip_lines. Returns the total number of wrapped lines.
  int drawWrapped(int x, int y, int w, int max_lines, int skip_lines, const char* str);
  // Draws str clipped with an ellipsis to max_w.
  void drawClipped(int x, int y, int max_w, const char* str);
};
