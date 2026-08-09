#include "HeltecV4Board.h"

void HeltecV4Board::begin() {
    ESP32Board::begin();


    pinMode(PIN_ADC_CTRL, OUTPUT);
    digitalWrite(PIN_ADC_CTRL, LOW); // Initially inactive

    loRaFEMControl.init();

    periph_power.begin();
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_DEEPSLEEP) {
      long wakeup_source = esp_sleep_get_ext1_wakeup_status();
      if (wakeup_source & (1 << P_LORA_DIO_1)) {  // received a LoRa packet (while in deep sleep)
        startup_reason = BD_STARTUP_RX_PACKET;
    }

      rtc_gpio_hold_dis((gpio_num_t)P_LORA_NSS);
      rtc_gpio_deinit((gpio_num_t)P_LORA_DIO_1);
    }
  }

  void HeltecV4Board::onBeforeTransmit(void) {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED on
    loRaFEMControl.setTxModeEnable();
  }

  void HeltecV4Board::onAfterTransmit(void) {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED off
    loRaFEMControl.setRxModeEnable();
  }

  void HeltecV4Board::powerOff() {
    // Turn off PA
    digitalWrite(P_LORA_PA_POWER, LOW);
    rtc_gpio_hold_en((gpio_num_t)P_LORA_PA_POWER);

    ESP32Board::powerOff();
  }

  void HeltecV4Board::hibernateButtonWake(int pin_btn) {
    // Button-only hibernate. Unlike enterDeepSleep()/powerOff(), this does NOT
    // arm the LoRa DIO1 wake source, so the node stays asleep through incoming
    // mesh traffic and only the user button revives it.
    //
    // PIN_USER_BTN (GPIO0) is active-low, so we wake on it being driven LOW.
    // It's also the BOOT strapping pin: wait for release before sleeping (so we
    // don't wake instantly), and on wake a brief TAP is safest -- holding GPIO0
    // through the reset can drop the chip into serial-download mode.
    pinMode(pin_btn, INPUT_PULLUP);
    unsigned long t0 = millis();
    while (digitalRead(pin_btn) == LOW && (unsigned long)(millis() - t0) < 10000) {
      delay(10);   // wait (<=10 s) for the user to let go
    }

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    // Keep the radio's SPI chip-select held so it stays quiescent during sleep.
    rtc_gpio_hold_en((gpio_num_t)P_LORA_NSS);

    // Arm the button (and only the button) as an RTC wake source on a LOW level.
    rtc_gpio_init((gpio_num_t)pin_btn);
    rtc_gpio_set_direction((gpio_num_t)pin_btn, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis((gpio_num_t)pin_btn);
    rtc_gpio_pullup_en((gpio_num_t)pin_btn);
    esp_sleep_enable_ext1_wakeup((1ULL << pin_btn), ESP_EXT1_WAKEUP_ANY_LOW);

    esp_deep_sleep_start();   // CPU halts here and never returns!
  }

  uint16_t HeltecV4Board::getBattMilliVolts()  {
    analogReadResolution(10);
    digitalWrite(PIN_ADC_CTRL, HIGH);
    delay(10);
    uint32_t raw = 0;
    for (int i = 0; i < 8; i++) {
      raw += analogRead(PIN_VBAT_READ);
    }
    raw = raw / 8;

    digitalWrite(PIN_ADC_CTRL, LOW);

    return (adc_mult * (3.3 / 1024.0) * raw) * 1000;
  }

  const char* HeltecV4Board::getManufacturerName() const {
#ifdef HELTEC_LORA_V4_TFT
    return loRaFEMControl.getFEMType() == KCT8103L_PA ? "Heltec V4.3 TFT" : "Heltec V4 TFT";
#else
    return loRaFEMControl.getFEMType() == KCT8103L_PA ? "Heltec V4.3 OLED" : "Heltec V4 OLED";
#endif
  }

  bool HeltecV4Board::setLoRaFemLnaEnabled(bool enable) {
    if (!loRaFEMControl.isLnaCanControl()) {
      return false;
    }

    loRaFEMControl.setLNAEnable(enable);
    loRaFEMControl.setRxModeEnable();
    return true;
  }

  bool HeltecV4Board::canControlLoRaFemLna() const {
    return loRaFEMControl.isLnaCanControl();
  }

  bool HeltecV4Board::isLoRaFemLnaEnabled() const {
    return loRaFEMControl.isLNAEnabled();
  }
