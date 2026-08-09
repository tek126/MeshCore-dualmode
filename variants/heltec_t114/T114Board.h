#pragma once

#include <MeshCore.h>
#include <Arduino.h>
#include <helpers/NRF52Board.h>

// built-ins
#define  PIN_VBAT_READ    4
#define  PIN_BAT_CTL      6
#define  MV_LSB   (3000.0F / 4096.0F) // 12-bit ADC with 3.0V input range

class T114Board : public NRF52BoardDCDC {
protected:
#ifdef NRF52_POWER_MANAGEMENT
  void initiateShutdown(uint8_t reason) override;
#endif

public:
  T114Board() : NRF52Board("T114_OTA") {}
  void begin();

#if defined(P_LORA_TX_LED)
  void onBeforeTransmit() override {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED on
  }
  void onAfterTransmit() override {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED off
  }
#endif

  uint16_t getBattMilliVolts() override {
    int adcvalue = 0;
    analogReadResolution(12);
    analogReference(AR_INTERNAL_3_0);
    pinMode(PIN_BAT_CTL, OUTPUT);          // battery adc can be read only ctrl pin 6 set to high
    digitalWrite(PIN_BAT_CTL, 1);

    delay(10);
    adcvalue = analogRead(PIN_VBAT_READ);
    digitalWrite(6, 0);

    return (uint16_t)((float)adcvalue * MV_LSB * 4.9);
  }

  const char* getManufacturerName() const override {
    return "Heltec T114";
  }

  void powerOff() override {
#ifdef LED_PIN
    digitalWrite(LED_PIN, HIGH);
#endif

    NRF52Board::powerOff();
  }

  // Hibernate waking on the user button. Waits for the button to be released
  // (so DETECT isn't already latched), arms a GPIO SENSE wake on it, then enters
  // SYSTEMOFF. SD-aware: sd_power_system_off() just RETURNS without powering off
  // when the SoftDevice isn't enabled (e.g. a non-BLE repeater build), so fall
  // back to the POWER register directly, then reset if even that returns. Waking
  // from SYSTEMOFF is a reset, so the node reboots. Does not return.
  void hibernateButtonWake(int pin_btn) {
#ifdef LED_PIN
    digitalWrite(LED_PIN, HIGH);
#endif
#if ENV_INCLUDE_GPS == 1
    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, LOW);
#endif
    // wait (<=10 s) for the user to let go, else the sense wake trips instantly
    pinMode(pin_btn, INPUT_PULLUP);
    unsigned long t0 = millis();
    while (digitalRead(pin_btn) == LOW && (unsigned long)(millis() - t0) < 10000) delay(10);

    nrf_gpio_cfg_sense_input(g_ADigitalPinMap[pin_btn],
                             NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

    Serial.flush();
    uint8_t sd_enabled = 0;
    sd_softdevice_is_enabled(&sd_enabled);
    if (sd_enabled) {
      if (sd_power_system_off() == NRF_ERROR_SOFTDEVICE_NOT_ENABLED) sd_enabled = 0;
    }
    if (!sd_enabled) {
      NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;   // SoftDevice off: direct register
    }
    NVIC_SystemReset();   // never reached in normal operation
  }
};
