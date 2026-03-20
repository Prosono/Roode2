#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X_ULD.h"

namespace {

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint16_t READ_INTERVAL_MS = 200;

VL53L1X_ULD sensor;

bool print_error(const char *action, VL53L1_Error status) {
  if (status == VL53L1_ERROR_NONE) {
    return false;
  }

  Serial.printf("%s feilet. Feilkode: %d\n", action, status);
  return true;
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1500);

  Serial.println();
  Serial.println("Starter VL53L1X seriell test");
  Serial.printf("I2C SDA=%u, SCL=%u\n", SDA_PIN, SCL_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQUENCY_HZ);

  auto status = sensor.Begin();
  if (print_error("Kunne ikke initialisere sensoren", status)) {
    Serial.println("Sjekk kabling, strom og I2C-adresse.");
    while (true) {
      delay(1000);
    }
  }

  status = sensor.StartRanging();
  if (print_error("Kunne ikke starte ranging", status)) {
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Sensor initialisert. Leser avstand i millimeter...");
}

void loop() {
  uint8_t data_ready = 0;
  auto status = sensor.CheckForDataReady(&data_ready);
  if (print_error("Kunne ikke sjekke data-ready", status)) {
    delay(1000);
    return;
  }

  if (!data_ready) {
    delay(5);
    return;
  }

  uint16_t distance_mm = 0;
  status = sensor.GetDistanceInMm(&distance_mm);
  if (print_error("Kunne ikke lese avstand", status)) {
    delay(1000);
    return;
  }

  status = sensor.ClearInterrupt();
  if (print_error("Kunne ikke cleare interrupt", status)) {
    delay(1000);
    return;
  }

  Serial.printf("Avstand: %u mm\n", distance_mm);
  delay(READ_INTERVAL_MS);
}
