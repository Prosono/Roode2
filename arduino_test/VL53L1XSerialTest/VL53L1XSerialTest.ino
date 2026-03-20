#include <Wire.h>
#include <VL53L1X.h>

const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;
const uint32_t SERIAL_BAUD = 115200;
const uint32_t I2C_FREQUENCY_HZ = 100000;
const uint16_t REPORT_INTERVAL_MS = 400;
const uint16_t MIN_VALID_DISTANCE_MM = 40;
const uint16_t MAX_VALID_DISTANCE_MM = 4000;
const uint8_t AVERAGE_WINDOW = 4;

VL53L1X sensor;
unsigned long lastStatusMs = 0;
unsigned long lastInvalidLogMs = 0;
uint16_t validSamples[AVERAGE_WINDOW] = {};
uint8_t sampleCount = 0;
uint8_t nextSampleIndex = 0;

void addSample(uint16_t distanceMm) {
  validSamples[nextSampleIndex] = distanceMm;
  nextSampleIndex = (nextSampleIndex + 1) % AVERAGE_WINDOW;
  if (sampleCount < AVERAGE_WINDOW) {
    sampleCount++;
  }
}

uint16_t averageDistance() {
  if (sampleCount == 0) {
    return 0;
  }

  uint32_t total = 0;
  for (uint8_t i = 0; i < sampleCount; i++) {
    total += validSamples[i];
  }
  return total / sampleCount;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1500);

  Serial.println();
  Serial.println("Starter enkel VL53L1X test");
  Serial.print("I2C SDA=");
  Serial.print(SDA_PIN);
  Serial.print(", SCL=");
  Serial.println(SCL_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQUENCY_HZ);

  sensor.setTimeout(0);
  if (!sensor.init()) {
    Serial.println("Fant ikke VL53L1X. Sjekk kabling og strom.");
    while (true) {
      delay(1000);
    }
  }

  sensor.setDistanceMode(VL53L1X::Short);
  if (!sensor.setMeasurementTimingBudget(50000)) {
    Serial.println("Kunne ikke sette timing budget.");
    while (true) {
      delay(1000);
    }
  }

  sensor.startContinuous(120);
  Serial.println("Sensor klar. Hold et matt objekt 15-50 cm foran sensoren.");
}

void loop() {
  if (!sensor.dataReady()) {
    if (millis() - lastStatusMs > 1000) {
      lastStatusMs = millis();
      Serial.println("Venter paa ny maaling...");
    }
    delay(10);
    return;
  }

  static unsigned long lastReportMs = 0;
  if (millis() - lastReportMs < REPORT_INTERVAL_MS) {
    sensor.read(false);
    return;
  }
  lastReportMs = millis();

  uint16_t distanceMm = sensor.read(false);
  bool statusValid = sensor.ranging_data.range_status == VL53L1X::RangeValid;
  bool distanceValid = distanceMm >= MIN_VALID_DISTANCE_MM && distanceMm <= MAX_VALID_DISTANCE_MM;

  if (!statusValid || !distanceValid) {
    if (millis() - lastInvalidLogMs > 1000) {
      lastInvalidLogMs = millis();
      Serial.print("Ignorerer ugyldig maaling: ");
      Serial.print(distanceMm);
      Serial.print(" mm");
      Serial.print(" | Status: ");
      Serial.println(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
    }
    return;
  }

  addSample(distanceMm);
  uint16_t smoothedDistanceMm = averageDistance();

  Serial.print("Avstand: ");
  Serial.print(distanceMm);
  Serial.print(" mm");
  Serial.print(" | Glattet: ");
  Serial.print(smoothedDistanceMm);
  Serial.println(" mm");
}
