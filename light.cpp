/*
  Project: Automatic Street Light System
  Features:
  - LDR analog read with moving average smoothing
  - Day/Night hysteresis (separate ON/OFF thresholds)
  - Optional auto-calibration (learns ambient min/max at startup window)
  - Relay output (active HIGH or LOW configurable)
  - Status LEDs (optional)
  - Periodic telemetry over Serial
  - Simple Serial CLI: STATUS, SET ON x, SET OFF x, SAVE, HELP
*/

#include <EEPROM.h>

// ------------------ USER CONFIG ------------------
const uint8_t PIN_LDR = A0;
const uint8_t PIN_RELAY = 8;       // Connect relay IN pin
const bool RELAY_ACTIVE_HIGH = true;

const uint8_t PIN_LED_STATUS = 13; // onboard LED (optional)
const bool USE_STATUS_LED = true;

// Hysteresis thresholds (raw 0..1023)
uint16_t THRESH_ON  = 480;   // go ON when LDR reading <= THRESH_ON (darker)
uint16_t THRESH_OFF = 520;   // go OFF when LDR reading >= THRESH_OFF (brighter)
const uint16_t MIN_GAP = 20; // force gap between ON/OFF

// Auto-calibration
const bool ENABLE_AUTO_CALIB = true;
const uint32_t CALIB_WINDOW_MS = 15000;  // learn min/max for first 15s

// Smoothing
const uint8_t AVG_WINDOW = 20;     // moving average length (max 50)
const uint16_t SAMPLE_INTERVAL_MS = 50;

// Telemetry
const uint32_t TELEMETRY_MS = 2000;

// EEPROM layout
const int EE_ADDR_MAGIC = 0;
const int EE_ADDR_ON = 2;
const int EE_ADDR_OFF = 4;
const uint16_t MAGIC = 0x5AA5;
// -------------------------------------------------

// Ring buffer for moving average
uint16_t buf[50];
uint8_t head = 0;
uint8_t count = 0;
uint32_t lastSample = 0;
uint32_t lastTelemetry = 0;

enum LightState { LS_OFF = 0, LS_ON = 1 };
LightState lightState = LS_OFF;

uint16_t rawMin = 1023, rawMax = 0;
uint32_t calibStart = 0;
bool calibDone = false;

void setRelay(bool on) {
  bool level = RELAY_ACTIVE_HIGH ? on : !on;
  digitalWrite(PIN_RELAY, level ? HIGH : LOW);
  if (USE_STATUS_LED) digitalWrite(PIN_LED_STATUS, on ? HIGH : LOW);
  lightState = on ? LS_ON : LS_OFF;
}

uint16_t readLDR() {
  return analogRead(PIN_LDR);
}

uint16_t smooth(uint16_t v) {
  buf[head] = v;
  head = (head + 1) % AVG_WINDOW;
  if (count < AVG_WINDOW) count++;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < count; i++) sum += buf[i];
  return (uint16_t)(sum / count);
}

void eepromLoad() {
  uint16_t m = 0;
  EEPROM.get(EE_ADDR_MAGIC, m);
  if (m == MAGIC) {
    EEPROM.get(EE_ADDR_ON, THRESH_ON);
    EEPROM.get(EE_ADDR_OFF, THRESH_OFF);
  }
}

void eepromSave() {
  EEPROM.put(EE_ADDR_MAGIC, MAGIC);
  EEPROM.put(EE_ADDR_ON, THRESH_ON);
  EEPROM.put(EE_ADDR_OFF, THRESH_OFF);
}

void applyHysteresis(uint16_t value) {
  if (lightState == LS_OFF && value <= THRESH_ON) {
    setRelay(true);
  } else if (lightState == LS_ON && value >= THRESH_OFF) {
    setRelay(false);
  }
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  STATUS           -> print thresholds, state, value"));
  Serial.println(F("  SET ON <0-1023>  -> set ON threshold"));
  Serial.println(F("  SET OFF <0-1023> -> set OFF threshold"));
  Serial.println(F("  SAVE             -> save thresholds to EEPROM"));
  Serial.println(F("  HELP             -> show this help"));
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toUpperCase();

  if (line == "HELP") {
    printHelp();
    return;
  }
  if (line == "STATUS") {
    Serial.print(F("ON=")); Serial.print(THRESH_ON);
    Serial.print(F(" OFF=")); Serial.print(THRESH_OFF);
    Serial.print(F(" STATE=")); Serial.print(lightState == LS_ON ? "ON" : "OFF");
    Serial.print(F(" RAWMIN=")); Serial.print(rawMin);
    Serial.print(F(" RAWMAX=")); Serial.print(rawMax);
    Serial.println();
    return;
  }
  if (line.startsWith("SET ON ")) {
    int v = line.substring(7).toInt();
    v = constrain(v, 0, 1023);
    THRESH_ON = v;
    if (THRESH_OFF < THRESH_ON + MIN_GAP) THRESH_OFF = min((int)THRESH_ON + MIN_GAP, 1023);
    Serial.println(F("OK"));
    return;
  }
  if (line.startsWith("SET OFF ")) {
    int v = line.substring(8).toInt();
    v = constrain(v, 0, 1023);
    THRESH_OFF = v;
    if (THRESH_ON > THRESH_OFF - MIN_GAP) THRESH_ON = max((int)THRESH_OFF - MIN_GAP, 0);
    Serial.println(F("OK"));
    return;
  }
  if (line == "SAVE") {
    eepromSave();
    Serial.println(F("SAVED"));
    return;
  }
  Serial.println(F("ERR: Unknown. Type HELP"));
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  if (USE_STATUS_LED) pinMode(PIN_LED_STATUS, OUTPUT);
  Serial.begin(9600);
  eepromLoad();
  setRelay(false);

  calibStart = millis();
  Serial.println(F("StreetLight: boot"));
  printHelp();
}

void loop() {
  handleSerial();

  uint32_t now = millis();

  if (!calibDone && ENABLE_AUTO_CALIB) {
    if (now - calibStart <= CALIB_WINDOW_MS) {
      uint16_t r = readLDR();
      if (r < rawMin) rawMin = r;
      if (r > rawMax) rawMax = r;
    } else {
      calibDone = true;
      // Place ON/OFF roughly around learned range if sensible
      if (rawMax > rawMin + 50) {
        THRESH_ON  = rawMin + (rawMax - rawMin) * 0.35; // darker
        THRESH_OFF = rawMin + (rawMax - rawMin) * 0.55; // brighter
        if (THRESH_OFF < THRESH_ON + MIN_GAP) THRESH_OFF = THRESH_ON + MIN_GAP;
      }
      Serial.println(F("Auto-calibration completed"));
      Serial.print(F("New ON/OFF: ")); Serial.print(THRESH_ON); Serial.print('/'); Serial.println(THRESH_OFF);
    }
  }

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;
    uint16_t raw = readLDR();
    if (raw < rawMin) rawMin = raw;
    if (raw > rawMax) rawMax = raw;

    uint16_t val = smooth(raw);
    applyHysteresis(val);
  }

  if (now - lastTelemetry >= TELEMETRY_MS) {
    lastTelemetry = now;
    Serial.print(F("VAL="));
    Serial.print(buf[(head + AVG_WINDOW - 1) % AVG_WINDOW]);
    Serial.print(F(" AVG="));
    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) sum += buf[i];
    Serial.print(sum / max<uint8_t>(1, count));
    Serial.print(F(" STATE=")); Serial.print(lightState == LS_ON ? "ON" : "OFF");
    Serial.print(F(" ON=")); Serial.print(THRESH_ON);
    Serial.print(F(" OFF=")); Serial.print(THRESH_OFF);
    Serial.println();
  }
}
