/*
 * Intelligent Predictive Protection and Optimisation System for Microgrids
 * ESP32 DC microgrid controller firmware
 *
 * Features:
 *  - millis()-based sequential load energisation (10 s / 20 s / 30 s)
 *  - Dual-relay source switching for Load 3 (microgrid <-> SMPS)
 *  - ACS712 current sensing, resistive-divider voltage sensing
 *  - SH1106 OLED status display via U8g2
 *
 * Board:    ESP32 Dev Module
 * Libraries: U8g2 (Oliver Kraus)
 */

#include <Wire.h>
#include <U8g2lib.h>

#define SDA_PIN 21
#define SCK_PIN 22
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, SCK_PIN, SDA_PIN, U8X8_PIN_NONE);

// ── Pins ──
#define SYS_CURR_PIN   35  // Total System Current
#define LOAD3_CURR_PIN 33  // Dedicated Load 3 Current
#define BAT_VOLT_PIN   34  // Battery Voltage
#define SMPS_VOLT_PIN  32  // SMPS Status Voltage
#define RELAY2_PIN     25  // Load 1
#define RELAY3_PIN     26  // Load 2
#define RELAY4_PIN     18  // Load 3 -> Microgrid
#define RELAY5_PIN     12  // Load 3 -> SMPS

// ── Calibration ──
float zeroL3 = 0, zeroSys = 0;
float OFFSET_SYS = 0.05;
float OFFSET_L3  = 0.29;

// ── Thresholds ──
#define L3_CURRENT_HIGH        0.800  // Amps, forward switching threshold (battery to SMPS)
#define L3_CURRENT_THRESHOLD   0.700  // Amps, recovery threshold (SMPS to battery)
#define SMPS_VOLTAGE_THRESHOLD 1.5    // Volts, minimum to consider SMPS online
#define SMPS_CHECK_INTERVAL    5000   // ms, how often to poll SMPS when L3 is off
#define STABILITY_WINDOW       7000   // ms, settle time before return-to-grid check

// ── Load 3 State Machine ──
// L3 can be in one of four states:
//   MICROGRID , normal operation, powered from microgrid
//   SMPS      , overcurrent detected, running on SMPS
//   OFF       , overcurrent detected, SMPS was offline, load is shed
//   TESTING   , SMPS just came back, measuring current before deciding routing
typedef enum {
  L3_MICROGRID,
  L3_SMPS,
  L3_OFF,
  L3_TESTING
} Load3State;

Load3State l3State = L3_MICROGRID;

// ── System Variables ──
bool smpsOnline = false;
float batVoltage = 0;
unsigned long startTime = 0;
unsigned long stabilityTimer = 0;  // used for return-to-grid settle period
unsigned long smpsCheckTimer = 0;  // used for periodic SMPS polling while L3 is OFF
unsigned long testStartTimer = 0;  // used to allow current to settle during TESTING state

// ── Helper: Read Current ──
float getAmps(int pin, float zeroV, float manualOffset) {
  long sum = 0;
  for (int i = 0; i < 200; i++) sum += analogRead(pin);
  float voltage = (sum / 200.0) * 3.3 / 4095.0;
  float amps = (voltage - zeroV) / 0.185;
  amps = amps - manualOffset;
  return (amps < 0.05) ? 0.000 : amps;
}

// ── Helper: Read Voltage ──
float getVoltage(int pin) {
  long sum = 0;
  for (int i = 0; i < 50; i++) sum += analogRead(pin);
  float v = (sum / 50.0) * 3.3 / 4095.0;
  return v * 3.64;
}

// ── Helper: Disconnect L3 from everything ──
void disconnectL3() {
  digitalWrite(RELAY4_PIN, HIGH);  // Open microgrid relay
  delay(100);
  digitalWrite(RELAY5_PIN, HIGH);  // Open SMPS relay
}

// ── Helper: Connect L3 to microgrid only ──
void connectL3ToMicrogrid() {
  digitalWrite(RELAY5_PIN, HIGH);  // Ensure SMPS relay open first
  delay(200);
  digitalWrite(RELAY4_PIN, LOW);   // Close microgrid relay
}

// ── Helper: Connect L3 to SMPS only ──
void connectL3ToSMPS() {
  digitalWrite(RELAY4_PIN, HIGH);  // Open microgrid relay first
  delay(200);
  digitalWrite(RELAY5_PIN, LOW);   // Close SMPS relay
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(RELAY2_PIN, OUTPUT); digitalWrite(RELAY2_PIN, HIGH);
  pinMode(RELAY3_PIN, OUTPUT); digitalWrite(RELAY3_PIN, HIGH);
  pinMode(RELAY4_PIN, OUTPUT); digitalWrite(RELAY4_PIN, HIGH);
  pinMode(RELAY5_PIN, OUTPUT); digitalWrite(RELAY5_PIN, HIGH);

  u8g2.begin();

  // Baseline calibration
  zeroSys = (analogRead(SYS_CURR_PIN) / 4095.0) * 3.3;
  zeroL3  = (analogRead(LOAD3_CURR_PIN) / 4095.0) * 3.3;

  startTime = millis();
  smpsCheckTimer = millis();
}

void loop() {
  unsigned long now = millis();

  // ── 1. Read All Sensors ──
  float currentL3  = getAmps(LOAD3_CURR_PIN, zeroL3, OFFSET_L3);
  float currentSys = getAmps(SYS_CURR_PIN, zeroSys, OFFSET_SYS);
  batVoltage = getVoltage(BAT_VOLT_PIN);
  float smpsDetectV = getVoltage(SMPS_VOLT_PIN);
  smpsOnline = (smpsDetectV > SMPS_VOLTAGE_THRESHOLD);

  // ── 2. Startup Sequence (staggered load connection) ──
  if (now - startTime > 10000 && digitalRead(RELAY2_PIN) == HIGH)
    digitalWrite(RELAY2_PIN, LOW);

  if (now - startTime > 20000 && digitalRead(RELAY3_PIN) == HIGH)
    digitalWrite(RELAY3_PIN, LOW);

  if (now - startTime > 30000 && l3State == L3_MICROGRID && digitalRead(RELAY4_PIN) == HIGH) {
    connectL3ToMicrogrid();
  }

  // ── 3. Load 3 State Machine (active after startup completes) ──
  if (now - startTime > 31000) {
    switch (l3State) {

      // ── State: L3 running on microgrid ──
      case L3_MICROGRID:
        if (currentL3 > L3_CURRENT_HIGH) {
          // Overcurrent detected, decide where to route based on SMPS status
          Serial.println("[L3] Overcurrent detected on microgrid.");
          if (smpsOnline) {
            // SMPS is available, transfer load 3 to it
            Serial.println("[L3] SMPS online. Transferring L3 to SMPS.");
            connectL3ToSMPS();
            l3State = L3_SMPS;
            stabilityTimer = now;
          } else {
            // SMPS is offline, shed load 3 entirely
            Serial.println("[L3] SMPS offline. Shedding L3.");
            disconnectL3();
            l3State = L3_OFF;
            smpsCheckTimer = now;
          }
        }
        break;

      // ── State: L3 running on SMPS ──
      case L3_SMPS:
        // After stability window, check if current has dropped enough to return to microgrid
        if (now - stabilityTimer > STABILITY_WINDOW) {
          if (currentL3 < L3_CURRENT_THRESHOLD) {
            Serial.println("[L3] Current nominal. Returning L3 to microgrid.");
            connectL3ToMicrogrid();
            l3State = L3_MICROGRID;
          } else {
            // Current still high, stay on SMPS, reset timer to check again later
            Serial.println("[L3] Current still high on SMPS. Staying on SMPS.");
            stabilityTimer = now;
          }
        }
        break;

      // ── State: L3 shed (off), waiting for SMPS to come back ──
      case L3_OFF:
        // Periodically poll SMPS status
        if (now - smpsCheckTimer > SMPS_CHECK_INTERVAL) {
          smpsCheckTimer = now;
          Serial.println("[L3] Polling SMPS status...");
          if (smpsOnline) {
            // SMPS restored, connect L3 to SMPS and enter testing state
            Serial.println("[L3] SMPS back online. Connecting L3 to SMPS for current test.");
            connectL3ToSMPS();
            l3State = L3_TESTING;
            testStartTimer = now;
          } else {
            Serial.println("[L3] SMPS still offline. L3 remains shed.");
          }
        }
        break;

      // ── State: L3 just connected to SMPS after restoration, measuring current ──
      case L3_TESTING:
        // Wait 2 seconds for current to stabilise before measuring
        if (now - testStartTimer > 2000) {
          Serial.print("[L3] SMPS restoration test, L3 current: ");
          Serial.print(currentL3, 3);
          Serial.println("A");
          if (currentL3 < L3_CURRENT_THRESHOLD) {
            // Load is within limits on SMPS, move it back to microgrid
            Serial.println("[L3] Current acceptable. Transferring L3 back to microgrid.");
            connectL3ToMicrogrid();
            l3State = L3_MICROGRID;
          } else {
            // Load still drawing too much, keep it on SMPS
            Serial.println("[L3] Current still high. Keeping L3 on SMPS.");
            l3State = L3_SMPS;
            stabilityTimer = now;
          }
        }
        break;
    }
  }

  // ── 4. Update Display ──
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);

  // Row 1: Voltage & SMPS Status
  u8g2.setCursor(0, 10);
  u8g2.print("BAT:"); u8g2.print(batVoltage, 1); u8g2.print("V");
  u8g2.setCursor(70, 10);
  u8g2.print("SMPS:"); u8g2.print(smpsOnline ? "ON" : "OFF");
  u8g2.drawHLine(0, 12, 128);

  // Row 2: Current Readings
  u8g2.setCursor(0, 24);
  u8g2.print("SYS: "); u8g2.print(currentSys, 3); u8g2.print("A");
  u8g2.setCursor(0, 36);
  u8g2.print("L3: "); u8g2.print(currentL3, 3); u8g2.print("A");
  u8g2.drawHLine(0, 38, 128);

  // Row 3: Load 1 & Load 2 status
  u8g2.setCursor(0, 50);
  u8g2.print(digitalRead(RELAY2_PIN) == LOW ? "L1:ON  " : "L1:OFF ");
  u8g2.print(digitalRead(RELAY3_PIN) == LOW ? "L2:ON" : "L2:OFF");

  // Row 4: Load 3 state
  u8g2.setCursor(0, 62);
  u8g2.print("L3:");
  switch (l3State) {
    case L3_MICROGRID: u8g2.print("ON(GRID)");  break;
    case L3_SMPS:      u8g2.print("ON(SMPS)");  break;
    case L3_OFF:       u8g2.print("SHED");      break;
    case L3_TESTING:   u8g2.print("TESTING.."); break;
  }
  u8g2.sendBuffer();

  // ── 5. Serial Monitor Telemetry ──
  // Prints a formatted status line every loop cycle so readings are
  // visible in the Arduino IDE Serial Monitor alongside event messages.
  Serial.print("[IPPOS] ");
  Serial.print("BAT:");  Serial.print(batVoltage, 2); Serial.print("V ");
  Serial.print("SMPS:"); Serial.print(smpsOnline ? "ONLINE " : "OFFLINE ");
  Serial.print("SYS:");  Serial.print(currentSys, 3); Serial.print("A ");
  Serial.print("L3:");   Serial.print(currentL3, 3);  Serial.print("A ");
  Serial.print("L1:");   Serial.print(digitalRead(RELAY2_PIN) == LOW ? "ON " : "OFF ");
  Serial.print("L2:");   Serial.print(digitalRead(RELAY3_PIN) == LOW ? "ON " : "OFF ");
  Serial.print("L3_STATE:");
  switch (l3State) {
    case L3_MICROGRID: Serial.println("MICROGRID"); break;
    case L3_SMPS:      Serial.println("SMPS");      break;
    case L3_OFF:       Serial.println("SHED");      break;
    case L3_TESTING:   Serial.println("TESTING");   break;
  }

  delay(100);
}
