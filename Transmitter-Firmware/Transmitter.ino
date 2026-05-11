#include <SPI.h>
#include <RF24.h>
#include <TFT_eSPI.h>

// ===== NRF24 pins on ESP32 (separate SPI bus) =====
#define NRF_CE   16
#define NRF_CSN  15
#define NRF_SCK  14
#define NRF_MISO 19
#define NRF_MOSI 13

// ===== TFT =====
#define TFT_CS   5

// ===== Transmitter Battery Voltage =====
#define TX_BAT_PIN      36   // Change to 39 if you prefer
// Transmitter battery voltage
float txBatteryVoltage = 0.0f;
float prevTxBat = -999.0f;
// ===== Inputs =====
#define THROTTLE_PIN    34
#define YAW_PIN         35
#define PITCH_PIN       32
#define ROLL_PIN        33
#define ARM_SWITCH_PIN  25

// ===== Screen switch button =====
#define ENC_SW 17

#define THROTTLE_IDLE 1000


RF24 radio(NRF_CE, NRF_CSN);
SPIClass nrfSPI(HSPI);
TFT_eSPI tft = TFT_eSPI();

const byte address[6] = "DRONE";

struct RadioData {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  uint8_t arm;
};

struct TelemetryData {
  float gx;
  float gy;
  float gz;
  float batteryVoltage;
  uint8_t armed;
  uint8_t failsafe;
};

RadioData data;
TelemetryData telemetry;

// Raw ADC
int rawThrottle = 0, rawYaw = 0, rawPitch = 0, rawRoll = 0;

static float throttleValue = 1000.0f;

const int THR_CENTER = 2048;
const int THR_DEADZONE = 80;
const float THR_SPEED = 0.6f;
const float THR_MAX_STEP = 8.0f;
// Display vars
int16_t roll_t = 0;
int16_t pitch_t = 0;
int16_t yaw_t = 0;

bool radioOk = false;
bool rfSent = false;
bool rfReceived = false;

// Screen state
enum ScreenState {
  SCREEN_MAIN,
  SCREEN_DEBUG
};

ScreenState currentScreen = SCREEN_MAIN;

bool mainStaticDrawn = false;
bool debugStaticDrawn = false;

// Previous values
int prevRoll = -999, prevPitch = -999, prevYaw = -999;
int prevThr = -1, prevYawRc = -1, prevPitchRc = -1, prevRollRc = -1;
int prevArm = -1;
float prevBat = -999.0f;
bool prevRfSent = false, prevRfReceived = false;
int prevArmed = -1, prevFailsafe = -1;
int prevRadioOk = -1;

unsigned long lastScreenUpdate = 0;

static float readTxBatteryVoltage() {
  const float DIVIDER_RATIO = 2.0f;
  const float ADC_REFERENCE = 3.3f;
  int raw = analogRead(TX_BAT_PIN);
  float voltageAtPin = (raw / 4095.0f) * ADC_REFERENCE;
  return voltageAtPin * DIVIDER_RATIO;
}

static uint16_t mapAxisToRC(int rawValue, int center, int minVal, int maxVal, bool invert = false) {
  const int DEADZONE = 100; //buvo 80
  const float EXPO = 0.35f;

  if (invert) {
    rawValue = maxVal - (rawValue - minVal);
  }

  int delta = rawValue - center;

  if (abs(delta) < DEADZONE) delta = 0;

  float range = (delta > 0) ? (maxVal - center) : (center - minVal);
  float x = (float)delta / range;

  x = x * (1 - EXPO) + x * x * x * EXPO;

  int out = 1500 + (int)(x * 500);

  return constrain(out, 1000, 2000);
}

static void setDefaults() {
  data.throttle = 1000;
  data.yaw = 1500; //1500
  data.pitch = 1500; //1500
  data.roll = 1500; //1500
  data.arm = 0;

  telemetry.gx = 0.0f;
  telemetry.gy = 0.0f;
  telemetry.gz = 0.0f;
  telemetry.batteryVoltage = 0.0f;
  telemetry.armed = 0;
  telemetry.failsafe = 1;
}

void resetPrevValues() {
  prevRoll = prevPitch = prevYaw = -999;
  prevThr = prevYawRc = prevPitchRc = prevRollRc = -1;
  prevArm = -1;
  prevBat = -999.0f;
  prevTxBat = -999.0f;
  prevRfSent = !rfSent;
  prevRfReceived = !rfReceived;
  prevArmed = -1;
  prevFailsafe = -1;
  prevRadioOk = -1;
}
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(TX_BAT_PIN, INPUT);
    // Optional: set higher attenuation for better range
    analogSetAttenuation(ADC_11db);   // Allows up to ~3.3V at the pin

  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

  pinMode(NRF_CSN, OUTPUT);
  digitalWrite(NRF_CSN, HIGH);

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(YAW_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT);
  pinMode(ROLL_PIN, INPUT);
  pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  // ===== TFT init =====
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(20, 20);
  tft.println("BOOT...");

  setDefaults();

  // ===== NRF init on separate HSPI =====
  nrfSPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN);

  radioOk = radio.begin(&nrfSPI);

  if (!radioOk) {
    Serial.println("radio.begin FAILED");
  } else {
    radio.setAutoAck(true);
    radio.enableAckPayload();
    radio.enableDynamicPayloads();
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.setChannel(108);
    radio.openWritingPipe(address);
    radio.stopListening();
    Serial.println("radio.begin OK");
  }

  Serial.println("ESP32 transmitter ready");

  tft.fillScreen(TFT_BLACK);
  mainStaticDrawn = false;
  debugStaticDrawn = false;
  resetPrevValues();
}

void loop() {
  // Read transmitter battery voltage (update less frequently is fine)
  static unsigned long lastBatRead = 0;
  if (millis() - lastBatRead >= 500) {        // every 500 ms is plenty
    txBatteryVoltage = readTxBatteryVoltage();
    lastBatRead = millis();
  }
  
  
  
  // ===== SCREEN SWITCH =====
  static bool lastEncSW = HIGH;
  bool nowEncSW = digitalRead(ENC_SW);

  if (lastEncSW == HIGH && nowEncSW == LOW) {
    currentScreen = (currentScreen == SCREEN_MAIN) ? SCREEN_DEBUG : SCREEN_MAIN;

    tft.fillScreen(TFT_BLACK);
    mainStaticDrawn = false;
    debugStaticDrawn = false;
    resetPrevValues();

    delay(200);
  }
  lastEncSW = nowEncSW;

  // ===== ADC =====
  rawThrottle = analogRead(THROTTLE_PIN);
  rawYaw      = analogRead(YAW_PIN);
  rawPitch    = analogRead(PITCH_PIN);
  rawRoll     = analogRead(ROLL_PIN);

  //invertuotas TRUE, ne invert Fa0lse
  static float rollF = 1500; //1500
  static float pitchF = 2048; //1500
  static float yawF = 1500;

  float alpha = 0.15f;

  uint16_t rollMapped  = mapAxisToRC(rawRoll, 2048, 0, 4095, false); //2048
  uint16_t pitchMapped = mapAxisToRC(rawPitch, 2048, 0, 4095, true); //2048
  uint16_t yawMapped   = mapAxisToRC(rawYaw, 2048, 0, 4095, true); //2048

  rollF  = rollF  * (1 - alpha) + rollMapped  * alpha;
  pitchF = pitchF * (1 - alpha) + pitchMapped * alpha;
  yawF   = yawF   * (1 - alpha) + yawMapped   * alpha;

  data.roll  = constrain((uint16_t)rollF + 53, 1000, 2000);
  data.pitch = constrain((uint16_t)pitchF - 50 , 1000, 2000);
  data.yaw   = constrain((uint16_t)yawF - 42, 1000, 2000);
  data.arm      = digitalRead(ARM_SWITCH_PIN) ? 0 : 1;

  // ===== THROTTLE HOLD 
  int diff = rawThrottle - 2000;

  // deadzone
  if (abs(diff) < 30) diff = 0;

  // normalizuojam -1..1
  float norm = diff / 2048.0f;

  // expo
  norm = norm * norm * norm;

  // keičiam throttle
  float delta = norm * 50.0f; // greitis

  throttleValue += delta;

  // ribos
  throttleValue = constrain(throttleValue, THROTTLE_IDLE, 2000);

  // disarm
  if (!data.arm) throttleValue = 1000;

  data.throttle = (uint16_t)throttleValue;
  //int diff = rawThrottle - THR_CENTER;

  // deadzone
  //if (abs(diff) < THR_DEADZONE) {
  //  diff = 0;
  //}

  // normalizuojam (-1 ... 1)
  //float norm = diff / 2048.0f;

  // expo (smooth center)
  //norm = norm * norm * norm;

  // kiek keisti throttle
  //float delta = norm * THR_SPEED * THR_MAX_STEP;

  // update
  //throttleValue += delta;

  // ribos
  //throttleValue = constrain(throttleValue, THROTTLE_IDLE, 2000.0f);

  // SAFETY: jei disarm → reset
  //if (!data.arm) {
  //  throttleValue = 1000.0f;
  //}

 // data.throttle = (uint16_t)throttleValue;

  // ===== RF SEND =====
  rfSent = false;
  rfReceived = false;

  if (radioOk) {
    rfSent = radio.write(&data, sizeof(data));

    if (rfSent && radio.isAckPayloadAvailable()) {
      radio.read(&telemetry, sizeof(telemetry));
      rfReceived = true;

      roll_t  = (int16_t)telemetry.gx;
      pitch_t = (int16_t)telemetry.gy;
      yaw_t   = (int16_t)telemetry.gz;
    }
  }

  // ===== SERIAL =====
  Serial.print("RADIO:");
  Serial.print(radioOk ? "OK" : "FAIL");

  Serial.print(" TX:");
  Serial.print(rfSent ? "OK" : "FAIL");

  Serial.print(" ARM_SW:");
  Serial.print(data.arm);

  Serial.print(" THR:");
  Serial.print(data.throttle);

  Serial.print(" Y:");
  Serial.print(data.yaw);

  Serial.print(" P:");
  Serial.print(data.pitch);

  Serial.print(" R:");
  Serial.print(data.roll);

  Serial.print(" | BAT:");
  Serial.print(telemetry.batteryVoltage, 2);
  Serial.print("V");

  Serial.print(" ARMED:");
  Serial.print(telemetry.armed);

  Serial.print(" FS:");
  Serial.print(telemetry.failsafe);

  Serial.print(" | Gx:");
  Serial.print(telemetry.gx, 1);

  Serial.print(" Gy:");
  Serial.print(telemetry.gy, 1);

  Serial.print(" Gz:");
  Serial.println(telemetry.gz, 1);

  // ===== DRAW only every 150 ms =====
  if (millis() - lastScreenUpdate >= 150) {
    lastScreenUpdate = millis();

    if (currentScreen == SCREEN_MAIN) {
      drawMain();
    } else {
      drawDebug();
    }
  }

  delay(20);
}

// ================= MAIN =================
void drawMain() {
  if (!mainStaticDrawn) {
    tft.setTextSize(2);

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(10, 10);   tft.print("RADIO:");
    tft.setCursor(170, 10);  tft.print("BAT:");

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(10, 40);   tft.print("ARM:");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 200);  tft.print("R:");
    tft.setCursor(110, 200); tft.print("P:");
    tft.setCursor(220, 200); tft.print("Y:");

    mainStaticDrawn = true;
  }

  if ((int)radioOk != prevRadioOk) {
    tft.fillRect(90, 10, 110, 20, TFT_BLACK);
    tft.setCursor(90, 10);
    tft.setTextColor(radioOk ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print(radioOk ? "OK" : "INIT FAIL");
    prevRadioOk = radioOk;
  }

  if (data.arm != prevArm) {
    tft.fillRect(70, 40, 60, 20, TFT_BLACK);
    tft.setCursor(70, 40);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(data.arm ? "ON" : "OFF");
    prevArm = data.arm;
  }

  if (abs(telemetry.batteryVoltage - prevBat) >= 0.01f) {
    tft.fillRect(220, 10, 90, 20, TFT_BLACK);
    tft.setCursor(220, 10);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.print(telemetry.batteryVoltage, 2);
    tft.print("V");
    prevBat = telemetry.batteryVoltage;
  }

  if (roll_t != prevRoll) {
    tft.fillRect(60, 200, 40, 20, TFT_BLACK);
    tft.setCursor(60, 200);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(roll_t);
    prevRoll = roll_t;
  }

  if (pitch_t != prevPitch) {
    tft.fillRect(170, 200, 40, 20, TFT_BLACK);
    tft.setCursor(170, 200);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(pitch_t);
    prevPitch = pitch_t;
  }

  if (yaw_t != prevYaw) {
    tft.fillRect(260, 200, 50, 20, TFT_BLACK);
    tft.setCursor(260, 200);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(yaw_t);
    prevYaw = yaw_t;
  }
}

// ================= DEBUG =================
void drawDebug() {
  if (!debugStaticDrawn) {
    tft.setTextSize(2);

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, 10); tft.print("DEBUG");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 40);   tft.print("RADIO:");
    tft.setCursor(160, 40);  tft.print("RF RX:");
    tft.setCursor(10, 70);   tft.print("RF TX:");
    tft.setCursor(10, 100);  tft.print("THR:");
    tft.setCursor(10, 120);  tft.print("YAW:");
    tft.setCursor(10, 140);  tft.print("PIT:");
    tft.setCursor(10, 160);  tft.print("ROL:");
    tft.setCursor(10, 185);  tft.print("ARM:");
    tft.setCursor(120, 185); tft.print("BAT:");     // this was drone battery
    tft.setCursor(10, 205);  tft.print("TXBAT:");   // ← NEW: transmitter battery
    tft.setCursor(10, 225);  tft.print("FS:");
    tft.setCursor(110, 225); tft.print("RXARM:");

    debugStaticDrawn = true;
  }

  if ((int)radioOk != prevRadioOk) {
    tft.fillRect(90, 40, 120, 20, TFT_BLACK);
    tft.setCursor(90, 40);
    tft.setTextColor(radioOk ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print(radioOk ? "OK" : "INIT FAIL");
    prevRadioOk = radioOk;
  }

  if (rfReceived != prevRfReceived) {
    tft.fillRect(240, 40, 40, 20, TFT_BLACK);
    tft.setCursor(240, 40);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(rfReceived ? "1" : "0");
    prevRfReceived = rfReceived;
  }

  if (rfSent != prevRfSent) {
    tft.fillRect(90, 70, 50, 20, TFT_BLACK);
    tft.setCursor(90, 70);
    tft.setTextColor(rfSent ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print(rfSent ? "OK" : "FAIL");
    prevRfSent = rfSent;
  }

  if ((int)data.throttle != prevThr) {
    tft.fillRect(80, 100, 80, 20, TFT_BLACK);
    tft.setCursor(80, 100);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(data.throttle);
    prevThr = data.throttle;
  }

  if ((int)data.yaw != prevYawRc) {
    tft.fillRect(80, 120, 80, 20, TFT_BLACK);
    tft.setCursor(80, 120);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(data.yaw);
    prevYawRc = data.yaw;
  }

  if ((int)data.pitch != prevPitchRc) {
    tft.fillRect(80, 140, 80, 20, TFT_BLACK);
    tft.setCursor(80, 140);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(data.pitch);
    prevPitchRc = data.pitch;
  }

  if ((int)data.roll != prevRollRc) {
    tft.fillRect(80, 160, 80, 20, TFT_BLACK);
    tft.setCursor(80, 160);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(data.roll);
    prevRollRc = data.roll;
  }

  if (data.arm != prevArm) {
    tft.fillRect(60, 185, 30, 20, TFT_BLACK);
    tft.setCursor(60, 185);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(data.arm);
    prevArm = data.arm;
  }

  // === Drone battery (existing) ===
  if (abs(telemetry.batteryVoltage - prevBat) >= 0.01f) {
    tft.fillRect(170, 185, 80, 20, TFT_BLACK);
    tft.setCursor(170, 185);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.print(telemetry.batteryVoltage, 2);
    tft.print("V");
    prevBat = telemetry.batteryVoltage;
  }

  // === NEW: Transmitter battery ===
  if (abs(txBatteryVoltage - prevTxBat) >= 0.01f) {
    tft.fillRect(80, 205, 80, 20, TFT_BLACK);     // adjust position if needed
    tft.setCursor(80, 205);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.print(txBatteryVoltage, 2);
    tft.print("V");
    prevTxBat = txBatteryVoltage;
  }

  if ((int)telemetry.failsafe != prevFailsafe) {
    tft.fillRect(40, 210, 30, 20, TFT_BLACK);
    tft.setCursor(40, 210);
    tft.setTextColor(telemetry.failsafe ? TFT_RED : TFT_GREEN, TFT_BLACK);
    tft.print(telemetry.failsafe);
    prevFailsafe = telemetry.failsafe;
  }

  if ((int)telemetry.armed != prevArmed) {
    tft.fillRect(190, 210, 40, 20, TFT_BLACK);
    tft.setCursor(190, 210);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(telemetry.armed);
    prevArmed = telemetry.armed;
  }
}
