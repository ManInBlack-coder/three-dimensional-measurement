/*
  ESP32 LiDAR scanner (TF-Luna + pan/tilt servos)
  -----------------------------------------------
  Rotates the LiDAR horizontally (pan) and vertically (tilt) and sends every
  measurement to the computer. All computation (3D points, merging, volume)
  happens on the computer in ruumala.py.

  Hardware
    - ESP32 dev board
    - TF-Luna LiDAR (UART mode, 115200 baud by default)
    - 2× servo (e.g. MG996R or DS3218, metal gear), pan-tilt bracket
    - SEPARATE 5 V supply for the servos (at least 2 A). Powering the servos
      from the ESP32 5V pin causes voltage drops and ESP32 resets.

  Wiring
    TF-Luna            ESP32
    pin1 5V      ->    5V (or from the external 5 V supply)
    pin2 RXD     ->    GPIO 17 (TX2)
    pin3 TXD     ->    GPIO 16 (RX2)
    pin4 GND     ->    GND
    (pin5 unconnected = UART mode)

    Pan servo signal   -> GPIO 13
    Tilt servo signal  -> GPIO 14
    Servo + and −      -> external 5 V supply; also connect the supply − to ESP32 GND!

  Mechanics
    - The LiDAR must point straight ahead when the tilt servo is at TILT_HORIZON degrees.
      Calibrate: send the command "TILT <angle>" and find the angle at which the
      LiDAR is level (spirit level). Put that value into ruumala.py (--tilt-horizon).
    - The scanner must stand level (base horizontal).

  Library: Arduino Library Manager -> "ESP32Servo" (Kevin Harrington).

  Protocol (115200 baud, lines end with '\n')
    Computer -> ESP32:
      PING                                  -> PONG
      HOME                                  -> servos to center
      PAN <angle> / TILT <angle>            -> move a servo (for calibration)
      READ                                  -> one measurement: R,<cm>,<strength>
      SCAN <panMin> <panMax> <panStep> <tiltMin> <tiltMax> <tiltStep>
    ESP32 -> computer while scanning:
      START,<number of points>
      P,<pan>,<tilt>,<distance_cm>,<strength>   (each point)
      DONE
*/

#include <ESP32Servo.h>

#define PAN_PIN   13
#define TILT_PIN  14
#define LIDAR_RX  16
#define LIDAR_TX  17

// Servo pulse widths (µs). If a servo doesn't turn the full 180°, adjust these.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Settle time after a move (ms): small step / new column
const int SETTLE_SMALL_MS = 35;
const int SETTLE_BIG_MS   = 400;
const int SAMPLES_PER_POINT = 3;   // number of readings averaged

Servo panServo, tiltServo;
HardwareSerial Lidar(2);

int curPan = 90, curTilt = 90;

// ---------- TF-Luna ----------
// Frame: 0x59 0x59 DistL DistH AmpL AmpH TempL TempH Checksum
bool readLunaFrame(uint16_t &dist, uint16_t &amp, uint32_t timeoutMs = 100) {
  uint8_t buf[9];
  uint32_t t0 = millis();
  int idx = 0;
  while (millis() - t0 < timeoutMs) {
    if (!Lidar.available()) continue;
    uint8_t b = Lidar.read();
    if (idx < 2) {
      if (b == 0x59) buf[idx++] = b; else idx = 0;
      continue;
    }
    buf[idx++] = b;
    if (idx == 9) {
      uint8_t sum = 0;
      for (int i = 0; i < 8; i++) sum += buf[i];
      if (sum == buf[8]) {
        dist = buf[2] | (buf[3] << 8);
        amp  = buf[4] | (buf[5] << 8);
        return true;
      }
      idx = 0;
    }
  }
  return false;
}

// Fresh measurement: flushes the buffer so a stale reading (taken while moving) doesn't interfere
bool measure(uint16_t &distCm, uint16_t &ampOut) {
  while (Lidar.available()) Lidar.read();
  uint32_t sumD = 0, sumA = 0;
  int n = 0;
  for (int i = 0; i < SAMPLES_PER_POINT + 1; i++) {
    uint16_t d, a;
    if (!readLunaFrame(d, a)) continue;
    if (i == 0) continue;                 // first frame may be partial/stale
    if (a < 100 || a == 65535) continue;  // too weak or saturated
    sumD += d; sumA += a; n++;
  }
  if (n == 0) { distCm = 0; ampOut = 0; return false; }
  distCm = sumD / n;
  ampOut = sumA / n;
  return true;
}

// ---------- Servos ----------
void moveTo(int pan, int tilt) {
  pan = constrain(pan, 0, 180);
  tilt = constrain(tilt, 0, 180);
  int jump = max(abs(pan - curPan), abs(tilt - curTilt));
  panServo.write(pan);
  tiltServo.write(tilt);
  curPan = pan; curTilt = tilt;
  delay(jump > 5 ? SETTLE_BIG_MS : SETTLE_SMALL_MS);
}

// ---------- Scanning ----------
void scan(int panMin, int panMax, int panStep, int tiltMin, int tiltMax, int tiltStep) {
  if (panStep < 1) panStep = 1;
  if (tiltStep < 1) tiltStep = 1;
  int cols = (panMax - panMin) / panStep + 1;
  int rows = (tiltMax - tiltMin) / tiltStep + 1;
  Serial.printf("START,%d\n", cols * rows);

  for (int c = 0; c < cols; c++) {
    int pan = panMin + c * panStep;
    for (int r = 0; r < rows; r++) {
      // "snake": odd columns run backwards (less servo travel), same tilt angles both ways
      int tilt = tiltMin + (c % 2 ? rows - 1 - r : r) * tiltStep;
      moveTo(pan, tilt);
      uint16_t d = 0, a = 0;
      measure(d, a);
      Serial.printf("P,%d,%d,%u,%u\n", pan, tilt, d, a);

      if (Serial.available() && Serial.peek() == 'X') {  // abort
        Serial.read();
        Serial.println("ABORTED");
        return;
      }
    }
  }
  Serial.println("DONE");
  moveTo(90, 90);
}

// ---------- Commands ----------
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  char cmd[8] = {0};
  int a[6] = {0};
  int n = sscanf(line.c_str(), "%7s %d %d %d %d %d %d", cmd, &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]);
  auto is = [&](const char *s) { return strcasecmp(cmd, s) == 0; };

  if (is("PING")) {
    Serial.println("PONG");
  } else if (is("HOME")) {
    moveTo(90, 90); Serial.println("OK");
  } else if (is("PAN") && n >= 2) {
    moveTo(a[0], curTilt); Serial.println("OK");
  } else if (is("TILT") && n >= 2) {
    moveTo(curPan, a[0]); Serial.println("OK");
  } else if (is("READ")) {
    uint16_t d, am;
    measure(d, am);
    Serial.printf("R,%u,%u\n", d, am);
  } else if (is("SCAN") && n >= 7) {
    scan(a[0], a[1], a[2], a[3], a[4], a[5]);
  } else {
    Serial.println("ERR tundmatu käsk");
  }
}

void setup() {
  Serial.begin(115200);
  Lidar.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_PIN, SERVO_MIN_US, SERVO_MAX_US);
  tiltServo.attach(TILT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  moveTo(90, 90);

  uint16_t d, am;
  if (readLunaFrame(d, am, 500)) Serial.println("READY lidar ok");
  else Serial.println("READY HOIATUS: TF-Lunast ei tule andmeid (kontrolli RX/TX ja toidet)");
}

void loop() {
  if (Serial.available()) handleLine(Serial.readStringUntil('\n'));  // trim() in handleLine drops '\r'
}
