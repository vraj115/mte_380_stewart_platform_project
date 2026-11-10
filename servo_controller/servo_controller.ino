// THIS IS THE MAIN CODE

// Three-servo Stewart platform controller for Adafruit PCA9685 (I2C address 0x40).
// Servo 1 -> shield channel 0; Servo 2 -> channel 3; Servo 3 -> channel 7.
// USB serial (115200 baud): "angle1,angle2,angle3\n" in degrees.
//   Angles may be whole numbers ("30,41,45") or decimals ("30.4,41.2,44.8").
// Each servo has its own calibrated min/neutral/max from stewart_config.json.
// SAFETY: no automatic servo-position command during Arduino startup.
//
// Changes from the previous version:
//   * 115200 baud instead of 9600. At 9600 the reply to every command took
//     ~26 ms on the wire, which caps the control rate and adds loop delay.
//   * Fractional angles. The PCA9685 at 50 Hz resolves 0.4 deg per tick, so
//     whole-degree commands were throwing away more than half the available
//     resolution. Whole-degree commands map to exactly the same pulse as
//     before, so the existing neutral calibration is unchanged.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdlib.h>
#include <math.h>

const uint8_t SHIELD_ADDR = 0x40;
Adafruit_PWMServoDriver pwm(SHIELD_ADDR);

const uint8_t CHANNELS[3] = {0, 3, 7};
// Keep these identical to min_angles_deg / max_angles_deg in stewart_config.json.
const float MIN_ANGLES[3] = {10.0, 15.0, 21.0};
const float NEUTRAL_ANGLES[3] = {30.0, 41.0, 45.0};  // Reference only, no auto-move
const float MAX_ANGLES[3] = {40.0, 50.0, 59.0};

// Values verified in the user's single-servo test. Calibrate each servo as needed.
const uint16_t SERVOMIN = 150;
const uint16_t SERVOMAX = 600;
const uint16_t SERVO_FREQ = 50;

char commandBuffer[40];
uint8_t commandLength = 0;
bool commandTooLong = false;

void setServoAngle(uint8_t channel, float angle) {
  // floor() reproduces the old integer map() for whole-degree angles.
  float ticks = SERVOMIN + angle * (float)(SERVOMAX - SERVOMIN) / 180.0;
  uint16_t pulse = (uint16_t)floor(ticks + 0.001);
  pwm.setPWM(channel, 0, pulse);
}

bool parseAngles(const char* text, float angles[3]) {
  const char* cursor = text;
  for (uint8_t i = 0; i < 3; i++) {
    while (*cursor == ' ' || *cursor == '\t') cursor++;
    char* end = NULL;
    double angle = strtod(cursor, &end);
    if (end == cursor || isnan(angle) ||
        angle < MIN_ANGLES[i] || angle > MAX_ANGLES[i]) {
      return false;
    }
    while (*end == ' ' || *end == '\t') end++;
    angles[i] = (float)angle;
    if (i < 2) {
      if (*end != ',') return false;
      cursor = end + 1;
    } else if (*end != '\0') {
      return false;
    }
  }
  return true;
}

void applyCommand(const char* command) {
  float angles[3];
  if (!parseAngles(command, angles)) {
    Serial.println("ERR: out-of-range/invalid command; expected 3 angles: ch0 10-40, ch3 15-50, ch7 21-59");
    return;
  }

  for (uint8_t i = 0; i < 3; i++) {
    setServoAngle(CHANNELS[i], angles[i]);
  }
  Serial.print("Angles set: ");
  Serial.print(angles[0], 1);
  Serial.print(", ");
  Serial.print(angles[1], 1);
  Serial.print(", ");
  Serial.println(angles[2], 1);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(3000, true);  // Arduino Mega

  Wire.beginTransmission(SHIELD_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERR: PCA9685 shield not found at 0x40");
    while (true) delay(1000);
  }

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("READY: PCA9685 channels 0,3,7; awaiting angle triplet (115200 baud, decimals ok)");
  Serial.println("HOME command: 30,41,45");
  // No position command here, to avoid unexpected motion on reset/USB connect.
}

void loop() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();
    if (incoming == '\r') continue;

    if (incoming == '\n') {
      if (commandTooLong) {
        Serial.println("ERR: command too long");
      } else if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        applyCommand(commandBuffer);
      }
      commandLength = 0;
      commandTooLong = false;
    } else if (!commandTooLong) {
      if (commandLength < sizeof(commandBuffer) - 1) {
        commandBuffer[commandLength++] = incoming;
      } else {
        commandTooLong = true;
      }
    }
  }
}
