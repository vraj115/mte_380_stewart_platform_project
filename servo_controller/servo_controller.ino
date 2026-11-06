// THIS IS THE MAIN CODE

// Three-servo Stewart platform controller for Adafruit PCA9685 (I2C address 0x40).
// Servo 1 -> shield channel 0; Servo 2 -> channel 3; Servo 3 -> channel 7.
// USB serial (9600 baud): "angle1,angle2,angle3\n" (integers in degrees).
// Each servo has its own calibrated min/neutral/max from stewart_config.json.
// SAFETY: no automatic servo-position command during Arduino startup.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdlib.h>

const uint8_t SHIELD_ADDR = 0x40;
Adafruit_PWMServoDriver pwm(SHIELD_ADDR);

const uint8_t CHANNELS[3] = {0, 3, 7};
const int MIN_ANGLES[3] = {10, 15, 21};
const int NEUTRAL_ANGLES[3] = {30, 41, 45};  // Reference only, no auto-move
const int MAX_ANGLES[3] = {40, 50, 59};

// Values verified in the user's single-servo test. Calibrate each servo as needed.
const uint16_t SERVOMIN = 150;
const uint16_t SERVOMAX = 600;
const uint16_t SERVO_FREQ = 50;

char commandBuffer[32];
uint8_t commandLength = 0;
bool commandTooLong = false;

void setServoAngle(uint8_t channel, int angle) {
  uint16_t pulse = (uint16_t)map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

bool parseAngles(const char* text, int angles[3]) {
  const char* cursor = text;
  for (uint8_t i = 0; i < 3; i++) {
    while (*cursor == ' ' || *cursor == '\t') cursor++;
    char* end = NULL;
    long angle = strtol(cursor, &end, 10);
    if (end == cursor || angle < MIN_ANGLES[i] || angle > MAX_ANGLES[i]) {
      return false;
    }
    while (*end == ' ' || *end == '\t') end++;
    angles[i] = (int)angle;
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
  int angles[3];
  if (!parseAngles(command, angles)) {
    Serial.println("ERR: out-of-range/invalid command; expected 3 integer angles: ch0 10-40, ch3 15-50, ch7 21-59");
    return;
  }

  for (uint8_t i = 0; i < 3; i++) {
    setServoAngle(CHANNELS[i], angles[i]);
  }
  Serial.print("Angles set: ");
  Serial.print(angles[0]);
  Serial.print(", ");
  Serial.print(angles[1]);
  Serial.print(", ");
  Serial.println(angles[2]);
}

void setup() {
  Serial.begin(9600);
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

  Serial.println("READY: PCA9685 channels 0,3,7; awaiting angle triplet");
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
