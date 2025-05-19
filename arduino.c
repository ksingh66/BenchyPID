#include <Servo.h>

#define PAN_PIN 3
#define TILT_PIN 9

// PID parameters for each axis
float Kp[2] = {0.2, 0.2};  // [pan, tilt]
float Ki[2] = {0.0, 0.0};  // [pan, tilt]
float Kd[2] = {0.0, 0.0};  // [pan, tilt]

// PID state
float integral[2] = {0, 0};
float previous_error[2] = {0, 0};

// Servo objects
Servo panServo;
Servo tiltServo;

// Current servo angles - Centered
int panAngle = 90;
int tiltAngle = 90;

// Time tracking
unsigned long lastUpdate = 0;
unsigned long updateInterval = 50;  // 50 ms refresh rate- lower this to increase responsiveness

void setup() {
  Serial.begin(115200);
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("PT:")) {
      command.remove(0, 3);
      int commaIndex = command.indexOf(',');
      if (commaIndex > 0) {
        float errorPan = command.substring(0, commaIndex).toFloat();
        float errorTilt = command.substring(commaIndex + 1).toFloat();

        unsigned long now = millis();
        if (now - lastUpdate >= updateInterval) {
          applyPID(errorPan, errorTilt);
          lastUpdate = now;

          // Report current position back
          Serial.print("PT:");
          Serial.print(panAngle);
          Serial.print(",");
          Serial.println(tiltAngle);
        }
      }

    } else if (command.startsWith("TUNE:")) {
      int servoIndex = command.substring(5, 6).toInt();
      int firstComma = command.indexOf(',', 6);
      int secondComma = command.indexOf(',', firstComma + 1);
      if (servoIndex == 0 || servoIndex == 1) {
        Kp[servoIndex] = command.substring(firstComma + 1, secondComma).toFloat();
        Ki[servoIndex] = command.substring(secondComma + 1, command.lastIndexOf(',')).toFloat();
        Kd[servoIndex] = command.substring(command.lastIndexOf(',') + 1).toFloat();
      }

    } else if (command == "RESET") {
      resetServos();
    } else if (command == "CENTERED") {
      // dont do nothing if already centered
    }
  }
}

void applyPID(float errorPan, float errorTilt) {
  float dt = updateInterval / 1000.0;

  // PID for pan (index 0)
  float outputPan = computePID(0, errorPan, dt);
  panAngle = constrain(panAngle + outputPan, 0, 180);
  panServo.write(panAngle);

  // PID for tilt (index 1)
  float outputTilt = computePID(1, errorTilt, dt);
  tiltAngle = constrain(tiltAngle + outputTilt, 0, 180);
  tiltServo.write(tiltAngle);
}

float computePID(int axis, float error, float dt) {
  integral[axis] += error * dt;
  float derivative = (error - previous_error[axis]) / dt;
  float output = Kp[axis] * error + Ki[axis] * integral[axis] + Kd[axis] * derivative;
  previous_error[axis] = error;
  return output;
}

void resetServos() {
  panAngle = 90;
  tiltAngle = 90;
  integral[0] = integral[1] = 0;
  previous_error[0] = previous_error[1] = 0;
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
}
