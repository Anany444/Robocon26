#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_VL53L0X.h>

// ==========================================
// HARDWARE PIN DEFINITIONS (ARDUINO MEGA)
// ==========================================
// Omni X-Drive Motors (Mega PWM Pins: 2-13, 44-46)
#define LF_DIR 10
#define LF_PWM 5

#define RF_DIR 8
#define RF_PWM 9

#define RR_DIR 6
#define RR_PWM 7

#define LR_DIR 12
#define LR_PWM 13

// Arm Servos (Controlling single arm together)
#define SERVO1_PIN 99
#define SERVO2_PIN 11

// Weapon Servos
#define WEAPON_SERVO_PIN 33
#define WEAPON_GRIPPER_PIN 35

// Relays
#define RELAY_GRIPPER 22  // Arm Gripper Relay
#define RELAY_FRONT   23  // Front Pneumatic Actuator Relay
#define RELAY_BACK    24  // Back Pneumatic Actuator Relay

// I2C Sensors (VL53L0X LiDAR & BNO085 IMU share the hardware I2C bus)
#define I2C_SDA_PIN   20  // Arduino Mega Hardware SDA Pin
#define I2C_SCL_PIN   21  // Arduino Mega Hardware SCL Pin
#define VL53L0X_XSHUT 25  // Optional XSHUT pin to reset/enable VL53L0X (set HIGH to enable)

// ==========================================
// GLOBAL STATE VARIABLES
// ==========================================
Servo servoArm1;
Servo servoArm2;
Servo servoWeapon;
Servo servoWeaponGripper;

int currentAngle = 60; // Initial angle 60
int currentServoStepSpeed = 0;
unsigned long lastServoTick = 0;
bool overrideActive = false;
const int MAX_MOTOR_PWM = 170; // Default PWM for test motions
const int MIN_MOTOR_PWM = 50;  // Minimum PWM threshold to overcome static friction near goal


float factorLF = 1.0;
float factorRF = 1.0;
float factorRR = 1.0;
float factorLR = 1.0;

// --- BNO085 IMU Variables ---
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
bool imuReady = false;
unsigned long lastImuSendTime = 0;

float imu_r = 0, imu_p = 0, imu_y = 0;
float imu_ax = 0, imu_ay = 0, imu_az = 0;
float imu_gx = 0, imu_gy = 0, imu_gz = 0;

// --- VL53L0X 1D LiDAR Variables ---
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool loxReady = false;
unsigned long lastLoxSendTime = 0;

// ==========================================
// FUNCTION PROTOTYPES
// ==========================================
void move_base(float x, float y, float z);
void stopMotors();
void pollLidar();
void LF(bool dir, int pwm);
void RF(bool dir, int pwm);
void RR(bool dir, int pwm);
void LR(bool dir, int pwm);
void pollIMU();

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(LF_DIR, OUTPUT); pinMode(LF_PWM, OUTPUT);
  pinMode(RF_DIR, OUTPUT); pinMode(RF_PWM, OUTPUT);
  pinMode(RR_DIR, OUTPUT); pinMode(RR_PWM, OUTPUT);
  pinMode(LR_DIR, OUTPUT); pinMode(LR_PWM, OUTPUT);

  // Relay pins
  pinMode(RELAY_GRIPPER, OUTPUT);
  pinMode(RELAY_FRONT, OUTPUT);
  pinMode(RELAY_BACK, OUTPUT);

  // Active LOW relays OFF initially
  digitalWrite(RELAY_GRIPPER, HIGH);
  digitalWrite(RELAY_FRONT, HIGH);
  digitalWrite(RELAY_BACK, HIGH);

  // Attach servos
  servoArm1.attach(SERVO1_PIN);
  servoArm2.attach(SERVO2_PIN);
  servoWeapon.attach(WEAPON_SERVO_PIN);
  servoWeaponGripper.attach(WEAPON_GRIPPER_PIN);

  // Initialize arm servos to 60 degrees
  servoArm1.write(currentAngle);
  servoArm2.write(currentAngle);

  // Initialize weapon servos
  servoWeapon.write(180);
  servoWeaponGripper.write(0);

  stopMotors();

  // Give sensors time to power up and stabilize after board reset
  delay(500);

  // --- Initialize BNO085 over I2C ---
  Wire.begin();
  if (!bno08x.begin_I2C()) {
    Serial.println("INFO:BNO085 IMU not detected via I2C!");
  } else {
    Serial.println("INFO:BNO085 IMU detected successfully!");
    // Enable reports at 5000us (200Hz)
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
    imuReady = true;
  }

  // --- Initialize VL53L0X over I2C ---
  if (!lox.begin()) {
    Serial.println("INFO:VL53L0X 1D LiDAR not detected via I2C!");
  } else {
    Serial.println("INFO:VL53L0X 1D LiDAR detected successfully!");
    loxReady = true;
  }

  Serial.println("R2 ARDUINO MEGA JOYSTICK CONTROLLER READY");
}

void pollIMU() {
  if (!imuReady) return;

  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
  }

  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float r = sensorValue.un.gameRotationVector.real;
      float i = sensorValue.un.gameRotationVector.i;
      float j = sensorValue.un.gameRotationVector.j;
      float k = sensorValue.un.gameRotationVector.k;

      float sqr = r * r;
      float sqi = i * i;
      float sqj = j * j;
      float sqk = k * k;

      imu_r = atan2(2.0 * (j * k + i * r), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;
      imu_p = asin(-2.0 * (i * k - j * r)) * RAD_TO_DEG;
      imu_y = atan2(2.0 * (i * j + k * r), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
    }
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      imu_ax = sensorValue.un.linearAcceleration.x;
      imu_ay = sensorValue.un.linearAcceleration.y;
      imu_az = sensorValue.un.linearAcceleration.z;
    }
    else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      imu_gx = sensorValue.un.gyroscope.x;
      imu_gy = sensorValue.un.gyroscope.y;
      imu_gz = sensorValue.un.gyroscope.z;
    }
  }

  // Broadcast IMU packet at 200Hz (every 5ms)
  if (millis() - lastImuSendTime >= 5) {
    lastImuSendTime = millis();
    Serial.print("IMU:");
    Serial.print(imu_r, 2); Serial.print(",");
    Serial.print(imu_p, 2); Serial.print(",");
    Serial.print(imu_y, 2); Serial.print(",");
    Serial.print(imu_ax, 2); Serial.print(",");
    Serial.print(imu_ay, 2); Serial.print(",");
    Serial.print(imu_az, 2); Serial.print(",");
    Serial.print(imu_gx, 3); Serial.print(",");
    Serial.print(imu_gy, 3); Serial.print(",");
    Serial.println(imu_gz, 3);
  }
}

void pollLidar() {
  if (!loxReady) return;
  if (millis() - lastLoxSendTime >= 15) { // Max ~66Hz
    lastLoxSendTime = millis();
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // false = no debug printing
    if (measure.RangeStatus != 4) { // Phase failure / out of range check
      float dist_m = measure.RangeMilliMeter / 1000.0;
      Serial.print("DIST:");
      Serial.println(dist_m, 3);
    }
  }
}

void loop() {
  // Poll sensors continuously in loop
  pollIMU();
  pollLidar();

  // --- Non-blocking Servo Sweep ---
  if (currentServoStepSpeed != 0 && millis() - lastServoTick >= 20) {
    lastServoTick = millis();
    currentAngle = constrain(currentAngle + currentServoStepSpeed, 40, 120);
    servoArm1.write(currentAngle);
    servoArm2.write(currentAngle);
  }

  // --- Serial Command Parsing ---
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() == 0) return;

    if (command == "OVERRIDE_ON") {
      overrideActive = true;
      currentServoStepSpeed = 0;
      stopMotors();
      Serial.println("ACK:OVERRIDE_ON");
      return;
    } else if (command == "OVERRIDE_OFF") {
      overrideActive = false;
      Serial.println("ACK:OVERRIDE_OFF");
      return;
    }

    if (overrideActive) return; // Block all other commands if override is active

    // --- Parse Variable Commands ---
    if (command.startsWith("CMD_VEL:")) {
      if (currentServoStepSpeed != 0) return; // Lockout drive during servo motion

      String valStr = command.substring(8);
      int c1 = valStr.indexOf(',');
      int c2 = valStr.indexOf(',', c1 + 1);

      float lx = valStr.substring(0, c1).toFloat();
      float ly = valStr.substring(c1 + 1, c2).toFloat();
      float az = valStr.substring(c2 + 1).toFloat();

      move_base(lx, ly, az);
    }
    else if (command.startsWith("PWM_FACTORS:")) {
      String valStr = command.substring(12);
      int c1 = valStr.indexOf(',');
      int c2 = valStr.indexOf(',', c1 + 1);
      int c3 = valStr.indexOf(',', c2 + 1);

      if (c1 > 0 && c2 > 0 && c3 > 0) {
        factorLF = constrain(valStr.substring(0, c1).toFloat(), 0.0, 1.0);
        factorRF = constrain(valStr.substring(c1 + 1, c2).toFloat(), 0.0, 1.0);
        factorRR = constrain(valStr.substring(c2 + 1, c3).toFloat(), 0.0, 1.0);
        factorLR = constrain(valStr.substring(c3 + 1).toFloat(), 0.0, 1.0);
      }
    }
    else if (command.startsWith("SERVO_ARM_STEP:")) {
      currentServoStepSpeed = command.substring(15).toInt();
    }
    // --- Parse Discrete Relays ---
    else if (command == "GRIPPER_ON") {
      digitalWrite(RELAY_GRIPPER, LOW);
    } else if (command == "GRIPPER_OFF") {
      digitalWrite(RELAY_GRIPPER, HIGH);
    }
    else if (command == "FRONT_ON") {
      digitalWrite(RELAY_FRONT, LOW);
    } else if (command == "FRONT_OFF") {
      digitalWrite(RELAY_FRONT, HIGH);
    }
    else if (command == "BACK_ON") {
      digitalWrite(RELAY_BACK, LOW);
    } else if (command == "BACK_OFF") {
      digitalWrite(RELAY_BACK, HIGH);
    }
    // --- Parse Weapon Servos ---
    else if (command == "CLOSE_WEAPON_GRIPPER") {
      servoWeaponGripper.write(100);
    } else if (command == "OPEN_WEAPON_GRIPPER") {
      servoWeaponGripper.write(0);
    }
    else if (command == "WEAPON_SERVO_ROTATE" || command == "WEAPON_SERVO_90") {
      servoWeapon.write(90);
    } else if (command == "WEAPON_SERVO_REST" || command == "WEAPON_SERVO_180") {
      servoWeapon.write(180);
    } else if (command.startsWith("WEAPON_SERVO:")) {
      int angle = constrain(command.substring(13).toInt(), 0, 180);
      servoWeapon.write(angle);
    }
    else {
      Serial.print("Unknown Command: ");
      Serial.println(command);
    }
  }
}

// ==========================================
// MOTOR HELPERS (analogWrite 0-255)
// ==========================================
void LF(bool dir, int pwm) {
  digitalWrite(LF_DIR, dir);
  analogWrite(LF_PWM, pwm);
}

void RF(bool dir, int pwm) {
  digitalWrite(RF_DIR, dir);
  analogWrite(RF_PWM, pwm);
}

void RR(bool dir, int pwm) {
  digitalWrite(RR_DIR, dir);
  analogWrite(RR_PWM, pwm);
}

void LR(bool dir, int pwm) {
  digitalWrite(LR_DIR, dir);
  analogWrite(LR_PWM, pwm);
}

// ==========================================
// OMNI X-DRIVE KINEMATICS
// ==========================================
void move_base(float x, float y, float z) {
  
  if (abs(x) < 0.05 && abs(y) < 0.05 && abs(z) < 0.03) {
    stopMotors();
    return;
  }

  // 4-Wheel Omni 'X' Configuration Kinematics
  float lf =  x - y + z;
  float rf =  x + y - z;
  float rr =  x - y - z;
  float lr =  x + y + z;

  float max_val = max(max(abs(lf), abs(rf)), max(abs(rr), abs(lr)));

  if (max_val > 1.0) {
      lf /= max_val;
      rf /= max_val;
      rr /= max_val;
      lr /= max_val;
  }

  int pwmLF = wheelToPWM(lf, factorLF);
  int pwmRF = wheelToPWM(rf, factorRF);
  int pwmRR = wheelToPWM(rr, factorRR);
  int pwmLR = wheelToPWM(lr, factorLR);

  LF(lf >= 0, pwmLF);
  RF(rf >= 0, pwmRF);
  RR(rr >= 0, pwmRR);
  LR(lr >= 0, pwmLR);

}
  // Convert normalized wheel command [0,1] to PWM with deadband compensation
int wheelToPWM(float wheel, float factor)
{
    float mag = abs(wheel);

    if (mag < 0.01f)        // Stop completely
        return 0;

    // Linear mapping:
    // 0.01 -> MIN_MOTOR_PWM
    // 1.00 -> MAX_MOTOR_PWM
    float pwm = MIN_MOTOR_PWM +
                (mag - 0.01f) *
                (MAX_MOTOR_PWM - MIN_MOTOR_PWM) /
                (1.0f - 0.01f);

    pwm *= factor;

    return constrain((int)pwm, 0, MAX_MOTOR_PWM);
}


void stopMotors() {
  analogWrite(LF_PWM, 0);
  analogWrite(RF_PWM, 0);
  analogWrite(RR_PWM, 0);
  analogWrite(LR_PWM, 0);
}
