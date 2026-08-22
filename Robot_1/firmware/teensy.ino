#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

// ==========================================
// CONFIGURATION PARAMETERS
// ==========================================
// Base Motors
const int MAX_MOTOR_PWM = 200; // Maximum PWM value for Omni wheels (0-255)
const int MIN_MOTOR_PWM = 20;  // Minimum PWM to overcome static friction

// Stepper Motors
const float STEPPER_MAX_SPEED = 2000.0;
const float STEPPER_ACCEL = 1000.0;


// ==========================================
// HARDWARE PIN DEFINITIONS
// ==========================================
// Omni-Wheel Base Motors (Requires PWM capable pins for motorX_pwm)
const int motorF_pwm = 3; const int motorF_dir = 2; // Front Motor (X-axis movement)
const int motorB_pwm = 7; const int motorB_dir = 6; // Back Motor (X-axis movement)
const int motorL_pwm = 9; const int motorL_dir = 8; // Left Motor (Y-axis movement)
const int motorR_pwm = 5  ; const int motorR_dir = 4; // Right Motor (Y-axis movement)

// Stepper Motors
const int ARM_STEP_PIN1 = 11; const int ARM_DIR_PIN1 = 12; // Arm Leadscrew 1
const int ARM_STEP_PIN2 = 25; const int ARM_DIR_PIN2 = 24; // Arm Leadscrew 2
const int WEP_STEP_PIN  = 16; const int WEP_DIR_PIN  = 15; // Weapon Stepper

// Servo Motors
const int SERVO_ARM_PIN1 = 22;
const int SERVO_ARM_PIN2 = 33;
const int WEAPON_SERVO_PIN = 18;
const int WEAPON_GRIPPER_PIN = 19;

// Relays / Solenoids
const int ARM_GRIPPER_RELAY_PIN = 27; // Teensy digital pin 


// ==========================================
// GLOBAL STATE VARIABLES
// ==========================================
// Stepper Positions
int arm_stepper_pos = 0;
int weapon_stepper_pos = 0;

// Servo Angles
int servo_arm_angle = 110;
int weapon_servo_angle = 0;
int weapon_gripper_angle = 0;

// Relay States
bool gripper_state = false; // False = Open, True = Closed


// ==========================================
// OBJECT INSTANTIATION
// ==========================================
AccelStepper armStepper1(AccelStepper::DRIVER, ARM_STEP_PIN1, ARM_DIR_PIN1);
AccelStepper armStepper2(AccelStepper::DRIVER, ARM_STEP_PIN2, ARM_DIR_PIN2);
AccelStepper weaponStepper(AccelStepper::DRIVER, WEP_STEP_PIN, WEP_DIR_PIN);

Servo servoArm1;
Servo servoArm2;
Servo weaponServo;
Servo weaponGripperServo;


// ==========================================
// FUNCTION PROTOTYPES
// ==========================================
void moveServosStepLoop(int stepCmd);
void move_base(float x, float y, float z);
void stopMotors();


// ==========================================
// SETUP ROUTINE
// ==========================================
void setup() {
  // Initialize serial communication at 115200 (must match ROS serial node)
  Serial.begin(115200);
  Serial1.begin(115200); // Hardware UART RX/TX
  
  // Wait for serial to initialize (Teensy native USB needs this)
  unsigned long start = millis();
  while (!Serial && millis() - start < 1000);
  
  Serial.println("Teensy Robot Controller Initialized.");
  Serial.println("Waiting for ROS commands on Serial1 (RX/TX)...");

  // --- Initialize Base Motors ---
  pinMode(motorF_pwm, OUTPUT); pinMode(motorF_dir, OUTPUT);
  pinMode(motorB_pwm, OUTPUT); pinMode(motorB_dir, OUTPUT);
  pinMode(motorL_pwm, OUTPUT); pinMode(motorL_dir, OUTPUT);
  pinMode(motorR_pwm, OUTPUT); pinMode(motorR_dir, OUTPUT);

  // --- Initialize Relays ---
  pinMode(ARM_GRIPPER_RELAY_PIN, OUTPUT);
  digitalWrite(ARM_GRIPPER_RELAY_PIN, LOW); // Start with gripper open

  // --- Initialize Servos ---
  servoArm1.attach(SERVO_ARM_PIN1);
  servoArm2.attach(SERVO_ARM_PIN2);
  weaponServo.attach(WEAPON_SERVO_PIN);
  weaponGripperServo.attach(WEAPON_GRIPPER_PIN);
  
  // Set Safe Initial Angles for weapon
  weaponServo.write(weapon_servo_angle);
  weaponGripperServo.write(weapon_gripper_angle);

  // --- Initialize Steppers ---
  armStepper1.setMaxSpeed(STEPPER_MAX_SPEED);
  armStepper1.setAcceleration(STEPPER_ACCEL);
  
  armStepper2.setMaxSpeed(STEPPER_MAX_SPEED);
  armStepper2.setAcceleration(STEPPER_ACCEL);
  
  weaponStepper.setMaxSpeed(STEPPER_MAX_SPEED);
  weaponStepper.setAcceleration(STEPPER_ACCEL);
}


// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  // Check if data is available on the Serial1 port (RX/TX)
  if (Serial1.available() > 0) {
    // Read the string until newline '\n'
    String command = Serial1.readStringUntil('\n');
    command.trim(); // Remove any \r or extra whitespace

    if (command.length() == 0) return;

    // --------------------------------------------------------
    // Parse Variable Data Commands (Values sent from ROS)
    // --------------------------------------------------------
    if (command.startsWith("ARM_STEP:")) {
      String valueStr = command.substring(9);
      arm_stepper_pos = valueStr.toInt();
      if (arm_stepper_pos == 0) {
        armStepper1.stop();
        armStepper2.stop();
      } else {
        // .move() adds relative steps to the current target position
        armStepper1.move(arm_stepper_pos);
        armStepper2.move(arm_stepper_pos); // Command both identical motors
      }
    }  
    else if (command.startsWith("WEAPON_STEP:")) {
      String valueStr = command.substring(12);
      weapon_stepper_pos = valueStr.toInt();
      
      // .move() adds relative steps to the current target position
      weaponStepper.move(weapon_stepper_pos);
    }
    else if (command.startsWith("SERVO_ARM_STEP:")) {
      int stepDir = command.substring(15).toInt();
      if (stepDir != 0) {
        stopMotors();
        moveServosStepLoop(stepDir);
      }
    } 
    else if (command.startsWith("WEAPON_SERVO_ANGLE:")) {
      String valueStr = command.substring(19);
      weapon_servo_angle = valueStr.toInt();
      weaponServo.write(weapon_servo_angle);
    } 
    else if (command.startsWith("WEAPON_GRIPPER_ANGLE:")) {
      String valueStr = command.substring(21);
      weapon_gripper_angle = valueStr.toInt();
      weaponGripperServo.write(weapon_gripper_angle);
    } 
    else if (command.startsWith("CMD_VEL:")) {
      String valueStr = command.substring(8);
      int firstComma = valueStr.indexOf(',');
      int secondComma = valueStr.indexOf(',', firstComma + 1);
      
      float linear_x = valueStr.substring(0, firstComma).toFloat();
      float linear_y = valueStr.substring(firstComma + 1, secondComma).toFloat();
      float angular_z = valueStr.substring(secondComma + 1).toFloat();
      
      // Call our new kinematics function
      move_base(linear_x, linear_y, angular_z);
    }

    // --------------------------------------------------------
    // Parse Discrete/Toggle Commands
    // --------------------------------------------------------
    else if (command == "WEAPON_CW") {
      weapon_servo_angle -= 2; // Adjust this value to change rotation speed
      if (weapon_servo_angle < 0) weapon_servo_angle = 0;
      weaponServo.write(weapon_servo_angle);
    } 
    else if (command == "WEAPON_CCW") {
      weapon_servo_angle += 2; 
      if (weapon_servo_angle > 180) weapon_servo_angle = 180;
      weaponServo.write(weapon_servo_angle);
    } 
    else if (command == "GRIPPER_ON") {
      gripper_state = true;
      digitalWrite(ARM_GRIPPER_RELAY_PIN, HIGH);
    } 
    else if (command == "GRIPPER_OFF") {
      gripper_state = false;
      digitalWrite(ARM_GRIPPER_RELAY_PIN, LOW);
    } 
    else if (command == "WEAPON_GRIPPER_CLOSED") {
      // Toggle sets it to 100 degrees
      weapon_gripper_angle = 100;
      weaponGripperServo.write(weapon_gripper_angle);
    } 
    else if (command == "WEAPON_GRIPPER_OPEN") {
      // Toggle sets it to 0 degrees
      weapon_gripper_angle = 0;
      weaponGripperServo.write(weapon_gripper_angle);
    } 
    else {
      // If we receive something unexpected, we can echo it or ignore it.
      Serial.print("Unknown Command: ");
      Serial.println(command);
    }
  }
  
  // These MUST be called as fast as possible in the loop.
  // They will output a step pulse only if a step is due.
  armStepper1.run();
  armStepper2.run();
  weaponStepper.run();
}


// ==========================================
// HELPER FUNCTIONS
// ==========================================

// --- Servo Control ---
void moveServosStepLoop(int stepSpeed) {
  unsigned long lastServoTick = millis();

  while (stepSpeed != 0) {
    if (Serial1.available() > 0) {
      String cmd = Serial1.readStringUntil('\n');
      cmd.trim();
      if (cmd.startsWith("SERVO_ARM_STEP:")) {
        stepSpeed = cmd.substring(15).toInt();
      }
    }

    if (millis() - lastServoTick >= 20) {
      lastServoTick = millis();
      if (stepSpeed != 0) {
        servo_arm_angle = constrain(servo_arm_angle + stepSpeed, 0, 150);
        servoArm1.write(servo_arm_angle);
        servoArm2.write(servo_arm_angle);
      }
    }

  
  }
}

// --- Kinematics & Motor Control ---
void move_base(float x, float y, float z) {

  if (abs(x) < 0.02 && abs(y) < 0.02 && abs(z) < 0.02) {
    stopMotors();
    return;
  } 


  // 4-Wheel Omni '+' Configuration Kinematics
  // Assuming X is Forward/Backward, Y is Right/Left, Z is Rotation
  // Note: Depending on your specific wheel orientation (facing in vs facing out), 
  // you may need to flip the signs (+ to -) for specific wheels.
  float f =  y + z; // Front wheel moves robot left/right and rotates
  float b = -y + z; // Back wheel moves robot left/right and rotates
  float l =  x - z; // Left wheel moves robot forward/back and rotates
  float r = -x - z; // Right wheel moves robot forward/back and rotates

  // Find the maximum value to normalize speeds if they exceed 1.0
  float max_val = max(max(abs(f), abs(b)), max(abs(l), abs(r)));
  if (max_val > 1.0) {
    f /= max_val;
    b /= max_val;
    l /= max_val;
    r /= max_val;
  }

  // Convert to PWM and apply minimum threshold to overcome deadband
  int pwmF = abs(f) * MAX_MOTOR_PWM;
  int pwmB = abs(b) * MAX_MOTOR_PWM;
  int pwmL = abs(l) * MAX_MOTOR_PWM;
  int pwmR = abs(r) * MAX_MOTOR_PWM;

  // If a motor is commanded to move, ensure it receives at least MIN_MOTOR_PWM
  if (pwmF > 0 && pwmF < MIN_MOTOR_PWM) pwmF = MIN_MOTOR_PWM;
  if (pwmB > 0 && pwmB < MIN_MOTOR_PWM) pwmB = MIN_MOTOR_PWM;
  if (pwmL > 0 && pwmL < MIN_MOTOR_PWM) pwmL = MIN_MOTOR_PWM;
  if (pwmR > 0 && pwmR < MIN_MOTOR_PWM) pwmR = MIN_MOTOR_PWM;

  // Set Motor Directions (HIGH for forward, LOW for reverse)
  // You may need to invert HIGH/LOW depending on your motor wiring
  digitalWrite(motorF_dir, f >= 0 ? HIGH : LOW);
  digitalWrite(motorB_dir, b >= 0 ? HIGH : LOW);
  digitalWrite(motorL_dir, l >= 0 ? HIGH : LOW);
  digitalWrite(motorR_dir, r >= 0 ? HIGH : LOW);

  // Write PWM to motors
  analogWrite(motorF_pwm, pwmF);
  analogWrite(motorB_pwm, pwmB);
  analogWrite(motorL_pwm, pwmL);
  analogWrite(motorR_pwm, pwmR);
}

void stopMotors() {
  analogWrite(motorF_pwm, 0);
  analogWrite(motorB_pwm, 0);
  analogWrite(motorL_pwm, 0);
  analogWrite(motorR_pwm, 0);
}
