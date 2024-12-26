#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Arm segment lengths (in cm)
const float L1 = 4.355;  // Length of the first arm segment
const float L2 = 13.4;  // Length of the second arm segment
const float L3 = 12.5;  // Length of the third arm segment

// Servo range constants
#define BASE_SERVO_MIN 150
#define BASE_SERVO_MAX 600
#define SHOULDER_SERVO_MIN 150
#define SHOULDER_SERVO_MAX 600
#define ELBOW_SERVO_MIN 150
#define ELBOW_SERVO_MAX 600

void setup() {
  // Start serial communication
  Serial.begin(9600);
  Serial.println("3-DOF Robotic Arm Control");

  // Initialize PCA9685 driver
  pwm.begin();
  pwm.setPWMFreq(60);  // Standard 60 Hz for servos
  delay(10);
}

void loop() {
  if (Serial.available() > 0) {
    char mode = Serial.read();  // Read mode (F for Forward, I for Inverse)

    if (mode == 'F') {
      // Forward Kinematics Mode
      Serial.println("Enter Theta1 (Base servo), Theta2 (Shoulder servo), and Theta3 (Elbow servo), separated by space:");
      while (Serial.available() < 3)
        ;
      float theta1 = Serial.parseFloat();  // Base servo angle (0 to 90 degrees)
      float theta2 = Serial.parseFloat();  // Shoulder servo angle (0 to 180 degrees)
      float theta3 = Serial.parseFloat();  // Elbow servo angle (0 to 180 degrees)

      // Convert to radians
      theta1 = radians(theta1);
      theta2 = radians(theta2);
      theta3 = radians(theta3);

      // Calculate end effector position (X, Y, Z)
      float x = L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3);
      float y = L1 * sin(theta1) + L2 * sin(theta1 + theta2) + L3 * sin(theta1 + theta2 + theta3);
      float z = L3 * sin(theta3); // Assuming some vertical component contributed by L3

      // Display calculated position
      Serial.print("End Effector Position: X = ");
      Serial.print(x);
      Serial.print(", Y = ");
      Serial.print(y);
      Serial.print(", Z = ");
      Serial.println(z);

      // Map the angles to PWM values and set PWM for servos
      int basePWM = map(theta1 * 180.0 / PI, 0, 90, BASE_SERVO_MIN, BASE_SERVO_MAX);
      int shoulderPWM = map(theta2 * 180.0 / PI, 0, 180, SHOULDER_SERVO_MIN, SHOULDER_SERVO_MAX);
      int elbowPWM = map(theta3 * 180.0 / PI, 0, 180, ELBOW_SERVO_MIN, ELBOW_SERVO_MAX);

      pwm.setPWM(0, 0, basePWM);      // Move base servo
      pwm.setPWM(1, 0, shoulderPWM);  // Move shoulder servo
      pwm.setPWM(2, 0, elbowPWM);     // Move elbow servo

    } else if (mode == 'I') {
      // Inverse Kinematics Mode
      Serial.println("Enter X, Y, and Z (in cm), separated by space:");
      while (Serial.available() < 3)
        ;
      float x = Serial.parseFloat();  // X position
      float y = Serial.parseFloat();  // Y position
      float z = Serial.parseFloat();  // Z position

      // Inverse Kinematics for 3-DOF
      float r = sqrt(x * x + y * y);  // Radial distance
      float d = sqrt(r * r + z * z);  // Total distance from origin
      if (d > (L1 + L2 + L3)) {
        Serial.println("Target out of reach!");
      } else {
        // Base angle
        float theta1 = atan2(y, x);

        // Law of cosines for shoulder and elbow
        float cosTheta2 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        float theta2 = acos(cosTheta2); // Shoulder angle
        float theta3 = atan2(z, r);     // Elbow angle

        // Convert to degrees
        theta1 = degrees(theta1);
        theta2 = degrees(theta2);
        theta3 = degrees(theta3);

        // Display joint angles
        Serial.print("Theta1 = ");
        Serial.print(theta1);
        Serial.print(", Theta2 = ");
        Serial.print(theta2);
        Serial.print(", Theta3 = ");
        Serial.println(theta3);

        // Map the angles to PWM values and set PWM for servos
        int basePWM = map(theta1, 0, 90, BASE_SERVO_MIN, BASE_SERVO_MAX);
        int shoulderPWM = map(theta2, 0, 180, SHOULDER_SERVO_MIN, SHOULDER_SERVO_MAX);
        int elbowPWM = map(theta3, 0, 180, ELBOW_SERVO_MIN, ELBOW_SERVO_MAX);

        pwm.setPWM(0, 0, basePWM);      // Move base servo
        pwm.setPWM(1, 0, shoulderPWM);  // Move shoulder servo
        pwm.setPWM(2, 0, elbowPWM);     // Move elbow servo
      }
    }
  }
}