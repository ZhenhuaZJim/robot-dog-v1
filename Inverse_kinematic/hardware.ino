#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ----------------------Individual Servo Configuration----------------
// Back left leg
#define Servo0Min 610
#define Servo0Max 2580
#define Servo1Min 580
#define Servo1Max 2470
#define Servo2Min 650
#define Servo2Max 2560
// Back right leg
#define Servo3Min 620
#define Servo3Max 2590
#define Servo4Min 550
#define Servo4Max 2500
#define Servo5Min 540
#define Servo5Max 2480
// Front left leg
#define Servo6Min 610
#define Servo6Max 2510
#define Servo7Min 550
#define Servo7Max 2440
#define Servo8Min 650
#define Servo8Max 2590
// Front Right leg
#define Servo9Min 610
#define Servo9Max 2510
#define Servo10Min 530
#define Servo10Max 2420
#define Servo11Min 660
#define Servo11Max 2600

#define SERVOFREQ 50
#define DEFAULT_FREQUENCY 27000000

void initServos(){
  pwm.begin();
  pwm.setOscillatorFrequency(DEFAULT_FREQUENCY);
  pwm.setPWMFreq(SERVOFREQ);
}

// Back left leg
void set_joint_1(int theta) {
  uint16_t off = map(theta, -90, 90, Servo2Min, Servo2Max);
  pwm.writeMicroseconds(2, off);
}

void set_joint_2(int theta) {
  uint16_t off = map(theta, 90, -90, Servo1Min, Servo1Max);
  pwm.writeMicroseconds(1, off);
}

void set_joint_3(int theta) {
  uint16_t off = map(theta, 90, -90, Servo0Min, Servo0Max);
  pwm.writeMicroseconds(0, off);
}

// Back right leg
void set_joint_4(int theta) {
  uint16_t off = map(theta, -90, 90, Servo5Min, Servo5Max);
  pwm.writeMicroseconds(5, off);
}

void set_joint_5(int theta) {
  uint16_t off = map(theta, -90, 90, Servo4Min, Servo4Max);
  pwm.writeMicroseconds(4, off);
}

void set_joint_6(int theta) {
  uint16_t off = map(theta, -90, 90, Servo3Min, Servo3Max);
  pwm.writeMicroseconds(3, off);
}

// Front left leg
void set_joint_7(int theta) {
  uint16_t off = map(theta, 90, -90, Servo8Min, Servo8Max);
  pwm.writeMicroseconds(8, off);
}

void set_joint_8(int theta) {
  uint16_t off = map(theta, 90, -90, Servo7Min, Servo7Max);
  pwm.writeMicroseconds(7, off);
}

void set_joint_9(int theta) {
  uint16_t off = map(theta, 90, -90, Servo6Min, Servo6Max);
  pwm.writeMicroseconds(6, off);
}

// Front right leg
void set_joint_10(int theta) {
  uint16_t off = map(theta, 90, -90, Servo11Min, Servo11Max);
  pwm.writeMicroseconds(11, off);
}

void set_joint_11(int theta) {
  uint16_t off = map(theta, -90, 90, Servo10Min, Servo10Max);
  pwm.writeMicroseconds(10, off);
}

void set_joint_12(int theta) {
  uint16_t off = map(theta, -90, 90, Servo9Min, Servo9Max);
  pwm.writeMicroseconds(9, off);
}

void set_joint_array_bl(int *jointAngle) {
  set_joint_1(jointAngle[0]);
  set_joint_2(jointAngle[1]);
  set_joint_3(jointAngle[2]);
}

void set_joint_array_br(int *jointAngle) {
  set_joint_4(jointAngle[0]);
  set_joint_5(jointAngle[1]);
  set_joint_6(jointAngle[2]);
}

void set_joint_array_fl(int *jointAngle) {
  set_joint_7(jointAngle[0]);
  set_joint_8(jointAngle[1]);
  set_joint_9(jointAngle[2]);
}

void set_joint_array_fr(int *jointAngle) {
  set_joint_10(jointAngle[0]);
  set_joint_11(jointAngle[1]);
  set_joint_12(jointAngle[2]);
}

void set_join_array_leg(const int* flAngles, const int* frAngles, const int* blAngles, const int* brAngles) {
  set_joint_array_fl(flAngles);
  set_joint_array_fr(frAngles);
  set_joint_array_bl(blAngles);
  set_joint_array_br(brAngles);
}
