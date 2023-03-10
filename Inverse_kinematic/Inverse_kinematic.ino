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

// --------------------Robot configuration----------------------
// Leg configuration
float LENGTH_1 = 75; // mm
float LENGTH_2 = 70; // mm
float LENGTH_3 = 90; // mm

int n = -1;
int PWM = 0;
float default_pos_r[3] = {0, -75, -100};
float default_pos_l[3] = {0, 75, -100};

int mode = 0;

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

void set_join_array_leg(const int* flAngles, const int* frAngles, const int* blAngles, const int* brAngles){
  set_joint_array_fl(flAngles);
  set_joint_array_fr(frAngles);
  set_joint_array_bl(blAngles);
  set_joint_array_br(brAngles);
}

void calculate_ik_l(float *coordinate, int *jointAngle) {
  float x = coordinate[0];
  float y = coordinate[1];
  float z = coordinate[2];

  float theta1 = 0;
  if (y >= 0) {
    theta1 = acos(LENGTH_1 / (sqrt(sq(y) + sq(z)))) - acos(abs(y) / (sqrt(sq(y) + sq(z))));
  }
  else {
    theta1 = acos(LENGTH_1 / (sqrt(sq(y) + sq(z)))) - acos(abs(z) / (sqrt(sq(y) + sq(z)))) - 1.5708;
  }

  float theta2 = 0;
  float theta3 = 0;
  Serial.println("Original Z: " + String(z) + " and theta_1: " + String(theta1));
  z = z - LENGTH_1 * sin(theta1);
  float length_ = sqrt(sq(x) + sq(z));
  Serial.println("calculate new Z: " + String(z) + " and length " + String(length_));
  if (x >= 0) {
    theta2 = -( (acos(abs(z) / length_) - acos(-(sq(LENGTH_3) - sq(length_) - sq(LENGTH_2)) / (2 * length_ * LENGTH_2))));
    theta3 = acos(-(sq(length_) - sq(LENGTH_3) - sq(LENGTH_2)) / (2 * LENGTH_3 * LENGTH_2)) - 1.5708;
  }
  else {
    theta2 = acos(abs(z) / (sqrt(sq(x) + sq(z)))) + acos(-(sq(LENGTH_3) - sq(LENGTH_2) - sq(length_)) / (2 * LENGTH_2 * length_));
    theta3 = acos(-(sq(length_) - sq(LENGTH_2) - sq(LENGTH_3)) / (2 * LENGTH_2 * LENGTH_3)) - 1.5708;
  }
  Serial.println("calculate_ik_l - 1: " + String(theta1) + "--- 2: " + String(theta2) + "--- 3: " + String(theta3));

  jointAngle[0] = int(theta1 * (180 / 3.14));
  jointAngle[1] = int(theta2 * (180 / 3.14));
  jointAngle[2] = int(theta3 * (180 / 3.14));
}

void calculate_ik_r(float *coordinate, int *jointAngle) {
  float x = coordinate[0];
  float y = coordinate[1];
  float z = coordinate[2];

  float theta1 = 0;
  if (y >= 0) {
    theta1 = 1.5708 - acos(LENGTH_1 / (sqrt(sq(y) + sq(z)))) - acos(abs(z) / (sqrt(sq(y) + sq(z))));
  }
  else {
    theta1 = -( acos(LENGTH_1 / (sqrt(sq(y) + sq(z)))) - acos(abs(y) / (sqrt(sq(y) + sq(z)))) );
  }

  float theta2 = 0;
  float theta3 = 0;
  Serial.println("calculate_ik_r Original Z: " + String(z) + " and theta_1: " + String(theta1));
  z = z + LENGTH_1 * sin(theta1);
  float length_ = sqrt(sq(x) + sq(z));
  Serial.println("calculate_ik_r calculate new Z: " + String(z) + " and length " + String(length_));

  if (x >= 0) {
    theta2 = -( (acos(abs(z) / length_) - acos(-(sq(LENGTH_3) - sq(length_) - sq(LENGTH_2)) / (2 * length_ * LENGTH_2))));
    theta3 = acos(-(sq(length_) - sq(LENGTH_3) - sq(LENGTH_2)) / (2 * LENGTH_3 * LENGTH_2)) - 1.5708;
  }
  else {
    theta2 = acos(abs(z) / (sqrt(sq(x) + sq(z)))) + acos(-(sq(LENGTH_3) - sq(LENGTH_2) - sq(length_)) / (2 * LENGTH_2 * length_));
    theta3 = acos(-(sq(length_) - sq(LENGTH_2) - sq(LENGTH_3)) / (2 * LENGTH_2 * LENGTH_3)) - 1.5708;
  }
  Serial.println("calculate_ik_r - 1: " + String(theta1) + " --- 2: " + String(theta2) + " --- 3: " + String(theta3));
  jointAngle[0] = int(theta1 * (180 / 3.14));
  jointAngle[1] = int(theta2 * (180 / 3.14));
  jointAngle[2] = int(theta3 * (180 / 3.14));
}

void walk_cycle(int *theta_l, int *theta_r) {
  float pos_r[3];
  float pos_l[3];
  memcpy(pos_r, default_pos_r, sizeof(pos_r));
  memcpy(pos_l, default_pos_l, sizeof(pos_l));
  float walkRadius = 50;
  float walkInterval = 20;
  float z_offset = -120;
  int delay_ms = 60;
  for (float x = walkRadius; x > -walkRadius; x -= walkInterval) {
    pos_r[0] = x;
    pos_l[0] = x;
    pos_r[2] = z_offset;
    pos_l[2] = z_offset;
    calculate_ik_r(pos_r, theta_r);
    calculate_ik_l(pos_l, theta_l);
    set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
    delay(delay_ms);
  }
  for (float x = -walkRadius; x < walkRadius; x += walkInterval) {
    pos_r[0] = x;
    pos_l[0] = x;
    float actual_z = sqrt(sq(walkRadius) - sq(x)) + z_offset;
    pos_r[2] = actual_z;
    pos_l[2] = actual_z;
    calculate_ik_r(pos_r, theta_r);
    calculate_ik_l(pos_l, theta_l);
    set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
    delay(delay_ms);
  }
}

void walk_cycle_alternate() {
  float pos_bl[3];
  float pos_br[3];
  float pos_fl[3];
  float pos_fr[3];
  int theta_bl[3];
  int theta_br[3];
  int theta_fl[3];
  int theta_fr[3];
  memcpy(pos_bl, default_pos_l, sizeof(pos_bl));
  memcpy(pos_br, default_pos_r, sizeof(pos_br));
  memcpy(pos_fl, default_pos_l, sizeof(pos_fl));
  memcpy(pos_fr, default_pos_r, sizeof(pos_fr));
  float walkRadius = 50;
  float walkInterval = 15;
  float z_offset = -120;
  int delay_ms = 20;
  for (float x = walkRadius; x > -walkRadius; x -= walkInterval) {
    pos_bl[0] = x;
    pos_br[0] = -x;
    pos_fl[0] = -x;
    pos_fr[0] = x;
    float actual_z = sqrt(sq(walkRadius) - sq(x)) + z_offset;
    pos_bl[2] = z_offset;
    pos_br[2] = actual_z;
    pos_fl[2] = actual_z;
    pos_fr[2] = z_offset;
    calculate_ik_l(pos_bl, theta_bl);
    calculate_ik_r(pos_br, theta_br);
    calculate_ik_l(pos_fl, theta_fl);
    calculate_ik_r(pos_fr, theta_fr);
    set_join_array_leg(theta_bl, theta_br, theta_fl, theta_fr);
    delay(delay_ms);
  }
  for (float x = -walkRadius; x < walkRadius; x += walkInterval) {
    pos_bl[0] = x;
    pos_br[0] = -x;
    pos_fl[0] = -x;
    pos_fr[0] = x;
    float actual_z = sqrt(sq(walkRadius) - sq(x)) + z_offset;
    pos_bl[2] = actual_z;
    pos_br[2] = z_offset;
    pos_fl[2] = z_offset;
    pos_fr[2] = actual_z;
    calculate_ik_l(pos_bl, theta_bl);
    calculate_ik_r(pos_br, theta_br);
    calculate_ik_l(pos_fl, theta_fl);
    calculate_ik_r(pos_fr, theta_fr);
    set_join_array_leg(theta_bl, theta_br, theta_fl, theta_fr);
    delay(delay_ms);
  }
}

void stand_cycle(int *theta_l, int *theta_r) {
  float pos_r[3], pos_l[3];
  memcpy(pos_r, default_pos_r, sizeof(pos_r));
  memcpy(pos_l, default_pos_l, sizeof(pos_l));
  float standOffSet = 40;
  float standInterval = 5;
  float z_offset = -90;
  int delay_ms = 50;
  for (float z = standOffSet; z > -standOffSet; z -= standInterval) {
    float actual_z = z + z_offset;
    pos_r[2] = actual_z;
    pos_l[2] = actual_z;
//    Serial.println("Position_r - x " + String(pos_r[0]) + "--- y " + String(pos_r[1]) + "--- z " + String(pos_r[2]));
//    Serial.println("Position_l - x " + String(pos_l[0]) + "--- y " + String(pos_l[1]) + "--- z " + String(pos_l[2]));
    calculate_ik_r(pos_r, theta_r);
    calculate_ik_l(pos_l, theta_l);
    set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
//    Serial.println("joint_angle_r - 1 " + String(theta_r[0]) + "--- 2 " + String(theta_r[1]) + "--- 3 " + String(theta_r[2]));
//    Serial.println("joint_angle_l - 1 " + String(theta_l[0]) + "--- 2 " + String(theta_l[1]) + "--- 3 " + String(theta_l[2]));
    delay(delay_ms);
  }
  for (float z = -standOffSet; z < standOffSet; z += standInterval) {
    float actual_z = z + z_offset;
    pos_r[2] = actual_z;
    pos_l[2] = actual_z;
//    Serial.println("Position_r - x " + String(pos_r[0]) + "--- y " + String(pos_r[1]) + "--- z " + String(pos_r[2]));
//    Serial.println("Position_l - x " + String(pos_l[0]) + "--- y " + String(pos_l[1]) + "--- z " + String(pos_l[2]));
    calculate_ik_r(pos_r, theta_r);
    calculate_ik_l(pos_l, theta_l);
    set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
//    Serial.println("joint_angle_r - 1 " + String(theta_r[0]) + "--- 2 " + String(theta_r[1]) + "--- 3 " + String(theta_r[2]));
//    Serial.println("joint_angle_l - 1 " + String(theta_l[0]) + "--- 2 " + String(theta_l[1]) + "--- 3 " + String(theta_l[2]));
    delay(delay_ms);
  }
}

int serialSetX(int *theta_l, int *theta_r) {
  float pos_r[3], pos_l[3];
  memcpy(pos_r, default_pos_r, sizeof(pos_r));
  memcpy(pos_l, default_pos_l, sizeof(pos_l));  
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true){
    if(Serial.available()){
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput != 0){
      pos_r[0] = serialIntOutput;
      pos_l[0] = serialIntOutput;
      calculate_ik_r(pos_r, theta_r);
      calculate_ik_l(pos_l, theta_l);
      set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
    }
    if (serialIntOutput == -1){
      return 0;
    }
  }
}

int serialSetY(int *theta_l, int *theta_r) {
  float pos_bl[3];
  float pos_br[3];
  float pos_fl[3];
  float pos_fr[3];
  int theta_bl[3];
  int theta_br[3];
  int theta_fl[3];
  int theta_fr[3];
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true){
    if(Serial.available()){
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput != 0){
      memcpy(pos_bl, default_pos_l, sizeof(pos_bl));
      memcpy(pos_br, default_pos_r, sizeof(pos_br));
      memcpy(pos_fl, default_pos_l, sizeof(pos_fl));
      memcpy(pos_fr, default_pos_r, sizeof(pos_fr));
      pos_bl[1] = pos_bl[1] + serialIntOutput;
      pos_br[1] = pos_br[1] + serialIntOutput;
      pos_fl[1] = pos_fl[1] + serialIntOutput;
      pos_fr[1] = pos_fr[1] + serialIntOutput;
      calculate_ik_l(pos_bl, theta_bl);
      calculate_ik_r(pos_br, theta_br);
      calculate_ik_l(pos_fl, theta_fl);
      calculate_ik_r(pos_fr, theta_fr);
      Serial.println("pos_fl - 1 " + String(pos_fl[0]) + "--- 2 " + String(pos_fl[1]) + "--- 3 " + String(pos_fl[2]));
      Serial.println("pos_fr - 1 " + String(pos_fr[0]) + "--- 2 " + String(pos_fr[1]) + "--- 3 " + String(pos_fr[2]));
      Serial.println("pos_bl - 1 " + String(pos_bl[0]) + "--- 2 " + String(pos_bl[1]) + "--- 3 " + String(pos_bl[2]));
      Serial.println("pos_br - 1 " + String(pos_br[0]) + "--- 2 " + String(pos_br[1]) + "--- 3 " + String(pos_br[2]));
      Serial.println("theta_fl - 1 " + String(theta_fl[0]) + "--- 2 " + String(theta_fl[1]) + "--- 3 " + String(theta_fl[2]));
      Serial.println("theta_fr - 1 " + String(theta_fr[0]) + "--- 2 " + String(theta_fr[1]) + "--- 3 " + String(theta_fr[2]));
      Serial.println("theta_bl - 1 " + String(theta_bl[0]) + "--- 2 " + String(theta_bl[1]) + "--- 3 " + String(theta_bl[2]));
      Serial.println("theta_br - 1 " + String(theta_br[0]) + "--- 2 " + String(theta_br[1]) + "--- 3 " + String(theta_br[2]));
      set_join_array_leg(theta_fl, theta_fr, theta_bl, theta_br);
    }
    if (serialIntOutput == -1){
      return 0;
    }
  }
}

int serialSetZ(int *theta_l, int *theta_r) {
  float pos_r[3], pos_l[3];
  memcpy(pos_r, default_pos_r, sizeof(pos_r));
  memcpy(pos_l, default_pos_l, sizeof(pos_l));  
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true){
    if(Serial.available()){
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput < 0){
      pos_r[2] = serialIntOutput;
      pos_l[2] = serialIntOutput;
      calculate_ik_r(pos_r, theta_r);
      calculate_ik_l(pos_l, theta_l);
      set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
    }
    if (serialIntOutput == -1){
      return 0;
    }
  }
}

void serialSetBodyRotX(int *theta_l, int *theta_r){
  float pos_r[3], pos_l[3];
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true){
    if(Serial.available()){
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput != 0){
      memcpy(pos_r, default_pos_r, sizeof(pos_r));
      memcpy(pos_l, default_pos_l, sizeof(pos_l));  
      float rad = float(serialIntOutput) * (3.14 / 180);
      float dz = 232 / 2 * float(sin(rad)); // 232 is the body length in y direction
      pos_r[2] = pos_r[2] + dz;
      pos_l[2] = pos_l[2] - dz;
      calculate_ik_r(pos_r, theta_r);
      calculate_ik_l(pos_l, theta_l);
      set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
    }
    if (serialIntOutput == -1){
      return 0;
    }
  }
}

void serialSetBodyRotY(int *theta_l, int *theta_r){
  float pos_bl[3];
  float pos_br[3];
  float pos_fl[3];
  float pos_fr[3];
  int theta_bl[3];
  int theta_br[3];
  int theta_fl[3];
  int theta_fr[3];
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true){
    if(Serial.available()){
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput != 0){
      memcpy(pos_bl, default_pos_l, sizeof(pos_bl));
      memcpy(pos_br, default_pos_r, sizeof(pos_br));
      memcpy(pos_fl, default_pos_l, sizeof(pos_fl));
      memcpy(pos_fr, default_pos_r, sizeof(pos_fr));
      float rad = float(serialIntOutput) * (3.14 / 180);
      float dz = 256 / 2 * float(sin(rad)); // 232 is the body length in x direction
      pos_bl[2] = pos_bl[2] + dz;
      pos_br[2] = pos_br[2] + dz;
      pos_fl[2] = pos_fl[2] - dz;
      pos_fr[2] = pos_fr[2] - dz;
      calculate_ik_l(pos_bl, theta_bl);
      calculate_ik_r(pos_br, theta_br);
      calculate_ik_l(pos_fl, theta_fl);
      calculate_ik_r(pos_fr, theta_fr);
      set_join_array_leg(theta_fl, theta_fr, theta_bl, theta_br);
    }
    if (serialIntOutput == -1){
      return 0;
    }
  }
}

void serialSetBodyRotZ(int *theta_l, int *theta_r){
  float pos_bl[3];
  float pos_br[3];
  float pos_fl[3];
  float pos_fr[3];
  int theta_bl[3];
  int theta_br[3];
  int theta_fl[3];
  int theta_fr[3];
  int delay_ms = 50;
  int serialIntOutput = 0;
  while (true){
    if(Serial.available()){
      Serial.println("Pasrsing output");
      serialIntOutput = Serial.parseInt();
      Serial.println("Pasrsing output" + String(serialIntOutput));
    }
    if (serialIntOutput != 0){
      memcpy(pos_bl, default_pos_l, sizeof(pos_bl));
      memcpy(pos_br, default_pos_r, sizeof(pos_br));
      memcpy(pos_fl, default_pos_l, sizeof(pos_fl));
      memcpy(pos_fr, default_pos_r, sizeof(pos_fr));
      float rad = float(serialIntOutput) * (3.14 / 180);
      float dy = 256 / 2 * float(sin(rad)); // 256 is the body length in x direction
      pos_fl[1] = pos_fl[1] + dy;
      pos_fr[1] = pos_fr[1] - dy;
      pos_bl[1] = pos_bl[1] + dy;
      pos_br[1] = pos_br[1] - dy;
      calculate_ik_l(pos_fl, theta_fl);
      calculate_ik_r(pos_fr, theta_fr);
      calculate_ik_l(pos_bl, theta_bl);
      calculate_ik_r(pos_br, theta_br);
      Serial.println("pos_fl - 1 " + String(pos_fl[0]) + "--- 2 " + String(pos_fl[1]) + "--- 3 " + String(pos_fl[2]));
      Serial.println("pos_fr - 1 " + String(pos_fr[0]) + "--- 2 " + String(pos_fr[1]) + "--- 3 " + String(pos_fr[2]));
      Serial.println("pos_bl - 1 " + String(pos_bl[0]) + "--- 2 " + String(pos_bl[1]) + "--- 3 " + String(pos_bl[2]));
      Serial.println("pos_br - 1 " + String(pos_br[0]) + "--- 2 " + String(pos_br[1]) + "--- 3 " + String(pos_br[2]));
      Serial.println("theta_fl - 1 " + String(theta_fl[0]) + "--- 2 " + String(theta_fl[1]) + "--- 3 " + String(theta_fl[2]));
      Serial.println("theta_fr - 1 " + String(theta_fr[0]) + "--- 2 " + String(theta_fr[1]) + "--- 3 " + String(theta_fr[2]));
      Serial.println("theta_bl - 1 " + String(theta_bl[0]) + "--- 2 " + String(theta_bl[1]) + "--- 3 " + String(theta_bl[2]));
      Serial.println("theta_br - 1 " + String(theta_br[0]) + "--- 2 " + String(theta_br[1]) + "--- 3 " + String(theta_br[2]));
      set_join_array_leg(theta_fl, theta_fr, theta_bl, theta_br);
    }
    if (serialIntOutput == -1){
      return 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  // put your setup code here, to run once:
  pwm.begin();
  pwm.setOscillatorFrequency(DEFAULT_FREQUENCY);
  pwm.setPWMFreq(SERVOFREQ);
  yield();

  int theta_l[3] = {0, 0, 0};
  int theta_r[3] = {0, 0, 0};
  calculate_ik_l(default_pos_l, theta_l);
  calculate_ik_r(default_pos_r, theta_r);
  float *pos_l = default_pos_l;
  float *pos_r = default_pos_r;
  set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
  delay(100);
}

void loop() {
  int theta_r[3] = {0, 0, 0};
  int theta_l[3] = {0, 0, 0};
  int serialIntOutput = 0;

  Serial.println("Before Serial.available");
  Serial.println("mode: " + String(mode));
  if(Serial.available()){
    Serial.println("Pasrsing output");
    serialIntOutput = Serial.parseInt();
    Serial.println("Pasrsing output" + String(serialIntOutput));
  }
  Serial.println("After Serial.available");
  if (serialIntOutput != 0){
    mode = serialIntOutput;
  }
  switch (mode){
    case -1:
      float pos_r[3];
      float pos_l[3];
      memcpy(pos_r, default_pos_r, sizeof(pos_r));
      memcpy(pos_l, default_pos_l, sizeof(pos_l));
      calculate_ik_l(pos_l, theta_l);
      calculate_ik_r(pos_r, theta_r);
      set_join_array_leg(theta_l, theta_r, theta_l, theta_r);
      delay(100);
      break;
    case 1: walk_cycle(theta_l, theta_r); break;
    case 2: stand_cycle(theta_l, theta_r); break;
    case 3: 
      serialSetZ(theta_l, theta_r);
      mode = -1;
      break;
    case 4: 
      serialSetBodyRotX(theta_l, theta_r);
      mode = -1;
      break;
    case 5: 
      serialSetBodyRotY(theta_l, theta_r);
      mode = -1;
      break;
    case 6: 
      serialSetBodyRotZ(theta_l, theta_r);
      mode = -1;
      break;
    case 7: 
      serialSetY(theta_l, theta_r);
      mode = -1;
      break;
    case 8: 
      serialSetX(theta_l, theta_r);
      mode = -1;
      break;
    case 9:
      walk_cycle_alternate();
      break;
  }
}
