#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//#define Servo0Min 553
#define Servo0Min 610
#define Servo0Max 2580
#define Servo1Min 580
#define Servo1Max 2470
#define Servo2Min 650
#define Servo2Max 2560

#define Servo3Min 620
#define Servo3Max 2590
#define Servo4Min 550
#define Servo4Max 2500
#define Servo5Min 540
#define Servo5Max 2480

#define Servo6Min 610
#define Servo6Max 2510
#define Servo7Min 550
#define Servo7Max 2440
#define Servo8Min 650
#define Servo8Max 2590

#define Servo9Min 610
#define Servo9Max 2510
#define Servo10Min 530
#define Servo10Max 2420
#define Servo11Min 660
#define Servo11Max 2600

#define SERVOFREQ 50
#define DEFAULT_FREQUENCY 27000000

int n = -1;
int PWM = 0;
int theta[3] = {0, 0, 0};
char theta11_ = 0;
char theta22 = 0;
char theta33 = 0;
char theta44 = 0;
char theta55 = 0;

void theta0(int theta) {
  uint16_t off = map(theta, -90, 90, Servo0Min, Servo0Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(0, off);
}

void theta1(int theta) {
  uint16_t off = map(theta, -90, 90, Servo1Min, Servo1Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(1, off);
}

void theta2(int theta) {
  uint16_t off = map(theta, 90, -90, Servo2Min, Servo2Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(2, off);
}

void theta3(int theta) {
  uint16_t off = map(theta, -90, 90, Servo3Min, Servo3Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(3, off);
}

void theta4(int theta) {
  uint16_t off = map(theta, -90, 90, Servo4Min, Servo4Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(4, off);
}

void theta5(int theta) {
  uint16_t off = map(theta, -90, 90, Servo5Min, Servo5Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(5, off);
}

void theta6(int theta) {
  uint16_t off = map(theta, -90, 90, Servo6Min, Servo6Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(6, off);
}

void theta7(int theta) {
  uint16_t off = map(theta, -90, 90, Servo7Min, Servo7Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(7, off);
}

void theta8(int theta) {
  uint16_t off = map(theta, -90, 90, Servo8Min, Servo8Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(8, off);
}

void theta9(int theta) {
  uint16_t off = map(theta, -90, 90, Servo9Min, Servo9Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(9, off);
}

void theta10(int theta) {
  uint16_t off = map(theta, -90, 90, Servo10Min, Servo10Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(10, off);
}

void theta11(int theta) {
  uint16_t off = map(theta, -90, 90, Servo11Min, Servo11Max);
  Serial.println(String(off));
  pwm.writeMicroseconds(11, off);
}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pwm.begin();
  pwm.setOscillatorFrequency(DEFAULT_FREQUENCY);
  pwm.setPWMFreq(SERVOFREQ);
  yield();
  delay(1000);
  int b = 255;
  Serial.write(b);
  int a = 0;
  //    while(a != 10){
  //      a = Serial.read();
  //    }
}

void loop() {
  Serial.println("input Servo Number and Angle Number");
  while (!Serial.available()) {}
  Serial.println("Servo num " + String(n));
  Serial.println("PWM " + String(PWM));
  n = Serial.parseInt();
  PWM = Serial.parseInt();
  Serial.println("Servo num " + String(n));
  Serial.println("PWM " + String(PWM));
  Serial.parseInt();
  switch (n) {
    case 0: theta0(PWM); break;
    case 1: theta1(PWM); break;
    case 2: theta2(PWM); break;
    case 3: theta3(PWM); break;
    case 4: theta4(PWM); break;
    case 5: theta5(PWM); break;
    case 6: theta6(PWM); break;
    case 7: theta7(PWM); break;
    case 8: theta8(PWM); break;
    case 9: theta9(PWM); break;
    case 10: theta10(PWM); break;
    case 11: theta11(PWM); break;
  }
}
