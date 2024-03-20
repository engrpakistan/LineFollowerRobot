/*  this line follower is based on
  QTR Array
  T6612FNG motor driver 
  PID Controller 
   */

#include <QTRSensors.h>
#include <SparkFun_TB6612.h>


#define AIN1 5
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 3
#define PWMB 9
#define STBY 6

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

QTRSensors qtr;


const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int max_speed = 255;   // 225
int turn = 120;
int L = 0;
int R = 0;
int error = 0;
int adj = 0;

float Kp = 0.0728; //0.0728 @volttage 
float Ki =0.0090;  //0.0090
float Kd = 1;//1

int P;
int I;
int D;
int lastError = 0;

uint16_t position;
void setup() {
  brake(motor1, motor2);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){
                      A0, A1, A2, A3, A4, A5, A6, A7 },
                    SensorCount);
  qtr.setEmitterPin(2);
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }
  Serial.println("Done");
  Serial.begin(9600);
}

void loop() {
  // forward(225,225);
  PID_control();
  // checksensor();
}
void PID_control() {
  position = qtr.readLineWhite(sensorValues);
  Serial.print("QTR::");
  Serial.println(position);
  error = 3500 - position;
//  if (position > 1000 && position < 6000) {
    P = error;
    I = I + error;
    D = error - lastError;
    lastError = error;

    adj = P * Kp + I * Ki + D * Kd;

    L = max_speed + adj;
    R = max_speed - adj;

    if (L > max_speed) {
      L = max_speed;
    }
    if (R > max_speed) {
      R = max_speed;
    }
    if (L < 0) {
      L = 0;
    }
    if (R < 0) {
      R = 0;
    }
    forward(L, R);
  }
  //  else{
  //   sharp_left();
  // }

  //  else if (position >=6000 && position <= 7000) {
  //   sharp_right();
  // }
  // //else{
  //   //sharp_right();
  // //}
  




void forward(int L, int R) {
  motor1.drive(L);
  motor2.drive(R);
}
void sharp_right() {
  motor1.drive(-255);
  motor2.drive(255);
}
void sharp_left() {
  motor1.drive(255);
  motor2.drive(-255);
}
