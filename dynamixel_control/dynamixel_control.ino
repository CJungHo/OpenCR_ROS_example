#include "Motor_driver.h"

MotorDriver motor_driver;

void setup() {
  Serial.begin(9600);
  motor_driver.init();
}

void loop() {
  //속도 제어
  motor_driver.VelControl(1, 100); // id, vel(-255 ~ 255)

  //위치 제어
  motor_driver.PosControl(1, 2048); // id, pos(0~4096)

  //모터 위치값 읽기
  static int32_t pos;
  motor_driver.readEncoder(pos);
  Serial.print("pos : ");
  Serial.println(pos);
}


