// 主程序
/*
#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}
*/


/**
 * @Pr0ximah
 * @date 23.09.04
 * @brief debug模式测试代码
*/
#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include "drive.h"
#include "car.h"

bool firstTime_flag = true;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Serial.println(digitalRead(4));
  // delay(20);
  delay(5000);
  if (firstTime_flag) {
    Car car;
    car.drive.gotoPoint(0, 100);
    firstTime_flag = false;
  }
  // if (firstTime_flag) {
  //   firstTime_flag = false;
  //   Encoder enc(port_Encoder_FL_A, port_Encoder_FL_B, 799);
  //   enc.testCoefficient();
  // }
}