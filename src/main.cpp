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
  delay(500);
  if (firstTime_flag) {
    Car car;
    car.drive.driveByAngle(100, PI / 6);
    delay(5000);
    car.drive.stop();
    firstTime_flag = false;
  }
  // if (firstTime_flag) {
  //   firstTime_flag = false;
  //   Encoder enc(port_Encoder_FL_A, port_Encoder_FL_B, port_Encoder_FR_A, port_Encoder_FR_B, 799, 799);
  //   enc.testCoefficient(Encoder::R);
  // }
}