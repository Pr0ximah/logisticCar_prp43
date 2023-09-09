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
#include "encoder.h"

bool firstTime_flag = true;
Encoder ecd(2, 3, 778);

void setup() {
  Serial.begin(9600);
}

void loop() {
  ecd.update();
  Serial.println(ecd.getAbsoluteAngle());
}