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
 * @date 23.06.25
 * @brief debug模式测试代码1
*/
#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include "drive.h"

bool firstTime_flag = true;

void setup() {
  
}

void loop() {
  if (firstTime_flag) {
    DriveControl testModule;
    firstTime_flag = false;
    testModule.gotoPoint(0, 20);
  }
}