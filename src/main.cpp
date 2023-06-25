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
#include "car.h"

void setup() {

}

void loop() {
  Car car;
  car.drive.gotoPoint(1, 1);
}