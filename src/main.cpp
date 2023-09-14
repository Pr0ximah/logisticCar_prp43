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
#include "car.h"

bool firstTime_flag = true;

void setup() { Serial.begin(9600); }

void loop() {
    delay(1000);
    if (firstTime_flag) {
        Car car;
        car.drive.move(40, Point(0, 50));
        firstTime_flag = false;
    }
}