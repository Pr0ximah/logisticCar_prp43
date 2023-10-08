// // 主程序
// #include <Arduino.h>

// Car car;

// void setup() { Serial.begin(9600); }

// void loop() {}

/**
 * @Pr0ximah
 * @date 23.09.04
 * @brief debug模式测试代码
 */
#include <Arduino.h>

#include "car.h"

bool firstTime_flag = true;

void setup() {
    Serial.begin(9600);
}

void loop() {
    delay(1000);
    if (firstTime_flag) {
        Car car;
        // car.drive.encoders.encoderFL.testCoefficient();
        // car.drive.motorPIDTest(0);
        car.drive.move(100, Vector(0, 120));
        car.drive.rotate(60, 90);
        car.drive.move(40, Vector(0, 100));
        car.drive.rotate(60, -90);
        car.drive.move(100, Vector(60, 30));
        // car.drive.rotateByPercentageFL(100, DriveControl::motorDir::FWD);
        // car.drive.rotateByPercentageFR(100, DriveControl::motorDir::FWD);
        // car.drive.rotateByPercentageBL(100, DriveControl::motorDir::FWD);
        // car.drive.rotateByPercentageBR(100, DriveControl::motorDir::FWD);
        firstTime_flag = false;
    }
}