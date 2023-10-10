/**
 * @Pr0ximah
 * @date 23.06.25
 * @file car.h
 * @attention 小车控制集成
 * @brief 实现Car类，控制小车的所有行动
 */

#ifndef CAR_H
#define CAR_H

#include "drive.h"
#include "robot_arm.h"
class Car {
private:
public:
    DriveControl drive;  // 应该放private！做测试时写到public里
    RobotArm robotarm;
    void autoRun();
};

#endif  // CAR_H