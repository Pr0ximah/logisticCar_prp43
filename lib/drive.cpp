#include "drive.h"
#include "Arduino.h"
#include <math.h>
#include "portDef.h"

// 初始化设置
DriveControl::DriveControl() {
    // imu初始化
    imuInit();
    // 电机端口输出开

}

DriveControl::~DriveControl() {
    // 电机端口输出关

}

void DriveControl::setTar(Point p) {
    posTar.setXY(p);
}

void DriveControl::setTar(float _x, float _y) {
    posTar.setXY(_x, _y);
}

void DriveControl::gotoPoint(Point p) {
    Vector vecToMove = p - posCur;
    //先沿x走再沿y走
    moveX(vecToMove.getX());
    moveY(vecToMove.getY());
}

void DriveControl::gotoPoint(float x, float y) {
    gotoPoint(Point(x, y));
}

void DriveControl::gotoTar() {
    gotoPoint(posTar);
}

void DriveControl::moveX(float tarX) {
    PID pid = {0};
    float controlVal = 0;
    statusUpdate();
    while (fabs(posCur.getX() - tarX) > POS_ERROR_TOLERANCE) {
        right(controlVal);
        delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
        statusUpdate();
        controlVal = pid.update(angle);
    }
}

void DriveControl::moveY(float tarY) {
    PID pid = {0};
    float controlVal = 0;
    statusUpdate();
    while (fabs(posCur.getY() - tarY) > POS_ERROR_TOLERANCE) {
        forward(controlVal);
        delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
        statusUpdate();
        controlVal = pid.update(angle);
    }
}

void DriveControl::statusUpdate() {
    imuUpdate();
}

void DriveControl::forward(float controlVal) {
    // 电机控制
    // -----------------
    // -----------------
}

void DriveControl::imuUpdate() {
    static bool firstTime_flag = true;
    static int updateTimeCur;
    static int updateTimeLast;
    int imuUpdateTimeInterval;  // 实际采样时间间隔
    if (firstTime_flag) { //若第一次，则不改变角度值
        firstTime_flag = false;
        updateTimeLast = millis();
        return;
    } else {
        float angleAcce;
        updateTimeCur = millis();
        angleAcce = imuReadAngleAcce();
        imuUpdateTimeInterval = updateTimeCur - updateTimeLast;
        updateTimeLast = updateTimeCur;
        angle += angleAcce * imuUpdateTimeInterval / 1000;
    }
}

void DriveControl::imuInit() {
    float angleAcce0_sum = 0;       //初始化静态偏移量和变量
    int16_t angleAcceOri;  // 角加速度原始数值
    int16_t temp1, temp2, temp3, temp4, temp5;  // 不使用这些变量，仅用于imu函数传参
    Serial.println("IMU initializing starts...");
    delay(1500);                    //等待车辆静止
    for (int i = 0; i < IMUINIT_SAMPLE_NUM; i++) {
        imu.getMotion6(&temp1, &temp2, &temp3, &temp4, &temp5, &angleAcceOri); //获取三轴加速度和三轴角速度初始值
        float angleAcce0 = angleAcceOri / ANGLE_ACCE_COEFFICIENT;
        angleAcce0_sum += angleAcce0;
        delay(10);
    }
    angleAcce0_bias = angleAcce0_sum / IMUINIT_SAMPLE_NUM;
    Serial.println("IMU initialize finished!");
}

float DriveControl::imuReadAngleAcce() {
    int16_t angleAcceOri;  // 角加速度原始数值
    int16_t temp1, temp2, temp3, temp4, temp5;  // 不使用这些变量，仅用于imu函数传参
    imu.getMotion6(&temp1, &temp2, &temp3, &temp4, &temp5, &angleAcceOri); //获取三轴加速度和三轴角速度初始值
    return angleAcceOri / ANGLE_ACCE_COEFFICIENT - angleAcce0_bias;
}