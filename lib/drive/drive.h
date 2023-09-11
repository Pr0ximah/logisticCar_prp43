/**
 * @Pr0ximah
 * @date 23.06.25
 * @file drive.h
 * @brief 实现DriveControl类，可以设定坐标，让小车移动到对应的位置
 * @note 小车会一直保持车头向前，移动控制靠麦轮实现
 * @note 坐标系定义如下:
 * /\ y
 * ||
 * ||
 * ||
 * ||        x
 * [] =======>
 * 车头始终朝向y轴正向
 * 角度angle为车头方向与y轴正向的偏移角度，逆时针为正向
 * @todo 全向移动暂时做不到！只能前后左右加旋转
*/

#ifndef DRIVE_H
#define DRIVE_H

#include "point.h"
#include "constDef.h"
#include "portDef.h"
#include "PID.h"
#include "IR.h"
#include "encoder.h"
#include <MPU6050.h>

// 封装的移动控制，可以设置目标点，小车移动到对应位置
class DriveControl {
private:
    // float disOri[4];   // 原始距离信息[F, B, L, R]，用于初始化修正，修正后建立世界坐标系
    // float disLast[4];  // 上一次采样时的距离信息
    // float disCur[4];   // 本次采样的距离信息
    Point posCur, posTar; // 世界坐标系
    // double heading;       // 当前行驶方向
    // MPU6050 imu;  // 陀螺仪初始化
    // IR IRArray[4] = {IR(port_IR_F, IR_VOLTAGE), IR(port_IR_B, IR_VOLTAGE), IR(port_IR_L, IR_VOLTAGE), IR(port_IR_R, IR_VOLTAGE)};  // 红外传感器初始化
    // float angle = 0;   // 小车车头角度
    // float angleAcce0_bias;  // 陀螺仪角加速度值调0偏置
    Encoder encoders;     // 左前轮和右后轮的电机编码器
    enum driveDir{dFWD, dLFT, dRHT, dBCK};
public:
    DriveControl();
    void setTar(Point p);
    void setTar(float x, float y);
    void gotoPoint(Point p);
    void gotoPoint(float x, float y);
    void gotoTar();
    void stop();
private:
    // // imu初始化，消除零误差
    // void imuInit();

    // pos初始化，建立新的世界坐标系
    void posInit();

    // 更新车体位置状态
    void statusUpdate();

    // 移动的直接控制，通过函数控制移动的方向和启动、停止
    // 带方向控制，读取小车状态信息，自动修正车头方向

    // // speed_percent: 前进速度(百分制);  dir: 目标方向向量
    // void driveByVector(float speed_percent, Vector vecTar);

public:
    /// @todo: 先暂时放到public里，正式版放进private
    enum motorDir{FWD, BCK};

    // // speed_percent: 前进速度(百分制);  angleTar: 目标方向(弧度制)
    // void driveByAngle(float speed_percent, float angleTar);

    // 用方向控制前进
    void driveByDir(float speed_percent, driveDir dir);

    // 四轮电机驱动的百分制控制
    void rotateByPercentageFL(double percent, motorDir dir);
    void rotateByPercentageFR(double percent, motorDir dir);
    void rotateByPercentageBL(double percent, motorDir dir);
    void rotateByPercentageBR(double percent, motorDir dir);

    // 控制小车沿x轴或y轴走直线
    void moveX(float tarX);
    void moveY(float tarY);
private:
    // // 更新陀螺仪数据
    // float imuUpdate();

    // 更新位置数据
    void posUpdate();

    // // 读取陀螺仪当前读取到的角加速度值
    // float imuReadAngleAcce();


};

#endif //DRIVE_H