/**
 * @Pr0ximah
 * @date 23.06.25
 * @file drive.h
 * @brief 实现DriveControl类，可以设定坐标，让小车移动到对应的位置
 * @note 小车会一直保持车头向前，移动控制靠麦轮实现
 * @note 坐标系定义如下:
 *  ^ y
 *  |
 *  |
 *  |
 *  |        x
 * [c] ------>
 * 车头始终朝向y轴正向
 * 角度angle为车头方向与y轴正向的偏移角度，逆时针为正向
*/

#ifndef DRIVE_H
#define DRIVE_H

#include "point.h"
#include "constDef.h"
#include "PID.h"
#include <MPU6050.h>

// 封装的移动控制，可以设置目标点，小车移动到对应位置
class DriveControl {
private:
    Point posCur, posTar;
    MPU6050 imu;  // 陀螺仪初始化
    float angle = 0;
    float angleAcce0_bias;  // 陀螺仪角加速度值调0偏置
public:
    DriveControl();
    ~DriveControl();
    void setTar(Point p);
    void setTar(float x, float y);
    void gotoPoint(Point p);
    void gotoPoint(float x, float y);
    void gotoTar();
private:
    // 初始化，消除零误差
    void imuInit();

    // 控制小车沿x轴或y轴走直线
    void moveX(float tarX);
    void moveY(float tarY);

    // 更新车体位置状态
    void statusUpdate();

    // 移动的直接控制，通过函数控制移动的方向和启动、停止
    // 带方向控制，读取小车状态信息，自动修正车头方向
    void forward(float controlVal);
    void left(float controlVal);
    void right(float controlVal);
    void stop(float controlVal);

    // 更新陀螺仪数据
    void imuUpdate();

    // 读取陀螺仪当前读取到的角加速度值
    float imuReadAngleAcce();
};

#endif //DRIVE_H