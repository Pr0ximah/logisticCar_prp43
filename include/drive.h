/**
 * @Pr0ximah
 * @date 23.06.25
 * @file drive.h
 * @brief 实现DriveControl类，可以设定坐标，让小车移动到对应的位置
 * @note 小车会一直保持车头向前，移动控制靠麦轮实现
*/

#ifndef DRIVE_H
#define DRIVE_H

#include "point.h"
#include "constDef.h"

// 封装的移动控制，可以设置目标点，小车移动到对应位置
class DriveControl {
private:
    Point posCur, posTar;
    float angle = 0;
public:
    DriveControl();
    ~DriveControl();
    void setTar(Point p);
    void setTar(float x, float y);
    void gotoPoint(Point p);
    void gotoPoint(float x, float y);
    void gotoTar();
private:
    // 控制小车沿x轴或y轴走直线
    void moveX(float tarX);
    void moveY(float tarY);
    void updateStatus();

    // 移动的直接控制，通过函数控制移动的方向和启动、停止
    // 带方向控制，读取小车状态信息，自动修正车头方向
    void forward();
    void left();
    void right();
    void stop();

    // 控制车头方向
};

#endif //DRIVE_H