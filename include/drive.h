/**
 * @Pr0ximah
 * @date 23.06.25
 * @file drive.h
 * @brief 实现DriveControl类，可以设定坐标，让小车移动到对应的位置
*/

#ifndef DRIVE_H
#define DRIVE_H

#include "point.h"

class DriveControl {
private:
    Point posCur, posTar;
public:
    void setTar(Point p);
    void setTar(float x, float y);
    void gotoPoint(Point p);
    void gotoPoint(float x, float y);
    void gotoTar();
};

#endif //DRIVE_H