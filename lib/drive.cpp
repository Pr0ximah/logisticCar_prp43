#include "drive.h"
#include "Arduino.h"
#include <math.h>
#include "portDef.h"

// 初始化设置
DriveControl::DriveControl() {
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
    updateStatus();
    while (fabs(posCur.getX() - tarX) > POS_ERROR_TOLERANCE) {
        forward();
        updateStatus();
    }
}

void DriveControl::moveY(float tarY) {

}

void DriveControl::updateStatus() {
    
}

void forward() {
    
}