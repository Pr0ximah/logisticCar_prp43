#include "drive.h"

void DriveControl::setTar(Point p) {
    posTar.setXY(p);
}

void DriveControl::setTar(float _x, float _y) {
    posTar.setXY(_x, _y);
}

void DriveControl::gotoPoint(Point p) {
    gotoPoint(p.getX(), p.getY());
}

void DriveControl::gotoPoint(float x, float y) {
    
}