#include "point.h"

#include <math.h>

Point::Point() : x(0), y(0) {}

Point::Point(float _x, float _y) : x(_x), y(_y) {}

Point::Point(const Point &pt) : x(pt.x), y(pt.y) {}

float Point::getX() const { return x; }

float Point::getY() const { return y; }

void Point::setXY(float _x, float _y) {
    x = _x;
    y = _y;
}

void Point::setXY(const Point &pt) {
    if (this != &pt) {
        x = pt.x;
        y = pt.y;
    }
}

Point &Point::operator=(const Point &pt) {
    if (this != &pt) {
        x = pt.x;
        y = pt.y;
    }
    return *this;
}

Point Point::operator+(const Point &pt) { return Point(x + pt.x, y + pt.y); }

Vector Point::operator-(const Point &pt) { return Vector(x - pt.x, y - pt.y); }

Vector::Vector() {
    x = 0;
    y = 0;
}

Vector::Vector(float _x, float _y) {
    x = _x;
    y = _y;
}

Vector::Vector(const Point &pt) {
    x = pt.getX();
    y = pt.getY();
}

float Vector::getX() const { return x; }

float Vector::getY() const { return y; }

void Vector::setXY(float _x, float _y) {
    x = _x;
    y = _y;
}

float Vector::getNorm() const { return sqrt(x * x + y * y); }

Point Point::operator+(const Vector &vec) { return Point(x + vec.getX(), y + vec.getY()); }

float Vector::getAngle() const { return atan2(-x, y); }

Vector Vector::operator*(int k) { return Vector(this->x * k, this->y * k); }