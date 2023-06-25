/**
 * @Pr0ximah
 * @date 23.06.25
 * @file point.h
 * @brief 实现point类和vector类用于位置计算
*/

#ifndef POINT_H
#define POINT_H

// 二维点类
class Point {
private:
    // 坐标
    float x, y;
public:
    // 构造函数
    Point();
    Point(float _x, float _y);
    Point(const Point &pt);
    
    float getX() const;
    float getY() const;
    void setXY(float _x, float _y);
    void setXY(const Point &pt);

    Point &operator=(const Point &pt);
};

// 二维向量类
class Vector {
private:
    float x, y;
public:
    Vector();
    Vector(float x, float y);
    Vector(const Point &pt);

    float getX() const;
    float getY() const;
    void setXY(float _x, float _y);

    // 获取模长
    float getNorm() const;
};

#endif //POINT_H