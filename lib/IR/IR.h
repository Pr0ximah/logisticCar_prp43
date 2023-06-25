/**
 * @Pr0ximah
 * @date 23.06.25
 * @file IR.h
 * @brief 实现IR类，读取红外传感器返回的距离信息
*/

#ifndef IR_H
#define IR_H

class IR {
private:
    int inputPin;
    float voltage;
public:
    IR(int _inputPin, float _voltage);
    float IR_ReadDis();
};

#endif //IR_H