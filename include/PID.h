/**
 * @Pr0ximah
 * @date 23.06.25
 * @file PID.h
 * @brief 实现PID类，提供PID控制，并实现自动参数整定
 * @todo 参数自整定部分
*/

class PID {
public:
    // variables
    float errorNow, errorLast, errorDiff, errorInt;
    bool init_Flag;
    float P, I, D;
    float kp, ki, kd;
    float target;
    float outVal;
    int secStable;
    int secNow, secLast;

    // functions
    PID(float _target);

    // 更新PID参数值，输入当前值，返回控制量
    float update(float current);

    void setCoefficient(float _kp, float _ki, float _kd);
};

// 返回x的符号值，正返回1，负返回-1
int sign(float x);