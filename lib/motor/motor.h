/**
 * @Pr0ximah
 * @date 23.09.37
 * @file motor.h
 * @brief 实现motor类，可以设定电机转速，用PID调节
 */

class motor {
private:
    // 旋转方向
    enum dir { FWD, BCK };

    int speedPin, dirPin;
    Encoder encoder;

    void rotateByPercent();
public:
    motor(int speedPin, int dirPin, int encoderA, int encoderB);

    void update();

    void setSpeed();

}