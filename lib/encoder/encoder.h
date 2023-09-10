/**
 * @Pr0ximah
 * @date 23.09.05
 * @file encoder.h
 * @brief 实现Encoder类，配置电机编码器，提供读取车轮移动距离的接口
*/

#ifndef ENCODER_H
#define ENCODER_H

class Encoder {
public:
    int numRound;       // 转过圈数
    int pulseCount;     // 脉冲数量
    int portA, portB;   // AB相端口
    const int COEFFICIENT_PER_ROUND;    // 脉冲数-圈数映射系数
public:
    Encoder(int _portA, int _portB, int _coefficient);
    
    // 重置编码器读数
    void reset();

    // 获取编码器转过角度 [0, 360)
    float getAngle() const;

    // 获取编码器转过绝对角度 (-inf, inf)
    float getAbsoluteAngle() const;

    // 获取编码器转过圈数
    int getRound() const;

    // 获取轮子转过距离
    float getDisOfWheel() const;

    // 更新编码器数据
    void update();

    // 测定coefficient的值
    // 输出pulseCount
    // 串口输入RESET重置pulseCount
    // 输入STOP退出
    // 手动测试多转几圈拟合，得每圈系数
    void testCoefficient();
private:
    // 读取B相数据并更新count值，是A相下降沿触发的函数
    void updateCount();

    // attachInterrupt中断函数处理
    static void ISRHandler();
    void setupISRHandler(int pin, void(*ISR)(void), int state);
    static Encoder *_ISRPointer;
    int id;
    static int s_id;
    static Encoder *objArray[2];
};

#endif // ENCODER_H