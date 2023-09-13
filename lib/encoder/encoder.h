/**
 * @Pr0ximah
 * @date 23.09.05
 * @file encoder.h
 * @brief 实现Encoder类，配置电机编码器，提供读取车轮移动距离的接口
*/

#ifndef ENCODER_H
#define ENCODER_H

// 电机编码器类，左右编码器均在其中
class Encoder {
private:
    int numRoundL, numRoundR;       // 转过圈数
    int pulseCountL, pulseCountR;     // 脉冲数量
    int portA_L, portB_L, portA_R, portB_R;   // AB相端口
    int ISR_PortL, ISR_PortR;       // attachinterrupt函数调用端口值
    const int COEFFICIENT_PER_ROUND_L, COEFFICIENT_PER_ROUND_R;    // 脉冲数-圈数映射系数
    double angleVel[3] = {0, 0, 0};   // 角速度测量值(deg)
    bool firstTimeFlag = true;
    unsigned long timeLast, timeCur;
    double angleLast, angleCur;
    int countOfUpdate;  // 记录更新次数，每5次更新一次电机转速
public:
    enum side {L, R};   // 左右边选择

    Encoder(int _portA_L, int _portB_L, int _portA_R, int _portB_R, int _coefficient_L, int _coefficient_R);
    
    // 重置编码器读数
    void reset();

    // 获取编码器转过角度 [0, 360deg)
    float getAngle(Encoder::side side) const;

    // 获取编码器转过绝对角度 (-inf, inf)
    float getAbsoluteAngle(Encoder::side side) const;

    // 获取编码器转过圈数
    int getRound(Encoder::side side) const;

    // 获取轮子转过距离
    float getDisOfWheel(Encoder::side side) const;

    // 更新编码器数据
    void update();

    // 测定coefficient的值
    // 输出pulseCount
    // 串口输入RESET重置pulseCount
    // 输入STOP退出
    // 手动测试多转几圈拟合，得每圈系数
    void testCoefficient(Encoder::side side);

    double getAngleVel() const;
private:
    // 读取B相数据并更新count值，是A相下降沿触发的函数
    void updateCountL();
    void updateCountR();

    // attachInterrupt中断函数处理
    static void ISRHandlerL();
    static void ISRHandlerR();
    void setupISRHandler(int pin, void(*ISR)(void), int state);
    static Encoder *_ISRPointer;
};

#endif // ENCODER_H