/**
 * @Pr0ximah
 * @date 23.09.05
 * @file encoder.h
 * @brief 实现Encoder类，配置电机编码器，提供读取车轮移动距离的接口
 */

#ifndef ENCODER_H
#define ENCODER_H

class EncoderSet;

// 单个编码器数据保存
class Encoder {
   private:
    int numRound;                     // 转过圈数
    int pulseCount;                   // 脉冲数量
    int portA, portB;                 // AB相端口
    int ISR_Port;                     // attachinterrupt函数调用端口值
    const int COEFFICIENT_PER_ROUND;  // 脉冲数-圈数映射系数
    double angleVel[3] = {0, 0, 0};   // 角速度测量值(deg)
    // 以下为保存两次更新状态所用的变量
    bool firstTimeFlag = true;
    unsigned long timeLast, timeCur;
    double angleLast, angleCur;
    int countOfUpdate;  // 记录更新次数，每5次更新一次电机转速
    friend class EncoderSet;

   public:
    Encoder(int _portA, int _portB, int _coefficient);

    // 重置编码器读数
    void reset();

    // 获取编码器转过角度 [0, 360deg)
    float getAngle() const;

    // 获取编码器转过绝对角度 (-inf, inf)
    float getAbsoluteAngle() const;

    // 获取编码器转过圈数
    int getRound() const;

    // 获取轮子转过距离
    float getDisOfWheel() const;

    // 获取角速度
    double getAngleVel() const;

    // 测定coefficient的值
    // 输出pulseCount
    // 串口输入RESET重置pulseCount
    // 输入STOP退出
    // 手动测试多转几圈拟合，得每圈系数
    void testCoefficient();

    // 更新编码器数据
    void update();

   private:
    // 读取B相数据并更新count值，是A相下降沿触发的函数
    void updateCount();
};

// 编码器集合类
class EncoderSet {
   public:
    // 编码器对象
    static Encoder encoderFL, encoderFR;

   private:
    // 编码器指针
    static Encoder *PtEncoderFL, *PtEncoderFR;
    // 编码器更新函数
    static void updateFL();
    static void updateFR();

   public:
    // 初始化函数
    EncoderSet();
};

#endif  // ENCODER_H