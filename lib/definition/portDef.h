/**
 * @Pr0ximah
 * @date 23.06.25
 * @file portDef.h
 * @brief 存储端口定义
*/

#ifndef PORT_DEF_H
#define PORT_DEF_H

// 电机端口
const int port_motor_FL = 7;  // 左前电机PWM
const int port_motor_FR = 10;  // 右前电机PWM
const int port_motor_BL = 6;  // 左后电机PWM
const int port_motor_BR = 11;  // 右后电机PWM
const int port_dir_FL = 9;    // 左前电机方向
const int port_dir_FR = 12;    // 右前电机方向
const int port_dir_BL = 8;    // 左后电机方向
const int port_dir_BR = 13;    // 右后电机方向

// // 红外测距传感器端口
// const int port_IR_F = 5;  // 前置红外
// const int port_IR_B = 6;  // 后置红外
// const int port_IR_L = 7;  // 左侧红外
// const int port_IR_R = 8;  // 右侧红外

// 电机编码器接口
const int port_Encoder_FL_A = 2;  // 左前电机编码器A相
const int port_Encoder_FL_B = 5;  // 左前电机编码器B相
const int port_Encoder_FR_A = 3;  // 右前电机编码器A相
const int port_Encoder_FR_B = 4;  // 右前电机编码器B相
// const int port_Encoder_BL_A = 13;  // 左后电机编码器A相
// const int port_Encoder_BL_B = 14;  // 左后电机编码器B相
// const int port_Encoder_BR_A = 15;  // 右后电机编码器A相
// const int port_Encoder_BR_B = 16;  // 右后电机编码器B相

#endif //PORT_DEF_H