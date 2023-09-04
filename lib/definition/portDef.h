/**
 * @Pr0ximah
 * @date 23.06.25
 * @file portDef.h
 * @brief 存储端口定义
*/

#ifndef PORT_DEF_H
#define PORT_DEF_H

// 电机端口
const int port_motor_FL = 1;  // 左前电机PWM
const int port_motor_FR = 2;  // 右前电机PWM
const int port_motor_BL = 3;  // 左后电机PWM
const int port_motor_BR = 4;  // 右后电机PWM
const int port_dir_FL = 5;    // 左前电机方向
const int port_dir_FR = 6;    // 右前电机方向
const int port_dir_BL = 7;    // 左后电机方向
const int port_dir_BR = 8;    // 右后电机方向

// // 红外测距传感器端口
// const int port_IR_F = 5;  // 前置红外
// const int port_IR_B = 6;  // 后置红外
// const int port_IR_L = 7;  // 左侧红外
// const int port_IR_R = 8;  // 右侧红外

#endif //PORT_DEF_H