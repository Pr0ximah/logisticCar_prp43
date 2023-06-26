/**
 * @Pr0ximah
 * @date 23.06.25
 * @file portDef.h
 * @brief 存储端口定义
*/

#ifndef PORT_DEF_H
#define PORT_DEF_H

// 电机端口
const int port_motor_FL = 1;  // 驱动电机左前
const int port_motor_FR = 2;  // 驱动电机右前
const int port_motor_BL = 3;  // 驱动电机左后
const int port_motor_BR = 4;  // 驱动电机右后

// 红外测距传感器端口
const int port_IR_F = 5;  // 前置红外
const int port_IR_B = 6;  // 后置红外
const int port_IR_L = 7;  // 左侧红外
const int port_IR_R = 8;  // 右侧红外

#endif //PORT_DEF_H