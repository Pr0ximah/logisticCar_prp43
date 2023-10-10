/**
 * @Pr0ximah
 * @date 23.06.25
 * @file portDef.h
 * @brief 存储端口定义
 */

#ifndef PORT_DEF_H
#define PORT_DEF_H

// 电机端口
const int port_motor_FL = 11;  // 左前电机PWM
const int port_motor_FR = 10;  // 右前电机PWM
const int port_motor_BL = 5;   // 左后电机PWM
const int port_motor_BR = 4;   // 右后电机PWM
const int port_dir_FL = 13;    // 左前电机方向
const int port_dir_FR = 12;    // 右前电机方向
const int port_dir_BL = 7;     // 左后电机方向
const int port_dir_BR = 6;     // 右后电机方向

// 电机编码器接口
const int port_Encoder_FL_A = 2;   // 左前电机编码器A相
const int port_Encoder_FL_B = 31;  // 左前电机编码器B相
const int port_Encoder_FR_A = 18;  // 右前电机编码器A相
const int port_Encoder_FR_B = 29;  // 右前电机编码器B相
const int port_Encoder_BL_A = 3;   // 左后电机编码器A相
const int port_Encoder_BL_B = 30;  // 左后电机编码器B相
const int port_Encoder_BR_A = 19;  // 右后电机编码器A相
const int port_Encoder_BR_B = 28;  // 右后电机编码器B相



//机械臂控制接口
const int plat=0;
const int on_plat=1;
const int middle=2;
const int top=3;
const int top_seize=7;


#endif  // PORT_DEF_H