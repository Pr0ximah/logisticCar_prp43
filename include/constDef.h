/**
 * @Pr0ximah
 * @date 23.06.25
 * @file constDef.h
 * @brief 常量定义
*/

// 以下参数可以自行设置
// ===================================================
// 小车行进状态更新时间间隔(ms)
const int MOVE_STATUS_UPDATE_TIME_INTERVAL = 500;

// 小车位移偏差允许的最大值(cm)
const float POS_ERROR_TOLERANCE = 1;

// 初始化imu的采样数据数量
const int IMUINIT_SAMPLE_NUM = 500;

// PID相关
// -------------------------
//允许的误差范围
const float errorTol = 0.2;
// 允许的I项最大值
const float IMax = 20;
// 当P为多大时不再考虑I项
const float IRange = 50;
// -------------------------
// ===================================================

// 以下参数需要测试获得
// ===================================================
// 角加速度单位换算系数(换算为单位: g)
const float ANGLE_ACCE_COEFFICIENT = 65.5;
// ===================================================