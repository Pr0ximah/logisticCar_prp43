#include "drive.h"
#include <Arduino.h>
#include <math.h>

double Encoder_FL_Coefficient = 799;
double Encoder_FR_Coefficient = 799;

// 初始化设置
DriveControl::DriveControl() : 
    encoder_FL(port_Encoder_FL_A, port_Encoder_FL_B, Encoder_FL_Coefficient), 
    encoder_FR(port_Encoder_FR_A, port_Encoder_FR_B, Encoder_FR_Coefficient)
{
    // IMU禁用
    // // imu初始化
    // imuInit();

    // pos初始化
    posInit();

    // 电机端口初始化
    pinMode(port_motor_FL, OUTPUT);
    pinMode(port_motor_FR, OUTPUT);
    pinMode(port_motor_BL, OUTPUT);
    pinMode(port_motor_BR, OUTPUT);
}

void DriveControl::posInit() {
    posCur.setXY(0, 0);
}

void DriveControl::setTar(Point p) {
    posTar.setXY(p);
}

void DriveControl::setTar(float _x, float _y) {
    posTar.setXY(_x, _y);
}

void DriveControl::gotoPoint(Point p) {
    Vector vecToMove = p - posCur;
    float disTol = POS_ERROR_TOLERANCE;
    PID pid(0);
    pid.setCoefficient(1, 0, 0);
    // 当误差距离不小于阈值时
    // 移动控制主循环
    while (vecToMove.getNorm() >= disTol) {
        if (vecToMove.getNorm() >= 10) {
            driveByAngle(100, vecToMove.getAngle());
        } else {
            int contrlVal = pid.update(vecToMove.getNorm());
            driveByAngle(contrlVal, vecToMove.getAngle());
        }
        // 更新位置
        statusUpdate();
        vecToMove = p - posCur;
        delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
        // Serial.print(posCur.getX());
        // Serial.print(" ");
        // Serial.println(posCur.getY());
    }
}

void DriveControl::gotoPoint(float x, float y) {
    gotoPoint(Point(x, y));
}

void DriveControl::gotoTar() {
    gotoPoint(posTar);
}

void DriveControl::statusUpdate() {
    // imuUpdate();
    posUpdate();
}

void DriveControl::posUpdate() {
    encoder_FL.update();
    float disWheelFL = encoder_FL.getAngle();
    encoder_FR.update();
    float disWheelFR = encoder_FR.getAngle();
    // Serial.print(posCur.getX());
    Serial.print(disWheelFL);
    Serial.print(" ");
    // Serial.print(posCur.getY());
    Serial.print(disWheelFR);
    posCur = Point(disWheelFL - disWheelFR, disWheelFL + disWheelFR);
    heading = atan2(disWheelFR, disWheelFL) - PI / 4;
    Serial.print(" ");
    Serial.println(heading);
}

// void DriveControl::forward(float controlVal) {
//     // 电机控制
//     // -----------------
//     float controlArr[4] = {0, 0, 0, 0};
//     if (controlVal > 0) {  // 右偏
//         controlArr[0] = -controlVal;
//         controlArr[2] = -controlVal;
//     } else if (controlVal < 0) {  // 左偏
//         controlArr[1] = controlVal;
//         controlArr[3] = controlVal;
//     }
//     analogWrite(port_motor_FL, 254 + controlArr[0]);
//     analogWrite(port_motor_FR, 254 + controlArr[1]);
//     analogWrite(port_motor_BL, 254 + controlArr[2]);
//     analogWrite(port_motor_BR, 254 + controlArr[3]);
//     // -----------------
// }

void DriveControl::driveByAngle(float speed_percent, float angleTar) {
    while (angleTar < -PI) {
        angleTar += PI * 2;
    }
    while (angleTar > PI) {
        angleTar -= PI * 2;
    }
    if (0 <= angleTar && angleTar < PI / 2) {
        rotateByPercentageFR(speed_percent, motorDir::FWD);
        rotateByPercentageBL(speed_percent, motorDir::FWD);
        rotateByPercentageFL(speed_percent * tan(PI / 4 - angleTar), motorDir::FWD);
        rotateByPercentageBR(speed_percent * tan(PI / 4 - angleTar), motorDir::FWD);
    } else if (PI / 2 <= angleTar && angleTar <= PI) {
        rotateByPercentageFL(speed_percent, motorDir::BCK);
        rotateByPercentageBR(speed_percent, motorDir::BCK);
        rotateByPercentageFR(speed_percent * tan(3 * PI / 4 - angleTar), motorDir::FWD);
        rotateByPercentageBL(speed_percent * tan(3 * PI / 4 - angleTar), motorDir::FWD);
    } else if (-PI / 2 <= angleTar && angleTar < 0) {
        rotateByPercentageFL(speed_percent, motorDir::FWD);
        rotateByPercentageBR(speed_percent, motorDir::FWD);
        rotateByPercentageFR(speed_percent * tan(-PI / 4 - angleTar), motorDir::FWD);
        rotateByPercentageBL(speed_percent * tan(-PI / 4 - angleTar), motorDir::FWD);
    } else if (-PI <= angleTar && angleTar < -PI / 2) {
        rotateByPercentageFR(speed_percent, motorDir::BCK);
        rotateByPercentageBL(speed_percent, motorDir::BCK);
        rotateByPercentageFL(speed_percent * tan(-3 * PI / 4 - angleTar), motorDir::FWD);
        rotateByPercentageBR(speed_percent * tan(-3 * PI / 4 - angleTar), motorDir::FWD);
    }
}

void DriveControl::stop() {
    analogWrite(port_motor_FL, 0);
    analogWrite(port_motor_FR, 0);
    analogWrite(port_motor_BL, 0);
    analogWrite(port_motor_BR, 0);
}

// float DriveControl::imuUpdate() {
//     static bool firstTime_flag = true;
//     static int updateTimeCur;
//     static int updateTimeLast;
//     int imuUpdateTimeInterval;  // 实际采样时间间隔
//     if (firstTime_flag) { //若第一次，则不改变角度值
//         firstTime_flag = false;
//         updateTimeLast = millis();
//         return angle;
//     } else {
//         float angleAcce;
//         updateTimeCur = millis();
//         angleAcce = imuReadAngleAcce();
//         // 如果超过了允许的最大值则设置为最大值
//         if (angleAcce > ANGLE_ACCE_TOL) {
//             angleAcce = ANGLE_ACCE_TOL;
//         }
//         imuUpdateTimeInterval = updateTimeCur - updateTimeLast;
//         updateTimeLast = updateTimeCur;
//         angle += angleAcce * imuUpdateTimeInterval / 1000;
//     }
//     return angle;
// }

// void DriveControl::posUpdate() {
//
// }

// void DriveControl::posUpdate_IR() {
//     for (int i = 0; i < 4; i++) {  // 设置disLast
//         disLast[i] = disCur[i];
//     }
//     for (int i = 0; i < 4; i++) {
//         disCur[i] = IRArray[i].IR_ReadDis();
//     }
//     float deltaDisTemp[4];  // 临时存储现在的距离差值(若偏差过大会被置为-999)
//     for (int i = 0; i < 4; i++) {
//         if (fabs(disCur[i] - disLast[i]) > IR_DATA_TOLERANCE) {
//             deltaDisTemp[i] = -999;
//         } else {
//             deltaDisTemp[i] = disCur[i] - disLast[i];
//         }
//     }
//     // 设置y坐标
//     // -------------------------------
//     float deltaY;  // Y值变化
//     float tempY;  // 取平均值求和的临时变量
//     for (int i = 0; i < 2; i++) {
//         if (deltaDisTemp[i] == -999) { //该值无效
//             continue;
//         } else {
//             tempY += sign(deltaDisTemp[0]) * fabs(deltaDisTemp[i]);
//         }
//     }
//     tempY /= 2;
//     deltaY = -1 * tempY;
//     // -------------------------------
//     // 设置x坐标
//     // -------------------------------
//     float deltaX;  // X值变化
//     float tempX;  // 取平均值求和的临时变量
//     for (int i = 2; i < 4; i++) {
//         if (deltaDisTemp[i] == -999) { //该值无效
//             continue;
//         } else {
//             tempX += sign(deltaDisTemp[2]) * fabs(deltaDisTemp[i]);
//         }
//     }
//     tempX /= 2;
//     deltaX = tempX;
//     // -------------------------------
//     posCur = posCur + Vector(deltaX, deltaY);
// }

void DriveControl::rotateByPercentageFL(double percent, motorDir dir) {
    if (dir == motorDir::BCK) {
        percent = -percent;
    }
    if (fabs(percent) > 100) {
        percent = sign(percent) * 100;
    }
    if (sign(percent) == -1) {
        digitalWrite(port_dir_FL, LOW);
        analogWrite(port_motor_FL, -255 * percent / 100);
    } else {
        digitalWrite(port_dir_FL, HIGH);
        analogWrite(port_motor_FL, 255 * percent / 100);
    }
}

void DriveControl::rotateByPercentageFR(double percent, motorDir dir) {
    if (dir == motorDir::BCK) {
        percent = -percent;
    }
    if (fabs(percent) > 100) {
        percent = sign(percent) * 100;
    }
    if (sign(percent) == -1) {
        digitalWrite(port_dir_FR, LOW);
        analogWrite(port_motor_FR, -255 * percent / 100);
    } else {
        digitalWrite(port_dir_FR, HIGH);
        analogWrite(port_motor_FR, 255 * percent / 100);
    }
}

void DriveControl::rotateByPercentageBL(double percent, motorDir dir) {
    if (dir == motorDir::BCK) {
        percent = -percent;
    }
    if (fabs(percent) > 100) {
        percent = sign(percent) * 100;
    }
    if (sign(percent) == -1) {
        digitalWrite(port_dir_BL, LOW);
        analogWrite(port_motor_BL, -255 * percent / 100);
    } else {
        digitalWrite(port_dir_BL, HIGH);
        analogWrite(port_motor_BL, 255 * percent / 100);
    }
    
}

void DriveControl::rotateByPercentageBR(double percent, motorDir dir) {
    if (dir == motorDir::BCK) {
        percent = -percent;
    }
    if (fabs(percent) > 100) {
        percent = sign(percent) * 100;
    }
    if (sign(percent) == -1) {
        digitalWrite(port_dir_BR, LOW);
        analogWrite(port_motor_BR, -255 * percent / 100);
    } else {
        digitalWrite(port_dir_BR, HIGH);
        analogWrite(port_motor_BR, 255 * percent / 100);
    }
    
}

// 红外模块已禁用
// ------------
// void DriveControl::posInit() {
//     // 红外模块引脚初始化
//     // -----------------------
//     pinMode(port_IR_F, INPUT);
//     pinMode(port_IR_B, INPUT);
//     pinMode(port_IR_L, INPUT);
//     pinMode(port_IR_R, INPUT);
//     // -----------------------
//     for (int i = 0; i < 4; i++) {
//         float valueSum = 0;
//         for (int j = 0; j < 3; j++) {  // 测三次取平均值初始化
//             valueSum += IRArray[i].IR_ReadDis();
//             delay(100);
//         }
//         disOri[i] = valueSum / 3;
//         disCur[i] = disOri[i];  // 初始化一开始的位置
//     }
// }

// IMU已禁用
// --------
// void DriveControl::imuInit() {
//     float angleAcce0_sum = 0;       //初始化静态偏移量和变量
//     int16_t angleAcceOri;  // 角加速度原始数值
//     int16_t temp1, temp2, temp3, temp4, temp5;  // 不使用这些变量，仅用于imu函数传参
//     Serial.println("IMU initializing starts...");
//     delay(1500);                    //等待车辆静止
//     for (int i = 0; i < IMUINIT_SAMPLE_NUM; i++) {
//         imu.getMotion6(&temp1, &temp2, &temp3, &temp4, &temp5, &angleAcceOri); //获取三轴加速度和三轴角速度初始值
//         float angleAcce0 = angleAcceOri / ANGLE_ACCE_COEFFICIENT;
//         angleAcce0_sum += angleAcce0;
//         delay(10);
//     }
//     angleAcce0_bias = angleAcce0_sum / IMUINIT_SAMPLE_NUM;
//     Serial.println("IMU initialize finished!");
// }

// float DriveControl::imuReadAngleAcce() {
//     int16_t angleAcceOri;  // 角加速度原始数值
//     int16_t temp1, temp2, temp3, temp4, temp5;  // 不使用这些变量，仅用于imu函数传参
//     imu.getMotion6(&temp1, &temp2, &temp3, &temp4, &temp5, &angleAcceOri); //获取三轴加速度和三轴角速度初始值
//     return angleAcceOri / ANGLE_ACCE_COEFFICIENT - angleAcce0_bias;
// }