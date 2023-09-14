#include "drive.h"

#include <Arduino.h>
#include <math.h>

// 初始化设置
DriveControl::DriveControl() {
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

    // 电机转速归零
    for (int i = 0; i < 4; i++) {
        motorSpeed[i] = 0;
    }
}

void DriveControl::posInit() { posCur.setXY(0, 0); }

void DriveControl::setTar(Point p) { posTar.setXY(p); }

void DriveControl::setTar(float _x, float _y) { posTar.setXY(_x, _y); }

void DriveControl::gotoPoint(Point(p)) { return; }

void DriveControl::gotoPoint(float x, float y) { gotoPoint(Point(x, y)); }

void DriveControl::gotoTar() { gotoPoint(posTar); }

void DriveControl::statusUpdate() {
    // imuUpdate();
    posUpdate();
    motorSpeedUpdate();
}

void DriveControl::driveByAngle(float speed, float dirAngle) {
    float trans_Angle = dirAngle + PI / 4;
    double k = 1;
    double v_1, v_2, v_3, v_4;  // 1-左前,2-右前,3-左后,4-右后
                                // 左前和右后一直保持一致 右前和左后一直保持一致
    v_1 = speed * cos(trans_Angle);
    v_4 = speed * cos(trans_Angle);
    v_2 = speed * sin(trans_Angle);
    v_3 = speed * sin(trans_Angle);
    // 归一化处理
    if (fabs(v_1) > 100 || fabs(v_2) > 100) {
        k = (fabs(v_1) > fabs(v_2)) ? 100 / v_1 : 100 / v_2;
    }
    motorSpeed[0] = v_1 * k;
    motorSpeed[1] = v_2 * k;
    motorSpeed[2] = v_3 * k;
    motorSpeed[3] = v_4 * k;
    motorSpeedUpdate();
}

void DriveControl::rotateByDir(float speed, bool isClockwise) {
    double k = 1;
    double v_1, v_2, v_3, v_4;  // 1-左前,2-右前,3-左后,4-右后
                                // 左前和左后一致 右前和右后一致
    int signR = isClockwise ? -1 : 1;
    v_1 = -speed * signR;
    v_2 = speed * signR;
    v_3 = -speed * signR;
    v_4 = speed * signR;
    // 归一化处理
    if (fabs(v_1) > 100 || fabs(v_2) > 100) {
        k = (fabs(v_1) > fabs(v_2)) ? 100 / v_1 : 100 / v_2;
    }
    motorSpeed[0] = v_1 * k;
    motorSpeed[1] = v_2 * k;
    motorSpeed[2] = v_3 * k;
    motorSpeed[3] = v_4 * k;
    motorSpeedUpdate();
}

void DriveControl::stop() {
    for (int i = 0; i < 4; i++) {
        motorSpeed[i] = 0;
    }
}

void DriveControl::posUpdate() {
    float disWheel[4] = {encoders.encoderFL.getDisOfWheel(),
                         -encoders.encoderFR.getDisOfWheel(),
                         encoders.encoderFR.getDisOfWheel(),
                         -encoders.encoderFL.getDisOfWheel()};
    posCur.setXY((disWheel[0] - disWheel[1]) / 2,
                 (disWheel[0] + disWheel[1]) / 2);
    // Serial.println(String(posCur.getX()) + " | " + String(posCur.getY()));
}

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

void DriveControl::motorSpeedUpdate() {
    Encoder *eTemp[4] = {&encoders.encoderFL, &encoders.encoderFR,
                         &encoders.encoderFR, &encoders.encoderFL};
    void (DriveControl::*rotateFunc[4])(double, DriveControl::motorDir) = {
        &rotateByPercentageFL, &rotateByPercentageFR, &rotateByPercentageBL,
        &rotateByPercentageBR};
    for (int i = 0; i < 4; i++) {
        float velPercentTar = motorSpeed[i];
        if (velPercentTar == 0) {
            (this->*rotateFunc[i])(0, FWD);
        }
        float velTar = velPercentTar * MOTOR_MAX_SPEED / 100;
        PID pid(velTar);
        pid.setCoefficient(0.17, 0.006, 0.3, 0);
        eTemp[i]->update();
        double vel = eTemp[i]->getAngleVel();
        if (i % 2) {  // i为奇数，即右侧电机，编码器安装方向相反，需要取负
            vel = -vel;
        }
        double controlVal;
        if (velPercentTar <= 50) {
            if (fabs(vel - velTar) <= 50) {
                controlVal = pid.update(vel) + motorVel2VoltPercent(velTar);
            } else if (fabs(vel) < fabs(velTar)) {
                controlVal = motorVel2VoltPercent(velTar);
                if (fabs(controlVal) < 30) {
                    controlVal = sign(controlVal) * 30;
                }
            } else {
                controlVal = (fabs(motorVel2VoltPercent(velTar)) - 5) *
                             sign(motorVel2VoltPercent(velTar));
            }
        } else {
            if (fabs(vel - velTar) <= 1000) {
                controlVal = pid.update(vel) + motorVel2VoltPercent(velTar);
            } else {
                controlVal = sign(velTar) * 70;
            }
        }
        if (sign(controlVal) != sign(velTar)) {
            controlVal = 0;
        }
        (this->*rotateFunc[i])(controlVal, FWD);
    }
}

void DriveControl::motorPIDTest() {
    int velPercentTar = -30;
    float velTar = velPercentTar * MOTOR_MAX_SPEED / 100;
    PID pid(velTar);
    pid.setIRange(50);
    pid.setCoefficient(0.17, 0.006, 0.3, 0);
    encoders.encoderFL.reset();
    while (true) {
        encoders.encoderFL.update();
        double vel = encoders.encoderFL.getAngleVel();
        double controlVal;
        if (velPercentTar <= 50) {
            if (fabs(vel - velTar) <= 50) {
                controlVal = pid.update(vel) + motorVel2VoltPercent(velTar);
            } else if (fabs(vel) < fabs(velTar)) {
                controlVal = motorVel2VoltPercent(velTar);
                if (fabs(controlVal) < 30) {
                    controlVal = sign(controlVal) * 30;
                }
            } else {
                controlVal = (fabs(motorVel2VoltPercent(velTar)) - 5) *
                             sign(motorVel2VoltPercent(velTar));
            }
        } else {
            if (fabs(vel - velTar) <= 1000) {
                controlVal = pid.update(vel) + motorVel2VoltPercent(velTar);
            } else {
                controlVal = sign(velTar) * 70;
            }
        }
        if (sign(controlVal) != sign(velTar)) {
            controlVal = 0;
        }
        rotateByPercentageFR(controlVal, FWD);
        rotateByPercentageFL(70, FWD);
        rotateByPercentageBR(70, FWD);
        rotateByPercentageBL(controlVal, FWD);
        delay(10);
    }
}

void DriveControl::motorVoltVelTest() {
    for (int volt = 0; volt <= 100; volt += 5) {
        rotateByPercentageFR(volt, FWD);
        delay(5000);
        int numSample = 0;
        float sample[10];
        for (int i = 0; i < 10; i++) {
            sample[i] = 0;
        }
        int temp = 0;
        while (numSample < 10) {
            encoders.encoderFL.update();
            temp++;
            if (temp % 50 == 0) {
                sample[numSample] = encoders.encoderFL.getAngleVel();
                numSample++;
            }
            delay(10);
        }
        float sum = 0;
        for (int i = 0; i < 10; i++) {
            sum += sample[i];
        }
    }
    rotateByPercentageFR(0, FWD);
}

double DriveControl::motorVel2VoltPercent(double _vel) {
    double vel = fabs(_vel);
    return (0.0003 * pow(vel, 2) - 0.1494 * vel + 37.8040) * (sign(_vel));
}

void DriveControl::move(float speed, Vector vec) {
    if (speed < 0) {  // speed为负则反向
        vec = vec * (-1);
    }

    // 编码器(定位用)初始化
    encoders.encoderFL.reset();
    encoders.encoderFR.reset();

    Point pTar = posCur + vec;
    Vector vecToMove = pTar - posCur;
    float disTol = POS_ERROR_TOLERANCE;
    PID pid(0);
    pid.setCoefficient(1, 0, 0, POS_ERROR_TOLERANCE);
    // 当误差距离不小于阈值时
    // 移动控制主循环
    while (vecToMove.getNorm() >= disTol) {
        if (vecToMove.getNorm() >= 10) {
            driveByAngle(speed, vecToMove.getAngle());
        } else {
            int controlVal = -pid.update(vecToMove.getNorm());
            if (controlVal >= 30) {
                controlVal = 30;
            }
            driveByAngle(controlVal, vecToMove.getAngle());
        }
        // 更新位置&电机速度
        statusUpdate();
        vecToMove = pTar - posCur;
        delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
    }
}

void DriveControl::rotate(float speed, float angleTar) {
    // 目标角度归一化到[0, 2*pi)
    while (angleTar >= 2 * PI) {
        angleTar -= 2 * PI;
    }
    while (angleTar < 0) {
        angleTar += 2 * PI;
    }
    // 编码器(定位用)初始化
    encoders.encoderFL.reset();
    encoders.encoderFR.reset();
    // 已转角度
    double angleCur = 0;

    double angleToRotate = angleTar - angleCur;
    float angleTol = ANGLE_ERROR_TOLERANCE;
    PID pid(angleTar);
    pid.setCoefficient(1, 0, 0, ANGLE_ERROR_TOLERANCE);
    // 当误差距离不小于阈值时
    // 旋转控制主循环
    while (angleToRotate >= angleTol) {
        if (angleToRotate >= 10) {
            rotateByDir(speed, false);
        } else {
            int controlVal = pid.update(angleCur);
            if (controlVal >= 20) {
                controlVal = 20;
            }
            rotateByDir(controlVal, false);
        }
        // 更新电机速度
        motorSpeedUpdate();
        // 更新朝向角度
        float disWheel[4] = {encoders.encoderFL.getDisOfWheel(),
                             -encoders.encoderFR.getDisOfWheel(),
                             encoders.encoderFR.getDisOfWheel(),
                             -encoders.encoderFL.getDisOfWheel()};
        angleCur +=
            -(disWheel[0] + disWheel[1] + disWheel[2] + disWheel[3]) / 4;

        angleToRotate = angleTar - angleCur;
        delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
    }
}

// IMU已禁用
// --------------------------------------------------------------------------------
// void DriveControl::imuInit() {
//     float angleAcce0_sum = 0;       //初始化静态偏移量和变量
//     int16_t angleAcceOri;  // 角加速度原始数值
//     int16_t temp1, temp2, temp3, temp4, temp5;  //
//     不使用这些变量，仅用于imu函数传参 Serial.println("IMU initializing
//     starts..."); delay(1500);                    //等待车辆静止 for (int i =
//     0; i < IMUINIT_SAMPLE_NUM; i++) {
//         imu.getMotion6(&temp1, &temp2, &temp3, &temp4, &temp5,
//         &angleAcceOri); //获取三轴加速度和三轴角速度初始值 float angleAcce0 =
//         angleAcceOri / ANGLE_ACCE_COEFFICIENT; angleAcce0_sum += angleAcce0;
//         delay(10);
//     }
//     angleAcce0_bias = angleAcce0_sum / IMUINIT_SAMPLE_NUM;
//     Serial.println("IMU initialize finished!");
// }

// float DriveControl::imuReadAngleAcce() {
//     int16_t angleAcceOri;  // 角加速度原始数值
//     int16_t temp1, temp2, temp3, temp4, temp5;  //
//     不使用这些变量，仅用于imu函数传参 imu.getMotion6(&temp1, &temp2, &temp3,
//     &temp4, &temp5, &angleAcceOri); //获取三轴加速度和三轴角速度初始值 return
//     angleAcceOri / ANGLE_ACCE_COEFFICIENT - angleAcce0_bias;
// }
//     int16_t temp1, temp2, temp3, temp4, temp5;  //
//     不使用这些变量，仅用于imu函数传参 imu.getMotion6(&temp1, &temp2, &temp3,
//     &temp4, &temp5, &angleAcceOri); //获取三轴加速度和三轴角速度初始值 return
//     angleAcceOri / ANGLE_ACCE_COEFFICIENT - angleAcce0_bias;
// }

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
// --------------------------------------------------------------------------------