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

    for (int i = 0; i < 4; i++) {
        // 电机转速归零
        motorSpeed[i] = 0;
        // 控制量及PID初始化
        motorControlVal[i] = 0;
        motorPid[i].setCoefficient(1.4, 0.1, 11);
    }
}

void DriveControl::posInit() { posCur.setXY(0, 0); }

void DriveControl::gotoPoint(Point p) { return; }

void DriveControl::gotoPoint(float x, float y) { gotoPoint(Point(x, y)); }

void DriveControl::statusUpdate() {
    // imuUpdate();
    posUpdate();
    motorSpeedUpdate();
}

void DriveControl::driveByAngle(float speed, float dirAngle) {
    float trans_Angle = dirAngle + PI / 4;  // 角度换算
    double k = 1;                           // 归一化系数
    double v_1, v_2, v_3, v_4;              // 1-左前,2-右前,3-左后,4-右后
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
    // motorSpeedUpdate();
}

void DriveControl::rotateByDir(float speed) {
    double k = 1;
    double v_1, v_2, v_3, v_4;  // 1-左前,2-右前,3-左后,4-右后
                                // 左前和左后一致 右前和右后一致
    v_1 = -speed;
    v_2 = speed;
    v_3 = -speed;
    v_4 = speed;
    // 归一化处理
    if (fabs(v_1) > 100 || fabs(v_2) > 100) {
        k = (fabs(v_1) > fabs(v_2)) ? 100 / v_1 : 100 / v_2;
    }
    motorSpeed[0] = v_1 * k;
    motorSpeed[1] = v_2 * k;
    motorSpeed[2] = v_3 * k;
    motorSpeed[3] = v_4 * k;
    // motorSpeedUpdate();
}

void DriveControl::stop() {
    for (int i = 0; i < 4; i++) {
        motorSpeed[i] = 0;
    }
    // motorSpeedUpdate();
}

void DriveControl::posUpdate() {
    float disWheel[4] = {encoders.encoderFL.getDisOfWheel(), encoders.encoderFR.getDisOfWheel(),
                         encoders.encoderBL.getDisOfWheel(), encoders.encoderBR.getDisOfWheel()};
    posCur.setXY((disWheel[0] - disWheel[1] - disWheel[2] + disWheel[3]) / 4,
                 (disWheel[0] + disWheel[1] + disWheel[2] + disWheel[3]) / 4);
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
        digitalWrite(port_dir_FL, HIGH);
        analogWrite(port_motor_FL, -255 * percent / 100);
    } else {
        digitalWrite(port_dir_FL, LOW);
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
        digitalWrite(port_dir_BL, HIGH);
        analogWrite(port_motor_BL, -255 * percent / 100);
    } else {
        digitalWrite(port_dir_BL, LOW);
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
    Encoder *eTemp[4] = {&encoders.encoderFL, &encoders.encoderFR, &encoders.encoderBL, &encoders.encoderBR};
    void (DriveControl::*rotateFunc[4])(double, DriveControl::motorDir) = {
        &rotateByPercentageFL, &rotateByPercentageFR, &rotateByPercentageBL, &rotateByPercentageBR};

    for (int i = 0; i < 4; i++) {
        float velPercentTar = motorSpeed[i];
        if (fabs(velPercentTar) <= 0.1) {
            (this->*rotateFunc[i])(0, FWD);
            continue;
        }
        float velTar = motorSpeed[i] * MOTOR_MAX_SPEED / 100;
        motorPid[i].setTar(velTar);
        eTemp[i]->update(); 
        double vel = eTemp[i]->getAngleVel();
        motorControlVal[i] += motorPid[i].update(vel);
        (this->*rotateFunc[i])(motorControlVal[i], FWD);

        // 以上部分仅为测试，实际效果非常差劲，到位后u/controlVal会震荡，控制量突变，考虑是kd导致，具体原因待研究

        // if (velPercentTar <= 50) {
        //     if (fabs(vel - velTar) <= 50) {
        //         controlVal = pid.update(vel) + motorVelToVoltPercent(velTar);
        //     } else if (fabs(vel) < fabs(velTar)) {
        //         controlVal = motorVelToVoltPercent(velTar);
        //         if (fabs(controlVal) < 30) {
        //             controlVal = sign(controlVal) * 30;
        //         }
        //     } else {
        //         controlVal = (fabs(motorVelToVoltPercent(velTar)) - 5) * sign(motorVelToVoltPercent(velTar));
        //     }
        // } else {
        //     if (fabs(vel - velTar) <= 30) {
        //         controlVal = pid.update(vel) + motorVelToVoltPercent(velTar);
        //     } else {
        //         controlVal = sign(velTar) * 70;
        //     }
        // }
        // if (sign(controlVal) != sign(velTar)) {
        //     controlVal = 0;
        // }
        // (this->*rotateFunc[i])(controlVal, FWD);
        Serial.print(String(i) + "controlVal: " + motorControlVal[i] + " _ " + "tar: " + String(velTar) + " | ");
    }
    Serial.println();
}

void DriveControl::motorPIDTest(int i) {
    Encoder *eTemp[4] = {&encoders.encoderFL, &encoders.encoderFR, &encoders.encoderBL, &encoders.encoderBR};
    void (DriveControl::*rotateFuncs[4])(double, DriveControl::motorDir) = {
        &rotateByPercentageFL, &rotateByPercentageFR, &rotateByPercentageBL, &rotateByPercentageBR};

    Encoder *enc = eTemp[i % 4];
    void (DriveControl::*rotateFunc)(double, DriveControl::motorDir) = rotateFuncs[i % 4];
    int velPercentTar = 100;
    float velTar = velPercentTar * MOTOR_MAX_SPEED / 100;

    struct velPID  // 不退出的增量式PID
    {
        float kp, ki, kd, uTol = 50;
        float error[3] = {0, 0, 0};
        float u = 0;
        float tar;

        float update(float input) {
            error[0] = error[1];
            error[1] = error[2];
            error[2] = tar - input;
            double ki_temp = ki;  // 误差稳定后加ki
            if (fabs(error[2] - error[1]) > 5) {
                ki_temp = 0;
            }
            u = kp * (error[2] - error[1]) / 10 + ki_temp * (error[2]) / 10 +
                kd * (error[2] - 2 * error[1] + error[0]) / 10;
            return u;
        }

        void setCoefficient(float _kp, float _ki, float _kd, float _uTol = 50) {
            kp = _kp;
            ki = _ki;
            kd = _kd;
            uTol = _uTol;
        }

        void setTar(float _tar) { tar = _tar; }
    } pid;

    pid.setCoefficient(1.4, 0.1, 11);
    // pid.setCoefficient(1.4, 0.1, 4);     // 空转稳定参数
    pid.setTar(velTar);
    enc->reset();
    double controlVal = 0;
    while (true) {
        enc->update();
        double vel = enc->getAngleVel();

        // if (fabs(vel - velTar) > pidEnterVelDiffThreshold) {
        //     controlVal = ((vel - velTar < 0) ? 100 : 0);
        // } else {
        controlVal += pid.update(vel);
        // }

        // // 原PID
        // if (velPercentTar <= 50) {
        //     if (fabs(vel - velTar) <= 50) {
        //         controlVal = pid.update(vel) + motorVelToVoltPercent(velTar);
        //     } else if (fabs(vel) < fabs(velTar)) {
        //         controlVal = motorVelToVoltPercent(velTar);
        //         if (fabs(controlVal) < 30) {
        //             controlVal = sign(controlVal) * 30;
        //         }
        //     } else {
        //         controlVal = (fabs(motorVelToVoltPercent(velTar)) - 5) * sign(motorVelToVoltPercent(velTar));
        //     }
        // } else {
        //     if (fabs(vel - velTar) <= 30) {
        //         controlVal = pid.update(vel) + motorVelToVoltPercent(velTar);
        //     } else {
        //         controlVal = sign(velTar) * 70;
        //     }
        // }

        // if (sign(controlVal) != sign(velTar)) {
        //     controlVal = 0;
        // }

        (this->*rotateFunc)(controlVal, FWD);
        Serial.println(String(velTar) + " | " + String(vel) + " | " + String(controlVal));
        delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
    }
}

void DriveControl::motorVoltVelTest(int i) {
    Encoder *eTemp[4] = {&encoders.encoderFL, &encoders.encoderFR, &encoders.encoderBL, &encoders.encoderBR};
    void (DriveControl::*rotateFunc[4])(double, DriveControl::motorDir) = {
        &rotateByPercentageFL, &rotateByPercentageFR, &rotateByPercentageBL, &rotateByPercentageBR};
    Serial.println(" VOL | ANGLE_VEL");
    Serial.println(" --- | ---------");
    const int SAMPLE_NUM = 10;
    for (int volt = 0; volt <= 100; volt += 5) {
        (this->*rotateFunc[i])(volt, FWD);
        delay(5000);
        int sampleCount = 0;
        float sample[SAMPLE_NUM];
        for (int i = 0; i < SAMPLE_NUM; i++) {
            sample[i] = 0;
        }
        int temp = 0;
        while (sampleCount < SAMPLE_NUM) {
            eTemp[i]->update();
            temp++;
            if (temp % 50 == 0) {
                sample[sampleCount] = eTemp[i]->getAngleVel();
                sampleCount++;
            }
            delay(10);
        }
        float sum = 0;
        for (int i = 0; i < SAMPLE_NUM; i++) {
            sum += sample[i];
        }
        // 输出格式化
        String tempStr = "";
        if (volt < 100) {
            tempStr = " ";
        }
        Serial.println(" " + String(volt) + tempStr + " | " + String(sum / SAMPLE_NUM));
    }
    (this->*rotateFunc[i])(0, FWD);
}

double DriveControl::motorVelToVoltPercent(double _vel) {
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
    encoders.encoderBL.reset();
    encoders.encoderBR.reset();

    Point pTar = Point(0, 0) + vec;
    Vector vecToMove = pTar - posCur;
    float disTol = POS_ERROR_TOLERANCE;
    PID pid(0);
    if (speed > 70) {
        pid.setCoefficient(0.4, 0.00001, 1.9, POS_ERROR_TOLERANCE);
    } else {
        pid.setCoefficient(0.4, 0.00001, 1.9, POS_ERROR_TOLERANCE);
    }
    // 当误差距离不小于阈值时
    // 移动控制主循环
    while (vecToMove.getNorm() >= disTol || !pid.arriveFlag) {
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
        Serial.println(String(posCur.getX()) + ", " + String(posCur.getY()));
    }
    stop();
    delay(MOVE_STATUS_INTERVAL);
}

void DriveControl::rotate(float speed, float angleTar) {
    // 编码器(定位用)初始化
    encoders.encoderFL.reset();
    encoders.encoderFR.reset();
    encoders.encoderBL.reset();
    encoders.encoderBR.reset();
    // 已转角度
    double angleCur = 0;

    double angleToRotate = angleTar - angleCur;
    float angleTol = ANGLE_ERROR_TOLERANCE;
    Serial.println(angleTar);
    PID pid(angleTar);
    pid.setCoefficient(1, 0, 0, ANGLE_ERROR_TOLERANCE);
    // 当误差距离不小于阈值时
    // 旋转控制主循环
    while (fabs(angleToRotate) >= angleTol || !pid.arriveFlag) {
        if (fabs(angleToRotate) >= 10) {
            rotateByDir(speed * sign(angleToRotate));
        } else {
            int controlVal = pid.update(angleCur);
            if (fabs(controlVal) >= 20) {
                controlVal = 20 * sign(controlVal);
            }
            rotateByDir(controlVal);
        }
        // 更新电机速度
        motorSpeedUpdate();
        // 更新朝向角度
        float disWheel[4] = {encoders.encoderFL.getDisOfWheel(), encoders.encoderFR.getDisOfWheel(),
                             encoders.encoderBL.getDisOfWheel(), encoders.encoderBR.getDisOfWheel()};
        angleCur = (-disWheel[0] + disWheel[1] - disWheel[2] + disWheel[3]) * 180 / (4 * PI * DIS_WHEEL_TO_CENTER);

        angleToRotate = angleTar - angleCur;
        Serial.println(angleCur);
        delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
    }
    stop();
    delay(MOVE_STATUS_INTERVAL);
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