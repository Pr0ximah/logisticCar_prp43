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
        motorSpeed[i] = 0;
    }
}

void DriveControl::posInit() { posCur.setXY(0, 0); }

void DriveControl::setTar(Point p) { posTar.setXY(p); }

void DriveControl::setTar(float _x, float _y) { posTar.setXY(_x, _y); }

// void DriveControl::gotoPoint(Point p) {
//     Vector vecToMove = p - posCur;
//     float disTol = POS_ERROR_TOLERANCE;
//     PID pid(0);
//     pid.setCoefficient(1, 0, 0);
//     // 当误差距离不小于阈值时
//     // 移动控制主循环
//     while (vecToMove.getNorm() >= disTol) {
//         if (vecToMove.getNorm() >= 10) {
//             driveByAngle(100, vecToMove.getAngle());
//         } else {
//             int contrlVal = pid.update(vecToMove.getNorm());
//             driveByAngle(contrlVal, vecToMove.getAngle());
//         }
//         // 更新位置
//         statusUpdate();
//         vecToMove = p - posCur;
//         delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
//         // Serial.print(posCur.getX());
//         // Serial.print(" ");
//         // Serial.println(posCur.getY());
//     }
// }

void DriveControl::gotoPoint(Point(p)) {
    Vector mvec = p - posCur;
    moveX(mvec.getX());
    moveY(mvec.getY());
}

void DriveControl::gotoPoint(float x, float y) { gotoPoint(Point(x, y)); }

void DriveControl::gotoTar() { gotoPoint(posTar); }

void DriveControl::statusUpdate() {
    // imuUpdate();
    // posUpdate();
    motorSpeedUpdate();
}

// void DriveControl::posUpdate() {
//     encoders.update();
//     float disWheelFL = encoders.getDisOfWheel(Encoder::L);
//     float disWheelFR = encoders.getDisOfWheel(Encoder::R);
//     Serial.print(posCur.getX());
//     Serial.print(" ");
//     Serial.println(posCur.getY());

//     // heading = atan2(disWheelFR, disWheelFL) - PI / 4;
//     // Serial.print(" ");
//     // Serial.println(heading);
// }

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

// void DriveControl::driveByAngle(float speed_percent, float angleTar) {
//     while (angleTar < -PI) {
//         angleTar += PI * 2;
//     }
//     while (angleTar > PI) {
//         angleTar -= PI * 2;
//     }
//     if (0 <= angleTar && angleTar < PI / 2) {
//         rotateByPercentageFR(speed_percent, motorDir::FWD);
//         rotateByPercentageBL(speed_percent, motorDir::FWD);
//         rotateByPercentageFL(speed_percent * tan(PI / 4 - angleTar),
//         motorDir::FWD); rotateByPercentageBR(speed_percent * tan(PI / 4 -
//         angleTar), motorDir::FWD);
//     } else if (PI / 2 <= angleTar && angleTar <= PI) {
//         rotateByPercentageFL(speed_percent, motorDir::BCK);
//         rotateByPercentageBR(speed_percent, motorDir::BCK);
//         rotateByPercentageFR(speed_percent * tan(3 * PI / 4 - angleTar),
//         motorDir::FWD); rotateByPercentageBL(speed_percent * tan(3 * PI / 4 -
//         angleTar), motorDir::FWD);
//     } else if (-PI / 2 <= angleTar && angleTar < 0) {
//         rotateByPercentageFL(speed_percent, motorDir::FWD);
//         rotateByPercentageBR(speed_percent, motorDir::FWD);
//         rotateByPercentageFR(speed_percent * tan(-PI / 4 - angleTar),
//         motorDir::FWD); rotateByPercentageBL(speed_percent * tan(-PI / 4 -
//         angleTar), motorDir::FWD);
//     } else if (-PI <= angleTar && angleTar < -PI / 2) {
//         rotateByPercentageFR(speed_percent, motorDir::BCK);
//         rotateByPercentageBL(speed_percent, motorDir::BCK);
//         rotateByPercentageFL(speed_percent * tan(-3 * PI / 4 - angleTar),
//         motorDir::FWD); rotateByPercentageBR(speed_percent * tan(-3 * PI / 4
//         - angleTar), motorDir::FWD);
//     }
// }

void DriveControl::stop() {
    for (int i = 0; i < 4; i++) {
        motorSpeed[i] = 0;
    }
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

// void DriveControl::moveX(float tarX) {
//     float curX = 0;
//     float difX = tarX - curX;
//     float PID_THRESHOLD = 10;
//     PID pid(0);
//     pid.setCoefficient(18, 40, 320, POS_ERROR_TOLERANCE);
//     encoders.reset();
//     while (fabs(difX) >= POS_ERROR_TOLERANCE) {
//         if (fabs(difX) >= PID_THRESHOLD) {     // 全转速
//             driveByDir(sign(difX) * 100, dRHT);
//         } else {                    // PID转速
//             float contrlVal = -pid.update(difX);
//             driveByDir(contrlVal, dRHT);
//         }
//         encoders.update();
//         curX = (encoders.getDisOfWheel(Encoder::L) -
//         encoders.getDisOfWheel(Encoder::R)) / 2;
//         // Serial.println(curX);
//         difX = tarX - curX;
//         delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
//     }
//     posCur = posCur + Point(curX, 0);
//     Serial.print("X:");
//     Serial.println(posCur.getX());
// }

// void DriveControl::moveY(float tarY) {
//     float curY = 0;
//     float difY = tarY - curY;
//     float PID_THRESHOLD = 10;
//     PID pid(0);
//     pid.setCoefficient(18, 40, 380, POS_ERROR_TOLERANCE);
//     encoders.reset();
//     while (fabs(difY) >= POS_ERROR_TOLERANCE) {
//         if (fabs(difY) >= PID_THRESHOLD) {     // 全转速
//             driveByDir(sign(difY) * 100, dFWD);
//         } else {                    // PID转速
//             float contrlVal = -pid.update(difY);
//             driveByDir(contrlVal, dFWD);
//         }
//         encoders.update();
//         curY = (encoders.getDisOfWheel(Encoder::L) +
//         encoders.getDisOfWheel(Encoder::R)) / 2;
//         // Serial.println(curY);
//         difY = tarY - curY;
//         delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
//     }
//     posCur = posCur + Point(0, curY);
//     Serial.print("Y:");
//     Serial.println(posCur.getY());
// }

void DriveControl::driveByDir(float speed_percent, driveDir dir) {
    switch (dir) {
        case dFWD:
            rotateByPercentageFL(speed_percent, FWD);
            rotateByPercentageFR(speed_percent, FWD);
            rotateByPercentageBL(speed_percent, FWD);
            rotateByPercentageBR(speed_percent, FWD);
            break;
        case dBCK:
            rotateByPercentageFL(speed_percent, BCK);
            rotateByPercentageFR(speed_percent, BCK);
            rotateByPercentageBL(speed_percent, BCK);
            rotateByPercentageBR(speed_percent, BCK);
            break;
        case dLFT:
            rotateByPercentageFL(speed_percent, BCK);
            rotateByPercentageFR(speed_percent, FWD);
            rotateByPercentageBL(speed_percent, FWD);
            rotateByPercentageBR(speed_percent, BCK);
            break;
        case dRHT:
            rotateByPercentageFL(speed_percent, FWD);
            rotateByPercentageFR(speed_percent, BCK);
            rotateByPercentageBL(speed_percent, BCK);
            rotateByPercentageBR(speed_percent, FWD);
            break;
    }
}

void DriveControl::motorSpeedUpdate() {
    Encoder *eTemp[4] = {&encoders.encoderFL, &encoders.encoderFR, &encoders.encoderFR, &encoders.encoderFL};
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
        eTemp[i]->reset();
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
        // Serial.println(String(vel) + " | " + String(controlVal) + " | " +
        //                String(velTar) + " | " +
        //                String(motorVel2VoltPercent(velTar)));
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
        Serial.println(String(vel) + " | " + String(controlVal) + " | " +
                       String(velTar) + " | " +
                       String(motorVel2VoltPercent(velTar)));
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
        String str = String(volt) + " : " + String(sum / 10);
        Serial.println(str);
    }
    rotateByPercentageFR(0, FWD);
}

double DriveControl::motorVel2VoltPercent(double _vel) {
    double vel = fabs(_vel);
    return (0.0003 * pow(vel, 2) - 0.1494 * vel + 37.8040) * (sign(_vel));
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
//     int16_t temp1, temp2, temp3, temp4, temp5;  // 不使用这些变量，仅用于imu函数传参
//     imu.getMotion6(&temp1, &temp2, &temp3, &temp4, &temp5, &angleAcceOri); //获取三轴加速度和三轴角速度初始值
//     return angleAcceOri / ANGLE_ACCE_COEFFICIENT - angleAcce0_bias;
// }
void DriveControl::move(float speed,Vector vec){
    float trans_Angle=vec.getAngle()+PI/4;
    double k=1;
    double v_1,v_2,v_3,v_4;//1-左前,2-右前,3-左后,4-右后 左前和右后一直保持一致 右前和左后一直保持一致
    v_1=speed*cos(trans_Angle);
    v_4=speed*cos(trans_Angle);
    v_2=speed*sin(trans_Angle);
    v_3=speed*sin(trans_Angle);

    if(fabs(v_1)>100 || fabs(v_2)>100){
        k=(fabs(v_1)>fabs(v_2))? 100/v_1 :100/v_2;
    }
    motorSpeed[0] = v_1*k;
    motorSpeed[1] = v_2*k;
    motorSpeed[2] = v_3*k;
    motorSpeed[3] = v_4*k;
    String str;
    for (int i = 0; i < 4; i++) {
        str += motorSpeed[i];
        str += " | ";
    }
    Serial.println(str);
    while (true) {
        statusUpdate();
        delay(MOVE_STATUS_UPDATE_TIME_INTERVAL);
    }
}

void DriveControl::rotate(float speed,float theta){

}
