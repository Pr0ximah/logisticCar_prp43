#include "encoder.h"

#include <Arduino.h>

#include "constDef.h"
#include "portDef.h"

Encoder EncoderSet::encoderFL(port_Encoder_FL_A, port_Encoder_FL_B, Encoder_FL_Coefficient);
Encoder EncoderSet::encoderFR(port_Encoder_FR_A, port_Encoder_FR_B, Encoder_FR_Coefficient);
Encoder EncoderSet::encoderBL(port_Encoder_BL_A, port_Encoder_BL_B, Encoder_BL_Coefficient);
Encoder EncoderSet::encoderBR(port_Encoder_BR_A, port_Encoder_BR_B, Encoder_BR_Coefficient);
Encoder *EncoderSet::PtEncoderFL = nullptr;
Encoder *EncoderSet::PtEncoderFR = nullptr;
Encoder *EncoderSet::PtEncoderBL = nullptr;
Encoder *EncoderSet::PtEncoderBR = nullptr;

EncoderSet::EncoderSet() {
    PtEncoderFL = &encoderFL;
    PtEncoderFR = &encoderFR;
    PtEncoderBL = &encoderBL;
    PtEncoderBR = &encoderBR;
    attachInterrupt(encoderFL.ISR_Port, updateFL, CHANGE);
    attachInterrupt(encoderFR.ISR_Port, updateFR, CHANGE);
    attachInterrupt(encoderBL.ISR_Port, updateBL, CHANGE);
    attachInterrupt(encoderBR.ISR_Port, updateBR, CHANGE);
}

void EncoderSet::updateFL() { PtEncoderFL->updateCount(false); }

void EncoderSet::updateFR() { PtEncoderFR->updateCount(true); }

void EncoderSet::updateBL() { PtEncoderBL->updateCount(false); }

void EncoderSet::updateBR() { PtEncoderBR->updateCount(true); }

int Encoder::port_to_ISR(int port) {
    switch (port) {
        case 2:
            return 0;
        case 3:
            return 1;
        case 21:
            return 2;
        case 20:
            return 3;
        case 19:
            return 4;
        case 18:
            return 5;
    }
}

Encoder::Encoder(int _portA, int _portB, int _coefficient) : COEFFICIENT_PER_ROUND(_coefficient) {
    portA = _portA;
    portB = _portB;

    ISR_Port = port_to_ISR(portA);
    pulseCount = 0;
    numRound = 0;
    angleLast = 0;
    angleCur = 0;
    countOfUpdate = 0;
    for (int i = 0; i < 3; i++) {
        angleVel[i] = 0;
    }

    // 引脚模式设置
    pinMode(portA, INPUT);
    pinMode(portB, INPUT);
}

void Encoder::reset() {
    pulseCount = 0;
    numRound = 0;
    countOfUpdate = 0;
    for (int i = 0; i < 3; i++) {
        angleVel[i] = 0;
    }
    angleLast = 0;
    angleCur = 0;
}

float Encoder::getAngle() const { return double(pulseCount) / COEFFICIENT_PER_ROUND * 360; }

float Encoder::getAbsoluteAngle() const { return numRound * 360 + getAngle(); }

int Encoder::getRound() const { return numRound; }

float Encoder::getDisOfWheel() const { return PI * WHEEL_DIAMETER * getAbsoluteAngle() / 360; }

void Encoder::update() {
    if (pulseCount <= -COEFFICIENT_PER_ROUND) {
        pulseCount += COEFFICIENT_PER_ROUND;
        numRound--;
    }
    if (pulseCount >= COEFFICIENT_PER_ROUND) {
        pulseCount -= COEFFICIENT_PER_ROUND;
        numRound++;
    }

    if (pulseCount <= -COEFFICIENT_PER_ROUND) {
        pulseCount += COEFFICIENT_PER_ROUND;
        numRound--;
    }
    if (pulseCount >= COEFFICIENT_PER_ROUND) {
        pulseCount -= COEFFICIENT_PER_ROUND;
        numRound++;
    }
    countOfUpdate++;

    if (countOfUpdate % 1 == 0) {
        timeCur = millis();
        angleCur = getAbsoluteAngle();
        if (firstTimeFlag) {
            timeLast = timeCur;
            angleLast = angleCur;
            firstTimeFlag = false;
            return;
        } else {
            int timeInterval = timeCur - timeLast;
            double angleDiff = angleCur - angleLast;
            double angleVelTemp;
            if (timeInterval != 0) {
                angleVelTemp = angleDiff * 1000 / timeInterval;
            } else {
                angleVelTemp = 0;
            }
            angleVel[0] = angleVel[1];
            angleVel[1] = angleVel[2];
            angleVel[2] = angleVelTemp;
            timeLast = timeCur;
            angleLast = angleCur;
        }
    }
}

void Encoder::updateCount(bool R) {
    //  A  B  R   RES
    // --------------
    //  H  L  F    +
    //  H  H  T    +
    //  L  H  F    +
    //  L  L  T    +
    //  L  H  T    -
    //  H  L  T    -
    //  L  L  F    -
    //  H  H  F    -

    cli();
    bool flag;
    flag = ((digitalRead(portA) == digitalRead(portB)) && R) || ((digitalRead(portA) != digitalRead(portB) && !R));
    if (flag) {
        pulseCount++;
    } else {
        pulseCount--;
    }
    sei();
}

void Encoder::testCoefficient() {
    Serial.begin(9600);

    while (true) {
        String cmd = Serial.readString();
        if (cmd == "STOP") {
            break;
        } else if (cmd == "RESET") {
            pulseCount = 0;
            Serial.println("reset done");
        }
        Serial.println(pulseCount);
        delay(10);
    }
}

double Encoder::getAngleVel() const {
    double sum = 0;
    int num = 0;
    for (int i = 0; i < 3; i++) {
        if (angleVel[i] != 0) {
            sum += angleVel[i];
            num++;
        }
    }
    if (num == 0) {
        num = 1;
    }
    return sum / num;
}