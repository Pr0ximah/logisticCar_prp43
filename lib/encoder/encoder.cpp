#include "encoder.h"

#include <Arduino.h>

#include "constDef.h"
#include "portDef.h"

Encoder EncoderSet::encoderFL(port_Encoder_FL_A, port_Encoder_FL_B,
                              Encoder_FL_Coefficient);
Encoder EncoderSet::encoderFR(port_Encoder_FR_A, port_Encoder_FR_B,
                              Encoder_FR_Coefficient);
Encoder *EncoderSet::PtEncoderFL = nullptr;
Encoder *EncoderSet::PtEncoderFR = nullptr;

EncoderSet::EncoderSet() {
    PtEncoderFL = &encoderFL;
    PtEncoderFR = &encoderFR;
    attachInterrupt(encoderFL.ISR_Port, updateFL, CHANGE);
    attachInterrupt(encoderFR.ISR_Port, updateFR, CHANGE);
}

void EncoderSet::updateFL() { PtEncoderFL->updateCount(); }

void EncoderSet::updateFR() { PtEncoderFR->updateCount(); }

Encoder::Encoder(int _portA, int _portB, int _coefficient)
    : COEFFICIENT_PER_ROUND(_coefficient) {
    portA = _portA;
    portB = _portB;
    if (portA == 2) {
        ISR_Port = 0;
    } else {
        ISR_Port = 1;
    }
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

float Encoder::getAngle() const {
    return double(pulseCount) / COEFFICIENT_PER_ROUND * 360;
}

float Encoder::getAbsoluteAngle() const { return numRound * 360 + getAngle(); }

int Encoder::getRound() const { return numRound; }

float Encoder::getDisOfWheel() const {
    return PI * WHEEL_DIAMETER * getAbsoluteAngle() / 360;
}

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

    if (countOfUpdate % 3 == 0) {
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

void Encoder::updateCount() {
    cli();
    if (digitalRead(portA) == LOW) {  // 下降沿
        if (digitalRead(portB) == LOW) {
            pulseCount--;
        } else {
            pulseCount++;
        }
    } else {
        if (digitalRead(portB) == LOW) {  // 上升沿
            pulseCount++;
        } else {
            pulseCount--;
        }
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