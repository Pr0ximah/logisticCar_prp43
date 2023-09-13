#include <Arduino.h>
#include "encoder.h"
#include "constDef.h"

Encoder *Encoder::_ISRPointer = nullptr;

Encoder::Encoder(int _portA_L, int _portB_L, int _portA_R, int _portB_R, int _coefficient_L, int _coefficient_R) : COEFFICIENT_PER_ROUND_L(_coefficient_L), COEFFICIENT_PER_ROUND_R(_coefficient_R)
{
    portA_L = _portA_L;
    portB_L = _portB_L;
    portA_R = _portA_R;
    portB_R = _portB_R;
    if (portA_L == 2) {
        ISR_PortL = 0;
        ISR_PortR = 1;
    } else {
        ISR_PortL = 1;
        ISR_PortR = 0;
    }
    pulseCountL = 0;
    pulseCountR = 0;
    numRoundL = 0;
    numRoundR = 0;
    // angleVel[3];
    angleLast = 0;
    angleCur = 0;
    countOfUpdate = 0;

    // 引脚模式设置
    pinMode(portA_L, INPUT);
    pinMode(portB_L, INPUT);
    pinMode(portA_R, INPUT);
    pinMode(portB_R, INPUT);

    _ISRPointer = this;
}

void Encoder::reset() {
    pulseCountL = 0;
    pulseCountR = 0;
    numRoundL = 0;
    numRoundR = 0;
    countOfUpdate = 0;
    for (int i = 0; i < 3; i++) {
        angleVel[i] = 0;
    }
    angleLast = 0;
    angleCur = 0;
}

float Encoder::getAngle(Encoder::side side) const {
    switch (side) {
        case Encoder::L:
            return double(pulseCountL) / COEFFICIENT_PER_ROUND_L * 360;
        case Encoder::R:
            return double(pulseCountR) / COEFFICIENT_PER_ROUND_R * 360;
        default:
            return -1;
    }
}

float Encoder::getAbsoluteAngle(Encoder::side side) const {
    switch (side) {
        case Encoder::L:
            return numRoundL * 360 + getAngle(Encoder::L);
        case Encoder::R:
            return numRoundR * 360 + getAngle(Encoder::R);
        default:
            return -1;
    }
}

int Encoder::getRound(Encoder::side side) const {
    switch (side) {
        case Encoder::L:
            return numRoundL;
        case Encoder::R:
            return numRoundR;
        default:
            return -1;
    }
}

float Encoder::getDisOfWheel(Encoder::side side) const {
    switch (side) {
        case Encoder::L:
            return PI * WHEEL_DIAMETER * getAbsoluteAngle(Encoder::L) / 360;
        case Encoder::R:
            return PI * WHEEL_DIAMETER * getAbsoluteAngle(Encoder::R) / 360;
        default:
            return -1;
    }
}

void Encoder::update() {
    // attachInterrupt函数处理
    setupISRHandler(ISR_PortL, Encoder::ISRHandlerL, CHANGE);
    setupISRHandler(ISR_PortR, Encoder::ISRHandlerR, CHANGE);

    if (pulseCountL <= -COEFFICIENT_PER_ROUND_L) {
        pulseCountL += COEFFICIENT_PER_ROUND_L;
        numRoundL--;
    }
    if (pulseCountL >= COEFFICIENT_PER_ROUND_L) {
        pulseCountL -= COEFFICIENT_PER_ROUND_L;
        numRoundL++;
    }

    if (pulseCountR <= -COEFFICIENT_PER_ROUND_R) {
        pulseCountR += COEFFICIENT_PER_ROUND_R;
        numRoundR--;
    }
    if (pulseCountR >= COEFFICIENT_PER_ROUND_R) {
        pulseCountR -= COEFFICIENT_PER_ROUND_R;
        numRoundR++;
    }
    countOfUpdate++;

    if (countOfUpdate % 3 == 0) {
        timeCur = millis();
        angleCur = getAbsoluteAngle(Encoder::R);
        if (firstTimeFlag) {
            timeLast = timeCur;
            angleLast = angleCur;
            firstTimeFlag = false;
            return;
        } else {
            int timeInterval = timeCur - timeLast;
            // Serial.println(timeInterval);
            double angleDiff = angleCur - angleLast;
            double angleVelTemp;
            // Serial.println(angleDiff);
            if (timeInterval != 0) {
                angleVelTemp = angleDiff * 1000 / timeInterval;
                // Serial.println(angleVelTemp);
            } else {
                angleVelTemp = 0;
            }
            // if (fabs(angleDiff - (angleVel[1] - angleVel[0])) <= 20) {
            //     angleVel[0] = angleVel[1];
            //     angleVel[1] = angleVel[2];
            //     angleVel[2] = angleVelTemp;
            // }
            angleVel[0] = angleVel[1];
            angleVel[1] = angleVel[2];
            angleVel[2] = angleVelTemp;
            timeLast = timeCur;
            angleLast = angleCur;
        }
    }
}

void Encoder::setupISRHandler(int pin, void(*ISR)(void), int state) {
    attachInterrupt(pin, ISR, state);
}

void Encoder::ISRHandlerL() {
    _ISRPointer -> updateCountL();
}

void Encoder::ISRHandlerR() {
    _ISRPointer -> updateCountR();
}

void Encoder::updateCountL() {
    cli();
    // 左
    if (digitalRead(portA_L) == LOW) {    // 下降沿
        if (digitalRead(portB_L) == LOW) {
            pulseCountL--;
        } else {
            pulseCountL++;
        }
    } else {
        if (digitalRead(portB_L) == LOW) {    // 上升沿
            pulseCountL++;
        } else {
            pulseCountL--;
        }
    }
    sei();
}

void Encoder::updateCountR() {
    cli();
    // 右
    if (digitalRead(portA_R) == LOW) {    // 下降沿
        if (digitalRead(portB_R) == LOW) {
            pulseCountR++;
        } else {
            pulseCountR--;
        }
    } else {
        if (digitalRead(portB_R) == LOW) {    // 上升沿
            pulseCountR--;
        } else {
            pulseCountR++;
        }
    }
    sei();
}

void Encoder::testCoefficient(Encoder::side side) {
    // attachInterrupt函数处理
    setupISRHandler(ISR_PortL, Encoder::ISRHandlerL, CHANGE);
    setupISRHandler(ISR_PortR, Encoder::ISRHandlerR, CHANGE);

    
    Serial.begin(9600);

    // Serial.println(ISR_PortA);
    // Serial.println(portB);
    
    while (true) {
        // Serial.println(digitalRead(portB));
        switch (side) {
            case L:
                Serial.println(pulseCountL);
                break;
            case R:
                Serial.println(pulseCountR);
                break;
        }
        String cmd = Serial.readString();
        if (cmd == "STOP") {
            break;
        } else if (cmd == "RESET") {
            pulseCountL = 0;
            pulseCountR = 0;
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
    // Serial.println(sum / num);
    return sum / num;
}
