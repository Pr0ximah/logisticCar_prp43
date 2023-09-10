#include <Arduino.h>
#include "encoder.h"
#include "constDef.h"

Encoder *Encoder::_ISRPointer = nullptr;
int Encoder::s_id = 0;
Encoder *Encoder::objArray[2] = {nullptr, nullptr};

Encoder::Encoder(int _portA, int _portB, int _coefficient) : COEFFICIENT_PER_ROUND(_coefficient)
{
    // 当前对象id设置
    for (int i = 0; i < 2; i++) {
        if (objArray[i] == nullptr) {
            id = i;
            objArray[i] = this;
            break;
        }
    }

    portA = _portA;
    portB = _portB;
    pulseCount = 0;
    numRound = 0;

    // 引脚模式设置
    pinMode(portA, INPUT);
    pinMode(portB, INPUT);
}

void Encoder::reset() {
    pulseCount = 0;
    numRound = 0;
}

float Encoder::getAngle() const {
    return double(pulseCount) / COEFFICIENT_PER_ROUND * 360;
}

float Encoder::getAbsoluteAngle() const {
    return numRound * 360 + getAngle();
}

int Encoder::getRound() const {
    return numRound;
}

float Encoder::getDisOfWheel() const {
    // Serial.println(getAbsoluteAngle());
    return PI * WHEEL_DIAMETER * getAbsoluteAngle() / 360;
}

void Encoder::update() {
    // attachInterrupt函数处理
    s_id = id;
    int ISR_PortA;
    if (portA == 2) {
        ISR_PortA = 0;
    } else {
        ISR_PortA = 1;
    }
    setupISRHandler(ISR_PortA, Encoder::ISRHandler, FALLING);
    setupISRHandler(ISR_PortA, Encoder::ISRHandler, RISING);

    // Serial.println(pulseCount);

    if (pulseCount <= -COEFFICIENT_PER_ROUND) {
        pulseCount += COEFFICIENT_PER_ROUND;
        numRound--;
    }
    if (pulseCount >= COEFFICIENT_PER_ROUND) {
        pulseCount -= COEFFICIENT_PER_ROUND;
        numRound++;
    }
}

void Encoder::setupISRHandler(int pin, void(*ISR)(void), int state) {
    attachInterrupt(pin, ISR, state);
}

void Encoder::ISRHandler() {
    _ISRPointer = objArray[s_id];
    _ISRPointer -> updateCount();
}

void Encoder::updateCount() {
    cli();
    if (digitalRead(portA) == LOW) {    // 下降沿
        if (digitalRead(portB) == LOW) {
            pulseCount--;
        } else {
            pulseCount++;
        }
    } else {
        if (digitalRead(portB) == LOW) {    // 上升沿
            pulseCount++;
        } else {
            pulseCount--;
        }
    }
    sei();
}

void Encoder::testCoefficient() {
    // attachInterrupt函数处理
    s_id = id;
    int ISR_PortA;
    if (portA == 2) {
        ISR_PortA = 0;
    } else {
        ISR_PortA = 1;
    }
    setupISRHandler(ISR_PortA, Encoder::ISRHandler, FALLING);
    setupISRHandler(ISR_PortA, Encoder::ISRHandler, RISING);

    
    Serial.begin(9600);

    // Serial.println(ISR_PortA);
    // Serial.println(portB);
    
    while (true) {
        // Serial.println(digitalRead(portB));
        Serial.println(getAngle());
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
