#include <Arduino.h>
#include "encoder.h"
#include "constDef.h"

Encoder *Encoder::_ISRPointer = nullptr;
int Encoder::s_id = 0;
Encoder *Encoder::objArray[4] = {nullptr, nullptr, nullptr, nullptr};

Encoder::Encoder(int _portA, int _portB, int _coefficient) : COEFFICIENT_PER_ROUND(_coefficient)
{
    // 当前对象id设置
    for (int i = 0; i < 4; i++) {
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

int Encoder::getDisOfWheel() const {
    return PI * WHEEL_DIAMETER * getAbsoluteAngle();
}

void Encoder::update() {
    // attachInterrupt函数处理
    s_id = id;
    setupISRHandler(portA, Encoder::ISRHandler, FALLING);

    if (pulseCount <= -COEFFICIENT_PER_ROUND) {
        pulseCount += COEFFICIENT_PER_ROUND;
        numRound++;
    }
    if (pulseCount >= COEFFICIENT_PER_ROUND) {
        pulseCount -= COEFFICIENT_PER_ROUND;
        numRound--;
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
    if (digitalRead(portB) == LOW) {
        pulseCount--;
    } else {
        pulseCount++;
    }
    sei();
}

