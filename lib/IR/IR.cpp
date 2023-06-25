#include "IR.h"
#include <Arduino.h>

IR::IR(int _inputPin, float _voltage) {
    inputPin = _inputPin;
    voltage = _voltage;
}

float IR::IR_ReadDis() {
    float coefficient = voltage / 1024;
    float dataRead = analogRead(inputPin);
    return dataRead * coefficient;
}