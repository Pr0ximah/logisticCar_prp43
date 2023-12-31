#include "PID.h"
#include <Arduino.h>
#include <math.h>
#include <time.h>
#include "constDef.h"

float PID::update(float current) {
    errorNow = target - current;
    P = kp * errorNow;
    if (init_Flag) {
        errorLast = errorNow;
        init_Flag = false;
        errorInt = 0;
    }
    errorDiff = errorNow - errorLast;
    D = kd * errorDiff;
    if (fabs(P) >= IRangeLocal) {
        errorInt = 0;
    } else {
        errorInt += errorNow;
        if (fabs(errorInt) * ki > IMax) {
            errorInt = sign(errorInt) * IMax / ki;
        }
    }
    // if (sign(errorInt) != sign(errorNow) || fabs(errorNow) <= errorTol) {
    //     errorInt = 0;
    // }
    I = ki * errorInt;
    if (fabs(errorNow) <= errorTol) {
        if (secStable >= 5) {
            outVal = 0;
        } else {
            outVal = P + I + D;
        }
    } else {
        secStable = 0;
        outVal = P + I + D;
    }
    secNow = millis() / 1000;
    if (init_Flag) {
        secLast = secNow;
    }
    if (fabs(errorNow <= errorTol)) {
        secStable += secNow - secLast;
    }
    secLast = secNow;
    errorLast = errorNow;
    return outVal;
}

void PID::setCoefficient(float _kp, float _ki, float _kd, float _error_tol) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
    errorTol = _error_tol;
}

PID::PID(float _target): target(_target), init_Flag(true) {
    IRangeLocal = IRange;
}

int sign(float x) {
    if (x == 0) { return 0; }
    return (x > 0) ? 1 : -1;
}

void PID::setIRange(int _IRange) {
    IRangeLocal = _IRange;
}