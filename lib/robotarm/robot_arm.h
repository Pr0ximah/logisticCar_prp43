#include "portDef.h"
// #include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
// #include <Wire.h>

enum state_for_arm{CLAW,VIR,STORE};//fetch vir store
enum state_for_claw{close,open};
class RobotArm{
private:
    state_for_arm sa;
    state_for_claw sc;
    Adafruit_PWMServoDriver ArmPwm;
public:
    RobotArm();
    void ChangeArmState(state_for_arm SA);
    void ChangeClawState(state_for_claw SC);
    void FromVirToClaw();
    void FromClawToVir();
    void FromVirToStore();
    void FromStoreToVir();
    void FromStoreToClaw();
    void tight();
    void loose();
    void fetch();
    void store();
};
