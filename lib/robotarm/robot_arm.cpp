#include "portDef.h"
#include "robot_arm.h"
// #include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
// #include <Wire.h>

RobotArm::RobotArm(){
    sc=open;
    sa=STORE;
    ArmPwm = Adafruit_PWMServoDriver();
    ArmPwm.begin();
    ArmPwm.setOscillatorFrequency(27000000);//原27000000
    ArmPwm.setPWMFreq(50); 
    ArmPwm.setPWM(middle, 0,290);
    ArmPwm.setPWM(top, 0, 385);
    ArmPwm.setPWM(plat, 0, 492);
    ArmPwm.setPWM(on_plat, 0,320);
    ArmPwm.setPWM(top_seize,0,210);
}

void RobotArm::FromVirToStore(){
  for(int i=320;i>240;--i)
  {ArmPwm.setPWM(on_plat, 0, i);
  delay(5);}
  ArmPwm.setPWM(on_plat, 0,240);

  for (int i=385;i>334;--i)
  {ArmPwm.setPWM(top, 0, i);
  delay(3);}
  ArmPwm.setPWM(top, 0, 334);

  for (int i=290;i>95;--i)
  {ArmPwm.setPWM(middle, 0, i);
  delay(5);}
  ArmPwm.setPWM(middle, 0, 95);

  for(int i=240;i<290;++i)
  {ArmPwm.setPWM(on_plat, 0, i);
  delay(3);}
  ArmPwm.setPWM(on_plat, 0,290);
  this->ChangeArmState(STORE);
  delay(1000);
}

void RobotArm::FromStoreToClaw(){
   for (int i=95;i<378;++i)
  {ArmPwm.setPWM(middle, 0, i);
  delay(5);}
  ArmPwm.setPWM(middle, 0, 382);

    for (int i=334;i<450;++i)
  {ArmPwm.setPWM(top, 0, i);
  delay(3);}
  ArmPwm.setPWM(top, 0, 450);

  for(int i=280;i>220;--i)
  {ArmPwm.setPWM(on_plat, 0, i);
  delay(3);}
  ArmPwm.setPWM(on_plat, 0,220);
  delay(1000);
  this->ChangeArmState(CLAW);
    delay(1000);
}
// @ todo
void RobotArm::FromVirToClaw(){
    for (int i=95;i<378;++i)
   {ArmPwm.setPWM(middle, 0, i);
   delay(5);}
  ArmPwm.setPWM(middle, 0, 382);

    for (int i=334;i<450;++i)
  {ArmPwm.setPWM(top, 0, i);
  delay(3);}
  ArmPwm.setPWM(top, 0, 450);

  for(int i=280;i>220;--i)
  {ArmPwm.setPWM(on_plat, 0, i);
  delay(3);}
  ArmPwm.setPWM(on_plat, 0,220);
  delay(1000);
  this->ChangeArmState(CLAW);
    delay(1000);
}

void RobotArm::ChangeArmState(state_for_arm SA){
    sa=SA;
}

void RobotArm::FromClawToVir(){
    for(int i=220;i<320;++i)
  {ArmPwm.setPWM(on_plat, 0, i);
  delay(3);}
  ArmPwm.setPWM(on_plat, 0,320);

  for (int i=382;i>290;--i)
  {ArmPwm.setPWM(middle, 0, i);
  delay(5);}
  ArmPwm.setPWM(middle, 0,290);

  for (int i=450;i>385;--i)
  {ArmPwm.setPWM(top, 0, i);
  delay(3);}
  ArmPwm.setPWM(top, 0, 385);
  this->ChangeArmState(VIR);
   delay(1000);
}

void RobotArm::ChangeClawState(state_for_claw SC){
    sc=SC;
}

void RobotArm::loose(){
      ArmPwm.setPWM(top_seize, 0, 210);//松开
      this->ChangeClawState(open);
      delay(1000);
}

void RobotArm::tight(){
    ArmPwm.setPWM(top_seize, 0, 260);//夹紧
    sc=close;
    delay(1000);
}

void RobotArm::fetch(){
    if(sc==close){
        this->loose();
    }
    if(sa==STORE){
        this->FromStoreToClaw();
        delay(100);
        this->tight();
    }
    else if(sa==VIR){
        this->FromVirToClaw();
        delay(100);
        this->tight();
    }
    else if(sa==CLAW){
        this->tight();
    }
}

void RobotArm::store(){
    if(sa==STORE){
        this->loose();
    }
    else if(sa==CLAW){
        this->FromClawToVir();
        this->loose();
        this->tight();
        this->FromVirToStore();
        delay(100);
        this->loose();
    }
    else if(sa==VIR){
        this->FromVirToStore();
        this->loose();
    }
}

void RobotArm::FromStoreToVir(){
    for(int i=290;i>240;--i)
  {ArmPwm.setPWM(on_plat, 0, i);
  delay(5);}
  ArmPwm.setPWM(on_plat, 0,240);

  for (int i=95;i<290;++i)
  {ArmPwm.setPWM(middle, 0, i);
  delay(5);}
  ArmPwm.setPWM(middle, 0,290);

  for (int i=334;i<385;++i)
  {ArmPwm.setPWM(top, 0, i);
  delay(3);}
  ArmPwm.setPWM(top, 0, 385);

  for(int i=240;i<320;++i)
  {ArmPwm.setPWM(on_plat, 0, i);
  delay(3);}
  ArmPwm.setPWM(on_plat, 0,320);
  this->ChangeArmState(VIR);
  delay(1000);
}