/*
  This code is used for connecting arduino to serial ATK-IMU901 module, and test in arduino uno R3 board.
  connect map:
  arduino   ATK901 module
  VCC    5v/3.3v
  TX     RX<-0
  TX     TX->1
  GND    GND
*/
#include <Arduino.h>
unsigned char Re_buf[16], counter = 0 ;
unsigned char sign = 0;
float acc[3], Roll, Pitch, Yaw, Q[4], w[3];
void setup() {
  Serial.begin(115200);
}

void loop() {
  if (sign)
  {
    sign = 0;
    if (Re_buf[0] == 0x55)   //检查帧头
    {
      switch (Re_buf [2])
      {
        case 0x01:
          Roll  = (short(Re_buf [5] << 8 | Re_buf [4])) / 32768.0 * 180;
          Pitch = (float(Re_buf [7] << 8 | Re_buf [6])) / 32768.0 * 180;
          Yaw   = (float(Re_buf [9] << 8 | Re_buf [8])) / 32768.0 * 180;
          Serial.println("Angle:");
          Serial.print(Roll); Serial.print(" ");
          Serial.print(Pitch); Serial.print(" ");
          Serial.print(Yaw); Serial.println(" ");
          break;
        case 0x02:
          Q[0]  = (float(Re_buf [5] << 8 | Re_buf [4])) / 32768.0;
          Q[1]  = (float(Re_buf [7] << 8 | Re_buf [6])) / 32768.0;
          Q[2]  = (float(Re_buf [9] << 8 | Re_buf [8])) / 32768.0;
          Q[3]  = (float(Re_buf [11] << 8 | Re_buf [10])) / 32768.0;
          Serial.println("Quarternion:");
          Serial.print(Q[0]); Serial.print(" ");
          Serial.print(Q[1]); Serial.print(" ");
          Serial.print(Q[2]); Serial.print(" ");
          Serial.print(Q[3]); Serial.println(" ");
          break;
        case 0x03:
          acc[0] = (float(Re_buf [5] << 8 | Re_buf [4])) / 32768.0 * 2 * 9.8;  // 2 为设置的1G单位刻度
          acc[1] = (float(Re_buf [7] << 8 | Re_buf [6])) / 32768.0 * 2 * 9.8;
          acc[2] = (float(Re_buf [9] << 8 | Re_buf [8])) / 32768.0 * 2 * 9.8;
          w[0]   = (float(Re_buf [11] << 8 | Re_buf [10])) / 32768.0 * 90;
          w[1]   = (float(Re_buf [13] << 8 | Re_buf [12])) / 32768.0 * 90;
          w[2]   = (float(Re_buf [15] << 8 | Re_buf [14])) / 32768.0 * 90;
          Serial.println("Accelerate:");
          Serial.print(acc[0]); Serial.print(" ");
          Serial.print(acc[1]); Serial.print(" ");
          Serial.print(acc[2]); Serial.println(" ");
          Serial.println("Gravity:");
          Serial.print(w[0]); Serial.print(" ");
          Serial.print(w[1]); Serial.print(" ");
          Serial.print(w[2]); Serial.println(" ");
          break;
      }
    }
  }
}

void serialEvent() {

    counter=0;
    while (Serial.available())//检验帧头是否正确
    {
        Re_buf[counter] = (unsigned char) Serial.read();
        if(Re_buf[0] != 0x55){
            counter=0;
            continue;
        }
        else{
            counter++;
            Re_buf[counter] = (unsigned char) Serial.read();
            if(Re_buf[1]!=0x55){
                counter=0;
                continue;
            }
            else{
                counter++;
                break;
            }
        }
    }

    Re_buf[counter] = (unsigned char) Serial.read();
    switch (Re_buf [2])      // 根据数据帧类型判断
    {
      case 0x01://11位信号读取姿态角
        for(int i=3;i<=10;i++){
            Re_buf[i] = (unsigned char) Serial.read();
        }
        sign=1;
        break;
      case 0x02://13位信号读取四元数
        for(int i=3;i<=12;i++){
            Re_buf[i] = (unsigned char) Serial.read();
        }
        sign=1;
        break;
      case 0x03://17位信号读取加速度
        for(int i=3;i<=16;i++){
            Re_buf[i] = (unsigned char) Serial.read();
        }
        sign=1;
        break;
      default:      
        break;
    }
    
}
