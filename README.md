# logisticCar_prp43

43期PRP项目智能物流小车控制程序

## 说明

该项目是智能物流小车的控制程序。目前完成的部分是底盘移动控制，后续还要加入机械臂控制的部分。

### 底盘移动控制

通过读取电机编码器等传感器数据，通过分析解算获得小车定位，并通过PID控制小车移动，实现输入目标位置、小车自动行驶并停止的功能。

### 机械臂控制

*TODO....*

## 项目结构

|--lib                                                                  <br>
|  |--car            总控，小车类，包括底盘、机械臂控制两部分。               <br>
|  |--definition     常量定义、端口定义。                                  <br>
|  |--drive          移动控制，所有的初始化在此进行，移动控制函数也在这里。     <br>
|  |--encoder        编码器类，电机编码器的数据获取。                        <br>
|  |--PID            PID控制器类。                                       <br>
|  |--point          点类、向量类，用于计算。                              <br>
|  |--motor          电机控制类，包含了转速PID                             <br>
|                                                                       <br>
|--src                                                                  <br>
|  |--main.cpp       程序主函数，主入口。

# Maintainer

@Pr0ximah: https://github.com/Pr0ximah

@520AWEI: https://github.com/520AWEI
