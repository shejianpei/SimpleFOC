/**
Deng's FOC 闭环位置控制例程 测试库：SimpleFOC 2.1.1 测试硬件：灯哥开源FOC V2.0
在串口窗口中输入：T+位置，就可以使得两个电机闭环转动
比如让两个电机都转动180°，则输入其弧度制：T3.14
在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(14) 中的值，设置为自己的极对数数字
程序默认设置的供电电压为 16.8V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
默认PID针对的电机是 GB6010 ，使用自己的电机需要修改PID参数，才能实现更好效果
*/

#include <SimpleFOC.h>


MagneticSensorPWM sensor = MagneticSensorPWM(23,3, 925); 

//电机参数
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 5, 18, 4);  //IN1 IN2 IN3 ENA


//命令设置
float target_velocity = 0;
char flag=0;
float init_angle =0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  sensor.init();

  //连接motor对象与传感器对象
  motor.linkSensor(&sensor);

  //供电电压设置 [V]
  driver.voltage_power_supply = 12;
  driver.init();

  //连接电机和driver对象
  motor.linkDriver(&driver);
  
  //FOC模型选择
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //运动控制模式设置
  motor.controller = MotionControlType::angle;

  //速度PI环设置
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 10;   //20
  
  //角度P环设置 
  motor.P_angle.P = 10;    //15
  
  //最大电机限制电机
  motor.voltage_limit = 12;
  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01;
  //设置最大速度限制
  motor.velocity_limit = 10;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  
  //初始化电机
  motor.init();
  
  //初始化 FOC
  motor.initFOC();
  
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
}


void loop() {
  if(flag == 0){
       init_angle = sensor.getAngle();  
       flag = 1;
  }

  motor.loopFOC();
  motor.move(-init_angle+target_velocity);

  command.run();
}
