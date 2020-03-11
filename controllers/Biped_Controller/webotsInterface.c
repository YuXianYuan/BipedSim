/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 双足机器人 仿真环境 接口文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/inertial_unit.h>

#include "webotsInterface.h"

//-----------------------------------------------------------device
/*电机*/
WbDeviceTag L0_motor;
WbDeviceTag L1_motor;

WbDeviceTag R0_motor;
WbDeviceTag R1_motor;

/*电机编码器*/
WbDeviceTag L0_pos_sensor;
WbDeviceTag L1_pos_sensor;

WbDeviceTag R0_pos_sensor;
WbDeviceTag R1_pos_sensor;

/*足底触碰开关*/
WbDeviceTag L_touch_sensor;
WbDeviceTag R_touch_sensor;

/*惯导系统*/
WbDeviceTag IMU;

/*
函数功能：初始化devices
*/
void webots_device_init()
{
  //get device
  L0_motor          = wb_robot_get_device("FL1 rotational motor");
  L1_motor          = wb_robot_get_device("FL2 rotational motor");
  
  R0_motor          = wb_robot_get_device("FR1 rotational motor");
  R1_motor          = wb_robot_get_device("FR2 rotational motor");
  
  
  L0_pos_sensor     = wb_robot_get_device("FL1 position sensor");
  L1_pos_sensor     = wb_robot_get_device("FL2 position sensor");
  
  R0_pos_sensor     = wb_robot_get_device("FR1 position sensor");
  R1_pos_sensor     = wb_robot_get_device("FR2 position sensor");

  
  L_touch_sensor    = wb_robot_get_device("FL touch sensor");
  R_touch_sensor    = wb_robot_get_device("FR touch sensor");
  
  IMU                = wb_robot_get_device("inertial unit");
  
  //enable
  wb_position_sensor_enable(L0_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(L1_pos_sensor, TIME_STEP);

  wb_position_sensor_enable(R0_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(R1_pos_sensor, TIME_STEP);
  
  wb_touch_sensor_enable(L_touch_sensor, TIME_STEP);
  wb_touch_sensor_enable(R_touch_sensor, TIME_STEP);
  
  wb_inertial_unit_enable(IMU, TIME_STEP);
  
  wb_keyboard_enable(TIME_STEP);
}
//-----------------------------------------------------------motor
/*
函数功能：设置电机扭矩
*/
void set_motor_torque(motorNameTypeDef motorName, double torque)
{
  if(torque >  1800)torque =  1800;
  if(torque < -1800)torque = -1800;
  
  switch (motorName){
  case L0:  {  wb_motor_set_torque(L0_motor, torque);break;  }
  case L1:  {  wb_motor_set_torque(L1_motor, torque);break;  }
  
  case R0:  {  wb_motor_set_torque(R0_motor, torque);break;  }
  case R1:  {  wb_motor_set_torque(R1_motor, torque);break;  }

  default:break;
  }
}
//-----------------------------------------------------------sensor
/*
函数功能：获取电机角度,角度制
*/
double get_motor_angle(motorNameTypeDef motorName)
{
  double angle = 0;
  switch (motorName){
  case L0:  { angle = wb_position_sensor_get_value(L0_pos_sensor);break; }
  case L1:  { angle = wb_position_sensor_get_value(L1_pos_sensor);break; }
  
  case R0:  { angle = wb_position_sensor_get_value(R0_pos_sensor);break; }
  case R1:  { angle = wb_position_sensor_get_value(R1_pos_sensor);break; }
  default:break;
  }
  return angle*180.0f/PI;
}
/*
函数功能：检测足底是否接触地面
*/
bool is_foot_touching(legNameTypeDef legName)
{
  if(legName == L)
    return wb_touch_sensor_get_value(L_touch_sensor);
  if(legName == R)
    return wb_touch_sensor_get_value(R_touch_sensor);
  return true;
}

/*
函数功能：读取IMU数据
*/
eulerAngleTypeDef get_IMU_Angle()
{
  const double* data = wb_inertial_unit_get_roll_pitch_yaw(IMU);
  
  eulerAngleTypeDef eulerAngle;
  eulerAngle.roll  = data[1]*180.0f/PI;
  eulerAngle.pitch = data[0]*180.0f/PI;
  eulerAngle.yaw   = data[2]*180.0f/PI;
  
  return eulerAngle;
}

//-----------------------------------------------------------keyboard
/*
函数功能：读取键盘键值
*/
int get_keyboard()
{
  return wb_keyboard_get_key();
}

