 /**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 双足机器人 仿真环境或硬件接口 头文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#ifndef _WEBOTSINTERFACE_H_
#define _WEBOTSINTERFACE_H_

//-----------------------------------------------------------macro
#define PI           (3.141892654f)
#define TIME_STEP    (1)
//-----------------------------------------------------------typedef
/*
电机名称集合
*/
typedef enum
{
  L0   = 0x00,
  L1   = 0x01,
  
  R0   = 0x02,
  R1   = 0x03,
}motorNameTypeDef;

/*
腿名称集合
*/
typedef enum
{
  L = 0x00,   //左
  R = 0x01,   //右
}legNameTypeDef;

/* 
1，陀螺仪数据定义,为了方便调试采用角度制，注意，采用角度制
2，webots IMU模块采用RPY角度制，定系旋转，矩阵左乘，即：
       Rot=RotY(yaw)*RotZ(pitch)*RotX(roll);
3，eulerAngleTypeDef结构体描述了数学模型中的RPY，定系旋转，矩阵左乘，即：
       Rot=RotZ(yaw)*RotY(pitch)*RotX(roll);
4,由于webots默认坐标系和数学模型世界坐标系定义不同，因此二者RPY定义不同，对应关系如下：

==============================
*   数学模型   *   webots    *
------------------------------
*   X(roll)   *   Z(pitch)  *
*   Y(pitch)  *   X(roll)   *
*   Z(yaw)    *   Y(yaw)    *
==============================
5,暂不考虑上述两个坐标系因相乘顺序而引起的误差。
*/
typedef struct
{
  double roll;       //横滚，x轴
  double pitch;      //俯仰，z轴
  double yaw;        //偏航，y轴
}eulerAngleTypeDef;

//-----------------------------------------------------------extern
extern void              webots_device_init                                           ();
extern void              set_motor_torque    (motorNameTypeDef motorName, double torque);
extern double            get_motor_angle                    (motorNameTypeDef motorName);
extern bool              is_foot_touching                       (legNameTypeDef legName);
extern int               get_keyboard                                                 ();
extern eulerAngleTypeDef get_IMU_Angle                                                ();
#endif

