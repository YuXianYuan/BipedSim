/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 双足机器人 控制器 文件
**  Version    : 
**  Notes      :
**  Author     : 于宪元
**********************************************************/
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <webots/keyboard.h>

#include "easyMat.h"
#include "controller.h"

robotTypeDef robot;
//----------------------------------------------------------declaration
void update_vxd();
void update_IMU();
void update_foot_touch_sensor();
void phase_swap();
void update_theta();
void forwardKinematics();
void estimate_vx();
void create_transJ(matTypeDef* transJ, legNameTypeDef leg);
void curve(vect2TypeDef* ft, vect2TypeDef* dft, double xT, vect2TypeDef toe0, vect2TypeDef dtoe0);
void update_toe0_dtoe0();
/*
机器人参数初始化，在while（1）之前调用
*/
void robot_init()
{
  //------------------------------------------------------------------------控制参数 
   robot.Tf     = 0.25;
   robot.Ts     = 0.25;
   robot.hd     = 0.72;
   robot.zh     = -0.62;
   robot.Kp     = 4000;
   robot.Kdp    = 200;
   robot.Kh     = 8000;
   robot.Kdh    = 800;
   robot.Kvx    = 3.0;
   robot.kx     = 100;
   robot.kdx    = 100;
   robot.kz1    = 10000;
   robot.kz2    = 1000;
   robot.kdz    = 100;
   
   robot.vxd_default = 0.02;  //默认前向期望速度
   robot.vxd         = 0;
  //------------------------------------------------------------------------模型参数
   robot.a0 = 0.4;    //大腿连杆长度
   robot.a1 = 0.4;    //小腿连杆长度
   robot.m0 = 4;      //大腿连杆质量
   robot.m1 = 4;      //小腿连杆质量
   robot.M  = 58;     //总质量
  //------------------------------------------------------------------------状态区
   robot.t               = 0;      //从一步开始，经过的时间
   robot.st              = L;      //支撑腿
   robot.sw              = R;      //摆动腿
   robot.is_touching[L]  = false;  //足底传感器接触标志量
   robot.is_touching[R]  = false;  //足底传感器接触标志量
   robot.pitch           = 0;      //pitch
   robot.dpitch          = 0;      //pitch'
   for(legNameTypeDef legIdx = L; legIdx <= R; legIdx++)
   {
     robot.dtoe[legIdx].x = 0;
     robot.dtoe[legIdx].z = 0;
   }
   robot.vx              = 0;  //机器人实际x速度
   robot.dtoe0.x         = 0;
   robot.dtoe0.z         = 0;
}
/*
机器人状态更新，包括读取传感器的数据以及一些状态估计
*/
void updateRobotState()
{
  /*通过键盘修改期望速度*/
  update_vxd();
  /*IMU和IMU导数更新*/
  update_IMU();
  /*足底接触传感器更新*/
  update_foot_touch_sensor();
  /*相位切换*/
  phase_swap();
  /*更新关节角*/
  update_theta();
  /*运动学正解*/
  forwardKinematics();
  /*记录相位切换瞬间，摆动足的起始位置和起始速度*/
  update_toe0_dtoe0();
  /*机器人水平速度估计*/
  estimate_vx();  //在robot.t=0时候特殊处理
  /*时钟更新*/
  robot.t += 0.001*TIME_STEP;   
}
/*
机器人控制
*/
void robot_control()
{
  //分别提取支撑足与摆动足的足底坐标
  vect2TypeDef st_toe, sw_toe;
  if(robot.st == L)
  {
    st_toe = robot.toe[L];
    sw_toe = robot.toe[R];
  }
  else  //(robot.st == R)
  {
    st_toe = robot.toe[R];
    sw_toe = robot.toe[L];
  }
  //-----------------------------------------------------------------支撑相控制
  //求机身高度
  static double pre_h = 0.792;
  double h  = -st_toe.z;
  double dh = (h - pre_h)/(0.001*(double)TIME_STEP);
  pre_h = h;
  //求机身3维力(取消对Fx的控制)
  double Ty = -(robot.Kp*robot.pitch/180.0*PI + robot.Kdp*robot.dpitch/180.0*PI);
  double Fz = -(robot.Kh*(h - robot.hd) + robot.Kdh*dh);
  
  //求支撑足足底力
  matTypeDef st_f;
  easyMat_create(&st_f, 2, 1);
  st_f.data[1][0] =  Fz + robot.M*g*cos(robot.pitch/180.0*PI);
  st_f.data[0][0] =  (Ty + st_f.data[1][0]*robot.toe[robot.st].x)/robot.toe[robot.st].z;
  //求支撑足关节力
  matTypeDef st_transJ, st_tau;
  easyMat_create(&st_tau   , 2, 1);
  easyMat_create(&st_transJ, 2, 2);
  create_transJ(&st_transJ, robot.st);
  easyMat_mult(&st_tau, &st_transJ, &st_f);
  easyMat_mult_k(-1.0, &st_tau);
  //发送关节力到电机
  
  if(robot.st == L)
  {
    set_motor_torque(L0, st_tau.data[0][0]);
    set_motor_torque(L1, st_tau.data[1][0]);
  }
  else //(robot.st == R)
  {
    set_motor_torque(R0, st_tau.data[0][0]);
    set_motor_torque(R1, st_tau.data[1][0]);
  }
  //释放内存
  easyMat_free(&st_f     );
  easyMat_free(&st_transJ);
  easyMat_free(&st_tau   );
  //-----------------------------------------------------------------摆动相控制
  //求落足点

  double xT = robot.vx*robot.Ts/2.0 + robot.Kvx*(robot.vx - robot.vxd);
  //求摆动足实时期望足底坐标与速度
  vect2TypeDef toed, dtoed;
  curve(&toed, &dtoed, xT, robot.toe0, robot.dtoe0);
  //求足底力
  matTypeDef sw_f;
  easyMat_create(&sw_f, 2,1);
  
  double kz;
  if(robot.t <= 0.75*robot.Tf)
    kz = robot.kz1;
  else
    kz = robot.kz2;
  
  sw_f.data[0][0] = -(robot.kx*(sw_toe.x - toed.x) + robot.kdx*(robot.dtoe[robot.sw].x - dtoed.x));
  sw_f.data[1][0] = -(      kz*(sw_toe.z - toed.z) + robot.kdz*(robot.dtoe[robot.sw].z - dtoed.z));
  //求关节力
  matTypeDef sw_transJ, sw_tau;
  easyMat_create(&sw_tau   , 2, 1);
  easyMat_create(&sw_transJ, 2, 2);
  create_transJ(&sw_transJ, robot.sw);
  easyMat_mult(&sw_tau, &sw_transJ, &sw_f);
  //发送关节力到电机
  if(robot.sw == L)
  {
    set_motor_torque(L0, sw_tau.data[0][0]);
    set_motor_torque(L1, sw_tau.data[1][0]);
  }
  else if(robot.sw == R)
  {
    set_motor_torque(R0, sw_tau.data[0][0]);
    set_motor_torque(R1, sw_tau.data[1][0]);
  }
  //释放内存
  easyMat_free(&sw_f     );
  easyMat_free(&sw_transJ);
  easyMat_free(&sw_tau   );
}
/*
通过键盘修改期望速度
*/
void update_vxd()
{
   /*读取键盘，获取速度*/
  switch(get_keyboard())
  {
    case WB_KEYBOARD_UP:
    {
      robot.vxd = robot.vxd_default;
      break;
    }
    case WB_KEYBOARD_DOWN:
    {
      robot.vxd = -robot.vxd_default;
      break;
    }
    default:
    {
      robot.vxd = 0;
      break;
    }
  }
}
/*
足底传感器更新
*/
void update_foot_touch_sensor()
{
  robot.is_touching[L] = is_foot_touching(L);
  robot.is_touching[R] = is_foot_touching(R);
}
/*
IMU与IMU的导数更新
*/
void update_IMU()
{
  static eulerAngleTypeDef pre_eulerAngle = {0,0,0};
  eulerAngleTypeDef eulerAngle = get_IMU_Angle();
  //IMU
  robot.pitch  = eulerAngle.pitch;
  //IMU'
  static double pre_dpitch = 0;
  robot.dpitch = (eulerAngle.pitch - pre_eulerAngle.pitch)/((double)TIME_STEP/1000.0);
  robot.dpitch = robot.dpitch*0.1 + pre_dpitch*0.9;
  
  pre_dpitch = robot.dpitch;
  pre_eulerAngle = eulerAngle;
}
/*
更新关节角
*/
void update_theta()
{
  robot.theta[L][0] = get_motor_angle(L0);
  robot.theta[L][1] = get_motor_angle(L1);
  
  robot.theta[R][0] = get_motor_angle(R0);
  robot.theta[R][1] = get_motor_angle(R1);
}
/*
相位切换,通过时间和足底传感器确定支撑对角腿和摆动对角腿
*/

void phase_swap()
{
  if(robot.t > 0.75*robot.Tf)
  {
    if(robot.st == L)  //支撑腿是L
    {
      if(robot.is_touching[R])
      {
        robot.st = R;
        robot.sw = L;
        robot.t = 0;      //相位切换时候时间归零
      }
    }
    else if(robot.st == R)  //支撑腿是R
    {
      if(robot.is_touching[L])
      {
        robot.st = L;
        robot.sw = R;
        robot.t = 0;      //相位切换时候时间归零
      }
    }
  }
}
/*
运动学正解
*/
void forwardKinematics()
{
  double a0        = robot.a0;
  double a1        = robot.a1;
  
  //足底位置
  for(legNameTypeDef legIdx = L; legIdx <= R; legIdx++)
  {
    double s0  = sin( robot.theta[legIdx][0]/180.0*PI );
    double c0  = cos( robot.theta[legIdx][0]/180.0*PI );
    double s01 = sin( robot.theta[legIdx][0]/180.0*PI + robot.theta[legIdx][1]/180.0*PI);
    double c01 = cos( robot.theta[legIdx][0]/180.0*PI + robot.theta[legIdx][1]/180.0*PI);
    
    robot.toe[legIdx].x = -a0*s0 - a1*s01;
    robot.toe[legIdx].z = - a0*c0 - a1*c01;
  }
  
  //足底速度
  static vect2TypeDef pre_toe[2];
  static bool first_run_flag = true;
  if(first_run_flag == true)
  {
    first_run_flag = false;
  }
  else
  {
    for(legNameTypeDef legIdx = L; legIdx <= R; legIdx++)
    {
      robot.dtoe[legIdx].x = (robot.toe[legIdx].x - pre_toe[legIdx].x)/(0.001*(double)TIME_STEP);
      robot.dtoe[legIdx].z = (robot.toe[legIdx].z - pre_toe[legIdx].z)/(0.001*(double)TIME_STEP);
    }
  }
  for(legNameTypeDef legIdx = L; legIdx <= R; legIdx++)
  {
    pre_toe[legIdx] = robot.toe[legIdx];
  }
}
/*
机器人水平速度估计
*/
void estimate_vx()
{
  static vect2TypeDef pre_toe;
         vect2TypeDef     toe;
  
  toe = robot.toe[robot.st];
  
  if(robot.t != 0)
  {
    robot.vx = -((toe.x - pre_toe.x))/(0.001*(double)TIME_STEP);
  }
  
  static double pre_vx = 0;
  robot.vx = robot.vx*0.1 + pre_vx*0.9;
  
  pre_toe = toe;
}
/*
更新摆动足的起始位置和起始速度
*/
void update_toe0_dtoe0()
{
  static vect2TypeDef pre_st_toe = {0,0};
  static vect2TypeDef st_dtoe    = {0,0};
  if(robot.t == 0) //刚刚切换过对角腿，记录此时的摆动足即可
  {
    //记录初始位置
    robot.toe0 = robot.toe[robot.sw];
   //记录初始速度
    robot.dtoe0 = st_dtoe;
  }
  else
  {
    st_dtoe.x = (robot.toe[robot.st].x - pre_st_toe.x)/(0.001*((double)TIME_STEP));
    st_dtoe.z = (robot.toe[robot.st].z - pre_st_toe.z)/(0.001*((double)TIME_STEP));
  }
  pre_st_toe = robot.toe[robot.st];
}
/*
足底轨迹
*/
void curve(vect2TypeDef* ft, vect2TypeDef* dft, double xT, vect2TypeDef toe0, vect2TypeDef dtoe0)
{
  double t   = robot.t;
  double x0  = toe0.x;
  double z0  = toe0.z;
  double dx0 = dtoe0.x;
//  double dz0 = dtoe0.z;
  double Tf  = robot.Tf;
  double zh  = robot.zh;
  //x,y
  if(t < Tf/4.0)
  {
    ft->x  = -4*dx0*t*t/Tf + dx0*t + x0;
    dft->x = -8*dx0*t/Tf + dx0;
  }
  else if((t >= Tf/4.0)&&(t < 3.0*Tf/4.0))
  {
    ft->x  = ( -4*Tf*dx0 - 16*xT + 16*x0)*t*t*t/(Tf*Tf*Tf) + 
             (  7*Tf*dx0 + 24*xT - 24*x0)*t*t/(Tf*Tf) + 
             (-15*Tf*dx0 - 36*xT + 36*x0)*t/(4*Tf) + 
             (  9*Tf*dx0 + 16*xT)/16;
            
    dft->x = ( -4*Tf*dx0 - 16*xT + 16*x0)*3*t*t/(Tf*Tf*Tf) + 
             (  7*Tf*dx0 + 24*xT - 24*x0)*2*t/(Tf*Tf) + 
             (-15*Tf*dx0 - 36*xT + 36*x0)/(4*Tf);
  }
  else
  {
    ft->x  = xT;
    dft->x = 0;
  }
  //z
  if(t < Tf/2.0)
  {
    ft->z  = 16*(z0 - zh)*t*t*t/(Tf*Tf*Tf) + 12*(zh - z0)*t*t/(Tf*Tf) + z0;
    dft->z = 16*(z0 - zh)*3*t*t/(Tf*Tf*Tf) + 12*(zh - z0)*2*t/(Tf*Tf);
  }
  else
  {
    ft->z  = 4*(z0 - zh)*t*t/(Tf*Tf) - 4*(z0 - zh)*t/Tf + z0;
    dft->z = 4*(z0 - zh)*2*t/(Tf*Tf) - 4*(z0 - zh)/Tf;
  }
}
/*
创建力雅可比
*/
void create_transJ(matTypeDef* transJ, legNameTypeDef leg)
{
  double a0 = robot.a0;
  double a1 = robot.a1;
  
  double c0  = cos(robot.theta[leg][0]/180.0*PI);
  double s0  = sin(robot.theta[leg][0]/180.0*PI);
  double c01 = cos(robot.theta[leg][0]/180.0*PI + robot.theta[leg][1]/180.0*PI);
  double s01 = sin(robot.theta[leg][0]/180.0*PI + robot.theta[leg][1]/180.0*PI);
  
  transJ->data[0][0] = -a0*c0 - a1*c01;
  transJ->data[0][1] =  a0*s0 + a1*s01; 
  
  transJ->data[1][0] = -a1*c01;
  transJ->data[1][1] =  a1*s01;
}