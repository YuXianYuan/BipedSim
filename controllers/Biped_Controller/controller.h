/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 双足机器人 控制器头 文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdbool.h>

#include "easyMat.h"
#include "webotsInterface.h"

//-----------------------------------------------------------macro
#define max(a,b) ( ((a)>(b)) ? (a):(b) )
#define min(a,b) ( ((a)>(b)) ? (b):(a) )

#define g        (9.8)
//-----------------------------------------------------------typedef
/*
3维向量
*/
typedef struct
{
  double x;
  double z;
}vect2TypeDef;
/*
机器人定义,包括机器人状态和机器人参数
*/
typedef struct
{
  //------------------------------------------------------------------------控制参数
  double Tf     ;  // 0.5;
  double Ts     ;  // 0.5;
  double hd     ;  // 0.6;
  double zh     ;  // -0.5;
  double Kp     ;  // 10000;
  double Kdp    ;  // 800;
  double Kh     ;  // 8000;
  double Kdh    ;  // 800;
  double Kvx    ;  // 1000;
  double kx     ;  // 100;
  double kdx    ;  // 100;
  double kz1    ;  // 10000;
  double kz2    ;  // 1000;
  double kdz    ;  // 100;

  double vxd_default ;  // 0.6;  //默认前向期望速度
  double vxd ;  // 0;
  //------------------------------------------------------------------------模型参数
  double a0 ;  // 0.4;    //大腿连杆长度
  double a1 ;  // 0.4;    //小腿连杆长度
  double m0 ;  // 4;      //大腿连杆质量
  double m1 ;  // 4;      //小腿连杆质量
  double M  ;  // 50;    //躯干质量
  //------------------------------------------------------------------------状态区
  double            t              ;  // 0;      //从一步开始，经过的时间
  legNameTypeDef    st             ;  // L;     //前支撑腿
  legNameTypeDef    sw             ;  // R;     //前摆动腿
  bool              is_touching[2] ;  // false;  //足底传感器接触标志量,下标为L,R
  double            pitch          ;  // 0;      //pitch
  double            dpitch         ;  // 0;      //pitch'
  double            theta[2][2]    ;  //关节角度
  vect2TypeDef      toe[2]         ;  //足底坐标
  vect2TypeDef      dtoe[2]        ;  //足底速度
  double            vx             ;  //机器人实际x速度
  vect2TypeDef      toe0           ;  
  vect2TypeDef      dtoe0          ;  // {0,0,0};
  
}robotTypeDef;
//-----------------------------------------------------------extern
extern robotTypeDef robot;

extern void robot_init           ();
extern void updateRobotState     ();
extern void robot_control        ();

#endif

