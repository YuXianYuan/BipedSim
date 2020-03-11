/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 双足机器人 主 文件
**  Version    : 
**  Notes      : 使用键盘 ↑ ↓ 控制机器人运动方向
**  Author     : 于宪元
**********************************************************/
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/keyboard.h>

#include "webotsInterface.h"
#include "controller.h"
#include <math.h>
extern robotTypeDef robot;
int main(int argc, char **argv) {
  
  wb_robot_init();
  webots_device_init();                           //webots设备初始化
  robot_init();                                   //机器人初始化
  while (wb_robot_step(TIME_STEP) != -1) {
  
  updateRobotState();                             //机器人状态更新
  robot_control();                                //机器人控制
  }
  wb_robot_cleanup();
  return 0;
}


