/*
 * bsp_gpio.c
 *
 *  Created on: 2023年5月21日
 *      Author: ALEX
 */
#include "bsp_gpio.h"
// 上传的状态信号:PK2
// 电量指示灯:PK6
// 充电or 放电状态:PK5
// 高压指示灯:PK4

/***********************************************************************************
 * Port A set up
 ***********************************************************************************
 * Pin/Function                 P1SEL1  P1SEL0      Function
 * ------------                 ------  ------      --------
 * PA 0 = VeREF-                1       1           VeREF-/ADC正向输入参考电压/未使用
 *****/
void Set_gpio_mode()
{
  GPIODirModeSet
}
