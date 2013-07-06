//MPU9150 I2C library for ARM STM32F103xx Microcontrollers - Header file 
// has defines to choose MPU9150/MPU6050 INT pin. 
// 25/06/2013 by Harinadha Reddy Chintalapalli <harinath.ec@gmail.com>
// Changelog:
//     2013-07-06 - Interrupts configuration changed to pin PC9 .
//     2013-06-25 - Initial release. 
/* ============================================================================================
MPU9150 device I2C library code for ARM STM32F103xx is placed under the MIT license
Copyright (c) 2013 Harinadha Reddy Chintalapalli

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================================================================
*/
#ifndef __MPU9150_INTERRUPTS_H
#define __MPU9150_INTERRUPTS_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f10x.h"

typedef enum 
{  
  MPU9150_INT_MODE_GPIO = 0,
  MPU9150_INT_MODE_EXTI = 1
} MPU9150intMode_TypeDef;

/** Define the Interrupt pin */  
#define MPU9150_INT_PIN                          GPIO_Pin_9   
#define MPU9150_INT_GPIO_PORT                    GPIOC                 
#define MPU9150_INT_GPIO_CLK                     RCC_APB2Periph_GPIOC  
#define MPU9150_INT_EXTI_LINE                    EXTI_Line9      
#define MPU9150_INT_EXTI_PORT_SOURCE             GPIO_PortSourceGPIOC  
#define MPU9150_INT_EXTI_PIN_SOURCE              GPIO_PinSource9    
#define MPU9150_INT_EDGE                         EXTI_Trigger_Rising 
#define MPU9150_INT_EXTI_IRQn                    EXTI9_5_IRQn       

#define MPU9150_INT_EXTI_PREEMPTION_PRIORITY     14
#define MPU9150_INT_EXTI_SUB_PRIORITY            0

void MPU9150_Interrupt_Init(MPU9150intMode_TypeDef Button_Mode);
void MPU9150_Interrupt_Cmd(FunctionalState NewState);
uint32_t MPU9150_Get_INTpin_State();

#ifdef __cplusplus
}
#endif

#endif /* __MPU9150_INTERRUPTS_H */