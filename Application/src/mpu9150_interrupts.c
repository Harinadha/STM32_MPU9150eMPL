//MPU9150 I2C library for ARM STM32F103xx Microcontrollers - Source file
//Has functions for configuring MPU9150/MPU6050 INT pin.
// 25/06/2013 by Harinadha Reddy Chintalapalli <harinath.ec@gmail.com>
// Changelog:
//     2013-07-06 - A bug in interrupts configuration fixed.
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
================================================================================================
*/
#include "mpu9150_interrupts.h"

void MPU9150_Interrupt_Init(MPU9150intMode_TypeDef MPU9150int_Mode)
{

  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable MPU9150int GPIO clocks */
  RCC_APB2PeriphClockCmd(MPU9150_INT_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
  
  /* Configure MPU9150int pin as input floating */
  GPIO_InitStructure.GPIO_Pin = MPU9150_INT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(MPU9150_INT_GPIO_PORT, &GPIO_InitStructure);

  if (MPU9150int_Mode == MPU9150_INT_MODE_EXTI)   
  {
    /* Connect MPU9150int EXTI Line to MPU9150int GPIO Pin */
    GPIO_EXTILineConfig(MPU9150_INT_EXTI_PORT_SOURCE, MPU9150_INT_EXTI_PIN_SOURCE);
    
    /* Configure MPU9150int EXTI line */ 
    EXTI_InitStructure.EXTI_Line = MPU9150_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = MPU9150_INT_EDGE;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  }  
}

void MPU9150_Interrupt_Cmd(FunctionalState NewState)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable and set MPU9150int EXTI Interrupt priority */
  NVIC_InitStructure.NVIC_IRQChannel = MPU9150_INT_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MPU9150_INT_EXTI_PREEMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = MPU9150_INT_EXTI_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;
  NVIC_Init(&NVIC_InitStructure);  
}

uint32_t MPU9150_Get_INTpin_State()
{
  return GPIO_ReadInputDataBit(MPU9150_INT_GPIO_PORT, MPU9150_INT_PIN);
}