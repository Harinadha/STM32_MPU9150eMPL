#include "mpu9150_interrupts.h"

void MPU9150_Interrupt_Init(MPU9150int_TypeDef MPU9150int, MPU9150intMode_TypeDef MPU9150int_Mode)
{

  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable MPU9150int GPIO clocks */
  RCC_APB2PeriphClockCmd(MPU9150_INT_GPIO_CLK, ENABLE);
  
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

void MPU9150_Interrupt_Cmd(MPU9150int_TypeDef MPU9150int, FunctionalState NewState)
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