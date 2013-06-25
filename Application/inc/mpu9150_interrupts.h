#ifndef __MPU9150_INTERRUPTS_H
#define __MPU9150_INTERRUPTS_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f10x.h"
	 
typedef enum 
{  
  MPU9150_INT = 0,
} MPU9150int_TypeDef;

typedef enum 
{  
  MPU9150_INT_MODE_GPIO = 0,
  MPU9150_INT_MODE_EXTI = 1
} MPU9150intMode_TypeDef;

/** Define the Interrupt pin */  
#define MPU9150_INTn                                  1  
  
#define MPU9150_INT_PIN                          GPIO_Pin_12           

#define MPU9150_INT_GPIO_PORT                    GPIOC                 
#define MPU9150_INT_GPIO_CLK                     RCC_APB2Periph_GPIOC  

#define MPU9150_INT_EXTI_LINE                    EXTI_Line12           

#define MPU9150_INT_EXTI_PORT_SOURCE             GPIO_PortSourceGPIOC  

#define MPU9150_INT_EXTI_PIN_SOURCE              GPIO_PinSource12      

#define MPU9150_INT_EDGE                         EXTI_Trigger_Rising 
#define MPU9150_INT_EXTI_IRQn                    EXTI15_10_IRQn      

#define MPU9150_INT_EXTI_PREEMPTION_PRIORITY     14

#define MPU9150_INT_EXTI_SUB_PRIORITY            0

void MPU9150_Interrupt_Init(MPU9150int_TypeDef MPU9150int, MPU9150intMode_TypeDef Button_Mode);
void MPU9150_Interrupt_Cmd(MPU9150int_TypeDef MPU9150int, FunctionalState NewState);
uint32_t MPU9150_Get_INTpin_State();

#ifdef __cplusplus
}
#endif

#endif /* __MPU9150_INTERRUPTS_H */