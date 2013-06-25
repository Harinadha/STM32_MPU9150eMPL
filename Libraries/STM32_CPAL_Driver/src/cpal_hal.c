/**
  ******************************************************************************
  * @file    Libraries/STM32_CPAL_Driver/src/cpal_hal.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   This file contains all the functions for the CPAL_HAL common
  *          firmware layer.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
/* If STM32F10X family is used */
#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)\
 || defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_XL) || defined (STM32F10X_CL)
 #include "stm32f10x.h"
#endif

/* If STM32L1XX family is used */   
#ifdef STM32L1XX_MD
 #include "stm32l1xx.h"
#endif

/* If STM32F2XX family is used */   
#ifdef STM32F2XX
 #include "stm32f2xx.h"
#endif

#include "misc.h"
#include "cpal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure NVIC Priority Group.
  * @param  None.
  * @retval None. 
  */
void CPAL_HAL_NVICInit(void)
{
 /* Set NVIC Group Priority */
  NVIC_PriorityGroupConfig (CPAL_NVIC_PRIOGROUP);
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/


