//MPU9150 I2C library for ARM STM32F103xx Microcontrollers - Main source file
//Has bit, byte and buffer I2C R/W functions
// 25/06/2013 by Harinadha Reddy Chintalapalli <harinath.ec@gmail.com>
// Changelog:
//     2013-06-25 - Initial release. Thanks to Invensense for releasing MPU driver for msp430.
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

#include "stm32_CPAL_mpu9150.h"

typedef struct {
    //volatile stm32_i2c_state state;
    /* First slave register. */
    unsigned char slave_reg;
    /* 0 if slave register has not been written yet. */
    unsigned char slave_reg_written;
    unsigned char *data;
    unsigned short length;
    unsigned char enabled;
} stm32_i2c_info;

static stm32_i2c_info i2c = {
    .enabled = 0
};

CPAL_TransferTypeDef  MPU9150_TXTransfer = { 
                        /* Initialize TX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};

CPAL_TransferTypeDef  MPU9150_RXTransfer = {
                        /* Initialize RX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};

extern CPAL_InitTypeDef MPU9150_DevStructure;

__IO uint32_t  MPU9150_Timeout = MPU9150_TIMEOUT; 

//------------------------------------------
static void MPU9150_StructInit(void);
static void MPU9150_Init(void);
//------------------------------------------

void MPU9150_Config(void)
{
	MPU9150_StructInit ();
  MPU9150_Init();
}
void MPU9150_DeInit(void)
{
    /* Initialize CPAL peripheral */
  CPAL_I2C_DeInit(&MPU9150_DevStructure);
}
static void MPU9150_StructInit(void)
{
	/* Set CPAL structure parameters to their default values */  
  CPAL_I2C_StructInit(&MPU9150_DevStructure);
  
  /* Set I2C clock speed */
  MPU9150_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C_SPEED;
  
#ifdef MPU9150_IT
  /* Select Interrupt programming model and disable all options */
  MPU9150_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
  MPU9150_DevStructure.wCPAL_Options  = 0;
#else
  /* Select DMA programming model and activate TX_DMA_TC and RX_DMA_TC interrupts */
  MPU9150_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_DMA;
  MPU9150_DevStructure.wCPAL_Options  = CPAL_OPT_DMATX_TCIT | CPAL_OPT_DMARX_TCIT ;
#endif /* IOE_IT */
  
  /* point to CPAL_TransferTypeDef structure */
  MPU9150_DevStructure.pCPAL_TransferTx = &MPU9150_TXTransfer;
  MPU9150_DevStructure.pCPAL_TransferRx = &MPU9150_RXTransfer;
}

static void MPU9150_Init(void)
{
  /* Initialize CPAL peripheral */
  CPAL_I2C_Init(&MPU9150_DevStructure);
}

int stm32_i2c_write(unsigned char slave_addr,  unsigned char reg_addr, unsigned char length, unsigned char *data)
{    
  /* Disable all options */
  MPU9150_DevStructure.wCPAL_Options = 0;
  
  /* point to CPAL_TransferTypeDef structure */
  MPU9150_DevStructure.pCPAL_TransferTx = &MPU9150_TXTransfer;
  
  /* Configure transfer parameters */  
  MPU9150_DevStructure.pCPAL_TransferTx->wNumData = (uint32_t)length;
  MPU9150_DevStructure.pCPAL_TransferTx->pbBuffer = data ;
  MPU9150_DevStructure.pCPAL_TransferTx->wAddr1   = (uint32_t)slave_addr;
  MPU9150_DevStructure.pCPAL_TransferTx->wAddr2   = (uint32_t)reg_addr;
  
  /* Write Operation */
  if(CPAL_I2C_Write(&MPU9150_DevStructure) == CPAL_PASS)
  {
    while ((MPU9150_DevStructure.CPAL_State != CPAL_STATE_READY) && (MPU9150_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
    
    if (MPU9150_DevStructure.CPAL_State == CPAL_STATE_ERROR)
    {
      return -1;
    }
  }
  else
  {
    return -1;
  }
  
  return 0;
}

int stm32_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    if (!length)
        return 0;
  /* Disable all options */
  MPU9150_DevStructure.wCPAL_Options = 0;

  /* point to CPAL_TransferTypeDef structure */
  MPU9150_DevStructure.pCPAL_TransferRx = &MPU9150_RXTransfer;
  
  /* Configure transfer parameters */  
  MPU9150_DevStructure.pCPAL_TransferRx->wNumData = (uint32_t)length;
  MPU9150_DevStructure.pCPAL_TransferRx->pbBuffer = data ;
  MPU9150_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)slave_addr;
  MPU9150_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)reg_addr;
  
  /* Read Operation */
  if(CPAL_I2C_Read(&MPU9150_DevStructure) == CPAL_PASS)
  {
    while ((MPU9150_DevStructure.CPAL_State != CPAL_STATE_READY) && (MPU9150_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
		
		if (MPU9150_DevStructure.CPAL_State == CPAL_STATE_ERROR)
    {
      return -1;
    }
  }else
		return -1;
  
	return 0;
}

static uint32_t MPU9150_Status (void)
{
  MPU9150_DevStructure.pCPAL_TransferTx = &MPU9150_TXTransfer;     
  MPU9150_DevStructure.pCPAL_TransferTx->wAddr1 = (uint32_t)MPU9150_ADDR;
  
  return CPAL_I2C_IsDeviceReady(&MPU9150_DevStructure);
}

/**
  * @brief  Checks the MPU9150 status.
  * @param  None
  * @retval ErrorStatus: MPU9150 Status (ERROR or SUCCESS).
  */
ErrorStatus MPU9150_GetStatus(void)
{  
  /* Test if MPU9150 is ready */
  while ((MPU9150_Status() == CPAL_FAIL) && MPU9150_Timeout)  
  {
    MPU9150_Timeout--;
  }  
  /* If MPU9150 is not responding return ERROR */
  if (MPU9150_Timeout == 0)
  {
    return ERROR;
  }  
  /* In other case return SUCCESS */
  return SUCCESS;  
}
