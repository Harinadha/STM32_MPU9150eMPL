#include "STM32_I2C.h"

//#define I2C_TIMEOUT_MS  (2500)
typedef enum {
    STATE_WAITING,
    STATE_READING,
    STATE_WRITING
} stm32_i2c_state;

typedef struct {
    volatile stm32_i2c_state state;
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
/*-------------------------------------------------------------------------*/
extern CPAL_InitTypeDef MPU6050_DevStructure;
CPAL_TransferTypeDef  MPU6050_TXTransfer = { 
                        /* Initialize TX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};

CPAL_TransferTypeDef  MPU6050_RXTransfer = {
                        /* Initialize RX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};
static void MPU6050_StructInit(void);
static void MPU6050_Init(void);
//--------------------------------------------------------------------------

int stm32_i2c_write(unsigned char slave_addr,  unsigned char reg_addr, unsigned char length, unsigned char *data)
{    
  /* Disable all options */
  MPU6050_DevStructure.wCPAL_Options = 0;
  
  /* point to CPAL_TransferTypeDef structure */
  MPU6050_DevStructure.pCPAL_TransferTx = &MPU6050_TXTransfer;
  
  /* Configure transfer parameters */  
  MPU6050_DevStructure.pCPAL_TransferTx->wNumData = length;
  MPU6050_DevStructure.pCPAL_TransferTx->pbBuffer = data ;
  MPU6050_DevStructure.pCPAL_TransferTx->wAddr1   = (uint32_t)slave_addr;
  MPU6050_DevStructure.pCPAL_TransferTx->wAddr2   = (uint32_t)reg_addr;
  
  /* Write Operation */
  if(CPAL_I2C_Write(&MPU6050_DevStructure) == CPAL_PASS)
  {
    while ((MPU6050_DevStructure.CPAL_State != CPAL_STATE_READY) && (MPU6050_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
    
    if (MPU6050_DevStructure.CPAL_State == CPAL_STATE_ERROR)
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
	//uint16_t tmp = 0;
  if (!i2c.enabled)
        return -1;
    if (!length)
        return 0;
  /* Disable all options */
  MPU6050_DevStructure.wCPAL_Options = 0;

  /* point to CPAL_TransferTypeDef structure */
  MPU6050_DevStructure.pCPAL_TransferRx = &MPU6050_RXTransfer;
  
  /* Configure transfer parameters */  
  MPU6050_DevStructure.pCPAL_TransferRx->wNumData = length;
  MPU6050_DevStructure.pCPAL_TransferRx->pbBuffer = data ;
  MPU6050_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)slave_addr;
  MPU6050_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)reg_addr;
  
  /* Read Operation */
  if(CPAL_I2C_Read(&MPU6050_DevStructure) == CPAL_PASS)
  {
    while ((MPU6050_DevStructure.CPAL_State != CPAL_STATE_READY) && (MPU6050_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
    { }
  }
  
  /* Store MPU6050_I2C received data */
//  tmp = (uint16_t)(MPU6050_Buffer[0] << 8);
//  tmp |= MPU6050_Buffer[1];
  
  /* return a Reg value */
  //return (uint16_t)tmp;  
	return 0;
}

static void MPU6050_StructInit(void)
{
			/* Initialize CPAL peripheral */
  CPAL_I2C_Init(&MPU6050_DevStructure);  
	/* Set I2C clock speed */
  MPU6050_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C_SPEED;
  			 
#ifdef MPU6050_IT
  /* Select Interrupt programming model and disable all options */
  MPU6050_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
  MPU6050_DevStructure.wCPAL_Options  = 0;
#else
  /* Select DMA programming model and activate TX_DMA_TC and RX_DMA_TC interrupts */
  MPU6050_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_DMA;
  MPU6050_DevStructure.wCPAL_Options  = CPAL_OPT_DMATX_TCIT | CPAL_OPT_DMARX_TCIT ;
#endif /* MPU6050_IT */  

  /* point to CPAL_TransferTypeDef structure */
  MPU6050_DevStructure.pCPAL_TransferTx = &MPU6050_TXTransfer;
  MPU6050_DevStructure.pCPAL_TransferRx = &MPU6050_RXTransfer;
}
static void MPU6050_Init(void)
{
	CPAL_I2C_Init(&MPU6050_DevStructure);
}