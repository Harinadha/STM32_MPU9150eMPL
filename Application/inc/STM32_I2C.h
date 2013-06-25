#ifndef _STM32_I2C_H_
#define _STM32_I2C_H_

#ifdef __cplusplus
 extern "C" {
#endif 
	 
/* Includes */
//#include "stm32f10x.h"	 
#include "cpal_i2c.h"

#define MPU6050_DevStructure                I2C2_DevStructure  

/* Select clock Speed */
/* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
   input clock) must be a multiple of 10 MHz */

#define I2C_SPEED                        300000

/* Select interrupt programming model : By default DMA programming model is selected.
 To select interrupt programming model uncomment this define */
//#define MPU6050_IT

/* Maximum Timeout values for waiting until device is ready for communication.*/
   
#define MPU6050_TIMEOUT        ((uint32_t)0x3FFFF)
	
	 
/**
 *  @brief	Set up the I2C port and configure the stm32 as the master.
 *  @return	0 if successful.
 */
int stm32_i2c_enable(void);
/**
 *  @brief  Disable I2C communication.
 *  This function will disable the I2C hardware and should be called prior to
 *  entering low-power mode.
 *  @return 0 if successful.
 */
int stm32_i2c_disable(void);
/**
 *  @brief      Write to a device register.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be written to.
 *  @param[in]  length      Number of bytes to write.
 *  @param[out] data        Data to be written to register.
 *
 *  @return     0 if successful.
 */
int stm32_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char *data);
/**
 *  @brief      Read from a device.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be read from.
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Data from register.
 *
 *  @return     0 if successful.
 */
int stm32_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);
	 
	 
#endif /* __STM32_I2C_H */
