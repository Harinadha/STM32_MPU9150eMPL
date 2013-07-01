//MPU9150 I2C library for ARM STM32F103xx Microcontrollers - Header file has defines
// to choose I2C peripheral, speed & pins. 
// 25/06/2013 by Harinadha Reddy Chintalapalli <harinath.ec@gmail.com>
// Changelog:
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
#ifndef __STM32_CPAL_MPU9150_H
#define __STM32_CPAL_MPU9150_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "cpal_i2c.h"

typedef enum {
	  MPU9150_STATE_IDLE			= 0x01,
    MPU9150_STATE_WRITING		= 0x02,
    MPU9150_STATE_READING		= 0x03,
    MPU9150_STATE_ERROR 		= 0x04,
} MPU9150_StateTypeDef;
/*========= CPAL_MPU9150_TypeDef =========*/
/* MPU9150 Device structure definition */
typedef struct  
{
  CPAL_InitTypeDef* MPU9150_CPALStructure;  /*!< Pointer on a CPAL Device structure relative to the device 
                                             instantiated to communicate with EEPROM */
  
  uint8_t MPU9150MemoryAddrMode;            /*!< Bit-field value specifying Memory Addressing Mode. Can be 
                                             any combination of following values: 
                                             sEE_Memory_Addressing_Mode_Defines */ 
  
  __IO MPU9150_StateTypeDef MPU9150State;       /*!< Holds the current State of the EEPROM device. The state 
                                             parameter can be one of the following values: sEE_State_Enum  */
  
} MPU9150_InitTypeDef; 

#define MPU9150_DevStructure             I2C2_DevStructure  
#define I2C_SPEED                        100000 // 100KHz

#define MPU9150_TIMEOUT                  ((uint32_t)0x3FFFF)

#define MPU9150_ADDR           						(0x68)<<1   /*!< MPU9150 i2c address */
	 
void MPU9150_Config(void);	 
void MPU9150_DeInit(void);
ErrorStatus MPU9150_GetStatus(void);

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

#ifdef __cplusplus
}
#endif

#endif /* __STM32_CPAL_MPU9150_H */
/******************* (C) COPYRIGHT 2013 Harinadha Reddy Chintalapalli *****END OF FILE****/
