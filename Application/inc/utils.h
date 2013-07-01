// Source file has defines of some commonly used utility functions. 
// Changelog:
// 		 2013-06-25 - Initial release.
//     2012-07-20 - First version.
// Author: Harinadha Reddy Chintalapalli


/** \def
 *  Define to prevent recursive inclusion
 */
#ifndef __UTILS_H
#define __UTILS_H

#ifdef __cplusplus
 extern "C" {
#endif 
  

/**
 * \include
 * */
#include "stm32f10x.h"


/** \addtogroup Utils
 *  \{
*/

/** @enum TestStatus
*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/**\def
 *  Computes the absolute value of its argument \a x.
*/
#define abs(a) ((a)>0?(a):-(a))

/**\def
 *  Define the UID Address and a pointer to it.
*/
#define U_ID_Base_Register_Address   (0x1FFFF7E8)
#define MCU_ID ((const unsigned char *)(U_ID_Base_Register_Address))

/** \addtogroup Utils_Function
 *  \{
*/
void prvFindFactors(u32 n, u16 *a, u16 *b);
void Delay(vu32 nCount);
void CopyBuffer(unsigned char* pBufferOrigin, unsigned char* pBufferDestination,  u8 NBOfBytes);
void s16_to_u8_buffer(s16* refvalue, unsigned char* pBufferDestination);
void u16_to_u8_buffer(u16* refvalue, unsigned char* pBufferDestination);
void Fill_Buffer(u32 *pBuffer, u16 BufferLenght, u32 Offset);
TestStatus Buffercmp(u32* pBuffer1, u32* pBuffer2, u16 BufferLength);
TestStatus eBuffercmp(u32* pBuffer, u16 BufferLength);
void Float_To_Buffer(float t, u8* pBuffer);
void Buffer_To_Float(u8* pBuffer,float t);
s16 FloatToFixed(float floatValue, uint8_t q);
float FixedToFloat(s16 fixedValue, uint8_t q);
void InsertChecksum(unsigned char* pInputPacket, unsigned char* pOutputPacket, u8 NBofBytes);
void EncodePacket(unsigned char* pInputPacket,unsigned char* pEncodedPacket, u8 NBofBytes, u8* NBofBytesOut );
void DecodePacket(unsigned char* pInputPacket,unsigned char* pDecodedPacket, u8 NBofBytes );
void RightShiftByteArray(unsigned char* pInByteArray, u8 Length);
void LeftShiftByteArray(unsigned char* pInByteArray, u8 Length);
void QuaternionProduct(float* pInQuat1, float* pInQuat2, float* pOutQuat);
void QuaternionInverse(float* pInQuat,float* pOutQuat);
void Delay_ms(u16 ms);

static inline void delay_us(uint32_t us)
{
	us *= 14;
	us--;
	asm(" mov r0, us \n"
	"loop: subs r0, #1 \n"
	" bhi loop \n");
}

/**
 * \}
 */ /* end of group Utils_Function */ 

/**
 * \}
 */ /* end of group Utils */ 
#endif /* __UTILS_H */

/******************* (c) COPYRIGHT 2013 Harinadha Reddy Chintalapalli *****END OF FILE****/
