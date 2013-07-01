// Source file has defines of some commonly used utility functions. 
// Changelog:
// 		 2013-06-25 - Initial release.
//     2012-07-20 - First version.
// Author: Harinadha Reddy Chintalapalli

/**
 * \include
 */
#include "utils.h"
#include "math.h"
/**
* \defgroup Utils
* \{
*/

/** \addtogroup Utils_Function
 *  \{
*/

/**
 * \brief  Compute two integer value a and b such that n = a * b. It used to
 *                  setup the timer to generate an IRQ with a specified frequency n.
 * \param  n : the specified frequency
 * \param  a : prescaler factor
 * \param  b : period factor
 * \retval None
*/

void prvFindFactors(u32 n, u16 *a, u16 *b)
{
	/** This function is copied from the ST STR7 library and is
	 * copyright STMicroelectronics.  Reproduced with permission.
	*/

	u16 b0;
	u16 a0;
	long err, err_min=n;


	*a = a0 = ((n-1)/0xffff) + 1;
	*b = b0 = n / *a;

	for (; *a < 0xffff-1; (*a)++)
	{
		*b = n / *a;
		err = (long)*a * (long)*b - (long)n;
		if (abs(err) > (*a / 2))
		{
			(*b)++;
			err = (long)*a * (long)*b - (long)n;
		}
		if (abs(err) < abs(err_min))
		{
			err_min = err;
			a0 = *a;
			b0 = *b;
			if (err == 0) break;
		}
	}

	*a = a0;
	*b = b0;
}

/**
 * \brief  Inserts a delay time
 * \param  nCount: specifies the delay time length
 * \retval None
*/

void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}
/**
 *  \brief Copy one buffer to another
 *  \param pBufferOrigin
 *  \param pBufferDestination
 *  \param NBOfBytes : number of bytes to copy
 *  \retval None
 */
void CopyBuffer(unsigned char* pBufferOrigin, unsigned char* pBufferDestination,  u8 NBOfBytes)
{
  while(NBOfBytes!=0)
  {
    NBOfBytes--;
    *pBufferDestination=*pBufferOrigin;
    pBufferDestination++;
    pBufferOrigin++;
  }
}

/**
 *  \brief Put in a buffer as u8 an s16.
 *  \param refvalue : s16 recipient
 *  \param pBufferDestination : u8 buffer destination
 *  \retval None
 */

void s16_to_u8_buffer(s16* refvalue, unsigned char* pBufferDestination)
{
  u16 tmp=0x00;
  tmp = (u16)(*refvalue); 
  *pBufferDestination=(u8)(tmp>>8);
  pBufferDestination++; 
  *pBufferDestination=(u8)tmp;

}

/**
 *  \brief Put in a buffer as u8 an u16.
 *  \param refvalue : u16 recipient
 *  \param pBufferDestination : u8 buffer destination
 *  \retval None
 */
void u16_to_u8_buffer(u16* refvalue, unsigned char* pBufferDestination)
{
  u16 tmp=0x00;
  tmp = (u16)(*refvalue);
  *pBufferDestination=(u8)(tmp>>8);
  pBufferDestination++;
  *pBufferDestination=(u8)tmp;
}

/**
 *  \brief Compares two buffers
 *  \param pBuffer1 : buffers to be compared
 *  \param pBuffer2 : buffers to be compared
 *  \retval TestStatus :  PASSED if  pBuffer1 is identical to pBuffer2; or FAILED if pBuffer1 id differs from pBuffer2
 */

TestStatus Buffercmp(u32* pBuffer1, u32* pBuffer2, u16 BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}

/**
 *  \brief Fills buffer with user predefined data
 *  \param pBuffer : pointer on the Buffer to fill
 *  \param BufferLenght : size of the buffer to fill
 *  \param Offset : first value to fill on the Buffer
 *  \retval None
 */

void Fill_Buffer(u32 *pBuffer, u16 BufferLenght, u32 Offset)
{
  u16 index = 0;

  /* Put in global buffer same values */
  for (index = 0; index < BufferLenght; index++ )
  {
    pBuffer[index] = index + Offset;
  }
}
/**
 *  \brief Checks if a buffer has all its values are equal to zero
 *  \param pBuffer :  buffer to be compared.
 *  \param BufferLenght : buffer's length
 *  \retval TestStatus : PASSED pBuffer values are zero
 *                 		 FAILED At least one value from pBuffer buffer is diffrent
 *                          from zero.
 */

TestStatus eBuffercmp(u32* pBuffer, u16 BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer != 0x00)
    {
      return FAILED;
    }

    pBuffer++;
  }

  return PASSED;
}

/**
  * brief: Copy the four byte of a float, into a buffer starting from a position
  * @param pBuffer
  * @param t : the float to copy into the buffer
  * @retval None
  */

void Float_To_Buffer(float t, u8* pBuffer)
{
 char* s;
 s=(char*)(&t);

 for(int i=3;i>=0;i--)
 {
  pBuffer[i]=(char) (*s);
  s++;
 }
}

/**
  * brief: Copy four byte of a buffer into a float.
  * @param t : the float to copy into the buffer
  * @param pBuffer
  * @retval None
  */
void Buffer_To_Float(u8* pBuffer, float t)
{
 char* s;
 s=(char*)(&t);
 for(int i=0;i<4;i++)
 {
  *s=pBuffer[i];
   s++;
 }
}
/**
  * brief:  Returns 16-bit fixed-point value from specified floating-point value with saturation.
  * @param Qvals  : Number of fraction bits.
  * @retval None
  */
s16 FloatToFixed(float floatValue, uint8_t q)
{
    int temp = (int)((floatValue) * (int)(1 << ((int)q)));
    if (temp > 32767) temp = 32767;
    else if (temp < -32768) temp = -32768;
    return (s16)temp;
}

/**
  * brief: Returns floating-point value from specified 16-bit fixed-point.
  * @param Qvals : Number of fraction bits.
  * @retval None
  */
float FixedToFloat(s16 fixedValue, uint8_t q)
{
    return ((float)(fixedValue) / (float)(1 << ((int)q)));
}

/**
  * brief: Insert Checksum into the Packet.
  * @param pInputPacket : Input Bytes
  * @param pOutputPacket : Output Bytes
  * @param NBofBytes: Total number of bytes in Inputpacket
  * @retval None
  */
void InsertChecksum(unsigned char* pInputPacket, unsigned char* pOutputPacket, u8 NBofBytes)
{
    //*InputPacket[NBofBytes - 1] = 0;                        // zero current checksum
   u8  ChecksumByte = 0; 
   while(NBofBytes!=0)
   {
     NBofBytes--;
     ChecksumByte += *pInputPacket;
     *pOutputPacket=*pInputPacket;
     pOutputPacket++;     
     pInputPacket++;
   }
   *pOutputPacket = ChecksumByte;
}
/**
  * brief: Encodes packet with consecutive right shifts so that the msb of each encoded byte is clear. The msb of the final byte is set to indicate the end of the packet.
  * @param pInputPacket : Input packet
  * @param pEncodedPacket : Encoded packet
  * @param NBofBytes: Total number of bytes in Inputpacket
  * @retval None
  */
 void EncodePacket(unsigned char* pInputPacket,unsigned char* pEncodedPacket, u8 NBofBytesIn, u8* NBofBytesOut)
 {
    u8 encodedPacketLength = (int8_t)(ceil((((float)NBofBytesIn * 1.125f)) + 0.125f));
    u8 shiftRegister[24]={0}; //*pNBofBytesOut = encodedPacketLength;
    CopyBuffer(pInputPacket, &shiftRegister[0], NBofBytesIn);       // copy encoded packet to shift register
    for(int i = 0; i < encodedPacketLength; i++)
    {
        RightShiftByteArray(shiftRegister, encodedPacketLength);   // right shift to clear msb of byte i
        pEncodedPacket[i] = shiftRegister[i];                      // save encoded byte i
        shiftRegister[i] = 0;                                      // clear byte i in shift register
    }
    pEncodedPacket[encodedPacketLength-1] |= 0x80;                 // set msb of framing byte
    *NBofBytesOut = encodedPacketLength;
 }
/**
  * brief: Right shift a byte array by 1 bit. The LSB of byte x becomes the msb of byte x+1
  * @param pInByteArray : Input byte array, output also in the same byte array
  * @param Length: Length of pInByteArray
  * @retval None
  */
void RightShiftByteArray(unsigned char* pInByteArray, u8 Length)
{
    pInByteArray[Length - 1] >>= 1;
    for (int i = Length - 2; i >= 0; i--)
    {
        if ((pInByteArray[i] & 0x01) == 0x01) pInByteArray[i + 1] |= 0x80;
        pInByteArray[i] >>= 1;
    }
}
/**
  * brief: Decodes a packet with consecutive left shifts so that the msb of each encoded byte is removed.
  * @param pInputPacket : Input packet
  * @param pDecodedPacket : Decoded packet
  * @param NBofBytes: Total number of bytes in Inputpacket
  * @retval None
  */
void DecodePacket(unsigned char* pInputPacket,unsigned char* pDecodedPacket, u8 NBofBytes )
{
    u8 decodedPacketLength = (int8_t)(floor(((float)NBofBytes - 0.125f) / 1.125f));
    u8 shiftRegister[24] = {0};
    for (int i = NBofBytes - 1; i >= 0; i--)
    {
        shiftRegister[i] = pInputPacket[i];
        LeftShiftByteArray(shiftRegister, NBofBytes);
    }
    CopyBuffer(shiftRegister, pDecodedPacket, decodedPacketLength);   
}
/**
  * brief: Left shifts a byte array by 1 bit. The msb of byte x becomes the lsb of byte x-1.
  * @param pInByteArray : Input byte array, output also in the same byte array
  * @param Length: Length of pInByteArray
  * @retval None
  */
void LeftShiftByteArray(unsigned char* pInByteArray, u8 Length)
{
    pInByteArray[0] <<= 1;
    for (int i = 1; i < Length; i++)
    {
        if ((pInByteArray[i] & 0x80) == 0x80) // If the MSB of any byte is 1, We have to do OR operation with its previous byte
            pInByteArray[i - 1] |= 0x01;         
        pInByteArray[i] <<= 1;
    }
}

void QuaternionProduct(float* pInQuat1, float* pInQuat2, float* pOutQuat)
{
    pOutQuat[0] = pInQuat1[0] * pInQuat2[0] - pInQuat1[1] * pInQuat2[1] - pInQuat1[2] * pInQuat2[2] - pInQuat1[3] * pInQuat2[3];  
    pOutQuat[1] = pInQuat1[0] * pInQuat2[1] + pInQuat1[1] * pInQuat2[0] + pInQuat1[2] * pInQuat2[3] - pInQuat1[3] * pInQuat2[2];
    pOutQuat[2] = pInQuat1[0] * pInQuat2[2] - pInQuat1[1] * pInQuat2[3] + pInQuat1[2] * pInQuat2[0] + pInQuat1[3] * pInQuat2[1];
    pOutQuat[3] = pInQuat1[0] * pInQuat2[3] + pInQuat1[1] * pInQuat2[2] - pInQuat1[2] * pInQuat2[1] + pInQuat1[3] * pInQuat2[0];
}

void QuaternionInverse(float* pInQuat,float* pOutQuat)
{ 
    float norm = (float)sqrt(pInQuat[0] * pInQuat[0] + pInQuat[1] * pInQuat[1] + pInQuat[2] * pInQuat[2] + pInQuat[3] * pInQuat[3]);
    pOutQuat[0] = pInQuat[0]; pOutQuat[1] = -pInQuat[1]; pOutQuat[2] = -pInQuat[2]; pOutQuat[3] = -pInQuat[3]; // Conjugate
    if (norm != 0.0f)
    {// handle NaN
        norm = 1 / norm;        // use reciprocal for division
        pOutQuat[0] *= norm;
        pOutQuat[1] *= norm;
        pOutQuat[2] *= norm;
        pOutQuat[3] *= norm;
    }    
}

void Delay_ms(u16 ms)
{
        volatile u16 i, j;
        for(; ms>0; ms--)
                for (i = 0; i < 500; i++)
                        for (j = 0; j < 5; j++);

        /*
        u16 i;
        for(i=0; i<ms; i++)
                delay_1ms();
                */
}

/**
 * }
 */ /* end of group Utils_Function */

/**
 * @}
 */ /* end of group Utils */

/******************* (c) COPYRIGHT 2013 Harinadha Reddy Chintalapalli *****END OF FILE****/
