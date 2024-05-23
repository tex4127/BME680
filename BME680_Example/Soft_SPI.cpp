//////////////////////////////////////////////////////////
///
//  Copyright (c) 2024
//  Author: Jacob Garner, mlgtex4127@gmail.com
//  
//  Filename: BME680.h
//
//  Description:
//  This code is designed to leverage Software SPI communications for specific uses
//  on PCBs where the Hardware SPI pins are being used for other purposes
//
//  We are working from basic Arduino methods to keep code overhead low
///
//////////////////////////////////////////////////////////


#include "Soft_SPI.h"

/*!
 *  @brief Default Constructor | Sets all pins to their specified values
 */
SoftSPI_Dev::SoftSPI_Dev(uint8_t cs, uint8_t mosi, uint8_t miso, uint8_t sck)
{
  _cs = cs;
  _mosi = mosi;
  _miso = miso;
  _sck = sck;
  _begun = false;
}

/*!
 *  @brief Initializer of the object on the stack, sets pinmodes and configurations
 *  @param bitOrder the bit order for the device will use for communications
 */
void SoftSPI_Dev::begin(uint8_t bitOrder)
{
  //Set our pins
  pinMode(_cs, OUTPUT);
  pinMode(_mosi, OUTPUT);
  pinMode(_miso, INPUT);
  pinMode(_sck, OUTPUT);
  digitalWrite(_cs, HIGH);
  _bitOrder = (bitOrder == LSBFIRST) ? LSBFIRST : MSBFIRST;
  _begun = true;
}

/*!
 *  @brief Writes a buffer of bytes to a SPI Connected Deivce
 */
void SoftSPI_Dev::write(uint8_t* buf, size_t len)
{
  digitalWrite(_cs, LOW);
  for(size_t i = 0; i < len; i++)
  {
    shiftOut(_mosi, _sck, _bitOrder, buf[i]);
  }
  digitalWrite(_cs, HIGH);
}
  
/*!
 *  @brief Writes a single byte to a SPI Connected Device
 */
void SoftSPI_Dev::write8(uint8_t data)
{
  SoftSPI_Dev::write(&data, 1);
}
  
  
/*!
 *  @brief Writes 2 bytes or one 16 bit value to a SPI Connected Device
 */
void SoftSPI_Dev::write16(uint16_t data)
{
  //create a new buffer for our data then write it to our device
  uint8_t _buf[2] = {(uint8_t)(data >> 8), (uint8_t)data};
  SoftSPI_Dev::write(_buf, sizeof(_buf)/sizeof(_buf[0]));
}

/*!
 *  @brief Writes 3 bytes or one 24 bit value to a SPI Connected Device
 *  @param data This is 24 bits sotred in the lower 24 bits of a 32 bit unisnged integer
 */
void SoftSPI_Dev::write24(uint32_t data)
{
  uint8_t _buf[3] = { (uint8_t)(data >> 16),
                      (uint8_t)(data >> 8),
                      (uint8_t)data};
  SoftSPI_Dev::write(_buf, sizeof(_buf)/sizeof(_buf[0]));
}
  
/*!
 *  @brief Writes 4 bytes or one 32 bit value to a SPI Connected Device
 */
void SoftSPI_Dev::write32(uint32_t data)
{
  uint8_t _buf[4] = { (uint8_t)(data >> 24),
                      (uint8_t)(data >> 16),
                      (uint8_t)(data >> 8),
                      (uint8_t)data};
  SoftSPI_Dev::write(_buf, sizeof(_buf)/sizeof(_buf[0]));
}

/*!
 *  @brief reads available bytes from the SPI Device into a buffer
 */
void SoftSPI_Dev::read(uint8_t* buf, size_t len)
{
  //Wait for our miso line to go low before reading
  while(digitalRead(_miso)){/*Do Nothing*/}
  digitalWrite(_cs, LOW);
  for(size_t i = 0; i < len; i++)
  {
    buf[i] = shiftIn(_miso, _sck, _bitOrder);
  }
  digitalWrite(_cs, HIGH);
}

/*!
 *  @brief reads 1 byte or one 8 bit value from a SPI Connected Device
 */
uint8_t SoftSPI_Dev::read8()
{
  uint8_t inData = 0;
  SoftSPI_Dev::read(&inData, 1);
  return inData;
}

uint16_t SoftSPI_Dev::read16()
{
  uint8_t inData[2];
  SoftSPI_Dev::read(inData, sizeof(inData)/sizeof(inData[0]));
  return (inData[0] << 8) | inData[1];
}

uint32_t SoftSPI_Dev::read24()
{
  uint8_t inData[3];
  SoftSPI_Dev::read(inData, sizeof(inData)/sizeof(inData[0]));
  return (inData[0] << 16) | (inData[1] << 8) | inData[2];
}

uint32_t SoftSPI_Dev::read32()
{
  uint8_t inData[4];
  SoftSPI_Dev::read(inData, sizeof(inData)/sizeof(inData[0]));
  return (inData[0] << 24) | (inData[1] << 16) | (inData[2] << 8) | inData[3];
}

