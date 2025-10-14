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

#ifndef __SOFT_SPI_HH__
#define __SOFT_SPI_HH__

#include "Arduino.h"

class SoftSPI_Dev
{
  public:
  SoftSPI_Dev(uint8_t cs = 10, uint8_t mosi = 11, uint8_t miso = 12, uint8_t sck = 13);
  void begin(uint8_t bitOrder = MSBFIRST);
  
  //Write Methods (buffered or single values)
  void write8(uint8_t data);
  void write16(uint16_t data);
  void write24(uint32_t data);
  void write32(uint32_t data);

  //Read Methods (buffered or single values)
  uint8_t read8();
  uint16_t read16();
  uint32_t read24();
  uint32_t read32();
  
  private:
  uint8_t _cs;
  uint8_t _mosi;
  uint8_t _miso;
  uint8_t _sck;
  bool _begun;
  uint8_t _bitOrder;
  void read(uint8_t* buf, size_t len);
  void write(uint8_t* buf, size_t len);
};

#endif