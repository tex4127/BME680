////////////////////////////////////////////////////
///
//  copyright @ 2024
//  File -> BitBangSPI.h
//  Author -> Jacob Garner, mlgtex4127@gmail.com
//
//  Dexcription: This is the header file for the BitBangSPI library
//  for use with microcontoller devices using SPI communication via ShiftIn and ShiftOut.
//
///
////////////////////////////////////////////////////

#ifndef __BITBANGSPI_HH__
#define __BITBANGSPI_HH__

#include "Arduino.h"
#include "SPI.h"

class BBSPIClass{
    public:
    BBSPIClass(uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin, uint8_t _dataOrder = MSBFIRST, uint8_t _dataMode = SPI_MODE0);
    void begin();
    bool getBegun();
    void write(uint8_t byte);
    uint8_t read();
    void setDataMode(uint8_t mode);
    private:
    bool _begun = false;
    uint8_t mosi;
    uint8_t miso;
    uint8_t sck;
    uint8_t dataOrder;
    uint8_t dataMode;
    uint8_t cpol;
    //uint8_t cpha; //Due to the nature of BitBanged spi, we do not need to worry about the CPHA
};

#endif