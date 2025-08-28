////////////////////////////////////////////////////
///
//  copyright @ 2024
//  File -> SoftwareSPI.h
//  Author -> Jacob Garner, mlgtex4127@gmail.com
//
//  Dexcription: This is the header file for the SoftwareSPI library
//  for use with microcontoller devices using SPI communication when
//  the Hardware SPI Bus(es) is not available.
//
///
////////////////////////////////////////////////////

#ifndef __SOFTWARESPI_HH__
#define __SOFTWARESPI_HH__

#include "Arduino.h"
#include "SPI.h"

class SSPI : public SPIClass{
    public:
        SSPI(uint8_t cs, uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t dataOrder = MSBFIRST, uint8_t dataMode = SPI_MODE0, uint32_t freq = 4000000);
        void begin();
        void end();
        uint8_t transfer(uint8_t);
        void transfer(uint8_t* buffer, uint32_t size); 
        //uint16_t transfer16(uint16_t); // Might come back to this at a later date
    private:
        uint8_t cs;
        uint8_t mosi;
        uint8_t miso;
        uint8_t sck;
        uint8_t freq;
        uint8_t dataOrder;
        uint8_t dataMode;
        uint32_t freq;
};

#endif