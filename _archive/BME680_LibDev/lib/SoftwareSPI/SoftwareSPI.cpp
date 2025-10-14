////////////////////////////////////////////////////
///
//  copyright @ 2024
//  File -> SoftwareSPI.cpp
//  Author -> Jacob Garner, mlgtex4127@gmail.com
//
//  Dexcription: This is the header file for the SoftwareSPI library
//  for use with microcontoller devices using SPI communication when
//  the Hardware SPI Bus(es) is not available.
//
///
////////////////////////////////////////////////////

#include "SoftwareSPI.h"

/*
SSPI::SSPI(uint8_t cs, uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t dataOrder, uint8_t dataMode, uint32_t spiClockDiv) :
cs(cs), mosi(mosi), miso(miso), dataOrder(dataOrder), dataMode(dataMode), spiClockDiv(spiClockDiv) {}
*/
SSPIClass::SSPIClass(uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin, uint8_t _dataOrder, uint8_t _dataMode, uint32_t _spiClockDiv){
    mosi = mosiPin;
    miso = misoPin;
    sck = sckPin;
    dataOrder = _dataOrder;
    dataMode = _dataMode;
    spiClockDiv = _spiClockDiv;
    _begun = false;
}

/// @brief Stack initializer for the Software SPI Object.
void SSPIClass::begin(){
    if(_begun){
        return;
    }
    pinMode(mosi, OUTPUT);
    pinMode(miso, INPUT);
    pinMode(sck, OUTPUT);
	digitalWrite(mosi, HIGH);
    //We need to handle our SPI MODE | CPOL and CPHA
    setDataMode(dataMode);
    setClockDivider(spiClockDiv);
    _begun = true;
}

/// @brief Releases all pins attached to this device
void SSPIClass::end(){
    if(_begun){
        pinMode(mosi, INPUT);
        pinMode(miso, INPUT);
        pinMode(sck, INPUT);
        _begun = false;
    }
}

/// @brief Sets our clock divider based on the SPI Clock of the device
/// @param spiClockDiv SPI_CLOCK_DIV# value defined by preprocessor macros
void SSPIClass::setClockDivider(uint32_t clockDiv){
    spiClockDiv = clockDiv;
    switch (spiClockDiv){
        case SPI_CLOCK_DIV2:
            fDelay = 2;
        break;
        case SPI_CLOCK_DIV4:
            fDelay = 4;
        break;
        case SPI_CLOCK_DIV8:
            fDelay = 8;
        break;
        case SPI_CLOCK_DIV16:
            fDelay = 16;
        break;
        case SPI_CLOCK_DIV32:
            fDelay = 32;
        break;
        case SPI_CLOCK_DIV64:
            fDelay = 64;
        break;
        case SPI_CLOCK_DIV128:
            fDelay = 128;
        break;
        default:
            setClockDivider(SPI_CLOCK_DIV128);
        break;
    }
}

/// @brief Sets the Data Mode for clock polarity and phase. Forces state of clock to cpol
/// @param mode Pre-defined SPI_MODE# values 0-3
void SSPIClass::setDataMode(uint8_t mode){
    //  CPOL    |   States   
    //  0       |   Idle state is low, pulse is high
    //  1       |   Idle state is high, pulse is low
    //  CPHA    |   Sampling Time
    //  0       |   Data is sampled on the CPOL change and shifted out on when CPOL returns to idle
    //  1       |   Data is sampled on the CPOL return to idle state and shifted out when state changes 
    dataMode = mode;
    switch (dataMode){
        case SPI_MODE0:
            cpol = 0;   //Idle State low, Pulse State high
            cpha = 0;   //Sample on rising edge, shifted out on falling edge
        break;
        case SPI_MODE1:
            cpol = 0;   //Idle State low, Pulse State high
            cpha = 1;   //Sample on falling edge, shifted out on rising edge
        break;
        case SPI_MODE2:
            cpol = 1;   //Idle State high, Pulse State low
            cpha = 0;   //Sample on falling edge, shifted out on rising edge
        break;
        case SPI_MODE3:
            cpol = 1;   //Idle State High, Pulse State low
            cpol = 1;   //Sample on rising edge, shifted out on falling edge
        break;
        default:
            setDataMode(SPI_MODE0);
        break;
    }
    digitalWrite(sck, cpol);
}

/// @brief Runs ASM nop to delay number of cycles 
/// @param d number of cycles to delay/run nop for
void SSPIClass::wait(uint8_t d){
    for (uint8_t i = 0; i < d; i++){
        asm volatile("nop");
    }
}
