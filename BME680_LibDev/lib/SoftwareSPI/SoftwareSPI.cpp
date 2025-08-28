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

SSPI::SSPI(uint8_t cs, uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t dataOrder, uint8_t dataMode, uint32_t freq) :
cs(cs), mosi(mosi), miso(miso), dataOrder(dataOrder), dataMode(dataMode), freq(freq)
{}

void SSPI::begin(){
    pinMode(cs, OUTPUT);
    pinMode(mosi, OUTPUT);
    pinMode(miso, INPUT);
    pinMode(sck, OUTPUT);
}

/// @brief Releases all pins attached to this device
void SSPI::end(){
    pinMode(cs, INPUT);
    pinMode(mosi, INPUT);
    pinMode(miso, INPUT);
    pinMode(sck, INPUT);
}
