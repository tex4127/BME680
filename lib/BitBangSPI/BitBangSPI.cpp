////////////////////////////////////////////////////
///
//  copyright @ 2024
//  File -> BitBangSPI.cpp
//  Author -> Jacob Garner, mlgtex4127@gmail.com
//
//  Dexcription: This is the source file for the BitBangSPI library
//  for use with microcontoller devices using SPI communication via ShiftIn and ShiftOut.
//
///
////////////////////////////////////////////////////

#include "BitBangSPI.h"

/// @brief Standard BigBangSPI class constructor
/// @param mosiPin Master Out Slave In pin for data transfer
/// @param misoPin Master In Slave Out pin for data transfer
/// @param sckPin Clock pin to be pulsed for data transfer
/// @param _dataOrder 
BBSPIClass::BBSPIClass(uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin, uint8_t _dataOrder, uint8_t _dataMode) :
mosi(mosiPin), miso(misoPin), sck(sckPin), dataOrder(_dataOrder), dataMode(_dataMode)
{}

/// @brief On stack initializer for the BBSPIClass instance
void BBSPIClass::begin(){
    if(!_begun){
        Serial.printf("BBSPIClass Begin -> %u, %u, %u\n", mosi, miso, sck);
        pinMode(mosi, OUTPUT);
        pinMode(miso, INPUT);
        pinMode(sck, OUTPUT);
        digitalWrite(mosi, HIGH);
        BBSPIClass::setDataMode(dataMode);
        _begun = false;
    }
}

/// @brief Getter for the begun status of the BBSPIClass Instance
/// @return Boolean status of 
bool BBSPIClass::getBegun(){
    return _begun;
}

/// @brief Writes one byte of data via ShiftOut
/// @param byte data to be written
void BBSPIClass::write(uint8_t byte){
    shiftOut(mosi, sck, dataOrder, byte);
}

/// @brief Reads one byte of data via ShiftIn
/// @return byte of data read
uint8_t BBSPIClass::read(){
    return shiftIn(miso, sck, dataOrder);
}

/// @brief Sets the clock polarity (not Phase) for our communication
/// @param mode SPI_MODEX for 0-3
void BBSPIClass::setDataMode(uint8_t mode){
    if(mode == SPI_MODE0 || mode == SPI_MODE1){
        digitalWrite(sck, LOW);
    }else{
        digitalWrite(sck, HIGH);
    }
}
