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

class SSPIClass{ //: public SPIClass{
    public:
        SSPIClass(uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin, uint8_t _dataOrder = MSBFIRST, uint8_t _dataMode = SPI_MODE0, uint32_t _spiClockDiv = SPI_CLOCK_DIV128);
        void begin();
        void end();
        /// @brief Transfers 1 byte of data in and 1 byte of data out 
        /// @param word Byte to be transfered out
        /// @return Received byte from SPI Device
        inline uint8_t transfer(uint8_t word){
            //Serial.printf("SSPIClass::transfer(word) -> %u\n",word);
            //Serial.printf("bv->");
            uint8_t o = 0;  //Copy of the word
            if(dataOrder == MSBFIRST){
            word = 
                (word&0x01) << 7 |
                (word&0x02) << 5 |
                (word&0x04) << 3 |
                (word&0x08) << 1 |
                (word&0x10) >> 1 |
                (word&0x20) >> 3 |
                (word&0x40) >> 5 |
                (word&0x80) >> 7;
            }
			uint8_t bv = 0;
            uint8_t d = fDelay>>1;            //Delay divider value divded by 2, we perform a read and a write, each delay must be half the total delay
            uint8_t cpst = cpol ? HIGH : LOW;    //Copy of cpolarity to trigger change in clock states
            for (uint8_t bsh = 0; bsh < 8; bsh++){
                //Shift Clock State based on Clock Phase
                if(cpha){
                    cpst ^= 1;
                    digitalWrite(sck, cpst);
                    wait(d);
                }
                //Write Current bit to MOSI
                digitalWrite(mosi, (word & (1<<bsh)) ? HIGH : LOW);
                wait(d);
                //Shift Clock
                cpst ^= 1;
                digitalWrite(sck, cpst);
                //Read Bit
                bv = digitalRead(miso);
                //Serial.printf("%u|",bv);
                if(dataOrder == MSBFIRST){
                    o <<= 1;
                    o |= bv;
                }else{
                    o >>= 1;
                    o |= bv << 7;
                }
                wait(d);
                //Shift Clock State based on Clock Phase
                if(!cpha){
                    cpst ^= 1;
                    digitalWrite(sck, cpst);
                }
            }
            //Serial.printf("\nSSPIClass::transfer(word) o -> %u\n", o);
            return o;
        }
        inline uint16_t transfer16(uint16_t word){
            uint16_t o = 0;
            if(dataOrder == MSBFIRST){
                word = 
                    (word&0x0001) << 15 |
                    (word&0x0002) << 13 |
                    (word&0x0004) << 11 |
                    (word&0x0008) << 9 |
                    (word&0x0010) << 7 |
                    (word&0x0020) << 5 |
                    (word&0x0040) << 3 |
                    (word&0x0080) << 1 |
                    (word&0x0100) >> 1 |
                    (word&0x0200) >> 3 |
                    (word&0x0400) >> 5 |
                    (word&0x0800) >> 7 |
                    (word&0x1000) >> 9 |
                    (word&0x2000) >> 11 |
                    (word&0x4000) >> 13 |
                    (word&0x8000) >> 15;
            }
            uint16_t bv = 0;
            uint8_t d = fDelay >> 1;
            uint8_t cpst = cpol ? 1 : 0;
            for (uint8_t bsh = 0; bsh < 16; bsh++){
                if(cpha){
                    cpst ^= 1;
                    digitalWrite(sck, cpst);
                    wait(d); 
                }
                //Write current bit to mosi
                digitalWrite(mosi, (word & (1 << bsh)) ?  HIGH : LOW);
                wait(d);
                //Shift clock
                cpst ^= 1;
                digitalWrite(sck, cpst);
                bv = digitalRead(miso);
                //Store the bit in o based on data order
                if(dataOrder == MSBFIRST){
                    o <<= 1;
                    o |= bv;
                }else{
                    o >>= 1;
                    o |= bv << 15;
                }
                wait(d);
                if(!cpha){
                    cpst ^= 1;
                    digitalWrite(sck, cpst);
                }
            }
            return 0;
        }
        /// @brief Transfers a buffer of data to our SPI device and stores the response into the buffer
        /// @param buffer buffer of data to be written to to the SPI Device
        /// @param size number of bytes to be written
        inline void transfer(void* buffer, uint32_t size){
            if(size == 0)
            return;
            uint8_t *buf = (uint8_t *)buffer;
            for (uint32_t i = 0 ; i < size; i++){
                *buf = SSPIClass::transfer(*buf);
            }
        }
        //uint16_t transfer16(uint16_t); // Might come back to this at a later date
        void setClockDivider(uint32_t spiClockDiv);
        void setDataMode(uint8_t mode);
    private:
        uint8_t mosi;
        uint8_t miso;
        uint8_t sck;
        uint8_t dataOrder;
        uint8_t dataMode;
        uint8_t cpol;
        uint8_t cpha;
        uint32_t spiClockDiv;
        uint8_t fDelay;
        bool _begun;
        void wait(uint8_t);
        
};

#endif
