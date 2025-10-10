/**
 * Copyright (c) 2025 Jacob Garner
 * 
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * @file BME68X.cpp
 * @date 2025-01-01
 * @version v1.0.0
 */

#include "BME68X.h"

//***********************************************************//
//
//  BME68X Interface Specific Methods
//
//***********************************************************//

/// @brief Begin method, Starts interface comms and initialized chip
void BME68X::begin(){
    memset(&sensorConfig, 0, sizeof(BME68X_Config_t));
    switch (intfType){
        case 0x01:
            interface.i2c.m_i2c->begin();
            write = BME68X_I2CWrite;
            read = BME68X_I2CRead;
        break;
        case 0x02:
            interface.spi.m_spi->begin();
            pinMode(interface.spi.cs, OUTPUT);
            digitalWrite(interface.spi.cs, HIGH); delay(1);
            digitalWrite(interface.spi.cs, LOW); delay(1);
            digitalWrite(interface.spi.cs, HIGH);
            write = BME68X_SPIWrite;
            read = BME68X_SPIRead;
        break;
        case 0x04:
            interface.sspi.m_spi->begin();
            pinMode(interface.sspi.cs, OUTPUT);
            digitalWrite(interface.sspi.cs, HIGH); delay(1);
            digitalWrite(interface.sspi.cs, LOW); delay(1);
            digitalWrite(interface.sspi.cs, HIGH);
            write = BME68X_SSPIWrite;
            read = BME68X_SSPIRead;
        break;
        case 0x08:
            interface.bbspi.m_spi->begin();
            pinMode(interface.bbspi.cs, OUTPUT);
            digitalWrite(interface.bbspi.cs, HIGH); delay(1);
            digitalWrite(interface.bbspi.cs, LOW); delay(1);
            digitalWrite(interface.bbspi.cs, HIGH);
            write = BME68X_BBSPIWrite;
            read = BME68X_BBSPIRead;
        break;
        default:
            _begun = false;
            return;
        break;
    }
    delay_us = BME68X_delayUs;
    int8_t st = init();
    _begun = (BME_STATUS_OK == st) ? true : false ;
}

/// @brief Getter for Begun status
/// @return begun status of the sensor
bool BME68X::begun(){
    return this->_begun;
}

/// @brief Sensor Initializer
/// @return Sensor Status
int8_t BME68X::init(){
    int8_t rslt = BME_STATUS_OK;
    softReset();
    //Get Chip ID
    rslt = read(BME68X_REGISTER_CHIPID, &ChipID, 1, &interface);
    if(BME_STATUS_OK != rslt || BME68X_CHIP_ID != ChipID){
        return BME_E_DEV_NOT_FOUND;
    }
    rslt = read(BME68X_REGISTER_VARIANTID, &VariantID, 1, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    } 
}

/// @brief Soft Resets the sensor
/// @return status of the chip 
int8_t BME68X::softReset(){
    uint8_t regAddr = BME68X_REGISTER_SOFTRESET;
    uint8_t regData = 0xB6; //Soft Reset Command
    uint8_t regStatus = 0;
    uint8_t tryRun = 5;
    int8_t rslt = write(regAddr, &regData, 1, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    /* Might want to add a while loop to ensure reset has occured */
    delay_us(2000, &interface);
    if(intfType == 0x01){
        return rslt;
    }
    /* rslt = get_mem_page() It should reset to 0x00*/
    return rslt;
}

int8_t BME68X::getCalibData(){
    return 0;
}

/// @brief Gets the memory page from the Sensor for SPI Communication
/// @return Sensor Status
int8_t BME68X::getMemPage(){
    int8_t rslt;
    uint8_t regData;
    rslt = read(BME68X_REGISTER_MEM_PAGE, &regData, 1, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    memPage = regData & 0x10; //memory page mask
    return rslt;
}

/// @brief Sets the appropriate Sensor memory page based on the regAddr being written to
/// @param regAddr Address to be written to which determines which mem page to be set
/// @return Sensor Status
int8_t BME68X::setMemPage(uint8_t regAddr){
    int8_t rslt = BME_STATUS_OK;
    uint8_t regData;
    uint8_t t_memPage;
    if(regAddr > 0x7F){
        t_memPage = BME68X_SPI_MEMPAGE_1;
    }else{
        t_memPage = BME68X_SPI_MEMPAGE_0;
    }
    if(memPage != t_memPage){
        memPage = t_memPage;
        //The below method params was | 0x80, this is moved to the read/write commands directly
        rslt = read(BME68X_REGISTER_MEM_PAGE, &regData, 1, &interface);
        if(BME_STATUS_OK != rslt){
            return rslt;
        }
        regData &= ~0x10; //0x10 is the mask for SPI Memory Page
        regData |= (memPage & 0x10);
        rslt = write(BME68X_REGISTER_MEM_PAGE, &regData, 1, &interface);
        return rslt;
    }
    return rslt;
}

/// @brief Reads the variant ID for the Sensor (Gas Related for BME680)
/// @return Sensor Status
int8_t BME68X::getVariantId(){
    int8_t rslt = BME_STATUS_OK;
    rslt = read(BME68X_REGISTER_VARIANTID, &VariantID, 1, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    return rslt;
}

/// @brief Writes the sensor mode to SLEEP
/// @return Sensor Status
int8_t BME68X::putSensorToSleep(){
    int8_t rslt;
    uint8_t curMode;
    rslt = read(BME68X_REGISTER_CONTROL , &curMode, 1, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    rslt = softReset();
    return rslt;
}



//***********************************************************//
//
//  BME680 Specific Methods
//
//***********************************************************//

BME680::BME680(uint8_t addr, TwoWire *wire){
    interface.i2c = {wire, addr};
    intfType = 0x01;
}

BME680::BME680(uint8_t cs, SPIClass *spi){
    interface.spi = {spi, cs};
    intfType = 0x02;
}

BME680::BME680(uint8_t cs, SSPIClass *sspi){
    interface.sspi = {sspi, cs};
    intfType = 0x04;
}

BME680::BME680(uint8_t cs, BBSPIClass *bbspi){
    interface.bbspi = {bbspi, cs};
    intfType = 0x08;
}

//***********************************************************//
//
//  BME688 Specific Methods
//
//***********************************************************//

/*
BME688::BME688(uint8_t addr, TwoWire *wire){
    interface.i2c = {wire, addr};
    intfType = 0x01;
}

BME688::BME688(uint8_t cs, SPIClass *spi){
    interface.spi = {spi, cs};
    intfType = 0x02;
}

BME688::BME688(uint8_t cs, SSPIClass *sspi){
    interface.sspi = {sspi, cs};
    intfType = 0x04;
}

BME688::BME688(uint8_t cs, BBSPIClass *bbspi){
    interface.bbspi = {bbspi, cs};
    intfType = 0x08;
}
*/

//***********************************************************//
//
//  Interface Methods for API on BME
//
//***********************************************************//

/// @brief Delays for a set number of microseconds diuring idel states
/// @param period Period in microseconds for the standby delay
/// @param intfPtr pointer to the BME_Interface_u comm
void BME68X_delayUs(uint32_t period, void *intfPtr){
    delayMicroseconds(period);
}

/// @brief Hardware SPI Write Method
/// @param regAddr BME280 Regiser to which data is been written
/// @param regData Data to be written to the register (typically on byte, could be more)
/// @param len Number of data values to be written
/// @param intfPtr Pointer to our Digital Communcation interface
int8_t BME68X_SPIWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr){
#ifdef __DEBUG__
    Serial.printf("\nSPIWrite(%02x,%02x,%u,&intf)", regAddr, regData[0], len);
#endif
    BME_Interface_u *comm = NULL;
    int8_t rslt = BME_STATUS_OK;
    if(intfPtr){
        comm = (BME_Interface_u *)intfPtr;
        if(comm->spi.m_spi){
            digitalWrite(comm->spi.cs, LOW);
            comm->spi.m_spi->transfer(regAddr & ~0x80); //For all SPI Writes we must set bit 7 low
#ifdef __DEBUG__
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr);
#endif
            for (uint32_t i = 0; i < len; i++){
                comm->spi.m_spi->transfer(regData[i]);
#ifdef __DEBUG__
                Serial.printf("%02x(%03u),", regData[i], regData[i]);
#endif
            }
            digitalWrite(comm->spi.cs, HIGH);
#ifdef __DEBUG__
            Serial.printf("}\n");
#endif
        }
        else{
            rslt = BME_E_COMM_FAIL;
        }
    }else{
        rslt = BME_E_COMM_FAIL;
    }
    return rslt;
}

/// @brief Hardware SPI Read Method
/// @param regAddr BME280 Register to which data will be read from 
/// @param regData Data to be read from the register (uint8_t array)
/// @param len Number of data values (attempted) to be read
/// @param intfPtr Pointer to our Digital Communication interface
int8_t BME68X_SPIRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr){
#ifdef __DEBUG__
    Serial.printf("\nSPIRead(%02x,%02x,%u,&intf)", regAddr, regData[0], len);
#endif
    BME_Interface_u *comm = NULL;
    int8_t rslt = BME_STATUS_OK;
    if(intfPtr){
        comm = (BME_Interface_u *)intfPtr;
        if(comm->spi.m_spi){
            digitalWrite(comm->spi.cs, LOW);
            comm->spi.m_spi->transfer(regAddr | 0x80); //for all SPI Reads we must force bit 7 high
#ifdef __DEBUG__
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr);
#endif
            memset(regData, 0xFF, len); //Reset our data
            for(uint32_t i = 0 ; i < len; i++){
                regData[i] = comm->spi.m_spi->transfer(0xFF);
#ifdef __DEBUG__
                Serial.printf("%02x(%03u),", regData[i], regData[i]);
#endif
            }
            digitalWrite(comm->spi.cs, HIGH);
#ifdef __DEBUG__
            Serial.printf("}\n");
#endif
        }else{
            rslt = BME_E_COMM_FAIL;
        }
    } else{
        rslt = BME_E_COMM_FAIL;
    }
    return rslt;
}

/// @brief Software SPI Write Method
/// @param regAddr BME280 Regiser to which data is been written
/// @param regData Data to be written to the register (typically on byte, could be more)
/// @param len Number of data values to be written
/// @param intfPtr Pointer to our Digital Communcation interface
int8_t BME68X_SSPIWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr){
#ifdef __DEBUG__
    Serial.printf("\nSSPIWrite(%02x,%02x,%u,&intf)", regAddr, regData[0], len);
#endif
    BME_Interface_u *comm = NULL;
    int8_t rslt = BME_STATUS_OK;
    if(intfPtr){
        comm = (BME_Interface_u *)intfPtr;
        if(comm->sspi.m_spi){
            digitalWrite(comm->sspi.cs, LOW);
            comm->sspi.m_spi->transfer(regAddr & ~0x80);
#ifdef __DEBUG__
            Serial.printf("\n\tregAddr -> %02x Data -> {");
#endif
            for (uint32_t i = 0; i < len; i ++){
#ifdef __DEBUG__
                Serial.printf("%02x(%03u),", regData[i], regData[i]);
#endif
                comm->sspi.m_spi->transfer(regData[i]);
            }
            digitalWrite(comm->sspi.cs, HIGH);
#ifdef __DEBUG__
            Serial.printf("}");
#endif
        }
        else{
            rslt = BME_E_COMM_FAIL;
        }
    }else{
        rslt = BME_E_COMM_FAIL;
    }
    return rslt;
}

/// @brief Software SPI Read Method
/// @param regAddr BME280 Register to which data will be read from 
/// @param regData Data to be read from the register (uint8_t array)
/// @param len Number of data values (attempted) to be read
/// @param intfPtr Pointer to our Digital Communication interface
int8_t BME68X_SSPIRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr){
#ifdef __DEBUG__
    Serial.printf("\nSSPIRead(%02x,%02x,%u,&intf)", regAddr, regData[0], len);
#endif
    BME_Interface_u *comm = NULL;
    int8_t rslt = BME_STATUS_OK;
    if(intfPtr){
        comm = (BME_Interface_u *)intfPtr;
        if(comm->sspi.m_spi){
            digitalWrite(comm->sspi.cs, LOW);
            comm->sspi.m_spi->transfer(regAddr | 0x80);
#ifdef __DEBUG__
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr);
#endif
            memset(regData, 0xFF, len);
            for (uint32_t i = 0; i < len; i++){
                regData[i] = comm->sspi.m_spi->transfer(0xFF);
#ifdef __DEBUG__
                Serial.printf("%02x(%u),", regData[i]);
#endif
            }
            digitalWrite(comm->sspi.cs, HIGH);
#ifdef __DEBUG__
            Serial.printf("}");
#endif
        }else{
            rslt = BME_E_COMM_FAIL;
        }
    }else{
        rslt = BME_E_COMM_FAIL;
    }
    return rslt;
}

/// @brief Software SPI Write Method
/// @param regAddr BME280 Regiser to which data is been written
/// @param regData Data to be written to the register (typically on byte, could be more)
/// @param len Number of data values to be written
/// @param intfPtr Pointer to our Digital Communcation interface
int8_t BME68X_BBSPIWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr){
#ifdef __DEBUG__
    Serial.printf("\nBBSPIWrite(%02x,%02x,%u,&intf)", regAddr, regData[0], len);
#endif
    BME_Interface_u *comm = NULL;
    int8_t rslt = BME_STATUS_OK;
    if(intfPtr){
        comm = (BME_Interface_u *)intfPtr;
        if(comm->bbspi.m_spi){
            digitalWrite(comm->bbspi.cs, LOW);
#ifdef __DEBUG__
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr);
#endif
            comm->bbspi.m_spi->write(regAddr & ~0x80);
            for(uint32_t i = 0; i < len; i++){
                comm->bbspi.m_spi->write(regData[i]);
            }
            digitalWrite(comm->bbspi.cs, HIGH);
#ifdef __DEBUG__
            Serial.printf("}");
#endif
        }else{
            rslt = BME_E_COMM_FAIL;
        }
    }else{
        rslt = BME_E_COMM_FAIL;
    }
    return rslt;
}

/// @brief Software SPI Read Method
/// @param regAddr BME280 Register to which data will be read from 
/// @param regData Data to be read from the register (uint8_t array)
/// @param len Number of data values (attempted) to be read
/// @param intfPtr Pointer to our Digital Communication interface
int8_t BME68X_BBSPIRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr){
#ifdef __DEBUG__
    Serial.printf("\nBBSPIRead(%02x,%02x,%u,&intf)", regAddr, regData[0], len);
#endif
    BME_Interface_u *comm = NULL;
    int8_t rslt = BME_STATUS_OK;
    if(intfPtr){
        comm = (BME_Interface_u *)intfPtr;
        if(comm->bbspi.m_spi){
            digitalWrite(comm->bbspi.cs, LOW);
#ifdef __DEBUG__
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr);
#endif
            comm->bbspi.m_spi->write(regAddr | 0x80);
            memset(regData, 0x00, len);
            for (uint32_t i = 0; i < len; i++){
                regData[i] = comm->bbspi.m_spi->read();
#ifdef __DEBUG__
                Serial.printf("%02x(%u)", regData[i]);
#endif
            }
            digitalWrite(comm->bbspi.cs, HIGH);
#ifdef __DEBUG__
            Serial.printf("}");
#endif
        }else{
            rslt = BME_E_COMM_FAIL;
        }
    }else{
        rslt = BME_E_COMM_FAIL;
    }
    return rslt;
}

/// @brief Hardware I2C Write Method
/// @param regAddr BME280 Regiser to which data is been written
/// @param regData Data to be written to the register (typically on byte, could be more)
/// @param len Number of data values to be written
/// @param intfPtr Pointer to our Digital Communcation interface
int8_t BME68X_I2CWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr){
    BME_Interface_u *comm = NULL;
    int8_t rslt = BME_STATUS_OK;
    if(intfPtr){
        comm = (BME_Interface_u *)intfPtr;
        if(comm->i2c.m_i2c){
            comm->i2c.m_i2c->beginTransmission(comm->i2c.addr);
            comm->i2c.m_i2c->write(regAddr);
            for (uint32_t i = 0 ; i < len; i++){
                comm->i2c.m_i2c->write(regData[i]);
            }
            comm->i2c.m_i2c->endTransmission();
        }else{
            rslt = BME_E_COMM_FAIL;
        }
    }else{
        rslt = BME_E_COMM_FAIL;
    }
    return rslt;
}

/// @brief Hardware I2C Read Method
/// @param regAddr BME280 Register to which data will be read from 
/// @param regData Data to be read from the register (uint8_t array)
/// @param len Number of data values (attempted) to be read
/// @param intfPtr Pointer to our Digital Communication interface
int8_t BME68X_I2CRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr){
    BME_Interface_u *comm = NULL;
    int8_t rslt = BME_STATUS_OK;
    if(intfPtr){
        comm = (BME_Interface_u *)intfPtr;
        if(comm->i2c.m_i2c){
            comm->i2c.m_i2c->beginTransmission(comm->i2c.addr);
            comm->i2c.m_i2c->write(regAddr);
            if(comm->i2c.m_i2c->endTransmission()){
                rslt = false;
            }
            comm->i2c.m_i2c->requestFrom(comm->i2c.addr, len);
            for(uint32_t i = 0; (i < len) && comm->i2c.m_i2c->available(); i++){
                regData[i] = comm->i2c.m_i2c->read();
            }
        }
        else{
            rslt = BME_E_COMM_FAIL;
        }
    }else{
        rslt = BME_E_COMM_FAIL;
    }
    return rslt;
}
