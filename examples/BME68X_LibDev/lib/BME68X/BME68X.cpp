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
#ifdef __DEBUG__
    Serial.printf("BME68X::begin()\n");
#endif
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
    sensorData = {0};
    int8_t st = init();
    _begun = (BME_STATUS_OK == st) ? true : false ;
}

/// @brief Getter for Begun status
/// @return begun status of the sensor
bool BME68X::begun(){
    return this->_begun;
}

/// @brief Sets the oversampling, mode 
/// @param mode Mode to set 
/// @param osTemp Oversampling rate for temperature measurments
/// @param osPress Oversampling rate for pressure measurements
/// @param osHum Oversampling rate for Humidity measurements
/// @param filterSize Filter size set by filter coeffs
void BME68X::setConfig(BME68X_Mode_e mode,
                BME68X_OS_e osTemp,
                BME68X_OS_e osPress,
                BME68X_OS_e osHum,
                BME68X_FilterSize_e filterSize){
    uint8_t regAddr[] = {BME68X_REGISTER_CONTROLHUMID, 
                        BME68X_REGISTER_CONTROL, 
                        BME68X_REGISTER_CONFIG};
    uint8_t regData[] = {BME68X_CTRLHUM_REG(osHum),
                        BME68X_CTRLMEAS_REG(osTemp, osPress, mode),
                        BME68X_CONFIG_REG(filterSize, 0)};
    uint8_t regData_s = 0x00;
    //Put the sensor to sleep
    if(BME_STATUS_OK != write(BME68X_REGISTER_CONTROL, &regData_s, 1, &interface)){
#ifdef __DEBUG__
        Serial.printf("\nError Putting Device to sleep | BME68X::setConfig()\n");
#endif
        return;
    }
    if(BME_STATUS_OK != write(regAddr[0], &regData[0], 1, &interface)){
#ifdef __DEBUG__
        Serial.printf("\nError writting Humid Control Reg\n>>regAddr %02x\n>>regData %02x\n", regAddr[0], regData[0]);
#endif
        return;
    }
    if(BME_STATUS_OK != write(regAddr[2], &regData[2], 1, &interface)){
#ifdef __DEBUG__
        Serial.printf("\nError writting Config Reg\n>>regAddr %02x\n>>regData %02x\n", regAddr[2], regData[2]);
#endif
        return;
    }
    if(BME_STATUS_OK != write(regAddr[1], &regData[1], 1, &interface)){
#ifdef __DEBUG__
        Serial.printf("\nError writting Measure Control Reg\n>>regAddr %02x\n>>regData %02x\n", regAddr[1], regData[1]);
#endif
        return;
    }
    return;
}

/// @brief Reads the pertinent config registers and updates the config strucutre
/// @return pointer to config structure
const BME68X_Config_t BME68X::getConfig(){
    uint8_t regData[4] = {0};
    if(BME_STATUS_OK != read(BME68X_REGISTER_CONTROLHUMID, regData, 4, &interface)){
#ifdef __DEBUG__
        Serial.printf("\nError reading Config | BME68X::getConfig()\n");
#endif
    }
    //regAddr[0] == 0x72 | regData[0] == os_hum
    sensorConfig.os_hum = regData[0] & 0x07;
    //regAddr[2] == 0x74 | regData[2] == os_temp, os_press, mode
    sensorConfig.os_temp = (regData[2] & 0xE0) >> 5;
    sensorConfig.os_press = (regData[2] & 0x1C) >> 2;
    sensorConfig.mode = (regData[2] & 0x03);
    //regAddr[3] == 0x75 | regData[3] = filter, spi_3w_en
    sensorConfig.filter = (regData[3] & 0x1C) >> 2;
    //We dont care about 3 wire SPI
    return sensorConfig;
}

int8_t BME68X::readSensor(){
    uint8_t regAddr = BME68X_REGISTER_PRESS_ADC_0;
    uint8_t regData[8] = {0};
    int8_t rslt = BME_STATUS_OK;
    rslt = read(regAddr, regData, 8, &interface);
    if(BME_STATUS_OK != rslt){
#ifdef __DEBUG__
        Serial.printf("\nError BME68X::readSensor()\n>>regAddr %02x\n", regAddr);
#endif
    }
    //Now parse our data
    int32_t adc_p = ((regData[0] << 16) | (regData[1] << 8) | regData[2]) >> 4;
    int32_t adc_t = ((regData[3] << 16) | (regData[4] << 8) | regData[5]) >> 4;
    int32_t adc_h = (regData[6] << 8) | regData[7];
    //Now compensate them, reference page 25 and 26 of the datasheet
    sensorData.temperature = compTemperature(adc_t) / 100.0f;   //return is 1u = 0.01 degC
    sensorData.pressure = compPressure(adc_p) / 256.0f;         //Q24.8 format 
    sensorData.humidity = compHumidity(adc_h) / 1024.0f;         //Q22.10 format
    return rslt;
}

/// @brief Getter for sensor data (Temp, pressure and humidity)
/// @return const BME280_Data_t containing the sensor data
const BME68X_Data_t BME68X::getSensorData(){
    return sensorData;
}

/// @brief Sensor Initializer
/// @return Sensor Status
int8_t BME68X::init(){
#ifdef __DEBUG__
    Serial.printf("BME68X::init()\n");
#endif
    int8_t rslt = BME_STATUS_OK;
    softReset();
    //Get Chip ID
#ifdef __DEBUG__
    Serial.printf("Get ChipID\n");
#endif
    rslt = read(BME68X_REGISTER_CHIPID, &ChipID, 1, &interface);
    if(BME_STATUS_OK != rslt || BME68X_CHIP_ID != ChipID){
        return BME_E_DEV_NOT_FOUND;
    }
#ifdef __DEBUG__
    Serial.printf("BME68X::ChipID = %02X\n", ChipID);
#endif
    //Get Variant ID
#ifdef __DEBUG__
    Serial.printf("Get Variant ID\n");
#endif
    rslt = read(BME68X_REGISTER_VARIANTID, &VariantID, 1, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
#ifdef __DEBUG__
    Serial.printf("BME68X::VariantID = %02X\n", VariantID);
#endif
    //Get Calibration Data
    rslt = getCalibData(); 
    return rslt;
}

/// @brief Soft Resets the sensor
/// @return status of the chip 
int8_t BME68X::softReset(){
#ifdef __DEBUG__
    Serial.printf("BME68X::softReset()\n");
#endif
    uint8_t regAddr = BME68X_REGISTER_SOFTRESET;
    uint8_t regData = 0xB6; //Soft Reset Command
    //uint8_t regStatus = 0;
    //uint8_t tryRun = 5;
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

/// @brief Read the sensor calibration data
/// @return Sensor Status
int8_t BME68X::getCalibData(){
#ifdef __DEBUG__
    Serial.printf("BME68X::getCalibData()\n");
#endif
    int rslt;
    uint8_t regData[BME68X_LEN_COEFF_ALL];
    //Get our coeffs
    rslt = read(BME68X_REGISTER_COEFF_1, regData, BME68X_LEN_COEFF_1, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    rslt = read(BME68X_REGISTER_COEFF_2, &regData[BME68X_LEN_COEFF_1], BME68X_LEN_COEFF_2, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    rslt = read(BME68X_REGISTER_COEFF_3, &regData[BME68X_LEN_COEFF_1 + BME68X_LEN_COEFF_2], BME68X_LEN_COEFF_3, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    //Temp Coeffs
    sensorCalib.par_t1 = (uint16_t)BME68X_16BIT_CONCAT(regData[32], regData[31]);
    sensorCalib.par_t2 = (int16_t)BME68X_16BIT_CONCAT(regData[1], regData[0]);
    sensorCalib.par_t3 = (int8_t)regData[2];
    //Pressure Coeffs
    sensorCalib.par_p1 = (uint16_t)BME68X_16BIT_CONCAT(regData[5], regData[4]);
    sensorCalib.par_p2 = (int16_t)BME68X_16BIT_CONCAT(regData[7], regData[6]);
    sensorCalib.par_p3 = (int8_t)regData[8];
    sensorCalib.par_p4 = (int16_t)BME68X_16BIT_CONCAT(regData[11], regData[10]);
    sensorCalib.par_p5 = (int16_t)BME68X_16BIT_CONCAT(regData[13], regData[12]);
    sensorCalib.par_p6 = (int8_t)regData[15];
    sensorCalib.par_p7 = (int8_t)regData[14];
    sensorCalib.par_p8 = (int16_t)BME68X_16BIT_CONCAT(regData[19], regData[18]);
    sensorCalib.par_p9 = (int16_t)BME68X_16BIT_CONCAT(regData[21], regData[20]);
    sensorCalib.par_p10 = (uint8_t)regData[22];
    //Humidity Coeffs
    sensorCalib.par_h1 = (uint16_t)BME68X_12BIT_LOWER_CONCAT(regData[25], regData[24]);
    sensorCalib.par_h2 = (uint16_t)BME68X_12BIT_UPPER_CONCAT(regData[23], regData[24]);
    sensorCalib.par_h3 = (int8_t)regData[26];
    sensorCalib.par_h4 = (int8_t)regData[27];
    sensorCalib.par_h5 = (int8_t)regData[28];
    sensorCalib.par_h6 = (uint8_t)regData[29];
    sensorCalib.par_h7 = (int8_t)regData[30];
    //Gas Heater Coeffs
    sensorCalib.par_gh1 = (int8_t)regData[35];
    sensorCalib.par_gh2 = (int16_t)BME68X_16BIT_CONCAT(regData[34], regData[33]);
    sensorCalib.par_gh3 = (int8_t)regData[36];
    //MISC Coeffs
    sensorCalib.res_heat_range = (regData[39] & 0x30) / 16;
    sensorCalib.res_heat_val = (int8_t)regData[37];
    sensorCalib.range_sw_err = (int8_t)(regData[41] & 0xF0) / 16;
    return rslt;
}

/// @brief Gets the memory page from the Sensor for SPI Communication
/// @return Sensor Status
int8_t BME68X::getMemPage(){
#ifdef __DEBUG__
    Serial.printf("BME68X::getMemPage()\n");
#endif
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
#ifdef __DEBUG__
    Serial.printf("BME68X::setMemPage()\n");
#endif
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
#ifdef __DEBUG__
    Serial.printf("BME68X::getVariantId()\n");
#endif
    int8_t rslt = BME_STATUS_OK;
    rslt = read(BME68X_REGISTER_VARIANTID, &VariantID, 1, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    return rslt;
}

/// @brief Sets the Sensor mode to the desired operation mode
/// @param mode Mode to which the sensor will be set
/// @return Sensor Status
int8_t BME68X::setSensorMode(BME68X_Mode_e mode){
    int8_t rslt = BME_STATUS_OK;
    uint8_t regData;
    uint8_t powMode = 0;
    uint8_t regAddr = BME68X_REGISTER_CONTROL;
    do{
        rslt = read(regAddr, &regData, 1, &interface);
        if(BME_STATUS_OK != rslt){
            continue;
        }
        powMode = regData & 0x03;
        if(powMode != BME68X_POWERMODE_SLEEP){
            regData &= ~0x03;
            rslt = write(regAddr, &regData, 1, &interface);
            delay_us(10000, &interface);
        }
    }while((powMode != BME68X_POWERMODE_SLEEP) && (rslt == BME_STATUS_OK));
    regData = (regData & ~0x03) | (mode & 0x03);
    rslt = write(regAddr, &regData, 1, &interface);
    return rslt;
}

/// @brief Writes the sensor mode to SLEEP
/// @return Sensor Status
int8_t BME68X::putSensorToSleep(){
#ifdef __DEBUG__
    Serial.printf("BME68X::putSensorToSleep()\n");
#endif
    int8_t rslt;
    uint8_t curMode;
    rslt = read(BME68X_REGISTER_CONTROL , &curMode, 1, &interface);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    rslt = softReset();
    return rslt;
}


int16_t BME68X::compTemperature(int32_t adc_t){
    int32_t var1, var2, var3;
    var1 = ((int32_t)adc_t >> 3) - ((int32_t)sensorCalib.par_t1 << 1);
    var2 = (var1 * (int32_t)sensorCalib.par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = ((var3) * ((int32_t)sensorCalib.par_t3 << 4)) >> 14;
    t_fine = (int32_t)(var2 + var3);
    return (int16_t)(((t_fine * 5) + 128) >> 8);
}

uint32_t BME68X::compPressure(uint32_t adc_p){
    int32_t var1, var2, var3, tPress;
    var1 = (((int32_t)t_fine) >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)sensorCalib.par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)sensorCalib.par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)sensorCalib.par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)sensorCalib.par_p3 << 5)) >> 3) +
           (((int32_t)sensorCalib.par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)sensorCalib.par_p1) >> 15;
    tPress = 1048576 - adc_p;
    tPress = (int32_t)((tPress - (var2 >> 12)) * ((uint32_t)3125));
    /*TO FINISH Page 23 of BME688 Manual*/

}

uint32_t BME68X::compHumidity(){

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
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr);
#endif
            for (uint32_t i = 0; i < len; i ++){
                comm->sspi.m_spi->transfer(regData[i]);
#ifdef __DEBUG__
                Serial.printf("%02x(%03u),", regData[i], regData[i]);
#endif
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
                Serial.printf("%02x(%03u),", regData[i], regData[i]);
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
#ifdef __DEBUG__
                Serial.printf("%02x(%03u),", regData[i], regData[i]);
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
                Serial.printf("%02x(%u)", regData[i], regData[i]);
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
