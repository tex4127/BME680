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
    sensorCalib = {0};
    sensorConfig = {0};
    heaterConfig = {0};
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
/// @return const copy of the config structure
const BME68X_Config_t BME68X::getConfig(){
    uint8_t regData[4] = {0};
    if(BME_STATUS_OK != readRegisters(BME68X_REGISTER_CONTROLHUMID, regData, 4)){
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

int8_t BME68X::setOperationMode(BME68X_Mode_e mode){
    int rslt = BME_STATUS_OK;
    uint8_t regAddr = BME68X_REGISTER_CONTROL;
    uint8_t regData = 0;
    uint8_t powMode = 0;
    rslt = readRegisters(regAddr, &regData, 1);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    do{
        powMode = regData & 0x03;
        if(powMode != BME68X_POWERMODE_SLEEP){
            regData &= ~0x03;
            rslt = writeRegisters(regAddr, &regData, 1);
            delay_us(10000, &interface);
        }
    }while((powMode != BME68X_POWERMODE_SLEEP) && (rslt == BME_STATUS_OK));

    regData = (regData & ~0x03) | (mode & 0x03);
    rslt = writeRegisters(regAddr, &regData, 1);
    return rslt;
}

/// @brief Sets Heater Configuration for FORCED Mode
/// @param temp 
/// @param dur 
void BME68X::setHeaterConfig(uint16_t temp, uint16_t dur){
    heaterConfig.enable = BME68X_HEATER_ENABLE;
    heaterConfig.heatr_temp = temp;
    heaterConfig.heatr_dur = dur;
    configureHeater(sensorConfig.mode, &heaterConfig);
}

void BME68X::setHeaterConfig(uint16_t temp, uint16_t dur, uint8_t len){
    heaterConfig.enable = BME68X_HEATER_ENABLE;
    heaterConfig.heatr_temp_prof = &temp;
    heaterConfig.heatr_dur_prof = &dur;
    heaterConfig.profile_len = len;
    configureHeater(sensorConfig.mode, &heaterConfig);
}

void BME68X::setHeaterConfig(uint16_t temp, uint16_t reps, uint16_t dur, uint8_t len){
    heaterConfig.enable = BME68X_HEATER_ENABLE;
    heaterConfig.heatr_temp_prof = &temp;
    heaterConfig.heatr_dur_prof = &reps;
    heaterConfig.shared_heatr_dur = dur;
    heaterConfig.profile_len = len;
    configureHeater(sensorConfig.mode, &heaterConfig);
}

/// @brief Reads the pertinent heater config register(s) and updates the heater config strucutre
/// @return const copy of the heater config structure
const BME68X_HeaterConfig_t BME68X::getHeaterConfig(){
    return heaterConfig;
}

int8_t BME68X::readSensor(){
    uint8_t regAddr = 0x1D;
    uint8_t regData[17] = {0}; //this gets ALL Data
    uint8_t rangeGasL, rangeGasH;
    uint16_t adc_gh, adc_gl;
    int8_t rslt = BME_STATUS_OK;
    rslt = readRegisters(regAddr, regData, 17);
    if(BME_STATUS_OK != rslt){
#ifdef __DEBUG__
        Serial.printf("\nError BME68X::readSensor()\n>>regAddr %02x\n", regAddr);
        Serial.printf("\nNew Data Status -> %02x\n", regData[0] & 0x80); //gas measuring and measureing in this bit as well
#endif
    }
    //Now parse our data
    sensorData.status = regData[0] & 0x80;
    sensorData.gasIndex = regData[0] & 0x0F;
    sensorData.measIndex = regData[1];
    uint32_t adc_p = ((regData[2] * 4096) | (regData[3] * 16) | (regData[4] / 16));
    uint32_t adc_t = ((regData[5] * 4096) | (regData[6] * 16) | (regData[7] / 16));
    uint16_t adc_h = (regData[8] * 256) | regData[9];
    adc_gl = (uint16_t)((uint32_t)regData[13] * 4 | (((uint32_t)regData[14]) / 64));
    adc_gh = (uint16_t)((uint32_t)regData[15] * 4 | (((uint32_t)regData[16]) / 64));
    rangeGasL = regData[14] & 0x0F;
    rangeGasH = regData[16] & 0x0F;
    if(BME68X_VARIANTID_GAS_HIGH == VariantID){
        sensorData.status |= regData[16] & BME68X_GASM_VALID_MASK;
        sensorData.status |= regData[16] & BME68X_HEAT_STAB_MASK;
    }else{
        sensorData.status |= regData[14] & BME68X_GASM_VALID_MASK;
        sensorData.status |= regData[14] & BME68X_HEAT_STAB_MASK;
    }
    rslt = readRegisters(BME68X_REGISTER_RES_HEAT_0 + sensorData.gasIndex, &sensorData.resHeat, 1);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    rslt = readRegisters(BME68X_REG_IDAC_HEAT_0 + sensorData.gasIndex, &sensorData.idac, 1);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    rslt = readRegisters(BME68X_REGISTER_GAS_WAIT_0 + sensorData.gasIndex, &sensorData.gasWait, 1);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }

    //Now compensate them, reference page 25 and 26 of the datasheet
    sensorData.temperature = compTemperature(adc_t) / 100.0f;   //return is 1u = 0.01 degC
    sensorData.pressure = compPressure(adc_p) / 1.0f;         //Q24.8 format 
    sensorData.humidity = compHumidity(adc_h) / 1024.0f;        //Q22.10 format
    sensorData.resistance = (BME68X_VARIANTID_GAS_HIGH == VariantID) ? calcGasResistanceHigh(adc_gh, rangeGasH) \
                            : calcGasResistanceLow(adc_gl, rangeGasL);
    if(BME_STATUS_OK == rslt){
        delay_us(10000, &interface);
    }
    return rslt;
}

/// @brief Getter for sensor data (Temp, pressure and humidity)
/// @return const BME280_Data_t containing the sensor data
const BME68X_Data_t BME68X::getSensorData(){
    return sensorData;
}

/// @brief Gets the measurement time based on the mode and config
/// @return time in microseconds
uint32_t BME68X::getMeasurementTime(){
    uint8_t osrToCycles[6] = {0,1,2,4,8,16};
    uint32_t measCycles;
    uint32_t measDur = 0;
    measCycles = osrToCycles[sensorConfig.os_temp];
    measCycles += osrToCycles[sensorConfig.os_press];
    measCycles += osrToCycles[sensorConfig.os_hum];
    measDur = measCycles * 1963;
    measDur += 477 * 4;     //TPH Switching Duration
    measDur += 477 * 5;     //Gas Measurement Duration
    if(sensorConfig.mode != BME68X_POWERMODE_PARALLEL){
        measDur += 1000;
    }
    return measDur;
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
    rslt = readRegisters(BME68X_REGISTER_CHIPID, &ChipID, 1);
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
    rslt = readRegisters(BME68X_REGISTER_VARIANTID, &VariantID, 1);
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
    uint8_t regAddr = 0xE0; //Was 0xe0 per docs, but 0x60 works everytime
    uint8_t regData = 0xB6; //Soft Reset Command
    //uint8_t regStatus = 0;
    //uint8_t tryRun = 5;
    int8_t rslt = writeRegisters(regAddr, &regData, 1);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    /* Might want to add a while loop to ensure reset has occured */
    delay_us(10000, &interface);
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
    //Serial.printf("READ COEFF 1\n");
    rslt = readRegisters(BME68X_REGISTER_COEFF_1, regData, BME68X_LEN_COEFF_1);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    //Serial.printf("READ COEFF 2\n");
    rslt = readRegisters(BME68X_REGISTER_COEFF_2, &regData[BME68X_LEN_COEFF_1], BME68X_LEN_COEFF_2);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    //Serial.printf("READ COEFF 3\n");
    rslt = readRegisters(BME68X_REGISTER_COEFF_3, &regData[BME68X_LEN_COEFF_1 + BME68X_LEN_COEFF_2], BME68X_LEN_COEFF_3);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    //Temp Coeffs
    sensorCalib.par_t1 = (uint16_t)BME68X_16BIT_CONCAT(regData[32], regData[31]);
    sensorCalib.par_t2 = (int16_t)BME68X_16BIT_CONCAT(regData[1], regData[0]);
    sensorCalib.par_t3 = (int8_t)regData[2];
    Serial.printf("t1 -> %u\nt2 -> %d\nt3 -> %d\n", sensorCalib.par_t1,
                                                    sensorCalib.par_t2,
                                                    sensorCalib.par_t3);
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
    rslt = readRegisters(BME68X_REGISTER_MEM_PAGE, &regData, 1);
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
    rslt = readRegisters(BME68X_REGISTER_VARIANTID, &VariantID, 1);
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
        rslt = readRegisters(regAddr, &regData, 1);
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
    rslt = readRegisters(BME68X_REGISTER_CONTROL , &curMode, 1);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    rslt = softReset();
    return rslt;
}

/// @brief Calculates the Temperature from the raw measurement
/// @param adc_t Raw Temp Measurement
/// @return value is 1u = 0.01 degC
int16_t BME68X::compTemperature(uint32_t adc_t){
    int64_t var1, var2, var3;
    int16_t calc_temp;
    var1 = ((int32_t)adc_t >> 3) - ((int32_t)sensorCalib.par_t1 << 1);
    var2 = (var1 * (int32_t)sensorCalib.par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = ((var3) * ((int32_t)sensorCalib.par_t3 << 4)) >> 14;
    t_fine = (int32_t)(var2 + var3);
    calc_temp = (int16_t)(((t_fine * 5) + 128) >> 8);
    return calc_temp;
}

/// @brief Calculates the Pressure from the raw measurement
/// @param adc_p Raw Pressure Measurement
/// @return Q24.8 format 
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
    tPress = (tPress < 0x40000000) ? (tPress << 1) / var1 : (tPress / var1) << 1;
    var1 = ((int32_t)sensorCalib.par_p9 * (int32_t)(((tPress >> 3) * (tPress >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(tPress >> 2) * (int32_t)sensorCalib.par_p8) >> 13;
    var3 =((int32_t)(tPress >> 8) * (int32_t)(tPress >> 8) * (int32_t)(tPress >> 8) * (int32_t)sensorCalib.par_p10) >> 17;
    tPress = (int32_t)(tPress) + ((var1 + var2 + var3 + ((int32_t)sensorCalib.par_p7 << 7)) >> 4);
    return (uint32_t)tPress;
}

/// @brief Calculates the Humidity from the raw measurement
/// @param adc_h Raw Pressure Measurement
/// @return Q22.10 format
uint32_t BME68X::compHumidity(uint32_t adc_h){
    int32_t var1, var2, var3, var4, var5, var6, tScaled, cHum;
    tScaled = (((int32_t)t_fine * 5) + 128) >> 8;
    var1 = (int32_t)(adc_h - ((int32_t)((int32_t)sensorCalib.par_h1 * 16))) -
           (((tScaled * (int32_t)sensorCalib.par_h3) / ((int32_t)100)) >> 1);
    var2 =
        ((int32_t)sensorCalib.par_h2 *
         (((tScaled * (int32_t)sensorCalib.par_h4) / ((int32_t)100)) +
          (((tScaled * ((tScaled * (int32_t)sensorCalib.par_h5) / ((int32_t)100))) >> 6) / ((int32_t)100)) +
          (int32_t)(1 << 14))) >> 10;
    var3 = var1 * var2;
    var4 = (int32_t)sensorCalib.par_h6 << 7;
    var4 = ((var4) + ((tScaled * (int32_t)sensorCalib.par_h7) / ((int32_t)100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;
    cHum = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;
    /* Cap at 100%rH */
    if (cHum > 100000){
        cHum = 100000;
    }else if (cHum < 0){
        cHum = 0;
    }
    return (uint32_t)cHum;
}

uint8_t BME68X::calcHeaterDurationShared(uint16_t dur){
    uint8_t factor = 0;
    uint8_t heatdurval;

    if (dur >= 0x783){
        heatdurval = 0xff; /* Max duration */
    }
    else{
        /* Step size of 0.477ms */
        dur = (uint16_t)(((uint32_t)dur * 1000) / 477);
        while (dur > 0x3F){
            dur = dur >> 2;
            factor += 1;
        }
        heatdurval = (uint8_t)(dur + (factor * 64));
    }
    return heatdurval;
}

int8_t BME68X::configureHeater(uint8_t opMode, const BME68X_HeaterConfig_t* conf){
    int8_t rslt = BME_STATUS_OK;
    //uint8_t ctrlGasData[2];
    //uint8_t ctrlGasAddr[2] = {BME68X_REGISTER_CONTROL_GAS_0, BME68X_REGISTER_CONTROL_GAS_1};
    uint8_t writeLen = 0;
    uint8_t nbConv;
    uint8_t heaterDurSharedAddr = BME68X_REGISTER_SHD_HEATR_DUR;
    uint8_t sharedDur = 0;
    uint8_t rhRegAddr[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t rhRegData[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t gwRegAddr[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t gwRegData[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    rslt = setOperationMode(BME68X_Mode_e::BME68X_MODE_SLEEP);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    switch (opMode){
        case BME68X_Mode_e::BME68X_MODE_FORCED:
            rhRegAddr[0] = BME68X_REGISTER_HEAT_RANGE;
            rhRegData[0] = calcResistorHeat(conf->heatr_temp);
            gwRegAddr[0] = BME68X_REGISTER_GAS_WAIT_0;
            gwRegData[0] = calcGasWait(conf->heatr_temp);
            nbConv = 0;
            writeLen = 1;
        break;
        case BME68X_Mode_e::BME68X_MODE_SEQUENTIAL:
            if((!conf->heatr_temp_prof) || (!conf->heatr_dur_prof)){
                rslt = BME_E_NULL_PTR;
                break;
            }
            for(uint8_t i = 0; i < conf->profile_len; i++){
                rhRegAddr[i] = BME68X_REGISTER_HEAT_RANGE + i;
                rhRegData[i] = calcResistorHeat(conf->heatr_temp_prof[i]);
                gwRegAddr[i] = BME68X_REGISTER_GAS_WAIT_0 + i;
                gwRegData[i] = calcGasWait(conf->heatr_dur_prof[i]);
            }
            nbConv = conf->profile_len;
            writeLen = conf->profile_len;
        break;
        case BME68X_Mode_e::BME68X_MODE_PARLLEL:
            if ((!conf->heatr_dur_prof) || (!conf->heatr_temp_prof))
            {
                rslt = BME_E_NULL_PTR;
                break;
            }

            for (uint8_t i = 0; i < conf->profile_len; i++)
            {
                rhRegAddr[i] = BME68X_REGISTER_HEAT_RANGE + i;
                rhRegData[i] = calcResistorHeat(conf->heatr_temp_prof[i]);
                gwRegAddr[i] = BME68X_REGISTER_GAS_WAIT_0 + i;
                gwRegData[i] = (uint8_t) conf->heatr_dur_prof[i];
            }

            nbConv = conf->profile_len;
            writeLen = conf->profile_len;
            sharedDur = calcHeaterDurationShared(conf->shared_heatr_dur);
            if(BME_STATUS_OK != rslt){
                break;
            }
            rslt = writeRegisters(heaterDurSharedAddr, &sharedDur, 1);
        break;
        default:
            rslt = BME_W_DEFINE_OP_MODE;
        break;
    }
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    rslt = writeRegisters(rhRegAddr[0], rhRegData, writeLen);
    if(BME_STATUS_OK != rslt){
        return rslt;
    }
    rslt = writeRegisters(gwRegAddr[0], gwRegData, writeLen);
    return rslt;
}

uint8_t BME68X::calcGasWait(uint16_t dur){
    uint8_t factor = 0;
    uint8_t durVal;
    if(dur >= 0xFC0){
        durVal = 0xFF;
    }else{
        while(dur > 0x3F){
            dur = dur/4;
            factor += 1;
        }
        durVal = (uint8_t)(dur + (factor * 64));
    }
    return durVal;
}

uint32_t BME68X::calcGasResistanceLow(uint16_t adc_gr, uint8_t rangeGas){
    int64_t var1;
    uint64_t var2;
    int64_t var3;
    uint32_t calc_gas_res;
    uint32_t lookup_table1[16] = {
        UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
        UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777), UINT32_C(2147483647), UINT32_C(2147483647),
        UINT32_C(2143188679), UINT32_C(2136746228), UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647),
        UINT32_C(2147483647)
    };
    uint32_t lookup_table2[16] = {
        UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000), UINT32_C(255744255),
        UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016), UINT32_C(8000000), UINT32_C(
            4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000), UINT32_C(250000), UINT32_C(125000)
    };

    /*lint -save -e704 */
    var1 = (int64_t)((1340 + (5 * (int64_t)sensorCalib.range_sw_err)) * ((int64_t)lookup_table1[rangeGas])) >> 16;
    var2 = (((int64_t)((int64_t)adc_gr << 15) - (int64_t)(16777216)) + var1);
    var3 = (((int64_t)lookup_table2[rangeGas] * (int64_t)var1) >> 9);
    calc_gas_res = (uint32_t)((var3 + ((int64_t)var2 >> 1)) / (int64_t)var2);

    /*lint -restore */
    return calc_gas_res;
}

uint32_t BME68X::calcGasResistanceHigh(uint16_t adc_gr, uint8_t rangeGas){
    uint32_t calc_gas_res;
    uint32_t var1 = UINT32_C(262144) >> rangeGas;
    int32_t var2 = (int32_t)adc_gr - INT32_C(512);

    var2 *= INT32_C(3);
    var2 = INT32_C(4096) + var2;

    /* multiplying 10000 then dividing then multiplying by 100 instead of multiplying by 1000000 to prevent overflow */
    calc_gas_res = (UINT32_C(10000) * var1) / (uint32_t)var2;
    calc_gas_res = calc_gas_res * 100;

    return calc_gas_res;
}

uint8_t BME68X::calcResistorHeat(uint16_t temp){
    uint8_t heaterRes;
    int32_t var1, var2, var3, var4, var5; 
    int32_t heaterRes_X100;
    if(temp > 400){
        temp = 400;
    }
    var1 = (((int32_t)ambTemp * sensorCalib.par_gh3) / 1000) * 256;
    var2 = (sensorCalib.par_gh1 + 784) * (((((sensorCalib.par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (sensorCalib.res_heat_range + 4));
    var5 = (131 * sensorCalib.res_heat_val) + 65536;
    heaterRes_X100 = (int32_t)(((var4 / var5) - 250) * 34);
    heaterRes = (uint8_t)((heaterRes_X100 + 50) / 100);

    return heaterRes;
}




/// @brief Wrapper for read methods to check mempage
int8_t BME68X::readRegisters(uint8_t regAddr, uint8_t *regData, uint32_t len){
    if(intfType > 0x01){
        setMemPage(regAddr);
    }
    return read(regAddr, regData, len, &interface);
}

/// @brief Wrapper for write methods to check mempage 
int8_t BME68X::writeRegisters(uint8_t regAddr, const uint8_t *regData, uint32_t len){
    if(intfType > 0x01){
        regAddr &= ~0x80;
        setMemPage(regAddr);
        
    } 
    return write(regAddr, regData, len, &interface);
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
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr & ~0x80);
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
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr | 0x80);
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
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr & ~0x80);
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
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr | 0x80);
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
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr & ~0x80);
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
            Serial.printf("\n\tregAddr -> %02x Data -> {", regAddr | 0x80);
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
