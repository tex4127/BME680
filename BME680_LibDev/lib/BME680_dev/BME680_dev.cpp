#include "BME680_dev.h"

/// @brief Default Constructor
BME680::BME680(){
    intfType = 0x00;
}

/// @brief I2C Constructor
/// @param addr I2C Address | default is 0x77
/// @param wire I2C Bus to use for communication
BME680::BME680(uint8_t addr, TwoWire* wire){
    interface.i2c.i2cAddr = addr;
    interface.i2c.m_i2c = wire;
    intfType = 0x01;
}

/// @brief Hardware SPI Constructor
/// @param csPin Chip Select Pin
/// @param spi Hardware SPI Class to use (i.e SPI, SPI1 or SPI2)
BME680::BME680(uint8_t csPin, SPIClass* spi){
    interface.spi.cs = csPin;
    interface.spi.m_spi = spi;
    intfType = 0x02;
}

/// @brief Software SPI Constructor
/// @param csPin Chip Select pin
/// @param mosiPin Master Out Slave In pin
/// @param misoPin Master In Slave Out pin
/// @param sckPin SPI Clock pin
//BME680::BME680(uint8_t csPin, uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin){
BME680::BME680(uint8_t csPin, SSPIClass* sspi){
    interface.sspi.cs = csPin;
    interface.sspi.m_spi = sspi; //testing out if a null ptr is giving the issue here...
    Serial.printf("Reference to SSPI %i\n", interface.sspi.m_spi);
    intfType = 0x04;
}

/// @brief Stack begin method for the BME680 Device
/// @return True if chip found and intiliezed, False if not
void BME680::begin(){
    b_status = BME68X_OK;
    memset(&b_dev, 0, sizeof(b_dev));
    memset(&b_conf, 0 ,sizeof(b_conf));
    memset(&bh_conf, 0, sizeof(bh_conf));
    memset(b_data, 0, sizeof(b_data));
    b_dev.amb_temp = 25;
    nFields = 0;
    iFields = 0;
    b_mode = BME680_Mode_e::BME680_MODE_SLEEP;

    switch (intfType){
        case 0x00:
        /*No Interface Defined*/
        break;
        case 0x01:
        /*Start I2C Device*/
        Serial.printf("Starting I2C Comms\n");
        interface.i2c.m_i2c->begin();
        b_dev.intf = BME68X_I2C_INTF;
        b_dev.read = BME680_I2CRead;
        b_dev.write = BME680_I2CWrite;
        b_dev.delay_us = BME680_delayUs;
        b_dev.intf_ptr = &interface;
        b_status = bme68x_init(&b_dev);
        _begun = (BME68X_OK == b_status) ? true : false; //Might need to move this to outside of the switch?
        break;
        case 0x02:
        /*Start H_SPI Device*/
        Serial.printf("Starting Hardware SPI Comms\n");
        interface.spi.m_spi->begin();
        pinMode(interface.spi.cs, OUTPUT);
        digitalWrite(interface.spi.cs, HIGH);
        delay(1);
        digitalWrite(interface.spi.cs, LOW);
        delay(1);
        digitalWrite(interface.spi.cs, HIGH);
        b_dev.intf = BME68X_SPI_INTF;
        b_dev.read = BME680_SPIRead;
        b_dev.write = BME680_SPIWrite;
        b_dev.delay_us = BME680_delayUs;
        b_dev.intf_ptr = &interface;
        b_status = bme68x_init(&b_dev);
        _begun = (BME68X_OK == b_status) ? true : false;
        break;
        case 0x04:
        /*Start S_SPI Device*/
        Serial.printf("Starting Software SPI Comms\n");
        interface.sspi.m_spi->begin();
        digitalWrite(interface.sspi.cs, HIGH);
        delay(1);
        digitalWrite(interface.sspi.cs, LOW);
        delay(1);
        digitalWrite(interface.sspi.cs, HIGH);
        b_dev.intf = BME68X_SPI_INTF;
        b_dev.read = BME680_SSPIRead;
        b_dev.write = BME680_SSPIWrite;
        b_dev.delay_us = BME680_delayUs;
        b_dev.intf_ptr = &interface;
        b_status = bme68x_init(&b_dev);
        _begun = (BME68X_OK == b_status) ? true : false;
        break;
        default:
        _begun = false;
        break;
    }
}

/// @brief Getter for begin status of the BME680 Communications
/// @return True if BME680 Connected, False if not
bool BME680::begun(){
    return _begun;
}

/// @brief Reads one register and retruns the value of that register
/// @param regAddr 
/// @return 
uint8_t BME680::readRegister(uint8_t regAddr){
    uint8_t regData;
    bme68x_get_regs(regAddr, &regData, 1, &b_dev);
    return regData;
}

/// @brief Reads register data from one or multiple registers
/// @param regAddr register address
/// @param buf data buffer for register data
/// @param bufLen data buffer length
void BME680::readRegister(uint8_t regAddr, uint8_t* buf, uint32_t bufLen){
    b_status = bme68x_get_regs(regAddr, buf, bufLen, &b_dev);
}

/// @brief Writes data to one or multiple registers
/// @param regAddr pointer to regsiter address (allows for looping)
/// @param buf data to be written to buffer(s)
/// @param bufLen length of 
void BME680::writeRegister(uint8_t* regAddr, uint8_t* buf, uint32_t bufLen){
    b_status = bme68x_set_regs(regAddr, buf, bufLen, &b_dev);
}

/// @brief Soft resets the BME680
void BME680::softReset(){
    b_status = bme68x_soft_reset(&b_dev);
}


/// @brief Gets the operation mode of the BME680
/// @return uint8_t representing the operation mode
BME680_Mode_e BME680::getOperMode(){
    uint8_t opMode;
    b_status = bme68x_get_op_mode(&opMode, &b_dev);
    return BME680_Mode_e(opMode);
}

/// @brief Sets the BME680 into a specified mode
/// @param _mode BME680_Mode_e into which the BME680 will be set
void BME680::setOperMode(BME680_Mode_e _mode){
    b_status = bme68x_set_op_mode((uint8_t)_mode, &b_dev);
    if((BME68X_OK == b_status) && (BME680_MODE_SLEEP != _mode)){
        b_mode = _mode;
    }
}

/// @brief Gets the current oversampling configuration for Temperature, Humidity, and Pressure
void BME680::getSensor_OS(){
    b_status = bme68x_get_conf(&b_conf, &b_dev);
}

/// @brief Sets the oversampiling configurtion for Temperature, Humidity, and Pressure
/// @param osTemp BME680_OS_e configuration for temperature
/// @param osPres BME680_OS_e configuration for Pressure
/// @param osHum BME680_OS_e configuration for Humidity
void BME680::setSensor_OS(BME680_OS_e osTemp, BME680_OS_e osPres, BME680_OS_e osHum){
    b_status = bme68x_get_conf(&b_conf, &b_dev);
    if(BME68X_OK == b_status){
        b_conf.os_temp = (uint8_t)osTemp;
        b_conf.os_pres = (uint8_t)osPres;
        b_conf.os_hum = (uint8_t)osHum;
        b_status = bme68x_set_conf(&b_conf, &b_dev);
    }
}

/// @brief Gets the current filter size of the BME680
/// @return BME680_FilterSize_e value that the BME680 is currently using
BME680_FilterSize_e BME680::getFilter(){
    b_status = bme68x_get_conf(&b_conf, &b_dev);
    return BME680_FilterSize_e(b_conf.filter);
}

/// @brief Sets the filter size based on enumerated values
/// @param filter BME680_FilterSize_e value corresponding to a valid Filter Size
void BME680::setFilter(BME680_FilterSize_e filter){
    b_status = bme68x_get_conf(&b_conf, &b_dev);
    if(BME68X_OK == b_status){
        b_conf.filter = (uint8_t)filter;
        bme68x_set_conf(&b_conf, &b_dev);
    }
}

/// @brief Gets the current ODR (time) for sequential reads
/// @return BME680_ODR_e value that the BME680 is currently using
BME680_ODR_e BME680::getSequentialSleep(){
    b_status = bme68x_get_conf(&b_conf, &b_dev);
    return BME680_ODR_e(b_conf.odr);
}

/// @brief Sets the Sequential Sleep time based on enumerated values
/// @param odr BME680_ODR_e value corresponding to a valie Sequential Sleep Time
void BME680::setSequentialSleep(BME680_ODR_e odr){
    b_status = bme68x_get_conf(&b_conf, &b_dev);
    if(BME68X_OK == b_status){
        b_conf.odr = (uint8_t)odr;
        b_status = bme68x_set_conf(&b_conf, &b_dev);
    }
}

/// @brief Sets the Heater profile for Forced Mode
/// @param temp Temperature in degrees celcius
/// @param dur Heating duration in ms
void BME680::setHeaterProfile(uint16_t temp, uint16_t dur){
    bh_conf.enable = BME68X_ENABLE;
    bh_conf.heatr_temp = temp;
    bh_conf.heatr_dur = dur;
    b_status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bh_conf, &b_dev);
}

/// @brief Sets the Heater profile for Sequential Mode
/// @param temp Temperature in degress celcius
/// @param dur Heating duration in ms
/// @param len Length of the profile
void BME680::setHeaterProfile(uint16_t temp, uint16_t dur, uint8_t len){
    bh_conf.enable = BME68X_ENABLE;
    bh_conf.heatr_temp_prof = &temp;
    bh_conf.heatr_dur_prof = &dur;
    bh_conf.profile_len = len;
    b_status = bme68x_set_heatr_conf(BME68X_SEQUENTIAL_MODE, &bh_conf, &b_dev);
}

/// @brief Sets the Heater profile for Parallel Mode
/// @param temp Temperature in degrees celcius
/// @param mul Number or repititions
/// @param dur Shared heating duration in ms
/// @param len Length of the profile
void BME680::setHeaterProfile(uint16_t temp, uint16_t mul, uint16_t dur, uint8_t len){
    bh_conf.enable = BME68X_ENABLE;
    bh_conf.heatr_temp_prof = &temp;
    bh_conf.heatr_dur_prof = &mul;
    bh_conf.shared_heatr_dur = dur;
    bh_conf.profile_len = len;
    b_status = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &bh_conf, &b_dev);
}

/// @brief Getter for the heater configuration 
/// @return const copy of the heater configuration
const bme68x_heatr_conf* BME680::getHeaterConfiguration(){
    return &bh_conf;
}

/// @brief Gets data from the sensor into local buffer
/// @return Number of data fileds stored in b_data buffer
uint8_t BME680::fetchData(){
    nFields = 0;
    b_status = bme68x_get_data(b_mode, b_data, &nFields, &b_dev);
    iFields = 0;
    return nFields;
}


uint8_t BME680::getData(bme68x_data *data){
    if(BME68X_FORCED_MODE == b_mode){
        *data = b_data[0];
        return 0;
    }
    if(nFields){
        *data = b_data[iFields++];
        return nFields - iFields;
    }
    return 0;
}

/// @brief Retrieves entire Snsor Data Structure
/// @return pointer to bme68x_data structure containing sensor data
bme68x_data* BME680::getSensorData(){
    return b_data;
}

/// @brief Retrieves the sensors Unique ID
/// @return uint32_t containing the Unique ID 
uint32_t BME680::getUID(){
    uint8_t idRegs[4];
    readRegister(BME68X_REG_UNIQUE_ID, idRegs, 4);
    uint32_t uid = ((idRegs[3] << 24) & 0xF000) | ((idRegs[2] << 16) & 0x0F00) | ((idRegs[1] << 8) & 0x00F0) | (idRegs[0] &0x000F);
    return uid;
}

/// @brief Sets the ambient temp poriton of bme68x_dev
/// @param temp ambient temperature to apply to b_dev
void BME680::setAmbientTemp(int8_t temp){
    b_dev.amb_temp = temp;
}

/// @brief Getter for the BME680 Status
/// @return -1 == error, 1 == warning, 0 == nothing
int8_t BME680::getStatus(){
    return b_status;
}

/// @brief Getter for the error code of interface functions
/// @return Interface return code
int8_t BME680::intfError(){
    return b_dev.intf_rslt;
}

//***********************************************************//
//
//  Interface Methods for API on BME
//
//***********************************************************//

/// @brief Function to pass
/// @param us 
/// @param intf 
void BME680_delayUs(uint32_t us, void* intf){
    delayMicroseconds(us);
}

/// @brief Hardware SPI Write Method
/// @param regAddr Address to which data will be written
/// @param buf Buffer to be written to the BME680
/// @param len Length of data buffer to be written
/// @param intf Pointer to digital communication object
/// @return Error code (int8_t) bassed on communication state. 
int8_t BME680_SPIWrite(uint8_t regAddr, const uint8_t* buf, uint32_t len, void* intf){
    Serial.printf("Writting SPI -> reg Addr %u | len %u |\n", regAddr, len);
    BME_Interface_u *comm = NULL;
    int8_t res = BME68X_OK;
    if(intf){
        comm = (BME_Interface_u *)intf;
        if(comm->spi.m_spi){
            digitalWrite(comm->spi.cs, LOW);
            comm->spi.m_spi->transfer(regAddr);
            for (uint32_t i = 0; i < len; i++){
                comm->spi.m_spi->transfer(buf[i]);
                Serial.printf("buf[%u] -> %u\n", i, buf[i]);
            }
            digitalWrite(comm->spi.cs, HIGH);
        }else{
            res = BME68X_E_COM_FAIL;
        }
    }else{
        res = BME68X_E_COM_FAIL;
    }
    return res;
}

/// @brief Hardware SPI Read Method
/// @param regAddr Address to which data will be read
/// @param buf Buffer into which received data will be stored
/// @param len Length of data Buffer
/// @param intf Pointer to digital communication
/// @return Error code (int8_t) bassed on communication state
int8_t BME680_SPIRead(uint8_t regAddr, uint8_t* buf, uint32_t len, void* intf){
    Serial.printf("Reading SPI -> reg Addr %u | len %u |\n", regAddr, len);
    BME_Interface_u *comm= NULL;
    int8_t res = BME68X_OK;
    if(intf){
        comm = (BME_Interface_u *)intf;
        if(comm->spi.m_spi){
            digitalWrite(comm->spi.cs, LOW);
            comm->spi.m_spi->transfer(regAddr);
            memset(buf, 0xFF, len);
            for(uint32_t i = 0; i < len; i++){
                buf[i] = comm->spi.m_spi->transfer(0xFF);
                Serial.printf("buf[%u] -> %u\n", i, buf[i]);
            }
            digitalWrite(comm->spi.cs, HIGH);
        }else{
            res = BME68X_E_COM_FAIL;
        }
    }else{
        res = BME68X_E_COM_FAIL;
    }
    return res;
}

/// @brief Software Spi (Bit Banged) Write Method
/// @param regAddr Address to which data will be written
/// @param buf Buffer to be written to the BME680
/// @param len Length of data buffer to be written
/// @param intf Pointer to digital communication object
/// @return Error code (int8_t) bassed on communication state. 
int8_t BME680_SSPIWrite(uint8_t regAddr, const uint8_t* buf, uint32_t len, void* intf){
    Serial.printf("Writting SSPI -> reg Addr %u | len %u |\n", regAddr, len);
    BME_Interface_u *comm = (BME_Interface_u *)intf;
    int8_t res = BME68X_OK;
    if(intf){
        comm = (BME_Interface_u *)intf;
        if(comm->sspi.m_spi){
            digitalWrite(comm->sspi.cs, LOW);
            comm->sspi.m_spi->transfer(regAddr);
            for (uint32_t i = 0; i < len; i++){
                comm->sspi.m_spi->transfer(buf[i]);
                Serial.printf("buf[%u] -> %u\n", i, buf[i]);
            }
            digitalWrite(comm->sspi.cs, HIGH);
        }else{
            res = BME68X_E_COM_FAIL;
        }
    }else{
        res = BME68X_E_COM_FAIL;
    }
    Serial.printf("SSPIWrite res -> %i\n", res);
    return res;
}

/// @brief Software Spi (Bit Banged) Write Method
/// @param regAddr Address to which data will be read
/// @param buf Buffer into which received data will be stored
/// @param len Length of data Buffer
/// @param intf Pointer to digital communication
/// @return Error code (int8_t) bassed on communication state
int8_t BME680_SSPIRead(uint8_t regAddr, uint8_t* buf, uint32_t len, void* intf){
    Serial.printf("Reading SSPI -> reg Addr %u | len %u |\n", regAddr, len);
    BME_Interface_u *comm = (BME_Interface_u *)intf;
    int8_t res = BME68X_OK;
    if(intf){
        comm = (BME_Interface_u *)intf;
        if(comm->sspi.m_spi){
            digitalWrite(comm->sspi.cs, LOW);
            comm->sspi.m_spi->transfer(regAddr);
            memset(buf, 0xFF, len);
            for(uint32_t i = 0; i < len; i++){
                buf[i] = comm->sspi.m_spi->transfer(0xFF);
                Serial.printf("buf[%u] -> %u\n", i, buf[i]);
            }
            digitalWrite(comm->sspi.cs, HIGH);
        }else{
            res = BME68X_E_COM_FAIL;
        }
    }else{
        res = BME68X_E_COM_FAIL;
    }
    Serial.printf("SSPIRead res -> %i\n", res);
    return res;
}

/// @brief I2C Device read method to be passed to BME680 API
/// @param regAddr Address to which data will be written
/// @param buf Buffer to be written to the BME680
/// @param len Length of data buffer to be written
/// @param intf Pointer to digital communication object
/// @return Error code (int8_t) bassed on communication state. 
int8_t BME680_I2CWrite(uint8_t regAddr, const uint8_t* buf, uint32_t len, void* intf){
    int8_t res = BME68X_OK;
    BME_Interface_u* comm = NULL;
    if(intf){
        comm = (BME_Interface_u *)intf;
        if(comm->i2c.m_i2c){
            comm->i2c.m_i2c->beginTransmission(comm->i2c.i2cAddr);
            comm->i2c.m_i2c->write(regAddr);
            for(uint32_t i = 0; i < len; i++){
                comm->i2c.m_i2c->write(buf[i]);
            }
            if(comm->i2c.m_i2c->endTransmission()){
                res = BME68X_E_COM_FAIL;
            }
        }else{
            res = BME68X_E_NULL_PTR;
        }
    }else{
        res = BME68X_E_NULL_PTR;
    }
    return res;
}

/// @brief I2C Device read method to be passed to BME680 API
/// @param regAddr Address to which data will be read
/// @param buf Buffer into which received data will be stored
/// @param len Length of data Buffer
/// @param intf Pointer to digital communication
/// @return Error code (int8_t) bassed on communication state
int8_t BME680_I2CRead(uint8_t regAddr, uint8_t* buf, uint32_t len, void* intf){
    int8_t res = BME68X_OK;
    BME_Interface_u *comm = NULL;
    if(intf){
        comm = (BME_Interface_u *)intf;
        if(comm->i2c.m_i2c){
            comm->i2c.m_i2c->beginTransmission(comm->i2c.i2cAddr);
            comm->i2c.m_i2c->write(regAddr);
            if(comm->i2c.m_i2c->endTransmission()){
                return BME68X_E_COM_FAIL;
            }
            comm->i2c.m_i2c->requestFrom(comm->i2c.i2cAddr, len);
            for(uint32_t i = 0; (i < len) && comm->i2c.m_i2c->available(); i++){
                buf[i] = comm->i2c.m_i2c->read();
            }
        }else{
            res = BME68X_E_NULL_PTR;
        }
    }else{
        res = BME68X_E_NULL_PTR;
    }
    return res;
}