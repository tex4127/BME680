#ifndef __BME680_HH__
#define __BME680_HH__

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "../SoftwareSPI/SoftwareSPI.h"

#include "bme68x/bme68x.h"

//#define BME68X_ERROR
//#define BME68X_WARNING

#define BME680_I2C_ADDRESS_DEFAULT      0x77
#define BME680_I2C_ADDRESS_ALTERNATE    0x00

#define BME68X_MAX_READ_LENGTH 51

//Delay/Idle function
void BME680_delayUs(uint32_t us, void* intf);
//Hardware SPI
int8_t BME680_SPIWrite(uint8_t regAddr, const uint8_t* buf, uint32_t len, void* intf);
int8_t BME680_SPIRead(uint8_t regAddr, uint8_t* buf, uint32_t len, void* intf);
//Software SPI
int8_t BME680_SSPIWrite(uint8_t regAddr, const uint8_t* buf, uint32_t len, void* intf);
int8_t BME680_SSPIRead(uint8_t regAddr, uint8_t* buf, uint32_t len, void* intf);
//Hardware I2C
int8_t BME680_I2CWrite(uint8_t regAddr, const uint8_t* buf, uint32_t len, void* intf);
int8_t BME680_I2CRead(uint8_t regAddr, uint8_t* buf, uint32_t len, void* intf);

/// @brief Enumerated values for Oversampling
typedef enum{
    BME680_OS_NONE = BME68X_OS_NONE,
    BME680_OS_1X = BME68X_OS_1X,
    BME680_OS_2X = BME68X_OS_2X,
    BME680_OS_4X = BME68X_OS_4X,
    BME680_OS_8X = BME68X_OS_8X,
    BME680_OS_16X = BME68X_OS_16X
} BME680_OS_e;

/// @brief Enumerated values for BME680 Modes
typedef enum{
    BME680_MODE_SLEEP = BME68X_SLEEP_MODE,
    BME680_MODE_FORCED = BME68X_FORCED_MODE,
    BME680_MODE_PARALLEL = BME68X_PARALLEL_MODE,
    BME680_MODE_SEQUENTIAL = BME68X_SEQUENTIAL_MODE
} BME680_Mode_e;

/// @brief Enumerated values for BME680 Filter Size
typedef enum{
    BME680_FILTER_OFF       = BME68X_FILTER_OFF,
    BME680_FILTER_SIZE_1    = BME68X_FILTER_SIZE_1,
    BME680_FILTER_SIZE_3    = BME68X_FILTER_SIZE_3,
    BME680_FILTER_SIZE_7    = BME68X_FILTER_SIZE_7,
    BME680_FILTER_SIZE_15   = BME68X_FILTER_SIZE_15,
    BME680_FILTER_SIZE_31   = BME68X_FILTER_SIZE_31,
    BME680_FILTER_SIZE_63   = BME68X_FILTER_SIZE_63,
    BME680_FILTER_SIEZ_127  = BME68X_FILTER_SIZE_127
} BME680_FilterSize_e;

/// @brief Enumerated values for BME680 ODR timing
typedef enum{
    BME680_ODR_NONE         = BME68X_ODR_NONE,
    BME680_ODR_590us        = BME68X_ODR_0_59_MS,
    BME680_ODR_10ms         = BME68X_ODR_10_MS,
    BME680_ODR_20ms         = BME68X_ODR_20_MS,
    BME680_ODR_63ms         = BME68X_ODR_62_5_MS,
    BME680_ODR_125ms        = BME68X_ODR_125_MS,
    BME680_ODR_250ms        = BME68X_ODR_250_MS,
    BME680_ODR_500ms        = BME68X_ODR_500_MS,
    BME680_ODR_1000ms       = BME68X_ODR_1000_MS 
} BME680_ODR_e;

/// @brief Interface union to for use with BME680 API
typedef union{
    struct{
        TwoWire *m_i2c;
        uint8_t i2cAddr;
    } i2c;
    struct{
        SPIClass *m_spi;
        uint8_t cs;
    } spi;
    struct{
        SSPIClass *m_spi;
        uint8_t cs;
    }sspi;
} BME_Interface_u;

/// @brief Main BME680 Class 
class BME680{
    public:
    BME680();                                                                   //Default Constructor
    BME680(uint8_t addr = BME680_I2C_ADDRESS_DEFAULT, TwoWire* wire = &Wire);   //I2C Interface
    BME680(uint8_t csPin, SPIClass* spi = &SPI);                                //SPI (Hardware) Interface
    //BME680(uint8_t csPin, uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin);     //SPI (Software) Interface
    BME680(uint8_t csPin, SSPIClass* sspi);
    void begin();
    bool begun();
    uint8_t readRegister(uint8_t regAddr);
    void readRegister(uint8_t regAddr, uint8_t* buf, uint32_t bufLen);
    void writeRegister(uint8_t* regAddr, uint8_t* buf, uint32_t bufLen);
    void softReset();
    //Getters and Setters
    BME680_Mode_e getOperMode();
    void setOperMode(BME680_Mode_e);

    void getSensor_OS();
    void setSensor_OS(BME680_OS_e osTemp, BME680_OS_e osPres, BME680_OS_e osHum);

    BME680_FilterSize_e getFilter();
    void setFilter(BME680_FilterSize_e);

    BME680_ODR_e getSequentialSleep();
    void setSequentialSleep(BME680_ODR_e);

    void setHeaterProfile(uint16_t temp, uint16_t dur);
    void setHeaterProfile(uint16_t temp, uint16_t dur, uint8_t len);
    void setHeaterProfile(uint16_t temp, uint16_t reps, uint16_t dur, uint8_t len);
    const bme68x_heatr_conf* getHeaterConfiguration();

    uint8_t fetchData();
    uint8_t getData(bme68x_data*);
    bme68x_data* getSensorData();

    uint32_t getUID();

    void getMeasurementTime(uint8_t _mode = BME68X_SLEEP_MODE);
    void setAmbientTemp(int8_t temp=25);
    int8_t getStatus();
    int8_t intfError();

    private:
    bool _begun = false;
    uint8_t intfType = 0; //For ease of use: 0x00 ==none, 0x01 == I2C, 0x02 == H_SPI, 0x04 == S_SPI
    BME_Interface_u interface;
    //bme68x lib defs
    bme68x_data b_data[3];
    bme68x_dev b_dev;
    bme68x_conf b_conf;
    bme68x_heatr_conf bh_conf;
    int8_t b_status;
    BME680_Mode_e b_mode;
    uint8_t nFields, iFields;
};


#endif