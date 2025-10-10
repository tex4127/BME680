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
 * @file BME68X.h
 * @date 2025-01-01
 * @version v1.0.0
 */


#ifndef __BME68X_HH__
#define __BME68X_HH__

#define __DEBUG__

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "../SoftwareSPI/SoftwareSPI.h"
#include "../BitBangSPI/BitBangSPI.h"

/********************************************************/
/*! @name  BME68X Macros                                */
/********************************************************/

#define BME68X_16BIT_CONCAT(msb, lsb)               (((uint16_t)msb << 8)| \
                                                    (uint16_t)lsb)

#define BME68X_12BIT_LOWER_CONCAT(msb, lsb)         (uint16_t)(((uint16_t)msb << 4) | \
                                                    ((uint16_t)lsb & 0x0F))
#define BME68X_12BIT_UPPER_CONCAT(msb, lsb)         (uint16_t)(((uint16_t)msb << 4) | \
                                                    ((uint16_t)lsb >> 4))

/********************************************************/
/*! @name BME68X Addresses                              */
/********************************************************/

/********************************************************/
/*! @name BME68X Statuses                               */
/********************************************************/

#define BME68X_REGISTER_CHIPID      0xD0
#define BME68X_CHIP_ID              0x61
#define BME68X_REGISTER_STATUS      0x73
#define BME68X_REGISTER_VARIANTID   0xF0
#define BME68X_REGISTER_CONTROL     0x74
#define BME68X_REGISTER_SOFTRESET   0xE0
#define BME68X_REGISTER_MEM_PAGE    0xF3
#define BME68X_SPI_MEMPAGE_0        0x10
#define BME68X_SPI_MEMPAGE_1        0x00

#define BME68X_STATUS_IM_UPDATE     0x01

#define BME68X_POWERMODE_SLEEP      0x00
#define BME68X_POWERMODE_FORCED     0x01
#define BME68X_POWERMODE_PARALLEL   0x02
#define BME68X_POWERMODE_SEQUENTIAL 0x03

/********************************************************/
/*! @name  BME68X Data Registers                        */
/********************************************************/

/*!
 *                                      FULL REGISTER MAP
 *  Regsiter Name   |   I2C     |       SPI Addr    | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 | Reset State   | Notes
 *                  |   Addr    |   Addr    |   pg  |       |       |       |       |       |       |       |       |      
 *  Status          |   0x73    |   0x73    |   1   |   -   |   -   |   -   |mem pg |   -   |   -   |   -   |   -   |   0x01        |   SPI Memory Page (for burst read/writes)?  
 *  Variant_id      |   0xF0    |   0x70    |   0   |                   Variant ID  [7:0]                           |   0x01        |   
 *  Reset           |   0xE0    |   0x60    |   0   |                       Reset   [7:0]                           |   0x00        |
 *  Chip_id         |   0xD0    |   0x50    |   0   |                   Chip ID     [7:0]                           |   0x61        |
 *  Config          |   0x75    |   0x75    |   1   |   -   |   -   |   -   | Filter [2:0]          |   -   | spi3w |   0x00        |
 *  Ctrl_meas       |   0x74    |   0x74    |   1   |   osrs_t [2:0]        |   osrs_p [2:0]        |   mode [1:0]  |   0x00        |
 *  Ctrl_hum        |   0x72    |   0x72    |   1   |   -   |spi3wi |   -   |   -   |   -   |   osrs_h [2:0]        |   0x00        |
 *  Ctrl_gas_1      |   0x71    |   0x71    |   1   |   -   |   -   |run_gas|   -   |       nb_conv [3:0]           |   0x00        |
 *  Ctrl_gas_0      |   0x70    |   0x70    |   1   |   -   |   -   |   -   |heatoff|   -   |   -   |   -   |   -   |   0x00        |
 *  Gas_wait_shared |   0x6E    |   0x6E    |   1   |                   gas_wait_shared [7:0]                       |   0x00        |
 *  Gas_wait_x (9-0)|0x6D...0x64|0x6D...0x64|   1   |       Gas_wait_9 down to Gas_wait_0 [7:0]                     |   0x00        |
 *  Res_heat_x      |0x63...0x5A|0x63...0x5A|   1   |       res_heat_9 down to res_heat_0 [7:0]                     |   0x00        |
 *  Idac_heat_x     |0x59...0x50|0x59...0x50|   1   |       idac_heat_9 down to idac_heat_0 [7:0]                   |   0x00        |
 *  gas_r_lsb       |   0x4F    |   0x4F    |   1   |   gas_r [1:0] |gas_vld|heat_st|   gas_range_r [3:0]           |   0x00        |
 *  gas_r_msb       |   0x4E    |   0x4E    |   1   |                   gas_r [9:2]                                 |   0x00        |
 *  hum_lsb         |   0x48    |   0x48    |   1   |                   hum_lsb [7:0]                               |   0x00        |
 *  hum_msb         |   0x47    |   0x47    |   1   |                   hum_msb [7:0]                               |   0x80        |
 *  temp_xlsb       |   0x46    |   0x46    |   1   |   temp_xlsb [7:4]             |   -   |   -   |   -   |   -   |   0x00        |
 *  temp_lsb        |   0x45    |   0x45    |   1   |                   temp_lsb [7:0]                              |   0x00        |
 *  temp_msb        |   0x44    |   0x44    |   1   |                   temp_msb [7:0]                              |   0x80        |
 *  press_xlsb      |   0x43    |   0x43    |   1   |   press_xlsb [7:4]            |   -   |   -   |   -   |   -   |   0x00        |
 *  press_lsb       |   0x42    |   0x42    |   1   |                   press_lsb [7:0]                             |   0x00        |
 *  press_msb       |   0x41    |   0x41    |   1   |                   press_msb [7:0]                             |   0x80        |
 *  sub_meas_index  |   0x40    |   0x40    |   1   |                   sub_meas_index [7:0]                        |   0x00        |
 *  meas_status     |   0x3F    |   0x3F    |   1   |newdata|gasmeas|allmeas|   -   |   gas_meas_index [3:0]        |   0x00        |
 *                  |-----------------------    Duplicate Registers start at 0x3E   ------------------------------------------------|
 */ 

#define BME68X_LEN_COEFF_ALL                42
#define BME68X_LEN_COEFF_1                  23
#define BME68X_LEN_COEFF_2                  14
#define BME68X_LEN_COEFF_3                  5

#define BME68X_REGISTER_COEFF_1             0x8A
#define BME68X_REGISTER_COEFF_2             0xE1
#define BME68X_REGISTER_COEFF_3             0x00

//Temperature Calibration RegistersA
#define BME68X_REGISTER_CALIB_TEMP_START    0x8A    //Based on Bosch API, this is where temp coeffs start
#define BME68X_REGISTER_PAR_T1              0xE9    //uint16_t
#define BME68X_REGISTER_PAR_T1_LSB          0xE9    //[7:0]
#define BME68X_REGISTER_PAR_T1_MSB          0xEA    //[15:8]
#define BME68X_REGISTER_PAR_T2              0x8A    //uint16_t
#define BME68X_REGISTER_PAR_T2_LSB          0x8A    //[7:0]
#define BME68X_REGISTER_PAR_T2_MSB          0x8B    //[15:8]
#define BME68X_REGISTER_PAR_T3              0x8C    //uint8_t
//Temp ADC Registers
//Field 0
#define BME68X_REGISTER_TEMP_ADC_0          0x22    //20 bit value
#define BME68X_REGISTER_TEMP_ADC_0_MSB      0x22    //bits [19:12]
#define BME68X_REGISTER_TEMP_ADC_0_LSB      0x23    //bits [11:4]
#define BME68X_REGISTER_TEMP_ADC_0_XLSB     0x24    //bits [3:0]
//Field 1
#define BME68X_REGISTER_TEMP_ADC_1          0x33    //20 bit value
#define BME68X_REGISTER_TEMP_ADC_1_MSB      0x33    //bits [19:12]
#define BME68X_REGISTER_TEMP_ADC_1_LSB      0x34    //bits [11:4]
#define BME68X_REGISTER_TEMP_ADC_1_XLSB     0x35    //bits [3:0]
//Field 2
#define BME68X_REGISTER_TEMP_ADC_2          0x44    //20 bit value
#define BME68X_REGISTER_TEMP_ADC_2_MSB      0x44    //bits [19:12]
#define BME68X_REGISTER_TEMP_ADC_2_LSB      0x45    //bits [11:4]
#define BME68X_REGISTER_TEMP_ADC_2_XLSB     0x46    //bits [3:0]

//Pressure Calibration Registers
#define BME68X_REGISTER_CALIB_PRESS_START   0x8E
#define BME68X_REGISTER_PAR_P1              0x8E
#define BME68X_REGISTER_PAR_P1_LSB          0x8E
#define BME68X_REGISTER_PAR_P1_MSB          0x8F
#define BME68X_REGISTER_PAR_P2              0x90 //Had this as 0x92 as well?
#define BME68X_REGISTER_PAR_P2_LSB          0x90
#define BME68X_REGISTER_PAR_P2_MSB          0x91
#define BME68X_REGISTER_PAR_P3              0x94
#define BME68X_REGISTER_PAR_P3_LSB          0x94
#define BME68X_REGISTER_PAR_P3_MSB          0x95
#define BME68X_REGISTER_PAR_P4              0x96    
#define BME68X_REGISTER_PAR_P4_LSB          0x96
#define BME68X_REGISTER_PAR_P5_MSB          0x97
#define BME68X_REGISTER_PAR_P6              0x99
#define BME68X_REGISTER_PAR_P7              0x98
#define BME68X_REGISTER_PAR_P8              0x9C
#define BME68X_REGISTER_PAR_P8_LSB          0x9C
#define BME68X_REGISTER_PAR_P8_MSB          0x9D
#define BME68X_REGISTER_PAR_P9              0x9E
#define BME68X_REGISTER_PAR_P9_LSB          0x9E
#define BME68X_REGISTER_PAR_P9_MSB          0x9F
#define BME68X_REGISTER_PAR_P10             0x0A
//Pressure ADC Registers
//Field 0
#define BME68X_REGISTER_PRESS_ADC_0         0x1F    //20 bit value
#define BME68X_REGISTER_PRESS_ADC_0_MSB     0x1F    //bits [19:12]
#define BME68X_REGISTER_PRESS_ADC_0_LSB     0x20    //bits [11:4]
#define BME68X_REGISTER_PRESS_ADC_0_XLSB    0x21    //bits [3:0]
//Field 1
#define BME68X_REGISTER_PRESS_ADC_1         0x30    //20 bit value
#define BME68X_REGISTER_PRESS_ADC_1_MSB     0x30    //bits [19:12]
#define BME68X_REGISTER_PRESS_ADC_1_LSB     0x31    //bits [11:4]
#define BME68X_REGISTER_PRESS_ADC_1_XLSB    0x32    //bits [3:0]
//Field 2
#define BME68X_REGISTER_PRESS_ADC_2         0x41    //20 bit value
#define BME68X_REGISTER_PRESS_ADC_2_MSB     0x41    //bits [19:12]
#define BME68X_REGISTER_PRESS_ADC_2_LSB     0x42    //bits [11:4]
#define BME68X_REGISTER_PRESS_ADC_2_XLSB    0x43    //bits [3:0]

//Humidity Calibration Registers
#define BME68X_REGISTER_CALIB_HUM_START     0xE1
#define BME68X_REGISTER_PAR_H1              0xE2
#define BME68X_REGISTER_PAR_H1_LSB          0xE2    //[3:0]
#define BME68X_REGISTER_PAR_H1_MSB          0xE3    //[7:0]
#define BME68X_REGISTER_PAR_H2              0xE1
#define BME68X_REGISTER_PAR_H2_LSB          0xE2    //[7:4]
#define BME68X_REGISTER_PAR_H2_MSB          0xE1    //[7:0]
#define BME68X_REGISTER_PAR_H3              0xE4
#define BME68X_REGISTER_PAR_H4              0xE5
#define BME68X_REGISTER_PAR_H5              0xE6
#define BME68X_REGISTER_PAR_H6              0xE7
#define BME68X_REGISTER_PAR_H7              0xE8
//Humidity ADC Registers
//Field 0
#define BME68X_REGISTER_HUM_ADC_0           0x25
#define BME68X_REGISTER_HUM_ADC_0_MSB       0x25
#define BME68X_REGISTER_HUM_ADC_0_LSB       0x26
//Field 1
#define BME68X_REGISTER_HUM_ADC_1           0x36
#define BME68X_REGISTER_HUM_ADC_1_MSB       0x36
#define BME68X_REGISTER_HUM_ADC_1_LSB       0x37
//Field 2
#define BME68X_REGISTER_HUM_ADC_2           0x47
#define BME68X_REGISTER_HUM_ADC_2_MSB       0x47
#define BME68X_REGISTER_HUM_ADC_2_LSB       0x48

//Gas Resistance Calibration Registers
#define BME68X_REGISTER_CALIB_GAS_START     0xEB
#define BME68X_REGISTER_PAR_G1              0xED
#define BME68X_REGISTER_PAR_G2              0xEB
#define BME68X_REGISTER_PAR_G2_LSB          0xEB
#define BME68X_REGISTER_PAR_G2_MSB          0xEC
#define BME68X_REGISTER_PAR_G3              0xEE
//Gas Measurement Registers
#define BME68X_REGISTER_HEAT_RANGE          0x02    //[5:4]
#define BME68X_REGISTER_HEAT_VAL            0x00    

/********************************************************/
/*  BMEXXX Interface Union                              */
/********************************************************/

#ifndef __BME_INTERFACE_UNION__
#define __BME_INTERFACE_UNION__
typedef union{
    struct{
        TwoWire *m_i2c;
        uint8_t addr;
    }i2c;
    struct{
        SPIClass* m_spi;
        uint8_t cs;
    }spi;
    struct{
        SSPIClass *m_spi;
        uint8_t cs;
    }sspi;
    struct{
        BBSPIClass *m_spi;
        uint8_t cs;
    }bbspi;
}BME_Interface_u;

#endif

/********************************************************/
/*  BMEXXX Interface Methods                            */
/********************************************************/

#ifndef BME_RET_VALUE_TYPE
#define BME_RET_VALUE_TYPE               int8_t
#endif

#ifndef BME_RET_VALUES
#define BME_RET_VALUES
#define BME_STATUS_OK                   0
#define BME_E_NULL_PTR                  -1
#define BME_E_COMM_FAIL                 -2
#define BME_E_INVALID_LEN               -3
#define BME_E_DEV_NOT_FOUND             -4
#define BME_E_SLEEP_MODE_FAIL           -5
#define BME_E_NVM_COPY_FAIL             -6
#define BME_W_INVALID_ODR               1
#endif

typedef void (* BME68X_DELAY_FUNC)(uint32_t period, void *intrPtr);
typedef BME_RET_VALUE_TYPE (* BME68X_INTF_WRITE)(uint8_t, const uint8_t*, uint32_t, void*);
typedef BME_RET_VALUE_TYPE (* BME68X_INTF_READ)(uint8_t, uint8_t*, uint32_t, void*);


#ifndef BME68X_API_FUNTIONS
#define BME68X_API_FUNCTIONS
//Delay/Idle Function
void BME68X_delayUs(uint32_t period, void *intfPtr);
//Hardware SPI
int8_t BME68X_SPIWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr);
int8_t BME68X_SPIRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void* intfPtr);
//Software SPI
int8_t BME68X_SSPIWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr);
int8_t BME68X_SSPIRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr);
//BitBang SPI
int8_t BME68X_BBSPIWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr);
int8_t BME68X_BBSPIRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr);
//Hardware I2C
int8_t BME68X_I2CWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr);
int8_t BME68X_I2CRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr);
#endif

/********************************************************/
/*! @name  BME68X Data Structures                       */
/********************************************************/

/// @brief Structure to store all Calibration Coefficients for the BME68X chip set
typedef struct {
    /*! Calibration coefficient for the temperature sensor */
    uint16_t par_t1;
    /*! Calibration coefficient for the temperature sensor */
    int16_t par_t2;
    /*! Calibration coefficient for the temperature sensor */
    int8_t par_t3;
    /*! Calibration coefficient for the pressure sensor */
    uint16_t par_p1;
    /*! Calibration coefficient for the pressure sensor */
    int16_t par_p2;
    /*! Calibration coefficient for the pressure sensor */
    int8_t par_p3;
    /*! Calibration coefficient for the pressure sensor */
    int16_t par_p4;
    /*! Calibration coefficient for the pressure sensor */
    int16_t par_p5;
    /*! Calibration coefficient for the pressure sensor */
    int8_t par_p6;
    /*! Calibration coefficient for the pressure sensor */
    int8_t par_p7;
    /*! Calibration coefficient for the pressure sensor */
    int16_t par_p8;
    /*! Calibration coefficient for the pressure sensor */
    int16_t par_p9;
    /*! Calibration coefficient for the pressure sensor */
    uint8_t par_p10;
    /*! Calibration coefficient for the humidity sensor */
    uint16_t par_h1;
    /*! Calibration coefficient for the humidity sensor */
    uint16_t par_h2;
    /*! Calibration coefficient for the humidity sensor */
    int8_t par_h3;
    /*! Calibration coefficient for the humidity sensor */
    int8_t par_h4;
    /*! Calibration coefficient for the humidity sensor */
    int8_t par_h5;
    /*! Calibration coefficient for the humidity sensor */
    uint8_t par_h6;
    /*! Calibration coefficient for the humidity sensor */
    int8_t par_h7;
    /*! Calibration coefficient for the gas sensor */
    int8_t par_gh1;
    /*! Calibration coefficient for the gas sensor */
    int16_t par_gh2;
    /*! Calibration coefficient for the gas sensor */
    int8_t par_gh3;
    /*! Variable to store the intermediate temperature coefficient */
    float t_fine;
    /*! Heater resistance range coefficient */
    uint8_t res_heat_range;
    /*! Heater resistance value coefficient */
    int8_t res_heat_val;
    /*! Gas resistance range switching error coefficient */
    int8_t range_sw_err;
} BME68X_Calib_t;

/*!
 *  @brief BME28X Configuration Structure
 *  this structure stores the settings for the sensor
 *  Oversampling for each sensor measurement
 *  Standby time (odr) for idelt time between measurments in normal mode
 *  Mode of operation (sleep, forced, normal)
 */
typedef struct{
    /*! Humidity oversampling | Set to bits [2:0] of ctrl_hum(0xF2)
     * 000 -> Skipped, sets value to 100 on chip
     * 001 -> Oversampling x1
     * 010 -> Oversampling x2
     * 011 -> Oversampling x4
     * 100 -> Oversampling x8
     * 101 -> Oversampling x16
     */
    uint8_t os_hum;
    /*! Humidity oversampling | Set to bits [7:5] of ctrl_meas(0xF4)
     * 000 -> Skipped, sets value to 100 on chip
     * 001 -> Oversampling x1
     * 010 -> Oversampling x2
     * 011 -> Oversampling x4
     * 100 -> Oversampling x8
     * 101 -> Oversampling x16
     */
    uint8_t os_temp;
    /*! Humidity oversampling | Set to bits [4:2] of ctrl_meas(0xF4)
     * 000 -> Skipped, sets value to 100 on chip
     * 001 -> Oversampling x1
     * 010 -> Oversampling x2
     * 011 -> Oversampling x4
     * 100 -> Oversampling x8
     * 101 -> Oversampling x16
     */
    uint8_t os_press;
    /*! Filter coefficient | Set to bits [4:2] of config(0xF5)
     * 000 -> Filter Off
     * 001 -> Filter Coeff x2
     * 010 -> Filter Coeff x4
     * 011 -> Filter Coeff x8
     * 100 -> Filter Coeff x16
     */
    uint8_t filter;
    /*! Standby Time (t_sb) | Set to bits [7:5] of config(0xF5)
     * 000 -> 0.5ms
     * 001 -> 62.5ms
     * 010 -> 125ms
     * 011 -> 250ms
     * 100 -> 500ms
     * 101 -> 1000ms
     * 110 -> 10ms
     * 111 -> 20ms
     */
    uint8_t odr;
    /*! Sensor Mode | Set to bits [1:0] of ctrl_meas(0xF4)
     * 00 -> Sleep mode
     * 01 -> Forced Mode
     * 10 -> Forced Mode
     * 11 -> Normal Mode
     */
    uint8_t mode;
} BME68X_Config_t;

/// @brief BME68X Heater Config Structure
typedef struct{
    uint8_t enable;
    uint16_t heatr_temp;
    uint16_t heatr_dur;
    uint16_t *heatr_temp_prof;
    uint16_t *heatr_dur_prof;
    uint8_t profile_len;
    uint16_t shared_heatr_dur;
} BME68X_HeaterConfig_t;

typedef enum{
    BME68X_MODE_SLEEP       = BME68X_POWERMODE_SLEEP,
    BME68X_MODE_FORCED      = BME68X_POWERMODE_FORCED,
    BME68X_MODE_PARLLEL     = BME68X_POWERMODE_PARALLEL,
    BME68X_MODE_SEQUENTIAL  = BME68X_POWERMODE_SEQUENTIAL 
}BME68X_Mode_e;

/// @brief BME68X Sensor Data Structure
typedef struct{
    float temperature;
    float pressure;
    float humidity;
    float resistance;
} BME68X_Data_t;


class BME68X{
    public:
    void begin(void);
    bool begun(void);
    //void setConfig(void);
    //const BME68X_Config_t getConfig();

    //int8_t readSensor(void);
    //const BME68X_Data_t getSensorData(void);
    //uint32_t getMeasurementTime(void); 

    protected:
    virtual int8_t init();
    int8_t softReset(void);
    int8_t getCalibData(void);
    int8_t getMemPage(void);
    int8_t setMemPage(uint8_t regAddr);
    int8_t getVariantId(void);
    int8_t setSensorMode(BME68X_Mode_e mode);
    int8_t putSensorToSleep(void);
    //int32_t compTemperature(int32_t);
    //uint32_t compPressure(int32_t);
    //uint32_t compHumidity(int32_t);


    BME68X_INTF_WRITE write;
    BME68X_INTF_READ read;
    BME68X_DELAY_FUNC delay_us;
    bool _begun = false;
    BME_Interface_u interface;

    uint8_t intfType = 0x00;        //0x00 == NONE | 0x01 == I2C | 0x02 == HSPI | 0x04 == SSPI | 0x08 == BBSPI
    uint8_t ChipID;
    uint8_t VariantID;
    BME68X_Calib_t sensorCalib;
    BME68X_Config_t sensorConfig;
    uint8_t memPage = 0;
};

class BME680 : public BME68X{
    public:
        BME680(uint8_t addr, TwoWire *wire = &Wire);
        BME680(uint8_t addr, SPIClass *spi = &SPI);
        BME680(uint8_t cs, SSPIClass *sspi);
        BME680(uint8_t cs, BBSPIClass *bbspi);
    protected:

    private:
};

/*
class BME688 : public BME68X{
    public:
        BME688(uint8_t addr, TwoWire *wire = &Wire);
        BME688(uint8_t addr, SPIClass *spi = &SPI);
        BME688(uint8_t cs, SSPIClass *sspi);
        BME688(uint8_t cs, BBSPIClass *bbspi);
    protected:

    private:

};
*/


#endif