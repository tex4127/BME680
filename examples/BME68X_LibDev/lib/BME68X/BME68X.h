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

/********************************************************/
/*! @name  BME68X Macros                                */
/********************************************************/

/********************************************************/
/*! @name BME280 Addresses                              */
/********************************************************/

/********************************************************/
/*! @name BME280 Statuses                               */
/********************************************************/

/********************************************************/
/*! @name  BME280 Data Registers                        */
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

#define BME68X_REGISTER_PAR_TP          0xE9

//Temperature Calibration Registers
#define BME68X_REGISTER_CALIB_TEMP_START    0xE9
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
#define BME68X_REGISTER_PAR_P1              0x8E    //uint16_t
#define BME68X_REGISTER_PAR_P1_LSB          0x8E    //[7:0]
#define BME68X_REGISTER_PAR_P1_MSB          0x8F    //[15:8]
#define BME68X_REGISTER_PAR_P2              0x90    //uint16_t
#define BME68X_REGISTER_PAR_P2_LSB          0x90    //[7:0]
#define BME68X_REGISTER_PAR_P2_MSB          0x91    //[15:8]   
#define BME68X_REGISTER_PAR_P2              0x92    //uint8_t
#define BME68X_REGISTER_PAR_P3              0x94    //uint16_t
#define BME68X_REGISTER_PAR_P3_LSB          0x94    //[7:0]
#define BME68X_REGISTER_PAR_P3_MSB          0x95    //[15:8]
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




#endif