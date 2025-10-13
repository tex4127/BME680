#include <Arduino.h>
#include "BME68X.h"

//#define __USEHSPI__
//#define __USESSPI__
#define __USEBBSPI__

const char* fwName = "BME680 Example Script ";

BME68X *bme;
SSPIClass* sspi;
BBSPIClass* bbspi;


void setup() {
    Serial.begin(115200);
    delay(1000);
    while(!Serial)
        ;
    Serial.printf("Starting %s\n", fwName);
#if defined(__USEHSPI__)
    Serial.printf("Starting HardwareSPI Variant\n");
    bme = new BME680(10, &SPI);
#elif defined(__USESSPI__)
    Serial.printf("Starting SoftwareSPI Variant\n");
    sspi = new SSPIClass(11, 12, 13, MSBFIRST, SPI_MODE0, SPI_CLOCK_DIV2);
    bme = new BME680(10, sspi);
#elif defined(__USEBBSPI__)
    Serial.printf("Starting BitBangSPI Variant\n");
    bbspi = new BBSPIClass(11, 12, 13, MSBFIRST, SPI_MODE0);
    bme = new BME680(10, bbspi);
#endif
    bme->begin();
    Serial.printf("Begin Status -> %d\n", bme->begun());
    Serial.printf("Setting BME68X_Config\n");
    bme->setConfig(BME68X_Mode_e::BME68X_MODE_FORCED,
                    BME68X_OS_e::BME68X_OS_16X, BME68X_OS_e::BME68X_OS_16X, BME68X_OS_e::BME68X_OS_16X, 
                    BME68X_FilterSize_e::BME68X_FILTER_SIZE_15);
    Serial.printf("Getting BME68X_Config\n");
    BME68X_Config_t conf = bme->getConfig();
    Serial.printf("Mode  -> %02x\n", conf.mode);
    Serial.printf("osr_t -> %02x\n", conf.os_temp);
    Serial.printf("osr_p -> %02x\n", conf.os_press);
    Serial.printf("osr_h -> %02x\n", conf.os_hum);
    while(1)
        ;
}

void loop() {
  
}
