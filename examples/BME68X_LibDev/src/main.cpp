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
    while(1)
        ;
}

void loop() {
  
}
