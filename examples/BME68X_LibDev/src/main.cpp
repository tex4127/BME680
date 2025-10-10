#include <Arduino.h>
#include "BME68X.h"

#define __USEHSPI__
//#define __USESSPI__
//#define __USEBBSPI__

const char* fwName = "BME680 Example Script ";

BME68X *bme;

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
#elif defined(__USEBBSPI__)
    Serial.printf("Starting BitBangSPI Variant\n");
#endif
    bme->begin();
    while(1)
        ;
}

void loop() {
  
}
