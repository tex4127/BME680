#include <Arduino.h>
#include "BME680_dev.h"

BME680* bme;
SSPIClass *SSPI;

void setup() {
  Serial.begin(115200);
  Serial.printf("Starting Program %s\n", __FILE__);
  Serial.printf("BME680 Tester");
  while(!Serial)
    ;
  Serial.printf("Initializing BME680 Chip\n");
  //bme = new BME680(10,&SPI);
  
  SSPI = new SSPIClass(10, 11, 12, 13, MSBFIRST, SPI_MODE0, SPI_CLOCK_DIV128);
  SSPI->begin();
  bme = new BME680(10, SSPI);
  
  bme->begin();
  Serial.printf("BME Status -> %i\n", bme->getStatus());
  Serial.printf("State of SCK Pin %u\n", digitalRead(13));
  Serial.printf("State of CS Pin  %u\n", digitalRead(10));
  while(1)
    ; 
}

void loop() {

}
