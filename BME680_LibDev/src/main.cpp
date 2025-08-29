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
  Serial.printf("Reference to SSPI %i\n", SSPI);
  SSPI->begin();
  bme = new BME680(10, SSPI);
  
  bme->begin();
  Serial.printf("BME Status -> %i\n", bme->getStatus());
  Serial.printf("State of SCK Pin %u\n", digitalRead(13));
  Serial.printf("State of CS Pin  %u\n", digitalRead(10));
  if(bme->getStatus() != 0){
    while(1)
      ; 
  }

  Serial.printf("Configuring BME680\n");
  bme->setSensor_OS(BME680_OS_16X, BME680_OS_16X, BME680_OS_16X);
  bme->setHeaterProfile(300, 100);

  Serial.printf("Moving to Loop!\n");
}

void loop() {
  BME680_Data bmeData;

  bme->setOperMode(BME680_Mode_e::BME680_MODE_FORCED);
  delayMicroseconds(bme->getMeasurementTime());
  if(bme->fetchData()){
    bme->getData(&bmeData);
    Serial.printf("%010lu | %03.2f \u00B0C | %07.2f Pa | %03.2f \u00B0/\u2092 | %.2f \u03A9\n",millis(), bmeData.temperature, bmeData.pressure, bmeData.humidity, bmeData.gas_resistance);

  }
}
