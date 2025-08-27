#include <Arduino.h>
#include "BME680_dev.h"

BME680* bme;

void setup() {
  Serial.begin(115200);
  Serial.printf("Starting Program %s\n", __FILE__);
  Serial.printf("BME680 Tester");
  while(!Serial)
    ;
  Serial.printf("Initializing BME680 Chip\n");
  bme = new BME680(10,11,12,13);
  bme->begin();
}

void loop() {

}
