#include <Arduino.h>
#include "BME68X.h"
#include <SD.h>
//#include <SDFat.h>

#define SD_STATUS_OK            0
#define SD_ERROR_FILENOTFOUND   -1
#define SD_ERROR_DATANOTWRITTEN -2

#define __USEHSPI__
//#define __USESSPI__
//#define __USEBBSPI__

const char* fwName = "BME680 Example Script ";
const char* dir = "BME680_Example/";
uint16_t fileNum = 0;
char fileName[50];
bool sdCardPresent = false;
const char *header = "Temp \u00B0C,Pressure Pa,Humidity RH,RGA \u03A9\n";

BME68X *bme;
SSPIClass* sspi;
BBSPIClass* bbspi;

void printData(BME68X_Data_t d){
    Serial.printf("Temperature -> %02.2f \u00B0C\n", d.temperature);
    Serial.printf("Pressure    -> %06.2f pA\n", d.pressure);
    Serial.printf("Humidity    -> %02.2f perCent\n", d.humidity);
    Serial.printf("Resistance  -> %02.2f \u03A9\n", d.resistance);
}

void printSDContents(File dir){
    Serial.printf("printSDContents(%s)\n", dir.name());
    while(1){
        File entry = dir.openNextFile();
        if(!entry){
            break;
        }
        Serial.printf("%s", entry.name());
        if(entry.isDirectory()){
            Serial.printf("/");
            printSDContents(entry);
        }else{
            Serial.printf("\t\t %d\n", entry.size());
        }
    }
}

void createDirectories(const char* dirName){
    if(SD.exists(dirName)){
        Serial.printf("Dir Name Exists -> %s\n", dirName);
        return;
    }
    SD.mkdir(dirName);
    Serial.printf("Dir Created -> %s \n", dirName);
}

void updateFileName(char* fName, uint16_t *fNum){
    while(SD.exists(fName)){
        *fNum += 1;
        sprintf(fName, "%sData_%04u.csv", dir, *fNum);
    }
    Serial.printf("File Name -> %s\n", fName);
}

void logData(const char* fileName, const char* data){
    //Serial.printf("%s exist status -> %d\n", fileName, SD.exists(fileName));
    File f = SD.open(fileName, FILE_WRITE);
    if(!f){
        Serial.printf("Error, File Not Found -> %s\n", fileName);
        return;
    }
    f.printf("%s\n", data);
    f.close();
}

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
                    BME68X_FilterSize_e::BME68X_FILTER_SIZE_127);
    bme->setHeaterConfig(300, 100);
    Serial.printf("Getting BME68X_Config\n");
    BME68X_Config_t conf = bme->getConfig();
    Serial.printf("Mode  -> %02x\n", conf.mode);
    Serial.printf("osr_t -> %02x\n", conf.os_temp);
    Serial.printf("osr_p -> %02x\n", conf.os_press);
    Serial.printf("osr_h -> %02x\n", conf.os_hum);
    delay(1000);
    bme->readSensor();
    BME68X_Data_t data = bme->getSensorData();
    Serial.println();
    printData(data);
    sdCardPresent = SD.begin(BUILTIN_SDCARD);
    if(sdCardPresent){
        sprintf(fileName, "%sData_%04u.csv", dir, fileNum);
        Serial.printf("%s\n", fileName);
        createDirectories(dir);
        updateFileName(fileName, &fileNum);
        logData(fileName, header);
        //File root = SD.open("/");
        //printSDContents(root);
        //root.close();
    }
    delay(1000);
}

void loop() {
    unsigned long st = millis();
    bme->setOperationMode(BME68X_Mode_e::BME68X_MODE_FORCED);
    //printf("Delay -> %lu\n", bme->getMeasurementTime());
    delayMicroseconds(bme->getMeasurementTime());
    bme->readSensor();
    BME68X_Data_t data = bme->getSensorData();
    if(sdCardPresent){
        char dataLog[70];
        sprintf(dataLog, "%1.4e,%1.4e,%1.4e,%1.8e,%lu", data.temperature, data.pressure, data.humidity, data.resistance, millis());
        logData(fileName, dataLog);
        Serial.printf("%s\n", dataLog);
    }else{
        Serial.println();
        printData(data);
    }
    while(millis() - st < 5000)
        ;
}
