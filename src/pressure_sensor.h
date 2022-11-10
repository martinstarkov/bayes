// TODO: Add header include guards.

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

class PressureSensor {
public:
    void Init(uint8_t address) {
        bool status = bmp.begin(address);
        if (!status) {
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring"));
            Serial.print("SensorID was: 0x"); 
            Serial.println(bmp.sensorID(), 16);
            while (1);
        }

        // TODO: Check if this is necessary. What does it even do?
        // Default settings from datasheet.
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    }
    float GetTemperature() {
        return bmp.readTemperature();
    }
    float GetPressure() {
        return bmp.readPressure();
    }
    float GetAltitude(float sea_level_hPa = 1013.25) {
        return bmp.readAltitude(sea_level_hPa);
    }
private:
    Adafruit_BMP280 bmp;
};