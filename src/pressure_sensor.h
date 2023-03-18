// TODO: Add header include guards.

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

class PressureSensor {
private:
    Adafruit_BMP280 bmp;
public:
    void init(uint8_t address) {
        if (!bmp.begin(address)) {
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring"));
            while (1);
        }
    }

    float getTemperature() {
        return bmp.readTemperature();
    }

    float getPressure() {
        return bmp.readPressure();
    }

    float getAltitude(float sea_level_hPa = 1013.25) {
        return bmp.readAltitude(sea_level_hPa);
    }
};