//UVOS
#pragma once

#include "i2c.h"
#include "point3D.h"


#define DEFAULT_MPU9150_I2C_ADDRESS 0x68
#define DEFAULT_COMPASS_SUBDEVICE_I2C_ADDRESS 0x0C

class Mpu9150 : private I2cDevice
{
private:
    void writeRegsiter( uint8_t address, uint8_t data );
    const uint8_t _compassAddress;
public:
    
    Mpu9150(const uint8_t  address = DEFAULT_MPU9150_I2C_ADDRESS, const uint8_t compassAddress = DEFAULT_COMPASS_SUBDEVICE_I2C_ADDRESS, const uint32_t sclPin = DEFAULT_SCL_PIN, const uint32_t sdaPin = DEFAULT_SDA_PIN);
    
    Point3D <int16_t> getAccelData();
    Point3D <int16_t> getMagnData();
    
    int16_t getTemperature();
    
    void stop();
    void start();
    
};
