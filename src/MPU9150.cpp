//UVOS
#include "MPU9150.h"

//Register defines taken from MPU9150 datasheet 
#define MPU9150_SMPLRT_DIV         0x19   // R/W
#define MPU9150_CONFIG             0x1A   // R/W
#define MPU9150_GYRO_CONFIG        0x1B   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W
#define MPU9150_FIFO_EN            0x23   // R/W
#define MPU9150_I2C_MST_CTRL       0x24   // R/W
#define MPU9150_I2C_SLV0_ADDR      0x25   // R/W
#define MPU9150_I2C_SLV0_REG       0x26   // R/W
#define MPU9150_I2C_SLV0_CTRL      0x27   // R/W
#define MPU9150_I2C_SLV1_ADDR      0x28   // R/W
#define MPU9150_I2C_SLV1_REG       0x29   // R/W
#define MPU9150_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU9150_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU9150_I2C_SLV2_REG       0x2C   // R/W
#define MPU9150_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU9150_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU9150_I2C_SLV3_REG       0x2F   // R/W
#define MPU9150_I2C_SLV3_CTRL      0x30   // R/W
#define MPU9150_I2C_SLV4_ADDR      0x31   // R/W
#define MPU9150_I2C_SLV4_REG       0x32   // R/W
#define MPU9150_I2C_SLV4_DO        0x33   // R/W
#define MPU9150_I2C_SLV4_CTRL      0x34   // R/W
#define MPU9150_I2C_SLV4_DI        0x35   // R  
#define MPU9150_I2C_MST_STATUS     0x36   // R
#define MPU9150_INT_PIN_CFG        0x37   // R/W
#define MPU9150_INT_ENABLE         0x38   // R/W
#define MPU9150_INT_STATUS         0x3A   // R  
#define MPU9150_ACCEL_XOUT_H       0x3B   // R  
#define MPU9150_ACCEL_XOUT_L       0x3C   // R  
#define MPU9150_ACCEL_YOUT_H       0x3D   // R  
#define MPU9150_ACCEL_YOUT_L       0x3E   // R  
#define MPU9150_ACCEL_ZOUT_H       0x3F   // R  
#define MPU9150_ACCEL_ZOUT_L       0x40   // R  
#define MPU9150_TEMP_OUT_H         0x41   // R  
#define MPU9150_TEMP_OUT_L         0x42   // R  
#define MPU9150_GYRO_XOUT_H        0x43   // R  
#define MPU9150_GYRO_XOUT_L        0x44   // R  
#define MPU9150_GYRO_YOUT_H        0x45   // R  
#define MPU9150_GYRO_YOUT_L        0x46   // R  
#define MPU9150_GYRO_ZOUT_H        0x47   // R  
#define MPU9150_GYRO_ZOUT_L        0x48   // R  
#define MPU9150_MOT_DETECT_STATUS  0x61   // R  
#define MPU9150_I2C_SLV0_DO        0x63   // R/W
#define MPU9150_I2C_SLV1_DO        0x64   // R/W
#define MPU9150_I2C_SLV2_DO        0x65   // R/W
#define MPU9150_I2C_SLV3_DO        0x66   // R/W
#define MPU9150_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU9150_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU9150_MOT_DETECT_CTRL    0x69   // R/W
#define MPU9150_USER_CTRL          0x6A   // R/W
#define MPU9150_PWR_MGMT_1         0x6B   // R/W
#define MPU9150_PWR_MGMT_2         0x6C   // R/W
#define MPU9150_FIFO_COUNTH        0x72   // R/W
#define MPU9150_FIFO_COUNTL        0x73   // R/W
#define MPU9150_FIFO_R_W           0x74   // R/W
#define MPU9150_CMPS_XOUT_L        0x4A   // R
#define MPU9150_CMPS_XOUT_H        0x4B   // R
#define MPU9150_CMPS_YOUT_L        0x4C   // R
#define MPU9150_CMPS_YOUT_H        0x4D   // R
#define MPU9150_CMPS_ZOUT_L        0x4E   // R
#define MPU9150_CMPS_ZOUT_H        0x4F   // R


Mpu9150::Mpu9150(const uint8_t  address, const uint32_t sclPin, const uint32_t sdaPin): I2cDevice(address, sclPin, sdaPin)
{
    start();
    
    uint8_t mpuAdress = _devAdress;
    _devAdress = _compassAddress << 1;      //change Address to Compass
    
    //Subdevice init sequence taken from datasheet

    writeRegsiter(0x0A, 0x00); //PowerDownMode
    writeRegsiter(0x0A, 0x0F); //SelfTest
    writeRegsiter(0x0A, 0x00); //PowerDownMode

    _devAdress = mpuAdress;      //change Address to MPU

    writeRegsiter(0x24, 0x40); //Wait for Data at Slave0
    writeRegsiter(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
    writeRegsiter(0x26, 0x02); //Set where reading at slave 0 starts
    writeRegsiter(0x27, 0x88); //set offset at start reading and enable
    writeRegsiter(0x28, _compassAddress); //set i2c address at slv1 at 0x0C
    writeRegsiter(0x29, 0x0A); //Set where reading at slave 1 starts
    writeRegsiter(0x2A, 0x81); //Enable at set length to 1
    writeRegsiter(0x64, 0x01); //overvride register
    writeRegsiter(0x67, 0x03); //set delay rate
    writeRegsiter(0x01, 0x80);

    writeRegsiter(0x34, 0x04); //set i2c slv4 delay
    writeRegsiter(0x64, 0x00); //override register
    writeRegsiter(0x6A, 0x00); //clear usr setting
    writeRegsiter(0x64, 0x01); //override register
    writeRegsiter(0x6A, 0x20); //enable master i2c mode
    writeRegsiter(0x34, 0x13); //disable slv4
    
    //stop()
}

void Mpu9150::writeRegsiter( uint8_t address, uint8_t data )
{
    write(address, 1);
    write(data, 1);
}

void Mpu9150::start()
{
    writeRegsiter(MPU9150_PWR_MGMT_1, 0); //pull device out of sleep
}

void Mpu9150::stop()
{
    writeRegsiter(MPU9150_PWR_MGMT_1, 1); //put device to sleep
}

point3D <int16_t> Mpu9150::getAccelData()
{
    point3D <int16_t> result;
    
    result.x  = txRxSequence(MPU9150_ACCEL_XOUT_H) << 8;
    result.x += txRxSequence(MPU9150_ACCEL_XOUT_L);
    MPU9150_CMPS_XOUT_L
    
    result.y  = txRxSequence(MPU9150_ACCEL_YOUT_H) << 8;
    result.y += txRxSequence(MPU9150_ACCEL_YOUT_L);
    
    
    result.z  = txRxSequence(MPU9150_ACCEL_ZOUT_H) << 8;
    result.z += txRxSequence(MPU9150_ACCEL_ZOUT_L);
    
    return result;
}

point3D <int16_t> Mpu9150::getMagnData()
{
    point3D <int16_t> result;
    
    result.x  = txRxSequence(MPU9150_CMPS_XOUT_H) << 8;
    result.x += txRxSequence(MPU9150_CMPS_XOUT_L);
    
    
    result.y  = txRxSequence(MPU9150_CMPS_YOUT_H) << 8;
    result.y += txRxSequence(MPU9150_CMPS_YOUT_L);
    
    
    result.z  = txRxSequence(MPU9150_CMPS_ZOUT_H) << 8;
    result.z += txRxSequence(MPU9150_CMPS_ZOUT_L);
    
    return result;
}

int16_t Mpu9150::getTempareture()
{
   int16_t result = 0;
   
   result  = txRxSequence(MPU9150_TEMP_OUT_H) << 8;
   result += txRxSequence(MPU9150_TEMP_OUT_L);
   
   return result;
}
