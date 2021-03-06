/*UVOS*/

/* This file is part of TelemetrySystem.
 *
 * TelemetrySystem is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) version 3 as published by
 * the Free Software Foundation.
 *
 * TelemetrySystem is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with TelemetrySystem.  If not, see <http://www.gnu.org/licenses/>.
*/

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


Mpu9150::Mpu9150(const uint8_t  address, const uint8_t compassAddress): I2cDevice(address), _compassAddress(compassAddress)
{
    init();
}

void Mpu9150::init()
{
    start();
    
    writeRegsiter(MPU9150_ACCEL_CONFIG, 0b00010000);
    
    writeRegsiter(MPU9150_I2C_MST_CTRL, 0x40); //Wait for Data at Slave0
    
    //TO DO: figure out why we need to configure slave 0. What device is at 0x8C?
    writeRegsiter(MPU9150_I2C_SLV0_ADDR, 0x8C); //Set i2c address at slave0 at 0x0C
    writeRegsiter(MPU9150_I2C_SLV0_REG, 0x02); //Set where reading at slave 0 starts
    writeRegsiter(MPU9150_I2C_SLV0_CTRL, 0x88); //set offset at start reading and enable
    
    //Configure compass as slave 1.
    writeRegsiter(MPU9150_I2C_SLV1_ADDR, _compassAddress); //set i2c address at slv1 at 0x0C
    writeRegsiter(MPU9150_I2C_SLV1_REG, 0x0A); //Set where reading at slave 1 starts
    writeRegsiter(MPU9150_I2C_SLV1_CTRL, 0x81); //Enable at set length to 1
    writeRegsiter(MPU9150_I2C_SLV1_DO, 0x01); //overvride register
    writeRegsiter(MPU9150_I2C_MST_DELAY_CTRL, 0x03); //set delay rate
    
    writeRegsiter(0x01, 0x80);

    writeRegsiter(MPU9150_I2C_SLV1_DO, 0x00); //override register
    writeRegsiter(MPU9150_USER_CTRL, 0x00); //clear usr setting
    writeRegsiter(MPU9150_I2C_SLV1_DO, 0x01); //override register
    writeRegsiter(MPU9150_USER_CTRL, 0x20); //enable master i2c mode
    
    stop();
}

void Mpu9150::writeRegsiter( uint8_t address, uint8_t data )
{
    uint8_t registerSeq[2] = {address, data};
    write(registerSeq, 2, true);
}

void Mpu9150::start()
{
    writeRegsiter(MPU9150_PWR_MGMT_1, 0); //pull device out of sleep
}

void Mpu9150::stop()
{
    writeRegsiter(MPU9150_PWR_MGMT_1, 0b01000000); //put device to sleep
}

Point3D <int16_t> Mpu9150::getAccelData()
{
    Point3D <int16_t> result;
    
    result.x  = txRxSequence(MPU9150_ACCEL_XOUT_L) << 8;
    result.x += txRxSequence(MPU9150_ACCEL_XOUT_H);
    
    result.y  = txRxSequence(MPU9150_ACCEL_YOUT_L) << 8;
    result.y += txRxSequence(MPU9150_ACCEL_YOUT_H);
    
    result.z  = txRxSequence(MPU9150_ACCEL_ZOUT_L) << 8;
    result.z += txRxSequence(MPU9150_ACCEL_ZOUT_H);
    
    return result;
}

Point3D <int16_t> Mpu9150::getMagnData()
{
    Point3D <int16_t> result;
    
    result.x  = txRxSequence(MPU9150_GYRO_XOUT_L) << 8;
    result.x += txRxSequence(MPU9150_GYRO_XOUT_H);
    
    
    result.y  = txRxSequence(MPU9150_GYRO_YOUT_L) << 8;
    result.y += txRxSequence(MPU9150_GYRO_YOUT_H);
    
    
    result.z  = txRxSequence(MPU9150_GYRO_ZOUT_L) << 8;
    result.z += txRxSequence(MPU9150_GYRO_ZOUT_H);
    
    return result;
}

int16_t Mpu9150::getTemperature()
{
   int16_t result = 0;
   
   result  = txRxSequence(MPU9150_TEMP_OUT_H) << 8;
   result += txRxSequence(MPU9150_TEMP_OUT_L);
   
   result = result/340 + 35; //convert device internal units to celsius
   
   return result;
}
