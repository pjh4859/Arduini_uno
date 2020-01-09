/*
MPU9250.h
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MPU9250_h
#define MPU9250_h

#include<math.h>
#include "Arduino.h"
#include "Wire.h"    // I2C library
#include "SPI.h"     // SPI library

#define G  9.807
#define _d2r  3.14159265 / 180.0
#define RAD2DGR		180.0/3.14159265

// MPU9250 registers
#define SELF_TEST_X_GYRO	0x00
#define SELF_TEST_Y_GYRO	0x01
#define SELF_TEST_Z_GYRO	0x02
#define SELF_TEST_X_ACCEL  0x0D
#define SELF_TEST_Y_ACCEL  0x0E
#define SELF_TEST_Z_ACCEL  0x0F
#define SELF_TEST_A  0x10//?
#define XG_OFFSET_H  0x13
#define XG_OFFSET_L  0x14
#define YG_OFFSET_H  0x15
#define YG_OFFSET_L  0x16
#define ZG_OFFSET_H  0x17
#define ZG_OFFSET_L  0x18
#define SMPLRT_DIV  0x19
#define CONFIG  0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_CONFIG2  0x1D
#define LP_ACCEL_ODR  0x1E
#define WOM_THR  0x1F
#define I2C_MST_EN  0x20
#define MOT_DUR  0x20//?
#define ZMOT_THR  0x21//?
#define ZRMOT_DUR  0x22//?
#define FIFO_EN  0x23
#define I2C_MST_CTRL  0x24
#define I2C_SLV0_ADDR  0x25
#define I2C_SLV0_REG  0x26
#define I2C_SLV0_CTRL  0x27
#define I2C_SLV1_ADDR  0x28
#define I2C_SLV1_REG  0x29
#define I2C_SLV1_CTRL  0x2A
#define I2C_SLV2_ADDR  0x2B
#define I2C_SLV2_REG  0x2C
#define I2C_SLV2_CTRL  0x2D
#define I2C_SLV3_ADDR  0x2E
#define I2C_SLV3_REG  0x2F
#define I2C_SLV3_CTRL  0x30
#define I2C_SLV4_ADDR  0x31
#define I2C_SLV4_REG  0x32
#define I2C_SLV4_DO  0x33
#define I2C_SLV4_CTRL  0x34
#define I2C_SLV4_DI  0x35
#define I2C_MST_STATUS  0x36
#define INT_PIN_CFG  0x37
#define INT_ENABLE  0x38
#define DMP_INT_STATUS  0x39  // Check DMP interrupt
#define INT_STATUS  0x3A
#define ACCEL_XOUT_H  0x3B
#define ACCEL_XOUT_L  0x3C
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40
#define TEMP_OUT_H  0x41
#define TEMP_OUT_L  0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define EXT_SENS_DATA_00  0x49
#define EXT_SENS_DATA_01  0x4A
#define EXT_SENS_DATA_02  0x4B
#define EXT_SENS_DATA_03  0x4C
#define EXT_SENS_DATA_04  0x4D
#define EXT_SENS_DATA_05  0x4E
#define EXT_SENS_DATA_06  0x4F
#define EXT_SENS_DATA_07  0x50
#define EXT_SENS_DATA_08  0x51
#define EXT_SENS_DATA_09  0x52
#define EXT_SENS_DATA_10  0x53
#define EXT_SENS_DATA_11  0x54
#define EXT_SENS_DATA_12  0x55
#define EXT_SENS_DATA_13  0x56
#define EXT_SENS_DATA_14  0x57
#define EXT_SENS_DATA_15  0x58
#define EXT_SENS_DATA_16  0x59
#define EXT_SENS_DATA_17  0x5A
#define EXT_SENS_DATA_18  0x5B
#define EXT_SENS_DATA_19  0x5C
#define EXT_SENS_DATA_20  0x5D
#define EXT_SENS_DATA_21  0x5E
#define EXT_SENS_DATA_22  0x5F
#define EXT_SENS_DATA_23  0x60
#define MOT_DETECT_STATUS  0x61//?
#define I2C_SLV0_DO  0x63
#define I2C_SLV1_DO  0x64
#define I2C_SLV2_DO  0x65
#define I2C_SLV3_DO  0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL  0x6A
#define PWR_MGMT_1  0x6B
#define PWR_MGMT_2  0x6C
#define DMP_BANK  0x6D//?
#define DMP_RW_PNT  0x6E//?
#define DMP_REG  0x6F//?
#define DMP_REG_1  0x70//?
#define DMP_REG_2  0x71//?
#define FIFO_COUNTH  0x72
#define FIFO_COUNTL  0x73
#define FIFO_R_W  0x74
#define WHO_AM_I  0x75
#define XA_OFFSET_H  0x77
#define XA_OFFSET_L  0x78
#define YA_OFFSET_H  0x7A
#define YA_OFFSET_L  0x7B
#define ZA_OFFSET_H  0x7D
#define ZA_OFFSET_L  0x7E
#define I2C_SLV0_EN  0x80//?
#define I2C_READ_FLAG  0x80//?

//AK8963 Register-----------------------------------
#define AK8963_WHO_AM_I  0x00
#define AK8963_INFO  0x01
#define AK8963_ST1  0x02
#define AK8963_HX_L  0x03
#define AK8963_HX_H  0x04
#define AK8963_HY_L  0x05
#define AK8963_HY_H  0x06
#define AK8963_HZ_L  0x07
#define AK8963_HZ_H  0x08
#define AK8963_ST2  0x09
#define AK8963_CNTL1  0x0A
#define AK8963_CNTL2  0x0B
#define AK8963_ASTC  0x0C
#define AK8963_TS1  0x0D
#define AK8963_TS2  0x0E
#define AK8963_I2CDIS  0x0F
#define AK8963_ASAX  0x10
#define AK8963_ASAY  0x11
#define AK8963_ASAZ  0x12
#define AK8963_ASA  0x10

//Register setting------------------------------------
#define CLOCK_SEL_PLL  0x01
#define I2C_MST_CLK 0x0D
#define AK8963_PWR_DOWN 0x00
#define PWR_RESET  0x80
#define AK8963_RESET  0x01
#define SEN_ENABLE  0x00
#define ACCEL_FS_SEL_2G  0x00
#define ACCEL_FS_SEL_4G  0x08
#define ACCEL_FS_SEL_8G  0x10
#define ACCEL_FS_SEL_16G  0x18
#define GYRO_FS_SEL_250DPS  0x00
#define GYRO_FS_SEL_500DPS  0x08
#define GYRO_FS_SEL_1000DPS  0x10
#define GYRO_FS_SEL_2000DPS  0x18

#define ACCEL_DLPF_184  0x09 //3dB BW218.1 Hz
#define ACCEL_DLPF_92 0x0A//3dB BW99 Hz
#define ACCEL_DLPF_41  0x0B//3dB BW44.8 Hz
#define ACCEL_DLPF_20  0x0C//3dB BW21.2 Hz
#define ACCEL_DLPF_10  0x0D//3dB BW10.2 Hz
#define ACCEL_DLPF_5  0x0E//3dB BW5.5 Hz   먼가 좀 이상해서 고침

#define GYRO_DLPF_184  0x01
#define GYRO_DLPF_92  0x02
#define GYRO_DLPF_41  0x03
#define GYRO_DLPF_20  0x04
#define GYRO_DLPF_10  0x05
#define GYRO_DLPF_5  0x06

#define AK8963_FUSE_ROM  0x0F
#define AK8963_CNT_MEAS1  0x12
#define AK8963_CNT_MEAS2  0x16

class MPU9250{
  public:
    enum GyroRange
    {
      GYRO_RANGE_250DPS,
      GYRO_RANGE_500DPS,
      GYRO_RANGE_1000DPS,
      GYRO_RANGE_2000DPS
    };
    enum AccelRange
    {
      ACCEL_RANGE_2G,
      ACCEL_RANGE_4G,
      ACCEL_RANGE_8G,
      ACCEL_RANGE_16G    
    };
    enum DlpfBandwidth
    {
      DLPF_BANDWIDTH_184HZ,
      DLPF_BANDWIDTH_92HZ,
      DLPF_BANDWIDTH_41HZ,
      DLPF_BANDWIDTH_20HZ,
      DLPF_BANDWIDTH_10HZ,
      DLPF_BANDWIDTH_5HZ
    };
    enum LpAccelOdr
    {
      LP_ACCEL_ODR_0_24HZ = 0,
      LP_ACCEL_ODR_0_49HZ = 1,
      LP_ACCEL_ODR_0_98HZ = 2,
      LP_ACCEL_ODR_1_95HZ = 3,
      LP_ACCEL_ODR_3_91HZ = 4,
      LP_ACCEL_ODR_7_81HZ = 5,
      LP_ACCEL_ODR_15_63HZ = 6,
      LP_ACCEL_ODR_31_25HZ = 7,
      LP_ACCEL_ODR_62_50HZ = 8,
      LP_ACCEL_ODR_125HZ = 9,
      LP_ACCEL_ODR_250HZ = 10,
      LP_ACCEL_ODR_500HZ = 11
    };
       
    MPU9250(TwoWire &bus,uint8_t address);
    //MPU9250(SPIClass &bus,uint8_t csPin);
    int begin();
    int setAccelRange(AccelRange range);
    int setGyroRange(GyroRange range);
    int setDlpfBandwidth(DlpfBandwidth bandwidth);
    int setSrd(uint8_t srd);
    int enableDataReadyInterrupt();
    int disableDataReadyInterrupt();
    int enableWakeOnMotion(float womThresh_mg,LpAccelOdr odr);
    int readSensor();
    float getAccelX_mss();
    float getAccelY_mss();
    float getAccelZ_mss();
    float getGyroX_rads();
    float getGyroY_rads();
    float getGyroZ_rads();
    float getMagX_uT();
    float getMagY_uT();
    float getMagZ_uT();
    float getTemperature_C();
    //---------Add functions START------------------------------------
    void Accel_to_degree(float* pAcc, float* pAcc_dgr);
    float getAccelX_mss_dgr();
    float getAccelY_mss_dgr();
    float getAccelZ_mss_dgr();
       

    //---------Add functions END------------------------------------
    
    int calibrateGyro();
    float getGyroBiasX_rads();
    float getGyroBiasY_rads();
    float getGyroBiasZ_rads();
    void setGyroBiasX_rads(float bias);
    void setGyroBiasY_rads(float bias);
    void setGyroBiasZ_rads(float bias);
    int calibrateAccel();
    float getAccelBiasX_mss();
    float getAccelScaleFactorX();
    float getAccelBiasY_mss();
    float getAccelScaleFactorY();
    float getAccelBiasZ_mss();
    float getAccelScaleFactorZ();
    void setAccelCalX(float bias,float scaleFactor);
    void setAccelCalY(float bias,float scaleFactor);
    void setAccelCalZ(float bias,float scaleFactor);
    int calibrateMag();
    float getMagBiasX_uT();
    float getMagScaleFactorX();
    float getMagBiasY_uT();
    float getMagScaleFactorY();
    float getMagBiasZ_uT();
    float getMagScaleFactorZ();
    void setMagCalX(float bias,float scaleFactor);
    void setMagCalY(float bias,float scaleFactor);
    void setMagCalZ(float bias,float scaleFactor);
  protected:
    // i2c
    uint8_t _address;
    TwoWire *_i2c;
    const uint32_t _i2cRate = 400000; // 400 kHz
    size_t _numBytes; // number of bytes received from I2C
    // spi
    //SPIClass *_spi;
    //uint8_t _csPin;
    //bool _useSPI;
    //bool _useSPIHS;
    //const uint8_t SPI_READ = 0x80;
    //const uint32_t SPI_LS_CLOCK = 1000000;  // 1 MHz
    //const uint32_t SPI_HS_CLOCK = 15000000; // 15 MHz
    // track success of interacting with sensor
    int _status;
    // buffer for reading from sensor
    uint8_t _buffer[21];
    // data counts
    int16_t _axcounts,_aycounts,_azcounts;
    int16_t _gxcounts,_gycounts,_gzcounts;
    int16_t _hxcounts,_hycounts,_hzcounts;
    int16_t _tcounts;
    // data buffer
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _hx, _hy, _hz;
    float _t;
    // angle data------------------------------------
    typedef struct Gyrostruct {
        double	past_time;
        double	present_time;
        double  delta_time;
        float	gyro_sample[3];
        float	gyro_current[3];
        float	gyro_delta[3];
    }GYROSTRUCT;

    float Acc[3];
    float* pAcc = Acc;
    float Acc_dgr[3] = { 0, };
    float* pAcc_dgr = Acc_dgr;

    GYROSTRUCT gs = { 0, };
    GYROSTRUCT* pgs;

    short Mag[3];
    short* pMag = Mag;
    float Mag_dgr[3] = { 0, };
    float* pMag_dgr = Mag_dgr;
    float Mag_adj[3];
    float Mag_sensi[3];

    float compli_angle[3];
    
    // wake on motion
    uint8_t _womThreshold;
    // scale factors
    float _accelScale;
    float _gyroScale;
    float _magScaleX, _magScaleY, _magScaleZ;
    const float _tempScale = 333.87f;
    const float _tempOffset = 21.0f;
    // configuration
    AccelRange _accelRange;
    GyroRange _gyroRange;
    DlpfBandwidth _bandwidth;
    uint8_t _srd;
    // gyro bias estimation
    size_t _numSamples = 100;
    double _gxbD, _gybD, _gzbD;
    float _gxb, _gyb, _gzb;
    // accel bias and scale factor estimation
    double _axbD, _aybD, _azbD;
    float _axmax, _aymax, _azmax;
    float _axmin, _aymin, _azmin;
    float _axb, _ayb, _azb;
    float _axs = 1.0f;
    float _ays = 1.0f;
    float _azs = 1.0f;
    // magnetometer bias and scale factor estimation
    uint16_t _maxCounts = 1000;
    float _deltaThresh = 0.3f;
    uint8_t _coeff = 8;
    uint16_t _counter;
    float _framedelta, _delta;
    float _hxfilt, _hyfilt, _hzfilt;
    float _hxmax, _hymax, _hzmax;
    float _hxmin, _hymin, _hzmin;
    float _hxb, _hyb, _hzb;
    float _hxs = 1.0f;
    float _hys = 1.0f;
    float _hzs = 1.0f;
    float _avgs;
    // transformation matrix
    /* transform the accel and gyro axes to match the magnetometer axes */
    const int16_t tX[3] = {0,  1,  0}; 
    const int16_t tY[3] = {1,  0,  0};
    const int16_t tZ[3] = {0,  0, -1};
    // constants
    //const float G = 9.807f;
    //const float _d2r = 3.14159265359f/180.0f;
    // MPU9250 registers

    const uint8_t ACCEL_OUT = 0x3B;
    const uint8_t GYRO_OUT = 0x43;
    const uint8_t TEMP_OUT = 0x41;
    //const uint8_t EXT_SENS_DATA_00 = 0x49;
    //const uint8_t ACCEL_CONFIG = 0x1C;
    //const uint8_t ACCEL_FS_SEL_2G = 0x00;
    //const uint8_t ACCEL_FS_SEL_4G = 0x08;
    //const uint8_t ACCEL_FS_SEL_8G = 0x10;
    //const uint8_t ACCEL_FS_SEL_16G = 0x18;
    //const uint8_t GYRO_CONFIG = 0x1B;
    //const uint8_t GYRO_FS_SEL_250DPS = 0x00;
    //const uint8_t GYRO_FS_SEL_500DPS = 0x08;
    //const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
    //const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
    //const uint8_t ACCEL_CONFIG2 = 0x1D;
    //const uint8_t ACCEL_DLPF_184 = 0x01;
    //const uint8_t ACCEL_DLPF_92 = 0x02;
    //const uint8_t ACCEL_DLPF_41 = 0x03;
    //const uint8_t ACCEL_DLPF_20 = 0x04;
    //const uint8_t ACCEL_DLPF_10 = 0x05;
    //const uint8_t ACCEL_DLPF_5 = 0x06;
    //const uint8_t CONFIG = 0x1A;
    //const uint8_t GYRO_DLPF_184 = 0x01;
    //const uint8_t GYRO_DLPF_92 = 0x02;
    //const uint8_t GYRO_DLPF_41 = 0x03;
    //const uint8_t GYRO_DLPF_20 = 0x04;
    //const uint8_t GYRO_DLPF_10 = 0x05;
    //const uint8_t GYRO_DLPF_5 = 0x06;
    //const uint8_t SMPLRT_DIV = 0x19;
    //const uint8_t INT_PIN_CFG = 0x37;
    //const uint8_t INT_ENABLE = 0x38;
    const uint8_t INT_DISABLE = 0x00;
    const uint8_t INT_PULSE_50US = 0x00;
    const uint8_t INT_WOM_EN = 0x40;
    const uint8_t INT_RAW_RDY_EN = 0x01;
    //const uint8_t PWR_MGMT_1 = 0x6B;
    const uint8_t PWR_CYCLE = 0x20;
    //const uint8_t PWR_RESET = 0x80;
    //const uint8_t CLOCK_SEL_PLL = 0x01;
    //const uint8_t PWR_MGMT_2 = 0x6C;
    //const uint8_t SEN_ENABLE = 0x00;
    const uint8_t DIS_GYRO = 0x07;
    //const uint8_t USER_CTRL = 0x6A;
    //const uint8_t I2C_MST_EN = 0x20;
    //const uint8_t I2C_MST_CLK = 0x0D;
    //const uint8_t I2C_MST_CTRL = 0x24;
    //const uint8_t I2C_SLV0_ADDR = 0x25;
    //const uint8_t I2C_SLV0_REG = 0x26;
    //const uint8_t I2C_SLV0_DO = 0x63;
    //const uint8_t I2C_SLV0_CTRL = 0x27;
    //const uint8_t I2C_SLV0_EN = 0x80;
    //const uint8_t I2C_READ_FLAG = 0x80;
    //const uint8_t MOT_DETECT_CTRL = 0x69;
    const uint8_t ACCEL_INTEL_EN = 0x80;
    const uint8_t ACCEL_INTEL_MODE = 0x40;
    //const uint8_t LP_ACCEL_ODR = 0x1E;
    //const uint8_t WOM_THR = 0x1F;
    //const uint8_t WHO_AM_I = 0x75;
    //const uint8_t FIFO_EN = 0x23;
    const uint8_t FIFO_TEMP = 0x80;
    const uint8_t FIFO_GYRO = 0x70;
    const uint8_t FIFO_ACCEL = 0x08;
    const uint8_t FIFO_MAG = 0x01;
    const uint8_t FIFO_COUNT = 0x72;
    const uint8_t FIFO_READ = 0x74;
    // AK8963 registers
    const uint8_t AK8963_I2C_ADDR = 0x0C;
    const uint8_t AK8963_HXL = 0x03; 
    //const uint8_t AK8963_CNTL1 = 0x0A;
    //const uint8_t AK8963_PWR_DOWN = 0x00;
    //const uint8_t AK8963_CNT_MEAS1 = 0x12;
    //const uint8_t AK8963_CNT_MEAS2 = 0x16;
    //const uint8_t AK8963_FUSE_ROM = 0x0F;
    //const uint8_t AK8963_CNTL2 = 0x0B;
    //const uint8_t AK8963_RESET = 0x01;
    //const uint8_t AK8963_ASA = 0x10;
    //const uint8_t AK8963_WHO_AM_I = 0x00;
    // private functions
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int writeAK8963Register(uint8_t subAddress, uint8_t data);
    int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int whoAmI();
    int whoAmIAK8963();
};

//class MPU9250FIFO: public MPU9250 {
//  public:
//    using MPU9250::MPU9250;
//    int enableFifo(bool accel,bool gyro,bool mag,bool temp);
//    int readFifo();
//    void getFifoAccelX_mss(size_t *size,float* data);
//    void getFifoAccelY_mss(size_t *size,float* data);
//    void getFifoAccelZ_mss(size_t *size,float* data);
//    void getFifoGyroX_rads(size_t *size,float* data);
//    void getFifoGyroY_rads(size_t *size,float* data);
//    void getFifoGyroZ_rads(size_t *size,float* data);
//    void getFifoMagX_uT(size_t *size,float* data);
//    void getFifoMagY_uT(size_t *size,float* data);
//    void getFifoMagZ_uT(size_t *size,float* data);
//    void getFifoTemperature_C(size_t *size,float* data);
  //protected:
  //  // fifo
  //  bool _enFifoAccel,_enFifoGyro,_enFifoMag,_enFifoTemp;
  //  size_t _fifoSize,_fifoFrameSize;
  //  float _axFifo[85], _ayFifo[85], _azFifo[85];
  //  size_t _aSize;
  //  float _gxFifo[85], _gyFifo[85], _gzFifo[85];
  //  size_t _gSize;
  //  float _hxFifo[73], _hyFifo[73], _hzFifo[73];
  //  size_t _hSize;
  //  float _tFifo[256];
  //  size_t _tSize;
//};

#endif
