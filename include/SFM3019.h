/**
 * @file SFM3019.h
 * @author Robert R. Gomes
 * @brief Driver para leitura de sensor de fluxo SFM3019
 * @version 1.0
 * @date 11-11-2024
 * 
 * @copyright Copyright (c) 2024
 * 
*/

#ifndef _SFM3019_H_
#define _SFM3019_H_

#include <Arduino.h>
#include <Wire.h>


#define DEBUG_SFM   1

#define SFM_I2C_ADDRESS               0x2E

#define SFM_START_CONTINUOUS_READ     0x1000
#define SFM_READ_SCALE_FACTOR_OFFSET  0x3661
#define SFM_CONTINUOUS_READ_AIR       0x3608
#define SFM_CONTINUOUS_READ_O2        0x3603
#define SFM_CONTINUOUS_READ_AIR_O2    0x3632
#define SFM_STOP_CONTINUOUS_READ      0x3FF9
#define SFM_SOFTWARE_RESET            0x0006
#define SFM_CHANGE_O2_FRAC            0xE17D

#define SCALE_FACTOR_FLOW              170U
#define FLOW_OFFSET                    40960

#define SFM_MAX_FLOW                  0x00F0
#define SFM_MIN_FLOW                  0x000A


enum SFM_STATUS {
  SFM_OK       = 0,
  SFM_ERROR    = 1,
  CONNECTED    = 2,
  DISCONNECTED = 3
};

enum I2C_STATUS {
  I2C_SUCCESS      = 0,
  I2C_FULL_BUFFER  = 1,
  I2C_NACK_ADDR    = 2,
  I2C_NACK_ERROR   = 3,
  I2C_UNKNOW_ERROR = 4,
  I2C_PACK_ERROR   = 5
};

typedef union {
  struct {
    uint16_t cmd;
  };
  uint8_t u[2];  
} SFM_WRITE_CMD;

typedef union {
  struct {
    uint16_t cmd1;
    uint16_t cmd2;
    uint8_t  crc;
  };
  uint8_t u[5];
} SFM_WRITE_CMD_2;

typedef struct {
  uint16_t scaleFactor;
  uint8_t  crc1;
  uint16_t offset;
  uint8_t  crc2;
  uint16_t flowUnit;
  uint8_t  crc3;
} METER_SETTINGS;

typedef union {
	struct {
		uint16_t rawFlow;
		uint8_t crc;
	};
	uint8_t buffer[9];
} FLOW_DATA;

typedef struct {
  METER_SETTINGS settings;
  FLOW_DATA data;

  uint8_t status = DISCONNECTED;
  float rawFlow = 0;
} FLOW_METER;


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint8_t SFMInitialize(void);


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint8_t SFMStartContReadAir(void);


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint8_t SFMStopMeasurement(void);


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint8_t SFMSoftwareReset(void);


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint8_t SFMReadSettings(METER_SETTINGS *);


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint8_t SFMSetGasMixture(uint8_t o2Frac);


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint8_t SFMReadSensor(float *flow);


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint8_t SFMGenCRC(uint8_t *data, uint8_t len);


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint8_t I2C_Write(uint8_t address, uint8_t *data, uint8_t size);


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint8_t I2C_Read(uint8_t address, uint8_t *data, uint8_t size);


/**
 * @brief
 * 
 * @param
 * @return 
*/
uint16_t AdjustEndianness(uint16_t Val);



extern FLOW_METER   FlowMeter;


#endif