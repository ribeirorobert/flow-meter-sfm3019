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

#define DEBUG_SFM                1
#define SFM_I2C_ADDRESS          0x2E

#define SFM_CONT_READ_AIR        0x3608
#define SFM_CONT_READ_O2         0x3603
#define SFM_CONT_READ_AIR_O2     0x3632
#define SFM_CHANGE_O2_FRAC       0xE17D
#define SFM_STOP_CONT_READ       0x3FF9
#define SFM_SOFT_RESET           0x0006
#define SFM_READ_SCALE_OFFSET    0x3661

#define SCALE_FACTOR_FLOW    170U
#define FLOW_OFFSET          40960
#define MAX_ALLOWED_ERRORS   10

#define RESET_TIMER_MS       10
#define START_TIMER_MS       40
#define PACKET_TIMER_MS      1
#define READ_PERIOD_MS       10

enum SFM_STATUS {
  RESET          = 0,
  RESETING       = 1,
  STOP           = 2,
  STOPPING       = 3,
  READ_SETTINGS  = 4,
  START          = 5,
  STARTING       = 6,
  DISCARD_PACKET = 7,
  NEW_PACKET     = 8,

  INITIALIZED     = 9,
  NOT_INITIALIZED = 10,
};

enum I2C_STATUS {
  I2C_SUCCESS      = 0,
  I2C_FULL_BUFFER  = 1,
  I2C_NACK_ADDR    = 2,
  I2C_NACK_ERROR   = 3,
  I2C_UNKNOW_ERROR = 4,
  I2C_PACK_ERROR   = 5,
  I2C_CRC_ERROR    = 6
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

  float rawFlow;
  uint8_t state;
  
  uint8_t errorCounter;
  uint32_t readPeriod;
  uint32_t timer;
} FLOW_METER;


/**
 * @brief Inicializa o sensor
 * 
 * @param void
 * @return nothing
*/
void SFMInit(void);


/**
 * @brief Inicia a leitura contínua do fluxo de ar
 * 
 * @param void
 * @return status da comunicação com o sensor
*/
uint8_t SFMStartContReadAir(void);


/**
 * @brief Inicia a leitura contínua do fluxo de oxigênio
 * 
 * @param void
 * @return status da comunicação com o sensor
*/
uint8_t SFMStartContReadO2(void);


/**
 * @brief Inicia a leitura contínua do fluxo de uma mistura de ar e oxigênio
 * 
 * @param fração de O2
 * @return status da comunicação com o sensor
*/
uint8_t SFMStartContReadAirO2(uint8_t o2Frac);

/**
 * @brief Define a fração volumétrica de oxigênio
 * 
 * @param fração de O2
 * @return status da comunicação com o sensor
*/
uint8_t SFMSetVolumeFrac(uint8_t o2Frac);

/**
 * @brief Encerra a leitura contínua de fluxo
 * 
 * @param void
 * @return status da comunicação com o sensor
*/
uint8_t SFMStopContRead(void);


/**
 * @brief Executa uma reinicialização de software
 * 
 * @param void
 * @return status da comunicação com o sensor
*/
uint8_t SFMSoftwareReset(void);


/**
 * @brief Lê e configura as configurações do sensor
 * 
 * @param struct de configurações do sensor
 * @return status da comunicação com o sensor
*/
uint8_t SFMReadSettings(METER_SETTINGS *);


/**
 * @brief Lê o fluxo atual do sensor
 * 
 * @param fluxo ponteiro para armazenar o valor do fluxo lido
 * @return status da comunicação com o sensor
*/
uint8_t SFMReadSensor(float *flow);


/**
 * @brief Descarta um pacote de dados inválido
 * 
 * @param void
 * @return status da comunicação com o sensor
*/
uint8_t SFMDiscardPacket(void);


/**
 * @brief
 * 
 * @param
 * @return status da comunicação com o sensor
*/
uint8_t SFMInitStateMachine(void);


/**
 * @brief Cálcula o valor de CRC
 * 
 * @param data,len dados e tamanho para calc. do crc
 * @return crc cálculado
*/
uint8_t SFMCalcCRC(uint8_t *data, uint8_t len);


/**
 * @brief Envia dados para o dispositivo especificado através do barramento I2C
 * 
 * @param endereço, dados e tamanho
 * @return status da comunicação I2C
*/
uint8_t I2C_Write(uint8_t address, uint8_t *data, uint8_t size);


/**
 * @brief Lê dados do dispositivo especificado através do barramento I2C
 * 
 * @param endereço, dados e tamanho
 * @return status da comunicação I2C
*/
uint8_t I2C_Read(uint8_t address, uint8_t *data, uint8_t size);


/**
 * @brief Inverte a ordem dos bytes de um valor de 16 bits
 * 
 * @param val Valor de 16 bits a ser invertido
 * @return valor com os bytes invertidos
*/
uint16_t SwapBytes(uint16_t val);



extern FLOW_METER   FlowMeter;


#endif