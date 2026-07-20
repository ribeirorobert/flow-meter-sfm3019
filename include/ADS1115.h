#ifndef ADS1115_H_
#define ADS1115_H_

#include <Arduino.h>

#define ADS1115_ADDR    0x48
#define ADS1115_PGA     0.000125f  //PGA: ±4.096V 1bit = 0.125mV

typedef enum {
    ADS1115_CH0 = 0,
    ADS1115_CH1
} ADS1115_Channel_t;

typedef struct {
    uint8_t address;
    ADS1115_Channel_t current_channel;

    int16_t ch0;
    int16_t ch1;
} ADS1115_Handle_t;

void ADS1115_Init(ADS1115_Handle_t *dev, uint8_t addr);
void ADS1115_Update(ADS1115_Handle_t *dev);

#endif