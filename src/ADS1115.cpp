// ADS1115.c completo e corrigido

#include "ADS1115.h"
#include <Wire.h>

#define REG_CONVERSION  0x00
#define REG_CONFIG      0x01

static void writeRegister(uint8_t addr, uint8_t reg, uint16_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

static int16_t readConversion(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(REG_CONVERSION);
    Wire.endTransmission();
    Wire.requestFrom(addr, (uint8_t)2);
    int16_t result = (int16_t)((Wire.read() << 8) | Wire.read());
    return result;
}

static uint16_t buildConfig(ADS1115_Channel_t ch) {
    uint16_t mux;
    switch (ch) {
        case ADS1115_CH0: mux = (0x4 << 12); break; // AIN0 vs GND
        case ADS1115_CH1: mux = (0x5 << 12); break; // AIN1 vs GND
        default:          mux = (0x4 << 12); break;
    }

    return (1     << 15) |  // OS: inicia conversão
           mux            |  // MUX
           (0x1   <<  9)  |  // PGA: ±4.096V 1bit = 0.125mV
           (1     <<  8)  |  // MODE: single-shot
           (0x7   <<  5)  |  // DR: 860 SPS
           (0x3   <<  0);    // COMP_QUE: desabilita comparador
}

static ADS1115_Channel_t nextChannel(ADS1115_Channel_t ch) {
    return (ch == ADS1115_CH0) ? ADS1115_CH1 : ADS1115_CH0;
}

void ADS1115_Init(ADS1115_Handle_t *dev, uint8_t addr) {
    dev->address         = addr;
    dev->current_channel = ADS1115_CH0;
    dev->ch0             = 0;
    dev->ch1             = 0;

    writeRegister(dev->address, REG_CONFIG, buildConfig(ADS1115_CH0));
}

void ADS1115_Update(ADS1115_Handle_t *dev) {
    // Lê resultado da conversão disparada no ciclo anterior
    int16_t value = readConversion(dev->address);

    if (dev->current_channel == ADS1115_CH0)
        dev->ch0 = value;
    else
        dev->ch1 = value;

    // Avança canal e dispara próxima conversão
    dev->current_channel = nextChannel(dev->current_channel);
    writeRegister(dev->address, REG_CONFIG, buildConfig(dev->current_channel));
}