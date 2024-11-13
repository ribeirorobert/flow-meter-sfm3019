/**
 * @file SFM3019.cpp
 * @author Robert R. Gomes
 * @brief Driver para leitura de sensor de fluxo SFM3019
 * @version 1.0
 * @date 11-11-2024
 * 
 * @copyright Copyright (c) 2024
 * 
*/

#include "SFM3019.h"


FLOW_METER      FlowMeter;
SFM_WRITE_CMD   cAux;
SFM_WRITE_CMD_2 c2Aux;


uint8_t SFMInitialize(void) {
  
  if (SFMSoftwareReset() != I2C_SUCCESS) {
#if DEBUG_SFM
    Serial.println(F("SW RESET FAIL"));
#endif
    return SFM_ERROR;
  }
  delay(100);


  if (SFMStopMeasurement() != I2C_SUCCESS) {
#if DEBUG_SFM
    Serial.println(F("STOP FAIL"));
#endif
    return SFM_ERROR;
  }
  delay(100);


  if (SFMReadSettings(&FlowMeter.settings) != I2C_SUCCESS) {
#if DEBUG_SFM
    Serial.println(F("READ SETTINGS FAIL"));
#endif
    return SFM_ERROR;
  }
  delay(100);


  return SFM_OK;
}


uint8_t SFMStartContReadAir(void) {
  cAux.cmd = AdjustEndianness(SFM_CONTINUOUS_READ_AIR); 

#if DEBUG_SFM
  Serial.print(F("START [0]:"));
  Serial.print(cAux.u[0], HEX);
  Serial.print(F(" [1]:"));
  Serial.println(cAux.u[1], HEX);
#endif

  return I2C_Write(SFM_I2C_ADDRESS, cAux.u, 2);
}


uint8_t SFMStopMeasurement(void) {
  cAux.cmd = AdjustEndianness(SFM_STOP_CONTINUOUS_READ); 

#if DEBUG_SFM
  Serial.print(F("STOP [0]:"));
  Serial.print(cAux.u[0], HEX);
  Serial.print(F(" [1]:"));
  Serial.println(cAux.u[1], HEX);
#endif

  return I2C_Write(SFM_I2C_ADDRESS, cAux.u, 2);
}


uint8_t SFMSoftwareReset(void) {
  cAux.cmd = AdjustEndianness(SFM_SOFTWARE_RESET); 

#if DEBUG_SFM
  Serial.print(F("RESET [0]:"));
  Serial.print(cAux.u[0], HEX);
  Serial.print(F(" [1]:"));
  Serial.println(cAux.u[1], HEX);
#endif

  return I2C_Write(SFM_I2C_ADDRESS, cAux.u, 1);
}


uint8_t SFMReadSettings(METER_SETTINGS *sfm) {  
  c2Aux.cmd1 = AdjustEndianness(SFM_READ_SCALE_FACTOR_OFFSET);
  c2Aux.cmd2 = AdjustEndianness(SFM_CONTINUOUS_READ_AIR);
  c2Aux.crc  = SFMGenCRC(&c2Aux.u[2], 2);

// #if DEBUG_SFM
//   Serial.println(F("\nWRITTING SETTINGS"));
//   Serial.println(c2Aux.u[0], HEX); //36
//   Serial.println(c2Aux.u[1], HEX); //61
//   Serial.println(c2Aux.u[2], HEX); //36
//   Serial.println(c2Aux.u[3], HEX); //08
//   Serial.println(c2Aux.u[4], HEX); //D0
// #endif

  uint8_t ret = I2C_Write(SFM_I2C_ADDRESS, c2Aux.u, 5);

  if (ret != I2C_SUCCESS) {
#if DEBUG_SFM
    Serial.println(F("\nWRITTING SETTINGS ERROR"));
#endif
    return ret;
  }

  ret = I2C_Read(SFM_I2C_ADDRESS, (uint8_t *)sfm, 9);

  FlowMeter.settings.scaleFactor = AdjustEndianness(sfm->scaleFactor);
  FlowMeter.settings.offset = AdjustEndianness(sfm->offset);
  FlowMeter.settings.flowUnit = AdjustEndianness(sfm->flowUnit);

#if DEBUG_SFM
  Serial.print(F("SCALE FACTOR: "));
  Serial.print(FlowMeter.settings.scaleFactor);
  Serial.print(F(" OFFSET: "));
  Serial.print(FlowMeter.settings.offset);
  Serial.print(F(" FLOW UNIT: "));
  Serial.println(FlowMeter.settings.flowUnit);
#endif 

  return ret;
}


uint8_t SFMSetGasMixture(uint8_t o2Frac = 21) {
  if (o2Frac > 100) o2Frac = 100;
  if (o2Frac < 21) o2Frac = 21;

  c2Aux.cmd1 = AdjustEndianness(SFM_CONTINUOUS_READ_AIR_O2);
  c2Aux.cmd2 = AdjustEndianness((uint16_t)o2Frac);
  c2Aux.crc  = SFMGenCRC(&c2Aux.u[2], 2);

  return I2C_Write(SFM_I2C_ADDRESS, c2Aux.u, 5);
}


uint8_t SFMReadSensor(float *F) {
  FLOW_DATA data;
  data.rawFlow = 0;
  data.crc = 0;

  uint8_t ret = I2C_Read(SFM_I2C_ADDRESS, data.buffer, 3);

  if (ret == I2C_SUCCESS && data.crc != 0) {
    uint8_t crc = SFMGenCRC((uint8_t *)data.buffer, 2);
    if (crc == data.crc) {
      int32_t flowParsed = AdjustEndianness(data.rawFlow);

      *F = (flowParsed - (float)FlowMeter.settings.offset) / (float)FlowMeter.settings.scaleFactor;

      return I2C_SUCCESS;
    }
  }

  return ret;
}


uint8_t SFMGenCRC(uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;

  for (uint8_t i = 0; i < len; i ++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j ++) {
      if ((crc & 0x80) != 0)
        crc = (uint8_t)((crc << 1) ^ 0x31);
      else
        crc <<= 1;
    }
  }

  return crc;
}




uint8_t I2C_Write(uint8_t address, uint8_t *data, uint8_t size) {
  Wire.beginTransmission(address);

  for (uint8_t i = 0; i < size; i ++) {
    Wire.write(data[i]);
  }

  return Wire.endTransmission();
} 


uint8_t I2C_Read(uint8_t address, uint8_t *data, uint8_t size) {
  Wire.requestFrom(address, size);

  if (Wire.available() != size) {
#if DEBUG_SFM
    Serial.println(F("I2C_PACK_ERROR"));
#endif
    return I2C_PACK_ERROR;
  }

  for (uint8_t i = 0; i < size; i ++) data[i] = Wire.read();

  return I2C_SUCCESS;
} 


uint16_t AdjustEndianness(uint16_t Val) {
  return ((Val >> 8) & 0x00FF) | ((Val << 8) & 0xFF00);
}