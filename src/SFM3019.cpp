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


void SFMInit(void) {
  FlowMeter.state = RESET;

  SFMInitStateMachine();
}


uint8_t SFMStartContReadAir(void) {
  cAux.cmd = SwapBytes(SFM_CONT_READ_AIR); 

  return I2C_Write(SFM_I2C_ADDRESS, cAux.u, 2);
}


uint8_t SFMStartContReadO2(void) {
  cAux.cmd = SwapBytes(SFM_CONT_READ_O2); 

  return I2C_Write(SFM_I2C_ADDRESS, cAux.u, 2);
}


uint8_t SFMStartContReadAirO2(uint8_t o2Frac) {
  if (o2Frac > 100) o2Frac = 100;
  if (o2Frac < 0) o2Frac = 0;

  c2Aux.cmd1 = SwapBytes(SFM_CONT_READ_AIR_O2);
  c2Aux.cmd2 = SwapBytes((uint16_t)o2Frac);
  c2Aux.crc  = SFMCalcCRC(&c2Aux.u[2], 2);

  return I2C_Write(SFM_I2C_ADDRESS, c2Aux.u, 5);
}


uint8_t SFMSetVolumeFrac(uint8_t o2Frac) {
  if (o2Frac > 100) o2Frac = 100;
  if (o2Frac < 0) o2Frac = 0;

  c2Aux.cmd1 = SwapBytes(SFM_CHANGE_O2_FRAC);
  c2Aux.cmd2 = SwapBytes((uint16_t)o2Frac);
  c2Aux.crc  = SFMCalcCRC(&c2Aux.u[2], 2);

  return I2C_Write(SFM_I2C_ADDRESS, c2Aux.u, 5);
}


uint8_t SFMStopContRead(void) {
  cAux.cmd = SwapBytes(SFM_STOP_CONT_READ); 

  return I2C_Write(SFM_I2C_ADDRESS, cAux.u, 2);
}


uint8_t SFMSoftwareReset(void) {
  cAux.cmd = SwapBytes(SFM_SOFT_RESET); 

  return I2C_Write(SFM_I2C_ADDRESS, cAux.u, 1);
}


uint8_t SFMReadSettings(METER_SETTINGS *sfm) {  
  c2Aux.cmd1 = SwapBytes(SFM_READ_SCALE_OFFSET);
  c2Aux.cmd2 = SwapBytes(SFM_CONT_READ_AIR);
  c2Aux.crc  = SFMCalcCRC(&c2Aux.u[2], 2);

  uint8_t ret = I2C_Write(SFM_I2C_ADDRESS, c2Aux.u, 5);

  if (ret != I2C_SUCCESS) {
#if DEBUG_SFM
    Serial.println(F("WRITTING SETTINGS ERROR"));
#endif
    return ret;
  }

  ret = I2C_Read(SFM_I2C_ADDRESS, (uint8_t *)sfm, 9);

  FlowMeter.settings.scaleFactor = SwapBytes(sfm->scaleFactor);
  FlowMeter.settings.offset = SwapBytes(sfm->offset);
  FlowMeter.settings.flowUnit = SwapBytes(sfm->flowUnit);

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


uint8_t SFMReadSensor(float *F) {
  if (millis() - FlowMeter.readPeriod >= READ_PERIOD_MS) {
    FlowMeter.readPeriod = millis();

    if (FlowMeter.state == INITIALIZED) {
      FLOW_DATA data = {0};

      if (I2C_Read(SFM_I2C_ADDRESS, data.buffer, 3) == I2C_SUCCESS) {
        if (data.crc != 0) {
          uint8_t crc = SFMCalcCRC((uint8_t *)data.buffer, 2);
          if (crc == data.crc) {
            int32_t flowParsed = SwapBytes(data.rawFlow);
            *F = (flowParsed - (float)FlowMeter.settings.offset) / (float)FlowMeter.settings.scaleFactor;
          }
        }
      } else {
        FlowMeter.errorCounter ++;
      }
    }
  }

  return SFMInitStateMachine();
}


uint8_t SFMDiscardPacket(void) {
  FLOW_DATA data = {0};

  uint8_t ret = I2C_Read(SFM_I2C_ADDRESS, data.buffer, 3);
  if (ret == I2C_SUCCESS && data.crc != 0) {
    uint8_t crc = SFMCalcCRC((uint8_t *)data.buffer, 2);
    if (crc == data.crc) return I2C_SUCCESS;
  }

  return ret;
}


uint8_t SFMInitStateMachine(void) {
  switch (FlowMeter.state) {
    case RESET: {
      if (SFMSoftwareReset() == I2C_SUCCESS) {
        FlowMeter.errorCounter = 0;
        FlowMeter.timer = millis();
        FlowMeter.state = RESETING;
#if DEBUG_SFM
        Serial.println(F("SW RESET OK"));
#endif
      } else {
        FlowMeter.errorCounter ++;
#if DEBUG_SFM
        Serial.println(F("SW RESET FAIL"));
#endif
      }
    }
    break;

    case RESETING: {
      if (millis() - FlowMeter.timer >= RESET_TIMER_MS) {
        FlowMeter.state = STOP;
      }
    }
    break;

    case STOP: {
      if (SFMStopContRead() == I2C_SUCCESS) {
        FlowMeter.errorCounter = 0;
        FlowMeter.timer = millis();
        FlowMeter.state = STOPPING;
#if DEBUG_SFM
        Serial.println(F("STOP OK"));
#endif
      } else {
        FlowMeter.errorCounter ++;
#if DEBUG_SFM
        Serial.println(F("STOP FAIL"));
#endif
      }
    }
    break;

    case STOPPING: {
      if (millis() - FlowMeter.timer >= START_TIMER_MS) {
        FlowMeter.state = READ_SETTINGS;
      }
    }
    break;

    case READ_SETTINGS: {
      if (SFMReadSettings(&FlowMeter.settings) == I2C_SUCCESS) {
        FlowMeter.errorCounter = 0;
        FlowMeter.timer = millis();
        FlowMeter.state = START;
#if DEBUG_SFM
        Serial.println(F("READ SETTINGS OK"));
#endif
      } else {
        FlowMeter.errorCounter ++;
#if DEBUG_SFM
        Serial.println(F("READ SETTINGS FAIL"));
#endif
      }
    }
    break;

    case START: {
      if (SFMStartContReadAir() == I2C_SUCCESS) { 
        FlowMeter.errorCounter = 0;
        FlowMeter.timer = millis();
        FlowMeter.state = STARTING;
#if DEBUG_SFM
        Serial.println(F("CONT READ AIR OK"));
#endif
      } else {
        FlowMeter.errorCounter ++;
#if DEBUG_SFM
        Serial.println(F("CONT READ AIR FAIL"));
#endif
      }
    }
    break;

    case STARTING: {
      if (millis() - FlowMeter.timer >= START_TIMER_MS) {
        FlowMeter.state = DISCARD_PACKET;
      }
    }
    break;

    case DISCARD_PACKET: {
      if (SFMDiscardPacket() == I2C_SUCCESS) {
        FlowMeter.errorCounter = 0;
      } else {
        FlowMeter.errorCounter ++;
      }

      FlowMeter.timer = millis();
      FlowMeter.state = NEW_PACKET;
    }
    break;

    case NEW_PACKET: {
      if (millis() - FlowMeter.timer >= PACKET_TIMER_MS) {
        FlowMeter.errorCounter = 0;
        FlowMeter.timer = millis();
        FlowMeter.state = INITIALIZED;
      }
    }
    break;

    case INITIALIZED: {

    }
    break;

    case NOT_INITIALIZED: {

    }
    break;

    default: {
      FlowMeter.state = RESET;
    }
    break;
  }

  if (FlowMeter.errorCounter >= MAX_ALLOWED_ERRORS) {
    FlowMeter.state = RESET;
  }

  return FlowMeter.state; 
}


uint8_t SFMCalcCRC(uint8_t *data, uint8_t len) {
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


uint16_t SwapBytes(uint16_t Val) {
  return ((Val >> 8) & 0x00FF) | ((Val << 8) & 0xFF00);
}