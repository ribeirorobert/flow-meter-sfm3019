#include "FLOW_MODEL.h"
#include "MPX5010DP.h"
#include "MPX7007DP.h"
#include "MPX7002DP.h"
#include "SFM3019.h"
#include "ADS1115.h"
#include "PID_CTRL.h"
#include "P_COMP.h"

#define I2C_CLOCK_SPEED   400000UL
#define UART_BAUD_RATE    115200UL

#define BLOWER_PWM_PIN    7

uint8_t enable = 0;
uint8_t print_press = 0;
uint8_t print_flow = 0;
uint8_t print_gains = 0;

float u_c = 0, u_ff = 0.0;

float K_dP = 1.80, dP_DB = 0.5;
float K_dQ = 0.0, dQ_DB = 2.0, Q_DB = 60.0;

float dP_filt = 0.0, dQ_filt = 0.0;
float Q_set = 0.0, Q_meas = 0.0, Q_est = 0;
float P_filt = 0.0, beta = 0.50;

float P_set = 8.0, P_meas = 0.0, P_corr = 0.0, P_est = 0.0, P_comp = 0.0;
float x_est = 0.0, b_est = 0.0;

volatile uint8_t flag_2ms = 0;
uint8_t control_4ms = 0;


uint8_t p_test_enb = 0;
uint8_t u_test_enb = 0;


PidController_Typedef controller;
MPX5010_Typedef pCompSensor;
MPX7007_Typedef airwaySensor;
MPX7002_Typedef flowSensor;
ADS1115_Handle_t ads;

void setup_timer1(void);
void setup_pwm(void);
void read_cmds(void);
void flow_derivative(void);
void predictive_exhalation(void);
void blower_set_speed(uint8_t);
void monitor_serial(void);
float filter_3th(float input);
void pCompTest(void);
void unitStepTest(void);

ISR(TIMER1_COMPA_vect) {
  flag_2ms = 1;
}

void setup() {
  Serial.begin(UART_BAUD_RATE);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  pinMode(BLOWER_PWM_PIN, OUTPUT);

  setup_timer1();
  setup_pwm();
  blower_set_speed(0);

  delay(1000);

  SFMInit();
  ADS1115_Init(&ads, ADS1115_ADDR);
  MPX5010_init(&pCompSensor, MPX5010_PIN);
  MPX7007_init(&airwaySensor, A4);
  MPX7002_init(&flowSensor, A5);
  //MPX7002_init(&flowSensor, MPX7002_CHANNEL_1, &ads);

  PidControllerInit(&controller);
  PidControllerSetLimits(&controller, 0.0f, 255.0f);
  //PidControllerSetGains(&controller, 28.0f, 76.0f, 0.0f, 0.001f);
  PidControllerSetGains(&controller, 39.0f, 10.0f, 0.0f, 0.001f);
  //PidControllerSetGains(&controller, 1.0f, 0.0f, 0.0f, 0.001f);
}

void loop() {
  
  if (flag_2ms) {
    flag_2ms = 0;

    read_cmds();
    
    ADS1115_Update(&ads);
    MPX5010_update_pressure(&pCompSensor);
    MPX7007_update_pressure(&airwaySensor);
    MPX7002_update_pressure(&flowSensor);
    //MPX7002_update_pressure(&flowSensor, &ads);
    SFMUpdateFlow();

    control_4ms ++;
    if (control_4ms >= 2) {
      control_4ms = 0;

      P_meas = pCompSensor.filtPressure;
      P_comp = pCompSensor.filtPressure;
      P_corr = get_internal_ref(P_set);

      Q_meas = (0.8f * Q_meas) + (0.2f * FlowMeter.rawFlow * FLOW_FACTOR);
      Q_est  = flowFromDP(flowSensor.filtPressure);

      /** 
       * Ppaciente = 0.5640f * Pinterno + 0.0354f
       * teste em que o PWM vai de 20 a 250. Colotei Q_meas, Pinterno, Ppaciente
       * 
       * x_est = -0.005259f * Q_meas + 0.560285f;
       * teste em x_est foi ajustado para P_est convergir com o Ppaciente, restringindo o fluxo de saída de max. até min.
       */

      // Serial.print(Q_est);
      // Serial.print('\t');
      // Serial.print(P_comp);
      // Serial.print('\t');
      // Serial.print(P_est);
      // Serial.print('\t');
      // Serial.print(x_est);
      // Serial.print('\n');

      //x_est = -0.005259f * Q_est + 0.560285f;
      x_est = -0.00001935*Q_est*Q_est + -0.002422*Q_est + 0.479042;
      P_est = 0.5640f * P_meas + 0.0354f + (x_est * P_meas);

      flow_derivative();

      PidControllerUpdate(&controller, P_set, P_meas);
      //PidControllerUpdate(&controller, Q_set, Q_meas);
      u_c = u_ff + controller.output;
    
      predictive_exhalation();

      u_c = constrain(u_c, 20, 255);
      blower_set_speed(enable ? u_c : 0);
      monitor_serial();
    }

    // pCompTest();
    // unitStepTest();
  }

  /*
  uint32_t now = micros();
  static uint32_t timer_1 = 0;

  if (now - timer_1 >= 10000) {
    timer_1 = now;

    read_cmds();

    float P_meas = pCompSensor.filtPressure;
    float Q_meas = FlowMeter.rawFlow;

    static float lastPressure = 0.0f, lastFlow = 0.0f;
    static uint32_t lastTime = 0.0f;
    const float alpha = 0.2f;

    float dt = (now - lastTime) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f; // proteção

    float dP = (P_meas - lastPressure) / dt; // taxa de subida
    float dQ = (Q_meas - lastFlow) / dt; // taxa de subida
    
    dP_filt = alpha * dP + (1.0f - alpha) * dP_filt;  // derivada filtrada
    dQ_filt = alpha * dQ + (1.0f - alpha) * dQ_filt;
    // dP_filt = dP;
    // dQ_filt = dQ;

    lastTime     = now;
    lastPressure = P_meas;
    lastFlow     = Q_meas;

    float error = P_set - P_meas;
    pressure_control(error);
    u_c = u_ff + u_pid;

    if (P_meas > P_set + 0.5f) {
      if (dP_filt > dP_DB) {
        u_c -= K_dP * (dP_filt - dP_DB);
        flag_dP = 1;
      } else {
        flag_dP = 0;
      }

      if (Q_meas < Q_DB && dQ_filt < -dQ_DB) {
        u_c -= K_dQ * (-dQ_filt - dQ_DB);
        flag_dQ = 1;
      } else {
        flag_dQ = 0;
      }
    }

    u_c = constrain(u_c, 20, 255);
    blower_set_speed(enable ? u_c : 0);
    monitor_serial();
  }  
  */

}

void flow_derivative(void) {
  static float lastPressure = 0.0f, lastFlow = 0.0f;
  const float alpha = 0.2f;

  float dP = (P_meas - lastPressure) / 0.004f;
  float dQ = (Q_meas - lastFlow) / 0.004f;
  
  dP_filt = alpha * dP + (1.0f - alpha) * dP_filt;
  dQ_filt = alpha * dQ + (1.0f - alpha) * dQ_filt;

  lastPressure = P_meas;
  lastFlow     = Q_meas;
}

void predictive_exhalation(void) {
  if (P_meas > P_corr + 0.5f) {
    if (dP_filt > dP_DB) {
      u_c -= K_dP * (dP_filt - dP_DB);
    }

    if (Q_meas < Q_DB && dQ_filt < -dQ_DB) {
      u_c -= K_dQ * (-dQ_filt - dQ_DB);
    }
  } 
  // else if (P_meas < P_corr - 0.5f && dP_filt < -dP_DB) {
  //   u_c += 1.2 * (-dP_filt - dP_DB);
  // }
} 

void blower_set_speed(uint8_t pwmValue) {
  if (pwmValue < 0) pwmValue = 0;
  if (pwmValue > 255) pwmValue = 255;

  OCR4B = pwmValue;
}

void read_cmds(void) {
  static unsigned long _timer = 0;
  if (millis() - _timer < 100UL) return;
  _timer = millis();

  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '0') enable = !enable;

    if (c == 'c') {
      MPX5010_set_offset(&pCompSensor,  pCompSensor.filtPressure);
      MPX7007_set_offset(&airwaySensor, airwaySensor.filtPressure);
      MPX7002_set_offset(&flowSensor,   flowSensor.filtPressure);
    }

    if (c == 'i') print_flow = !print_flow;
    if (c == 'o') print_press = !print_press;
    if (c == 'p') print_gains = !print_gains;

    if (c == '1') P_set += 0.5;
    if (c == '2') P_set -= 0.5;
    if (c == '3') Q_set += 5;
    if (c == '4') Q_set -= 5;

    if (c == 'q') controller.kp += 1;
    if (c == 'w') controller.kp -= 1;
    if (c == 'a') controller.ki += 1;
    if (c == 's') controller.ki -= 1;
    if (c == 'z') controller.kd += 0.05;
    if (c == 'x') controller.kd -= 0.05;

    if (c == 'e') K_dP += 0.1;
    if (c == 'r') K_dP -= 0.1;
    if (c == 'd') dP_DB += 0.1;
    if (c == 'f') dP_DB -= 0.1;

    if (c == 't') K_dQ += 1.0;
    if (c == 'y') K_dQ -= 1.0;
    if (c == 'g') dQ_DB += 0.1;
    if (c == 'h') dQ_DB -= 0.1;
    if (c == 'c') Q_DB += 1.0;
    if (c == 'v') Q_DB -= 1.0;

    if (c == 'u') {
      p_test_enb = !p_test_enb;
      u_test_enb = !u_test_enb;
    }

    if (c == 'k') x_est += 0.01;
    if (c == 'l') x_est -= 0.01;

    if (c == 'n') b_est += 0.1;
    if (c == 'm') b_est -= 0.1;

    // if (c == 'n') u_ff += 5;
    // if (c == 'm') u_ff -= 5;

    PidControllerSetGains(&controller, controller.kp, controller.ki, controller.kd, 0.001f);
  }

}

void monitor_serial(void) {

  if (print_flow) {
    Serial.print(Q_set);
    Serial.print('\t');
    Serial.print(Q_meas);
    Serial.print('\t');
    Serial.print(Q_est);
    Serial.print('\t');
  }

  if (print_press) {
    Serial.print(P_set);
    Serial.print('\t');
    Serial.print(P_meas);
    Serial.print('\t');
    Serial.print(P_est);
    Serial.print('\t');
    Serial.print(P_comp);
    Serial.print('\t');
  }

  if (print_gains) {
    Serial.print(P_set-P_meas);
    Serial.print('\t');
    Serial.print(controller.kp);
    Serial.print('\t');
    Serial.print(controller.ki);
    Serial.print('\t');
    Serial.print(controller.kd);
    Serial.print('\t');
    Serial.print(u_c);
    Serial.print('\t');
    Serial.print(controller.pTerm);
    Serial.print('\t');
    Serial.print(controller.iTerm);
    Serial.print('\t');
    Serial.print(controller.dTerm);
    Serial.print('\t');
  }

  Serial.print('\n');
}

void setup_timer1(void) {
  cli(); // desabilita interrupções

  TCCR1A = 0;
  TCCR1B = 0;

  // modo CTC
  TCCR1B |= (1 << WGM12);

  // prescaler = 64
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // 16MHz / 64 = 250kHz → 1 tick = 4us
  // 2 ms / 4 us = 500 ticks
  OCR1A = 499;

  // habilita interrupção
  TIMSK1 |= (1 << OCIE1A);

  sei(); // habilita interrupções
}

void setup_pwm(void) {
  cli(); // desabilita interrupções

  TCCR4A = (1 << WGM40) | (1 << COM4B1);
  TCCR4B = (1 << WGM42) | (1 << CS41); // prescaler = 8

  sei(); // habilita interrupções
}

void pCompTest(void) {
  static uint8_t p_state = 0;
  static uint8_t p_pwm = 250;
  static uint32_t p_timer = 0;

  if (p_test_enb) {
    switch (p_state) {
      case 0: {
        p_timer = millis();
        p_state = 1;
      } 
      break;

      case 1: {
        if (millis() - p_timer >= 2000UL) {
          p_timer = millis();
          p_state = 2;
        }
      } 
      break;

      case 2: {
        p_pwm -= 10;

        Serial.print(Q_est);
        Serial.print('\t');
        Serial.print(P_meas);
        Serial.print('\t');
        Serial.print(P_comp);
        Serial.print('\n');

        if (p_pwm <= 0) {
          p_state = 3;
        } else {
          p_state = 0;
        }
      }
      break;

      case 3: {
        p_pwm = 0;
        p_test_enb = 0;
        p_state = 0;
      }
      break;
    }

    blower_set_speed(p_pwm);
  } else {
    p_pwm = 250;
  }
}

void unitStepTest(void) {
  static uint8_t u_state = 0;
  static uint8_t u_pwm = 0;
  static uint32_t u_timer = 0;

  if (u_test_enb) {
    switch (u_state) {
      case 0: {
        u_pwm = 0;
        u_state = 1;
        u_timer = millis();
      }
      break;

      case 1: {
        Serial.print(millis());
        Serial.print('\t');
        Serial.print(u_pwm);
        Serial.print('\t');
        Serial.print(P_meas);
        Serial.print('\t');
        Serial.print(P_comp);
        Serial.print('\n');

        if (millis() - u_timer >= 500) {
          u_pwm = 200;
          u_state = 2;
          u_timer = millis();
        }
      }
      break;

      case 2: {
        Serial.print(millis());
        Serial.print('\t');
        Serial.print(u_pwm);
        Serial.print('\t');
        Serial.print(P_meas);
        Serial.print('\t');
        Serial.print(P_comp);
        Serial.print('\n');

        if (millis() - u_timer >= 2000) {
          u_pwm = 0;
          u_state = 0;
          u_test_enb = 0;
        } 
      }
      break;
    }

    blower_set_speed(u_pwm);
  }

  
}

#define GAIN   4.553605266e+03
static float xv[4], yv[4];

float filter_3th(float input) {
  xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; 
  xv[3] = input / GAIN;
  yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; 
  yv[3] =   (xv[0] + xv[3]) + 3 * (xv[1] + xv[2])
                + (  0.7776385602 * yv[0]) + ( -2.5282312191 * yv[1])
                + (  2.7488358092 * yv[2]);
  return yv[3];
}