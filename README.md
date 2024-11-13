# Projeto de Interface com o Sensor de Fluxo SFM3019

Este projeto implementa uma interface para comunicação e controle do sensor de fluxo **SFM3019** via protocolo **I2C**. O sensor SFM3019 é projetado para medir o fluxo de ar e de oxigênio com alta precisão, sendo amplamente utilizado em aplicações médicas e industriais, como ventiladores mecânicos e sistemas de controle de fluxo de gás.

## Funcionalidades Principais

1. **Inicialização e Configuração**: Inclui funções para configurar o sensor e definir parâmetros como a fração volumétrica de oxigênio, essencial para medições em misturas de ar e oxigênio.

2. **Leitura Contínua de Fluxo**:
   - **Leitura de Fluxo de Ar**: Permite o monitoramento constante do fluxo de ar.
   - **Leitura de Fluxo de Oxigênio**: Suporta leituras contínuas do fluxo de oxigênio.
   - **Leitura de Mistura de Ar e Oxigênio**: Permite medir o fluxo de uma mistura com uma fração específica de oxigênio, configurada pelo usuário.

3. **Comandos de Controle**:
   - **Iniciar e Parar Leituras Contínuas**: Permite ativar e desativar a leitura contínua do sensor.
   - **Reinicialização por Software**: Força uma reinicialização do sensor para resolver erros de comunicação.
   - **Leitura de Configurações**: Função para consultar e aplicar configurações internas do sensor, incluindo fatores de escala e offset.

4. **Verificação de Erros**:
   - **Detecção de Erros de Conexão**: Detecta problemas de comunicação com o sensor e implementa um contador de tentativas para reconectar automaticamente.
   - **Verificação de CRC**: Implementa a verificação de CRC para garantir a integridade dos dados lidos do sensor.
   - **Controle de Tentativas de Reconexão**: Após um número predefinido de erros consecutivos, o sistema reinicializa o sensor para tentar resolver possíveis falhas.

## Estrutura de Código

O projeto está organizado de maneira modular para facilitar a leitura e manutenção:

- **Macros e Constantes**: Definem endereços I2C, comandos específicos do sensor e valores de tempo para controles temporais.
- **Funções de Controle do Sensor**: Funções para inicialização, controle de leitura contínua, configuração de frações de oxigênio e interrupção de leituras.
- **Funções de Comunicação I2C**: Implementam a comunicação I2C para leitura e escrita de dados.
- **Funções de Verificação de Dados**: Incluem verificação de CRC e funções auxiliares para interpretar os dados do sensor.

## Exemplo de Uso

Para utilizar o projeto, é necessário configurar o endereço I2C e os comandos de leitura apropriados. Exemplo de inicialização e leitura contínua do fluxo de ar:

```cpp
#include <Arduino.h>
#include "SFM3019.h"

void setup() {
  Serial.begin(38400);
  Wire.begin();
  delay(1000);

  SFMInit();
}

void loop() {
  if (SFMReadSensor(&FlowMeter.rawFlow) == INITIALIZED) {
    Serial.println(FlowMeter.rawFlow);
  }
}
