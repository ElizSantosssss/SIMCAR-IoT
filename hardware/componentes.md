## Plataforma de Desenvolvimento
- **NodeMCU ESP32 (ESP-WROOM-32)**
  - Processador: Dual-core Xtensa LX6 de 32 bits, 240MHz
  - Memória: 520 KB de SRAM, 4 MB de flash
  - Conectividade: Wi-Fi 802.11 b/g/n, Bluetooth 4.2
  - Alimentação: 5V via USB ou 3.3V diretamente

## Sensores
- **Sensor de Fluxo YF-S201**
  - Faixa de medição: 1-30L/min
  - Precisão: ±10%
  - Saída: 450 pulsos/litro
  - Conexão: Pino digital GPIO18 (CLK) e GPIO19 (DT)

- **Sensor de pH DFRobot SEN0161**
  - Faixa de medição: 0-14 pH
  - Precisão: ±0.1 pH
  - Saída: Analógica
  - Conexão: Pino analógico GPIO35

- **Sensor de Turbidez**
  - Faixa de medição: 0-1000 NTU
  - Saída: Analógica
  - Conexão: Pino analógico (simulado no protótipo)

## Atuadores
- **Válvula Solenoide IV-12V**
  - Tensão de operação: 12V/2A
  - Diâmetro: 1/2"
  - Controle: Via módulo relé conectado ao GPIO12

- **Módulo Relé SRD-05VDC**
  - Tensão de controle: 5V
  - Capacidade de chaveamento: 10A
  - Conexão: Pino digital GPIO12 (válvula) e GPIO13 (bomba)

- **Bomba de Água**
  - Tensão de operação: 12V
  - Controle: Via módulo relé conectado ao GPIO13

## Outros Componentes
- **Display LCD 16x2 I2C**
  - Interface: I2C (SDA: GPIO21, SCL: GPIO22)
  - Endereço I2C: 0x27
  - Função: Exibição de dados de fluxo, pH e status dos atuadores
