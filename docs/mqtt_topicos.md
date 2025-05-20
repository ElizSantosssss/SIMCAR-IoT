## Broker MQTT
- **Endereço**: broker.hivemq.com
- **Porta**: 1883 (não criptografada)
- **Autenticação**: Não utilizada (broker público)

## Tópicos de Publicação (ESP32 → Broker)

### Dados dos Sensores
- **simcar/residencia01/fluxo**
  - Formato: JSON `{"valor": X.XX, "unidade": "L/min"}`
  - Frequência: A cada 5 segundos
  - Exemplo: `{"valor": 5.20, "unidade": "L/min"}`

- **simcar/residencia01/ph**
  - Formato: JSON `{"valor": X.XX}`
  - Frequência: A cada 5 segundos
  - Exemplo: `{"valor": 7.20}`

- **simcar/residencia01/turbidez**
  - Formato: JSON `{"valor": XX.XX, "unidade": "NTU"}`
  - Frequência: A cada 5 segundos
  - Exemplo: `{"valor": 20.00, "unidade": "NTU"}`

### Status dos Atuadores
- **simcar/residencia01/status/valvula**
  - Formato: String `"ABERTA"` ou `"FECHADA"`
  - Frequência: A cada 5 segundos e quando o estado muda
  - Exemplo: `"FECHADA"`

- **simcar/residencia01/status/bomba**
  - Formato: String `"LIGADA"` ou `"DESLIGADA"`
  - Frequência: A cada 5 segundos e quando o estado muda
  - Exemplo: `"LIGADA"`

### Alertas
- **simcar/residencia01/alerta/vazamento**
  - Formato: String `"VAZAMENTO_DETECTADO"`
  - Frequência: Imediatamente quando um vazamento é detectado
  - Exemplo: `"VAZAMENTO_DETECTADO"`

## Tópicos de Subscrição (Broker → ESP32)
*(Implementação futura para controle remoto)*

- **simcar/residencia01/comando/valvula**
  - Formato: String `"ABRIR"` ou `"FECHAR"`
  - Função: Controlar remotamente a válvula
  - Exemplo: `"FECHAR"`

- **simcar/residencia01/comando/bomba**
  - Formato: String `"LIGAR"` ou `"DESLIGAR"`
  - Função: Controlar remotamente a bomba
  - Exemplo: `"LIGAR"`
