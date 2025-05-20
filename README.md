# SIMCAR - Sistema Inteligente de Monitoramento e Controle de Água em Residências

## Descrição
O SIMCAR é uma solução IoT para otimizar o uso da água, detectar vazamentos e promover o reuso de águas cinzas em residências. O sistema utiliza sensores para monitorar o fluxo e a qualidade da água, atuadores para controlar o fluxo e o reuso, e comunicação MQTT para transmitir dados e alertas.

## Funcionalidades
- Monitoramento de fluxo de água em tempo real
- Detecção automática de vazamentos
- Avaliação da qualidade da água para reuso (pH e turbidez)
- Controle automático de válvula para interrupção do fornecimento em caso de vazamento
- Controle de bomba para reuso de águas cinzas quando a qualidade é adequada
- Comunicação via MQTT para monitoramento remoto e alertas

## Como Reproduzir
1. Monte o hardware conforme o diagrama de montagem em `/docs/diagrama_montagem.png`
2. Instale as bibliotecas necessárias listadas em `requirements.txt`
3. Carregue o código `SIMCAR_ESP32.ino` no seu NodeMCU ESP32
4. Configure as credenciais Wi-Fi e o broker MQTT no código
5. Conecte-se ao broker MQTT para monitorar os dados e alertas

## Autor
Elizabeth Diniz dos Santos - Universidade Presbiteriana Mackenzie
