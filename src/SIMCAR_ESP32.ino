// Bibliotecas necessárias
#include <WiFi.h>
#include <PubSubClient.h>      // Para MQTT
#include <Wire.h>              // Para I2C (LCD)
#include <LiquidCrystal_I2C.h> // Para LCD

// Pinos para o Rotary Encoder (Sensor de Fluxo)
#define PINO_ENCODER_CLK 18
#define PINO_ENCODER_DT 19
volatile long contadorPulsosFluxo = 0;
volatile float fluxoAgua = 0.0; // L/min ou L - Tornar volátil pois é modificada na ISR
volatile bool mudancaFluxo = false; // Flag para indicar mudança no fluxo

// Pino para o Potenciômetro (Sensor de PH)
#define PINO_ADC_PH 35
float valorPH = 7.0;

// Pino para o LED (Válvula)
#define PINO_VALVULA 12

// Pino para o Buzzer (Bomba)
#define PINO_BOMBA 13 // Usaremos este pino para o buzzer

// Configuração do LCD I2C (endereço pode variar, 0x27 ou 0x3F são comuns)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variáveis de estado e controle do projeto (conforme PDF)
float turbidez = 20; // NTU - Simulado ou com outro potenciômetro
unsigned long tempoInicioVazamento = 0;
const int TEMPO_MAX_VAZAMENTO_SEGUNDOS = 120; // Pode ser reduzido para teste rápido (ex: 10 ou 20)
const float FLUXO_LIMITE_VAZAMENTO = 10.0; // Reduzido para teste rápido com encoder
unsigned long tempoFechamentoVazamento = 0;
const unsigned long TEMPO_REABERTURA = 60000; 

// Variáveis para o Rotary Encoder
volatile int clkAnteriorISR; // Usar volatile para variáveis de ISR

// Configurações de Wi-Fi
const char* ssid = "Wokwi-GUEST"; // SSID da rede Wi-Fi que o Wokwi simula
const char* password = "";         // Senha da rede Wi-Fi (deixe vazio para Wokwi-GUEST)

// Configurações do Broker MQTT
const char* mqtt_server = "broker.hivemq.com"; // Endereço do broker MQTT
const int mqtt_port = 1883;                   // Porta do broker MQTT
const char* mqtt_client_id = "SIMCAR_ESP32_Client_Wokwi"; // ID único para seu cliente MQTT

// Tópicos MQTT (ajuste conforme seu projeto no artigo)
const char* topic_fluxo = "simcar/residencia01/fluxo";
const char* topic_ph = "simcar/residencia01/ph";
const char* topic_turbidez = "simcar/residencia01/turbidez"; // Adicionado para consistência
const char* topic_alerta_vazamento = "simcar/residencia01/alerta/vazamento";
const char* topic_status_valvula = "simcar/residencia01/status/valvula";
const char* topic_status_bomba = "simcar/residencia01/status/bomba";

WiFiClient espClient;
PubSubClient client(espClient); // Cliente MQTT

unsigned long ultimaPublicacaoMQTT = 0;
const long intervaloPublicacaoMQTT = 5000; // Publica a cada 5 segundos
unsigned long ultimaAtualizacaoLCD = 0;
const long intervaloAtualizacaoLCD = 1000; // Atualiza LCD a cada 1 segundo

// Variáveis para medição de tempo
unsigned long tempoInicioSensorFluxo = 0;
unsigned long tempoFimSensorFluxo = 0;
unsigned long tempoInicioSensorPH = 0;
unsigned long tempoFimSensorPH = 0;
unsigned long tempoInicioAtuadorValvula = 0;
unsigned long tempoFimAtuadorValvula = 0;
unsigned long tempoInicioAtuadorBomba = 0;
unsigned long tempoFimAtuadorBomba = 0;

// Arrays para armazenar as medições
unsigned long medicoesTempoSensorFluxo[4] = {0, 0, 0, 0};
unsigned long medicoesTempoSensorPH[4] = {0, 0, 0, 0};
unsigned long medicoesTempoAtuadorValvula[4] = {0, 0, 0, 0};
unsigned long medicoesTempoAtuadorBomba[4] = {0, 0, 0, 0};

// Contadores para as medições
int contadorMedicaoFluxo = 0;
int contadorMedicaoPH = 0;
int contadorMedicaoValvula = 0;
int contadorMedicaoBomba = 0;

// Função para calcular a média
unsigned long calcularMedia(unsigned long medicoes[4]) {
  unsigned long soma = 0;
  for (int i = 0; i < 4; i++) {
    soma += medicoes[i];
  }
  return soma / 4;
}

void IRAM_ATTR isrEncoder() {
  int clkAtual = digitalRead(PINO_ENCODER_CLK);
  if (clkAtual != clkAnteriorISR && clkAtual == HIGH) { // Detecta borda de subida no CLK
    if (digitalRead(PINO_ENCODER_DT) != clkAtual) { // Sentido horário (aumenta fluxo)
      contadorPulsosFluxo++;
    } else { // Sentido anti-horário (diminui fluxo)
      contadorPulsosFluxo--;
      if (contadorPulsosFluxo < 0) contadorPulsosFluxo = 0;
    }
    fluxoAgua = contadorPulsosFluxo * 0.1; // Mapeamento simples: cada pulso = 0.1 L/min.
    mudancaFluxo = true;
  }
  clkAnteriorISR = clkAtual;
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectando WiFi");

  WiFi.begin(ssid, password);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
    delay(500);
    Serial.print(".");
    lcd.print(".");
    tentativas++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado!");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("WiFi Conectado!");
    lcd.setCursor(0,1);
    lcd.print(WiFi.localIP());
    delay(2000);
  } else {
    Serial.println("\nFalha ao conectar ao WiFi.");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Falha WiFi");
  }
}

void callback_mqtt(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem recebida no tópico: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  String mensagemPayload = "";
  for (unsigned int i = 0; i < length; i++) {
    mensagemPayload += (char)payload[i];
  }
  Serial.println(mensagemPayload);
  
  // Mostra a mensagem no LCD temporariamente
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MQTT Msg Rx:");
  lcd.setCursor(0,1);
  lcd.print(mensagemPayload.substring(0,15));
  delay(2000); // Mostra por 2 segundos
}

void reconnect_mqtt() {
  while (!client.connected() && WiFi.status() == WL_CONNECTED) {
    Serial.print("Tentando conexão MQTT...");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Conect. MQTT...");

    if (client.connect(mqtt_client_id)) {
      Serial.println("conectado ao broker MQTT!");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("MQTT Conectado!");
      delay(1000);
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      lcd.setCursor(0,1);
      lcd.print("Falha MQTT");
      delay(5000);
    }
  }
}

void exibirResultadosMedicoes() {
  if (contadorMedicaoFluxo >= 4 && contadorMedicaoPH >= 4 && 
      contadorMedicaoValvula >= 4 && contadorMedicaoBomba >= 4) {
    
    Serial.println("\n----- RESULTADOS DAS MEDIÇÕES DE TEMPO -----");
    
    Serial.println("\nSensor de Fluxo:");
    for (int i = 0; i < 4; i++) {
      Serial.print("Medição #");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(medicoesTempoSensorFluxo[i]);
      Serial.println(" ms");
    }
    Serial.print("Média: ");
    Serial.print(calcularMedia(medicoesTempoSensorFluxo));
    Serial.println(" ms");
    
    Serial.println("\nSensor de pH:");
    for (int i = 0; i < 4; i++) {
      Serial.print("Medição #");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(medicoesTempoSensorPH[i]);
      Serial.println(" ms");
    }
    Serial.print("Média: ");
    Serial.print(calcularMedia(medicoesTempoSensorPH));
    Serial.println(" ms");
    
    Serial.println("\nAtuador Válvula:");
    for (int i = 0; i < 4; i++) {
      Serial.print("Medição #");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(medicoesTempoAtuadorValvula[i]);
      Serial.println(" ms");
    }
    Serial.print("Média: ");
    Serial.print(calcularMedia(medicoesTempoAtuadorValvula));
    Serial.println(" ms");
    
    Serial.println("\nAtuador Bomba:");
    for (int i = 0; i < 4; i++) {
      Serial.print("Medição #");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(medicoesTempoAtuadorBomba[i]);
      Serial.println(" ms");
    }
    Serial.print("Média: ");
    Serial.print(calcularMedia(medicoesTempoAtuadorBomba));
    Serial.println(" ms");
    
    Serial.println("\n-----------------------------------------");
  }
}

void atualizarLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fluxo: "); lcd.print(fluxoAgua, 1); lcd.print("L/m");
  lcd.setCursor(0, 1);
  lcd.print("pH: "); lcd.print(valorPH, 1);
  if(digitalRead(PINO_VALVULA) == HIGH) { lcd.print(" V:F"); }
  if(digitalRead(PINO_BOMBA) == HIGH) { lcd.print(" B:ON"); }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nIniciando SIMCAR IoT - Simulação Wokwi com MQTT");

  // Inicializa LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("SIMCAR IoT");
  delay(1000);

  setup_wifi(); // Conecta ao Wi-Fi

  // Configura o servidor e porta MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback_mqtt);

  // Configura pinos dos sensores
  pinMode(PINO_ENCODER_CLK, INPUT_PULLUP);
  pinMode(PINO_ENCODER_DT, INPUT_PULLUP);
  clkAnteriorISR = digitalRead(PINO_ENCODER_CLK);
  attachInterrupt(digitalPinToInterrupt(PINO_ENCODER_CLK), isrEncoder, CHANGE);
  Serial.println("Encoder (Fluxo) configurado.");

  Serial.println("Potenciômetro (pH) pronto para leitura.");

  // Configura pinos dos atuadores
  pinMode(PINO_VALVULA, OUTPUT);
  pinMode(PINO_BOMBA, OUTPUT);
  digitalWrite(PINO_VALVULA, LOW); // Válvula inicialmente ABERTA (LED apagado)
  digitalWrite(PINO_BOMBA, LOW);   // Bomba inicialmente DESLIGADA (Buzzer quieto)
  Serial.println("Atuadores (LED/Válvula, Buzzer/Bomba) configurados.");
  Serial.println("--- Interaja com os sensores no Wokwi! ---");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect_mqtt();
    }
    client.loop();
  }

  // 1. Leitura dos Sensores
  if (mudancaFluxo) {
    tempoInicioSensorFluxo = millis();
    Serial.print("[DEBUG Fluxo] Pulsos: "); Serial.print(contadorPulsosFluxo);
    Serial.print(", Fluxo: "); Serial.println(fluxoAgua);
    mudancaFluxo = false;
  }

  tempoInicioSensorPH = millis();
  int leituraADC_PH = analogRead(PINO_ADC_PH);
  valorPH = map(leituraADC_PH, 0, 4095, 0, 140) / 10.0;
  tempoFimSensorPH = millis();
  
  if (contadorMedicaoPH < 4) {
    medicoesTempoSensorPH[contadorMedicaoPH] = tempoFimSensorPH - tempoInicioSensorPH;
    contadorMedicaoPH++;
    Serial.print("Medição de tempo Sensor pH #");
    Serial.print(contadorMedicaoPH);
    Serial.print(": ");
    Serial.print(medicoesTempoSensorPH[contadorMedicaoPH-1]);
    Serial.println(" ms");
  }

  // 2. Lógica de Controle - Vazamento 
bool vazamentoDetectadoEsteLoop = false;
if (fluxoAgua > FLUXO_LIMITE_VAZAMENTO) {
  if (tempoInicioVazamento == 0) {
    tempoInicioVazamento = millis();
    Serial.println("[DEBUG Válvula] Início contagem vazamento.");
  }
  
  unsigned long tempoDecorridoSegundos = (millis() - tempoInicioVazamento) / 1000;
  Serial.print("[DEBUG Válvula] Tempo decorrido fluxo alto: "); 
  Serial.print(tempoDecorridoSegundos); 
  Serial.println("s");

  if (tempoDecorridoSegundos > TEMPO_MAX_VAZAMENTO_SEGUNDOS && digitalRead(PINO_VALVULA) == LOW) {
    Serial.println("!!! ALERTA DE VAZAMENTO - FECHANDO VÁLVULA !!!");
    digitalWrite(PINO_VALVULA, HIGH); // Fecha válvula
    tempoFechamentoVazamento = millis(); // Registra quando fechou
    vazamentoDetectadoEsteLoop = true;
    
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      client.publish(topic_status_valvula, "FECHADA");
      client.publish(topic_alerta_vazamento, "VAZAMENTO_DETECTADO");
    }
  }
} 
else {
  // Se a válvula está fechada por vazamento E já passou 1 minuto OU o fluxo normalizou
  if (digitalRead(PINO_VALVULA) == HIGH && 
      (millis() - tempoFechamentoVazamento > TEMPO_REABERTURA || fluxoAgua <= FLUXO_LIMITE_VAZAMENTO)) {
    
    Serial.println("[DEBUG] Reabrindo válvula após normalização do fluxo");
    digitalWrite(PINO_VALVULA, LOW); // Reabre válvula
    
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      client.publish(topic_status_valvula, "ABERTA");
    }
  }
  tempoInicioVazamento = 0; // Reseta o contador de vazamento
}

  // Controle da Bomba
  bool condicaoBomba = (turbidez < 50 && valorPH >= 6.5 && valorPH <= 7.5);
  tempoInicioAtuadorBomba = millis();
  digitalWrite(PINO_BOMBA, condicaoBomba ? HIGH : LOW);
  tempoFimAtuadorBomba = millis();
  
  if (contadorMedicaoBomba < 4) {
    medicoesTempoAtuadorBomba[contadorMedicaoBomba] = tempoFimAtuadorBomba - tempoInicioAtuadorBomba;
    contadorMedicaoBomba++;
    Serial.print("Medição de tempo Atuador Bomba #");
    Serial.print(contadorMedicaoBomba);
    Serial.print(": ");
    Serial.print(medicoesTempoAtuadorBomba[contadorMedicaoBomba-1]);
    Serial.println(" ms");
  }

  // Atualização do LCD em intervalos regulares
  if (millis() - ultimaAtualizacaoLCD > intervaloAtualizacaoLCD) {
    ultimaAtualizacaoLCD = millis();
    atualizarLCD();
  }

  // 4. Publicação MQTT
  unsigned long agora = millis();
  if (WiFi.status() == WL_CONNECTED && client.connected() && (agora - ultimaPublicacaoMQTT > intervaloPublicacaoMQTT)) {
    ultimaPublicacaoMQTT = agora;
    Serial.println("-- Publicando dados MQTT --");

    tempoInicioSensorFluxo = millis();
    char msg_payload[64];
    snprintf(msg_payload, sizeof(msg_payload), "{\"valor\":%.2f, \"unidade\":\"L/min\"}", fluxoAgua);
    client.publish(topic_fluxo, msg_payload);
    tempoFimSensorFluxo = millis();
    
    if (contadorMedicaoFluxo < 4) {
      medicoesTempoSensorFluxo[contadorMedicaoFluxo] = tempoFimSensorFluxo - tempoInicioSensorFluxo;
      contadorMedicaoFluxo++;
      Serial.print("Medição de tempo Sensor Fluxo #");
      Serial.print(contadorMedicaoFluxo);
      Serial.print(": ");
      Serial.print(medicoesTempoSensorFluxo[contadorMedicaoFluxo-1]);
      Serial.println(" ms");
    }

    Serial.print("Publicado [Fluxo]: "); Serial.println(msg_payload);

    snprintf(msg_payload, sizeof(msg_payload), "{\"valor\":%.2f}", valorPH);
    client.publish(topic_ph, msg_payload);
    Serial.print("Publicado [pH]: "); Serial.println(msg_payload);
    
    snprintf(msg_payload, sizeof(msg_payload), "{\"valor\":%.2f, \"unidade\":\"NTU\"}", turbidez);
    client.publish(topic_turbidez, msg_payload);
    Serial.print("Publicado [Turbidez]: "); Serial.println(msg_payload);

    client.publish(topic_status_valvula, digitalRead(PINO_VALVULA) == HIGH ? "FECHADA" : "ABERTA");
    client.publish(topic_status_bomba, digitalRead(PINO_BOMBA) == HIGH ? "LIGADA" : "DESLIGADA");
  }

  // Exibir resultados quando todas as medições estiverem completas
  exibirResultadosMedicoes();

  delay(100);
}
