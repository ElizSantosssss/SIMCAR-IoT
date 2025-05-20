/ Bibliotecas necessárias
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

// Pino para o Potenciômetro (Sensor de pH)
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
const float FLUXO_LIMITE_VAZAMENTO = 1.0; // Reduzido para teste rápido com encoder

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
// const char* topic_comando_valvula = "simcar/residencia01/comando/valvula"; // Exemplo se for receber comandos

WiFiClient espClient;
PubSubClient client(espClient); // Cliente MQTT

unsigned long ultimaPublicacaoMQTT = 0;
const long intervaloPublicacaoMQTT = 5000; // Publica a cada 5 segundos

// --- Interação com Sensores no Wokwi ---
// Rotary Encoder (Fluxo): Clique no corpo do encoder e arraste o mouse para cima/baixo para girá-lo.
// Potenciômetro (pH): Clique no cursor (parte móvel) do potenciômetro e arraste-o para alterar a resistência/valor.

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
    // Prosseguir mesmo sem WiFi para testes locais, mas MQTT não funcionará
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
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MQTT Msg Rx:");
  lcd.setCursor(0,1);
  lcd.print(mensagemPayload.substring(0,15)); // Mostra parte da msg no LCD

  // Exemplo: Lógica para tratar comandos recebidos
  // if (String(topic) == topic_comando_valvula) {
  //   if (mensagemPayload == "FECHAR") {
  //     digitalWrite(PINO_VALVULA, HIGH);
  //     client.publish(topic_status_valvula, "FECHADA");
  //   } else if (mensagemPayload == "ABRIR") {
  //     digitalWrite(PINO_VALVULA, LOW);
  //     client.publish(topic_status_valvula, "ABERTA");
  //   }
  // }
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
      // Publica uma mensagem de status (opcional)
      // client.publish("simcar/residencia01/status/sistema", "ESP32 Online");
      // Se for receber comandos, se inscreva aqui:
      // client.subscribe(topic_comando_valvula);
      // Serial.print("Inscrito no tópico: "); Serial.println(topic_comando_valvula);
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
  // client.setCallback(callback_mqtt); // Descomente se for receber mensagens MQTT

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
    client.loop(); // Essencial para o cliente MQTT
  }

  // 1. Leitura dos Sensores
  if (mudancaFluxo) {
    Serial.print("[DEBUG Fluxo] Pulsos: "); Serial.print(contadorPulsosFluxo);
    Serial.print(", Fluxo: "); Serial.println(fluxoAgua);
    mudancaFluxo = false;
  }

  int leituraADC_PH = analogRead(PINO_ADC_PH);
  valorPH = map(leituraADC_PH, 0, 4095, 0, 140) / 10.0;
  Serial.print("[DEBUG pH] ADC: "); Serial.print(leituraADC_PH);
  Serial.print(", pH: "); Serial.println(valorPH);

  // 2. Lógica de Controle
  bool vazamentoDetectadoEsteLoop = false;
  if (fluxoAgua > FLUXO_LIMITE_VAZAMENTO) {
    if (tempoInicioVazamento == 0) {
      tempoInicioVazamento = millis();
      Serial.println("[DEBUG Válvula] Início contagem vazamento.");
    }
    unsigned long tempoDecorridoSegundos = (millis() - tempoInicioVazamento) / 1000;
    Serial.print("[DEBUG Válvula] Tempo decorrido fluxo alto: "); Serial.print(tempoDecorridoSegundos); Serial.println("s");

    if (tempoDecorridoSegundos > TEMPO_MAX_VAZAMENTO_SEGUNDOS) {
      Serial.println("!!! ALERTA DE VAZAMENTO - FECHANDO VÁLVULA (LED) !!!");
      digitalWrite(PINO_VALVULA, HIGH); // Fecha válvula (LED acende)
      vazamentoDetectadoEsteLoop = true;
    }
  } else {
    if (tempoInicioVazamento != 0) {
        Serial.println("[DEBUG Válvula] Fluxo normalizado, resetando tempo.");
        // Manter válvula fechada se já fechou por vazamento, ou aberta se não.
        // Para reabrir automaticamente, descomente: digitalWrite(PINO_VALVULA, LOW);
    }
    tempoInicioVazamento = 0;
  }

  bool condicaoBomba = (turbidez < 50 && valorPH >= 6.5 && valorPH <= 7.5);
  if (condicaoBomba) {
    Serial.println("ATIVA BOMBA/BUZZER");
    digitalWrite(PINO_BOMBA, HIGH);
  } else {
    digitalWrite(PINO_BOMBA, LOW);
  }

  // 3. Atualização do Display LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fluxo: "); lcd.print(fluxoAgua, 1); lcd.print("L/m");
  lcd.setCursor(0, 1);
  lcd.print("pH: "); lcd.print(valorPH, 1);
  if(digitalRead(PINO_VALVULA) == HIGH) { lcd.print(" V:F");}
  if(digitalRead(PINO_BOMBA) == HIGH) { lcd.print(" B:ON");}

  // 4. Publicação MQTT
  unsigned long agora = millis();
  if (WiFi.status() == WL_CONNECTED && client.connected() && (agora - ultimaPublicacaoMQTT > intervaloPublicacaoMQTT)) {
    ultimaPublicacaoMQTT = agora;
    Serial.println("-- Publicando dados MQTT --");

    char msg_payload[64];
    snprintf(msg_payload, sizeof(msg_payload), "{\"valor\":%.2f, \"unidade\":\"L/min\"}", fluxoAgua);
    client.publish(topic_fluxo, msg_payload);
    Serial.print("Publicado [Fluxo]: "); Serial.println(msg_payload);

    snprintf(msg_payload, sizeof(msg_payload), "{\"valor\":%.2f}", valorPH);
    client.publish(topic_ph, msg_payload);
    Serial.print("Publicado [pH]: "); Serial.println(msg_payload);
    
    snprintf(msg_payload, sizeof(msg_payload), "{\"valor\":%.2f, \"unidade\":\"NTU\"}", turbidez); // Publica turbidez fixa
    client.publish(topic_turbidez, msg_payload);
    Serial.print("Publicado [Turbidez]: "); Serial.println(msg_payload);

    client.publish(topic_status_valvula, digitalRead(PINO_VALVULA) == HIGH ? "FECHADA" : "ABERTA");
    client.publish(topic_status_bomba, digitalRead(PINO_BOMBA) == HIGH ? "LIGADA" : "DESLIGADA");

    if (vazamentoDetectadoEsteLoop) {
        client.publish(topic_alerta_vazamento, "VAZAMENTO_DETECTADO");
        Serial.println("Publicado [Alerta Vazamento]");
    }
  }

  Serial.println("--- STATUS ATUAL ---");
  Serial.print("Valvula (LED pino "); Serial.print(PINO_VALVULA); Serial.print("): ");
  Serial.println(digitalRead(PINO_VALVULA) == HIGH ? "FECHADA (LED ACESO)" : "ABERTA (LED APAGADO)");
  Serial.print("Bomba (Buzzer pino "); Serial.print(PINO_BOMBA); Serial.print("): ");
  Serial.println(digitalRead(PINO_BOMBA) == HIGH ? "LIGADA (BUZZER SOANDO)" : "DESLIGADA (BUZZER QUIETO)");
  Serial.println("----------------------");

  delay(1000); // Reduzido para publicações MQTT mais frequentes se necessário, mas loop principal mais rápido
}

