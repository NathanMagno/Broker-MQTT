#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// Definições de pinos
const int SERVO_PIN = 27;
const int BUZZER_PIN = 13;
const int LIGHT_PIN = 15;
const int LED_R = 21;
const int LED_G = 22;
const int LED_B = 23;


// Identificadores
const char* ID        = "nathan_mosquitto";
const char* moduleID  = "nathan_mosquitto123";

// Wi-Fi
const char* SSID      = "Wokwi-GUEST";
const char* PASSWORD  = "";

// MQTT Broker
const char* BROKER_MQTT  = "74.235.56.231";
const int   BROKER_PORT  = 1883;
const char* mqttUser     = "cleyton";
const char* mqttPassword = "mosquitto_passwd";

// Tópico MQTT
#define TOPICO_PUBLISH  "clima"

// MQTT
const char* mqtt_server = "broker.hivemq.com";  // ou IP da sua VM na Azure
const char* mqtt_topic = "fiap/iot/sensores";   // tópico correto aqui

WiFiClient espClient;
PubSubClient client(espClient);

Servo myservo;

void setup_wifi() {
  Serial.print("Conectando ao WiFi");
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado!");
}

void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Conectando ao MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println(" conectado!");
    } else {
      Serial.print(" falhou, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Setup Wi-Fi e MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // Pinos dos atuadores
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  myservo.attach(SERVO_PIN, 500, 2400);

  Serial.println("Demo de atuadores e sensores iniciada!");
}

void loop() {
  // Reconectar MQTT se necessário
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  // === Simular dados de sensores ===
  float temperatura = random(200, 350) / 10.0;  // 20°C a 35°C
  float umidade = random(400, 800) / 10.0;      // 40% a 80%
  float pressao = random(9800, 10500) / 10.0;   // 980 a 1050 hPa
  float altitude = random(0, 500);              // 0 a 500 m

  String payload = "{";
  payload += "\"temperatura\":" + String(temperatura, 1) + ",";
  payload += "\"umidade\":" + String(umidade, 1) + ",";
  payload += "\"pressao\":" + String(pressao, 1) + ",";
  payload += "\"altitude\":" + String(altitude, 0);
  payload += "}";

  Serial.println("Publicando dados MQTT:");
  Serial.println(payload);
  client.publish(mqtt_topic, payload.c_str());

  // === Ciclo de testes com atuadores ===

  // 1. Buzzer
  tone(BUZZER_PIN, 1000);
  delay(1000);
  noTone(BUZZER_PIN);
  delay(500);

  // 2. Luz
  digitalWrite(LIGHT_PIN, HIGH);
  delay(1000);
  digitalWrite(LIGHT_PIN, LOW);
  delay(500);

  // 3. LEDs RGB
  digitalWrite(LED_R, HIGH); digitalWrite(LED_G, LOW); digitalWrite(LED_B, LOW); delay(500);
  digitalWrite(LED_R, LOW);  digitalWrite(LED_G, HIGH);                          delay(500);
  digitalWrite(LED_G, LOW);  digitalWrite(LED_B, HIGH);                          delay(500);
  digitalWrite(LED_B, LOW);                                                     delay(500);

  // 4. Servo
  myservo.write(0); delay(1000);
  myservo.write(90); delay(1000);
  myservo.write(180); delay(1000);

  delay(3000); // aguarda antes do próximo envio
}
