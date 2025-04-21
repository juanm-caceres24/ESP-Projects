#include <Arduino.h>
#include <WiFi.h>

const char *ssid = "ESP_AccessPoint";
const char *password = "12345678";
const char *serverIP = "192.168.4.1";
const int serverPort = 80;

WiFiClient client;

void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando cliente...");
  WiFi.begin(ssid, password);
  Serial.print("Conectando al punto de acceso");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConexión establecida.");
  Serial.print("IP del cliente: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (client.connect(serverIP, serverPort)) {
      Serial.println("Conectado al servidor.");
      Serial.print("Enviando comando: ");
      Serial.println(command);
      client.println(command);
      while (client.connected() || client.available()) {
        if (client.available()) {
          String response = client.readStringUntil('\n');
          Serial.print("Respuesta del servidor: ");
          Serial.println(response);
        }
      }
      client.stop();
      Serial.println("Desconectado del servidor.");
    } else {
      Serial.println("No se pudo conectar al servidor.");
    }
  }
}
