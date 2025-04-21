#include <Arduino.h>
#include <WiFi.h>

#define LED_BUILTIN 2
#define DELAY_TIME 200

const char *ssid = "ESP_AccessPoint";
const char *password = "12345678";

WiFiServer wifiServer(80);

void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando...");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  WiFi.softAP(ssid, password);
  Serial.println("Punto de acceso configurado.");
  Serial.print("IP del punto de acceso: ");
  Serial.println(WiFi.softAPIP());
  wifiServer.begin();
  Serial.println("Servidor WiFi iniciado.");
}

void loop() {
  WiFiClient client = wifiServer.available();
  if (client) {
    Serial.println("Cliente conectado.");
    String request = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        request += c;
        if (c == '\n') {
          Serial.println("Solicitud recibida:");
          Serial.println(request);
          if (request.indexOf("LED=ON") != -1) {
            digitalWrite(LED_BUILTIN, HIGH);
            Serial.println("LED encendido.");
            client.println("LED encendido.");
          } else if (request.indexOf("LED=OFF") != -1) {
            digitalWrite(LED_BUILTIN, LOW);
            Serial.println("LED apagado.");
            client.println("LED apagado.");
          } else {
            client.println("Comando no reconocido.");
          }
          client.stop();
          Serial.println("Cliente desconectado.");
          break;
        }
      }
    }
  }
}