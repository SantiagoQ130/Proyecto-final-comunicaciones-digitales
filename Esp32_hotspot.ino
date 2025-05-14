#include <WiFi.h>
#include <DNSServer.h>
#include <esp_wifi.h>
#include <tcpip_adapter.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// Configuración de red
const char* ssid = "ESP32-123456";
const char* password = "12345678";
const int maxClientes = 20;
const int canal = 1;
const bool ocultarRed = false;
const IPAddress ip(192, 168, 4, 1);

// DNS cautivo
DNSServer dnsServer;
const byte DNS_PORT = 53;

// Servidor web HTTP
WebServer server(80);

// Variables de monitoreo
unsigned long ultimoTiempo = 0;
const unsigned long intervalo = 5000;  // 5 segundos

// ========= SETUP =========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===========================");
  Serial.println("Iniciando Punto de Acceso WiFi...");

  // Configurar como AP
  WiFi.mode(WIFI_AP);

  // Ajustes avanzados del AP
  wifi_config_t conf;
  esp_wifi_get_config(WIFI_IF_AP, &conf);
  conf.ap.beacon_interval = 100;
  conf.ap.authmode = WIFI_AUTH_WPA2_PSK;
  esp_wifi_set_config(WIFI_IF_AP, &conf);
  esp_wifi_set_max_tx_power(82);

  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));

  if (WiFi.softAP(ssid, password, canal, ocultarRed, maxClientes)) {
    Serial.println("¡AP iniciado correctamente!");
    Serial.print("IP del AP: ");
    Serial.println(WiFi.softAPIP());
    dnsServer.start(DNS_PORT, "*", ip);

    uint8_t mac[6];
    WiFi.softAPmacAddress(mac);
    Serial.printf("MAC del AP: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    Serial.println("ERROR: Fallo al iniciar el AP. Reiniciando...");
    ESP.restart();
  }

  // Configurar servidor web
  server.on("/rssi", HTTP_POST, handleRSSI);
  server.begin();
  Serial.println("Servidor HTTP iniciado en /rssi");
  Serial.println("===========================");
}

// ========= LOOP =========
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  gestionClientes();

  unsigned long tiempoActual = millis();
  if (tiempoActual - ultimoTiempo >= intervalo) {
    ultimoTiempo = tiempoActual;
    mostrarInfoClientes();
  }
}

// ========= Manejo POST RSSI =========
void handleRSSI() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Body no encontrado");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<200> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, body);
  if (error) {
    server.send(400, "application/json", "{\"error\":\"JSON inválido\"}");
    return;
  }

  int id = jsonDoc["id"];
  int rssi = jsonDoc["rssi"];

  Serial.printf("📡 Recibido RSSI -> ID: %d, RSSI: %d dBm\n", id, rssi);

  server.send(200, "application/json", "{\"status\":\"OK\"}");
}

// ========= Cliente =========
void gestionClientes() {
  if (WiFi.getMode() != WIFI_AP && WiFi.getMode() != WIFI_AP_STA) {
    Serial.println("AP desactivado inesperadamente. Reiniciando...");
    reiniciarAP();
  }
}

void mostrarInfoClientes() {
  int numClientes = WiFi.softAPgetStationNum();
  Serial.printf("Clientes conectados: %d\n", numClientes);

  wifi_sta_list_t stationList;
  tcpip_adapter_sta_list_t adapterList;

  if (esp_wifi_ap_get_sta_list(&stationList) == ESP_OK &&
      tcpip_adapter_get_sta_list(&stationList, &adapterList) == ESP_OK) {
    for (int i = 0; i < adapterList.num; i++) {
      tcpip_adapter_sta_info_t station = adapterList.sta[i];
      Serial.printf("  Cliente %d - MAC: %02X:%02X:%02X:%02X:%02X:%02X, IP: %s\n",
        i + 1,
        station.mac[0], station.mac[1], station.mac[2],
        station.mac[3], station.mac[4], station.mac[5],
        IPAddress(station.ip.addr).toString().c_str());
    }
  }

  Serial.printf("Memoria libre: %lu KB\n", ESP.getFreeHeap() / 1024);
  Serial.println("---------------------------");
}

void reiniciarAP() {
  Serial.println("Reiniciando AP...");
  WiFi.softAPdisconnect(true);
  delay(1000);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password, canal, ocultarRed, maxClientes);
}
