/*
 * Código combinado mejorado para ESP32 con sensor MPU6050 y Punto de Acceso WiFi
 * Incluye calibración, filtrado mejorado y manejo de deriva
 * Crea un punto de acceso WiFi para compartir los datos y recibir RSSI
 */

#include <WiFi.h>
#include <DNSServer.h>
#include <esp_wifi.h>
#include <tcpip_adapter.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

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

// Variables de monitoreo WiFi
unsigned long ultimoTiempoWiFi = 0;
const unsigned long intervaloWiFi = 5000;  // 5 segundos

// Declaración del objeto MPU6050
MPU6050 mpu;

// Variables para almacenar los datos del sensor
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Variables para mediciones
float accelX, accelY;               // Aceleración en m/s²
float velocX = 0, velocY = 0;        // Velocidad en m/s
float offsetX = 0, offsetY = 0;      // Offsets de calibración
bool calibrated = false;             // Bandera de calibración

// Constantes de configuración para el MPU6050
const float accelFactor = 9.81 / 16384.0; // Conversión a m/s² (±2g)
const int CALIBRATION_SAMPLES = 500;      // Muestras para calibración
const float noiseThreshold = 0.05;        // Umbral de ruido en m/s²
const float decayFactor = 0.9;            // Factor de decaimiento
const int sampleRate = 50;                // Tiempo entre muestras (ms)

// Variables de tiempo
unsigned long tiempoAnterior = 0;
unsigned long deltaT = 0;

// Variables para compartir datos del sensor
unsigned long ultimoTiempoSensor = 0;
const unsigned long intervaloSensor = 1000;  // 1 segundo

// ========= SETUP =========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===========================");
  Serial.println("Iniciando sistema combinado mejorado...");

  // Inicializar I2C y MPU6050
  Serial.println("Inicializando I2C...");
  Wire.begin();
  
  Serial.println("Inicializando MPU6050...");
  mpu.initialize();
  
  // Verificar la conexión
  Serial.println("Probando conexión con el MPU6050...");
  Serial.print("MPU6050 conexión: ");
  Serial.println(mpu.testConnection() ? "OK" : "FALLO");
  
  // Configuración del rango de acelerómetro a ±2g
  mpu.setFullScaleAccelRange(0); // 0 = ±2g
  
  Serial.println("Calibrando sensor... Mantenga el dispositivo inmóvil.");
  // Realizar la calibración
  calibrateSensor();
  
  tiempoAnterior = millis();
  
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
  server.on("/", HTTP_GET, handleRoot);
  server.on("/rssi", HTTP_POST, handleRSSI);
  server.on("/data", HTTP_GET, handleSensorData);
  server.on("/clients", HTTP_GET, handleClientData);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Servidor HTTP iniciado");
  Serial.println("Rutas disponibles: /, /rssi, /data, /clients");
  Serial.println("===========================");
}

void handleSensorData() {
  StaticJsonDocument<200> doc;
  
  doc["accelX"] = accelX;
  doc["accelY"] = accelY;
  doc["velocX"] = velocX;
  doc["velocY"] = velocY;
  
  String jsonResponse;
  serializeJson(doc, jsonResponse);
  
  server.send(200, "application/json", jsonResponse);
  Serial.println("Enviando datos del sensor por HTTP");
}

// ========= LOOP =========
void loop() {
  // Procesar solicitudes DNS y HTTP
  dnsServer.processNextRequest();
  server.handleClient();

  // Gestionar clientes WiFi y monitoreo
  unsigned long tiempoActual = millis();
  if (tiempoActual - ultimoTiempoWiFi >= intervaloWiFi) {
    ultimoTiempoWiFi = tiempoActual;
    gestionClientes();
    mostrarInfoClientes();
  }
  
  // Leer datos del sensor MPU6050
  leerDatosMPU();
}

// ========= Funciones MPU6050 =========

// Función para calibrar el sensor
void calibrateSensor() {
  float sumX = 0, sumY = 0;
  
  for(int i = 0; i < CALIBRATION_SAMPLES; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    sumX += ax;
    sumY += ay;
    delay(5);
    
    // Mostrar progreso cada 100 muestras
    if(i % 100 == 0) {
      Serial.print("Calibrando... ");
      Serial.print((i * 100) / CALIBRATION_SAMPLES);
      Serial.println("%");
    }
  }
  
  offsetX = sumX / CALIBRATION_SAMPLES;
  offsetY = sumY / CALIBRATION_SAMPLES;
  calibrated = true;
  
  Serial.println("Calibración completada.");
  Serial.print("Offsets calculados - X: ");
  Serial.print(offsetX);
  Serial.print(", Y: ");
  Serial.println(offsetY);
}

// Función para actualizar la velocidad con filtrado mejorado
void updateVelocity(float deltaT) {
  // Eje X
  if(abs(accelX) > noiseThreshold) {
    velocX += accelX * deltaT;
  } else {
    velocX *= decayFactor;
    // Forzar a cero cuando sea muy pequeño
    if(abs(velocX) < 0.01) velocX = 0;
  }
  
  // Eje Y
  if(abs(accelY) > noiseThreshold) {
    velocY += accelY * deltaT;
  } else {
    velocY *= decayFactor;
    // Forzar a cero cuando sea muy pequeño
    if(abs(velocY) < 0.01) velocY = 0;
  }
}

void leerDatosMPU() {
  // Leer datos de aceleración
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Aplicar calibración
  ax -= offsetX;
  ay -= offsetY;
  
  // Convertir a m/s²
  accelX = ax * accelFactor;
  accelY = ay * accelFactor;
  
  // Calcular tiempo transcurrido desde la última medición
  unsigned long tiempoActual = millis();
  deltaT = tiempoActual - tiempoAnterior;  // Delta de tiempo en milisegundos
  float deltaTSegundos = deltaT / 1000.0;  // Convertir a segundos
  
  // Actualizar velocidad con filtrado mejorado
  updateVelocity(deltaTSegundos);
  
  // Actualizar tiempo anterior
  tiempoAnterior = tiempoActual;
  
  // Mostrar resultados en el Monitor Serial periódicamente
  if (tiempoActual - ultimoTiempoSensor >= intervaloSensor) {
    ultimoTiempoSensor = tiempoActual;
    
    Serial.println("=== Datos del MPU6050 ===");
    Serial.print("Aceleración: X = ");
    Serial.print(accelX, 4);  // 4 decimales para mayor precisión
    Serial.print(" m/s², Y = ");
    Serial.print(accelY, 4);
    Serial.println(" m/s²");
    
    Serial.print("Velocidad: X = ");
    Serial.print(velocX, 4);
    Serial.print(" m/s, Y = ");
    Serial.print(velocY, 4);
    Serial.println(" m/s");
    
    Serial.print("Tiempo delta: ");
    Serial.print(deltaT);
    Serial.println(" ms");
    Serial.println("=======================");
  }
}

// ========= Manejadores HTTP =========
void handleRoot() {
  String html = "<html><head><title>ESP32 + MPU6050</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial;margin:20px;text-align:center;} ";
  html += "h1{color:#0066cc;} .data{margin:20px;padding:10px;border:1px solid #ddd;border-radius:5px;} ";
  html += "button{background:#0066cc;color:white;border:none;padding:10px 20px;border-radius:5px;margin:10px;cursor:pointer;} ";
  html += "button:hover{background:#0055aa;} ";
  html += ".client-list{margin:20px;padding:10px;border:1px solid #ddd;border-radius:5px;background:#f8f8f8;}</style>";
  html += "</head><body>";
  html += "<h1>ESP32 + MPU6050 Monitor</h1>";
  html += "<div class='data' id='sensorData'>Cargando datos...</div>";
  html += "<div class='client-list' id='clientData'>Cargando información de clientes...</div>";
  html += "<button onclick='fetchData()'>Actualizar datos</button>";
  html += "<button onclick='fetchClients()'>Actualizar clientes</button>";
  html += "<script>";
  html += "function fetchData() {";
  html += "  fetch('/data')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      let html = '<h2>Datos del sensor:</h2>';";
  html += "      html += '<p><strong>Aceleración X:</strong> ' + data.accelX.toFixed(2) + ' m/s²</p>';";
  html += "      html += '<p><strong>Aceleración Y:</strong> ' + data.accelY.toFixed(2) + ' m/s²</p>';";
  html += "      html += '<p><strong>Velocidad X:</strong> ' + data.velocX.toFixed(2) + ' m/s</p>';";
  html += "      html += '<p><strong>Velocidad Y:</strong> ' + data.velocY.toFixed(2) + ' m/s</p>';";
  html += "      document.getElementById('sensorData').innerHTML = html;";
  html += "    })";
  html += "    .catch(error => {";
  html += "      document.getElementById('sensorData').innerHTML = '<p>Error al obtener datos</p>';";
  html += "      console.error('Error:', error);";
  html += "    });";
  html += "}";
  html += "function fetchClients() {";
  html += "  fetch('/clients')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      let html = '<h2>Clientes conectados: ' + data.count + '</h2>';";
  html += "      if(data.count > 0) {";
  html += "        html += '<table style=\"width:100%; text-align:left;\"><tr><th>Cliente</th><th>MAC</th><th>IP</th></tr>';";
  html += "        data.clients.forEach((client, index) => {";
  html += "          html += '<tr><td>' + (index + 1) + '</td><td>' + client.mac + '</td><td>' + client.ip + '</td></tr>';";
  html += "        });";
  html += "        html += '</table>';";
  html += "      } else {";
  html += "        html += '<p>No hay clientes conectados actualmente</p>';";
  html += "      }";
  html += "      html += '<p><strong>Memoria libre:</strong> ' + data.freeHeap + ' KB</p>';";
  html += "      document.getElementById('clientData').innerHTML = html;";
  html += "    })";
  html += "    .catch(error => {";
  html += "      document.getElementById('clientData').innerHTML = '<p>Error al obtener información de clientes</p>';";
  html += "      console.error('Error:', error);";
  html += "    });";
  html += "}";
  html += "fetchData();";
  html += "fetchClients();";
  html += "setInterval(fetchData, 1000);";
  html += "setInterval(fetchClients, 5000);";
  html += "</script></body></html>";
  
  server.send(200, "text/html", html);
}

void handleNotFound() {
  String mensaje = "Ruta no encontrada\n\n";
  mensaje += "URI: ";
  mensaje += server.uri();
  mensaje += "\n";
  mensaje += "Método: ";
  mensaje += (server.method() == HTTP_GET ? "GET" : "POST");
  mensaje += "\n";
  
  server.send(404, "text/plain", mensaje);
  Serial.println("Petición 404: " + server.uri());
}

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

void handleClientData() {
  StaticJsonDocument<1024> doc;
  
  int numClientes = WiFi.softAPgetStationNum();
  doc["count"] = numClientes;
  doc["freeHeap"] = ESP.getFreeHeap() / 1024;
  
  if (numClientes > 0) {
    wifi_sta_list_t stationList;
    tcpip_adapter_sta_list_t adapterList;
    
    if (esp_wifi_ap_get_sta_list(&stationList) == ESP_OK &&
        tcpip_adapter_get_sta_list(&stationList, &adapterList) == ESP_OK) {
      
      JsonArray clients = doc.createNestedArray("clients");
      
      for (int i = 0; i < adapterList.num; i++) {
        tcpip_adapter_sta_info_t station = adapterList.sta[i];
        
        JsonObject clientObj = clients.createNestedObject();
        
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 station.mac[0], station.mac[1], station.mac[2], 
                 station.mac[3], station.mac[4], station.mac[5]);
        
        clientObj["mac"] = String(macStr);
        clientObj["ip"] = IPAddress(station.ip.addr).toString();
      }
    }
  } else {
    doc["clients"] = JsonArray();
  }
  
  String jsonResponse;
  serializeJson(doc, jsonResponse);
  
  server.send(200, "application/json", jsonResponse);
  Serial.println("Enviando información de clientes por HTTP");
}

// ========= Funciones WiFi =========
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

