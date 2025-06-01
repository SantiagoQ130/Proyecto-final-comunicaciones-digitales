#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// ========== CONFIGURACIÓN SISTEMA ==========
const char* ssid = "ESP32-MPU6050";
const char* password = "12345678";  // Mínimo 8 caracteres
WebServer server(80);
MPU6050 mpu;

// Constantes optimizadas para ESP32
#define MAX_CLIENTES 8                   // ESP32 tiene más memoria
#define TIMEOUT_HTTP 3000               // Timeout HTTP
#define SENSOR_FREQ_HZ 50               // 50Hz para mejor respuesta
#define SENSOR_INTERVAL_MS (1000/SENSOR_FREQ_HZ)
#define WIFI_SCAN_INTERVAL 15000        // Escaneo menos frecuente
#define REPORT_INTERVAL 2000            // Reportes cada 2 segundos
#define MAX_RECONNECT_ATTEMPTS 3        // Intentos de reconexión
#define KALMAN_Q 0.001f                 // Ruido del proceso
#define KALMAN_R 0.01f                  // Ruido de medición

// Tipos de dispositivos
enum DeviceType { WIFI_ONLY, HTTP_CLIENT, UNKNOWN_DEVICE };

// ========== ESTRUCTURAS DE DATOS ==========
struct KalmanFilter {
  float q;      // Ruido del proceso
  float r;      // Ruido de medición  
  float x;      // Valor estimado
  float p;      // Error de estimación
  float k;      // Ganancia de Kalman
};

struct SensorData {
  float accelX, accelY, accelZ;         // Aceleración (g)
  float gyroX, gyroY, gyroZ;            // Velocidad angular (°/s)
  float pitch, roll;                    // Ángulos calculados (°)
  float temperature;                    // Temperatura del sensor (°C)
  unsigned long timestamp;
  bool valido;
  uint8_t calidad;                      // 0-100%
};

struct ClientInfo {
  IPAddress ip;
  unsigned long lastSeen;
  unsigned long firstSeen;
  uint16_t requests;
  DeviceType type;
  String deviceName;
  float avgResponseTime;
  bool active;
};

struct SystemMetrics {
  unsigned long totalRequests;
  unsigned long errors404;
  unsigned long sensorErrors;
  unsigned long uptime;
  float cpuLoad;
  uint32_t freeMemory;
  uint8_t clientCount;
};

// ========== VARIABLES GLOBALES ==========
SensorData sensor;
SystemMetrics metrics;
ClientInfo clients[MAX_CLIENTES];
KalmanFilter kalman_ax, kalman_ay, kalman_gx, kalman_gy;

// Control de tiempo optimizado
unsigned long lastSensorRead = 0;
unsigned long lastReport = 0;
unsigned long lastWifiScan = 0;
unsigned long lastCleanup = 0;
unsigned long systemStart;

// Estado del sensor
bool sensorAvailable = false;
bool calibrationComplete = false;
int16_t accel_offset[3] = {0, 0, 0};
int16_t gyro_offset[3] = {0, 0, 0};
uint8_t reconnectAttempts = 0;

// ========== OPTIMIZACIONES ADICIONALES ==========
#define ENABLE_DEEP_ANALYSIS     // Análisis avanzado de movimiento
#define HISTORY_SIZE 20

struct SensorHistory {
  SensorData buffer[HISTORY_SIZE];
  uint8_t index;
  uint8_t count;
  float avgAccelMagnitude;
  float avgGyroMagnitude;
  bool movementDetected;
};

SensorHistory history;

// Estados del sistema para mejor control
enum SystemState {
  STATE_INITIALIZING,
  STATE_CALIBRATING, 
  STATE_NORMAL,
  STATE_ERROR_RECOVERY,
  STATE_LOW_POWER
};

SystemState currentState = STATE_INITIALIZING;

// ========== FUNCIONES DE UTILIDAD ==========
void logInfo(const String& msg) {
  Serial.print(F("[INFO] "));
  Serial.println(msg);
}

void logError(const String& msg) {
  Serial.print(F("[ERROR] "));
  Serial.println(msg);
}

void logDebug(const String& msg) {
  #ifdef DEBUG_MODE
  Serial.print(F("[DEBUG] "));
  Serial.println(msg);
  #endif
}

// Inicializar filtro Kalman
void initKalman(KalmanFilter* kf, float q, float r) {
  kf->q = q;
  kf->r = r;
  kf->x = 0.0f;
  kf->p = 1.0f;
}

// Aplicar filtro Kalman
float applyKalman(KalmanFilter* kf, float measurement) {
  // Predicción
  kf->p += kf->q;
  
  // Actualización
  kf->k = kf->p / (kf->p + kf->r);
  kf->x += kf->k * (measurement - kf->x);
  kf->p *= (1.0f - kf->k);
  
  return kf->x;
}

// ========== INICIALIZACIÓN ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  logInfo(F("=== ESP32-S3 MPU6050 Monitor v1.0 ==="));
  logInfo(F("Iniciando sistema optimizado..."));
  
  systemStart = millis();
  
  // Inicializar filtros Kalman
  initKalman(&kalman_ax, KALMAN_Q, KALMAN_R);
  initKalman(&kalman_ay, KALMAN_Q, KALMAN_R);
  initKalman(&kalman_gx, KALMAN_Q * 10, KALMAN_R * 10); // Más ruido para gyro
  initKalman(&kalman_gy, KALMAN_Q * 10, KALMAN_R * 10);
  
  // Inicializar I2C y MPU6050 - Pines para ESP32
  Wire.begin(21, 22); // SDA=21, SCL=22 en ESP32
  Wire.setClock(400000); // 400kHz para mejor rendimiento
  
  if (initializeMPU6050()) {
    logInfo(F("✓ MPU6050 inicializado"));
    sensorAvailable = true;
    calibrateSensor();
  } else {
    logError(F("✗ MPU6050 no detectado"));
  }
  
  // Configurar WiFi AP
  setupWiFiAP();
  
  // Configurar rutas del servidor web
  setupWebServer();
  
  // Inicializar métricas
  memset(&metrics, 0, sizeof(metrics));
  memset(clients, 0, sizeof(clients));
  memset(&history, 0, sizeof(history));
  
  logInfo(F("✓ Sistema listo"));
  logInfo("Conecta a WiFi: " + String(ssid));
  logInfo("Accede a: http://192.168.4.1");
  logInfo(F("================================"));
}

bool initializeMPU6050() {
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    return false;
  }
  
  // Configuración optimizada
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);    // ±4g para mejor sensibilidad
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);    // ±500°/s
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);               // Filtro 42Hz
  mpu.setRate(19);                                   // 50Hz (1000/(1+19))
  
  // Configurar interrupciones si es necesario
  mpu.setIntEnabled(0x01); // Data ready interrupt
  
  return true;
}

void setupWiFiAP() {
  logInfo(F("Configurando Access Point..."));
  
  // Configurar ESP32 como punto de acceso
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), 
                    IPAddress(192, 168, 4, 1), 
                    IPAddress(255, 255, 255, 0));
  
  bool result = WiFi.softAP(ssid, password);
  
  if(result) {
    logInfo("AP creado: " + String(ssid));
    logInfo("IP: " + WiFi.softAPIP().toString());
  } else {
    logError(F("Error al crear AP"));
    while (true) delay(1000);
  }
  
  delay(2000); // Estabilización
}

void setupWebServer() {
  // Configurar rutas del servidor web
  server.on("/", handleRoot);
  server.on("/data", handleSensorData);
  server.on("/status", handleStatus);
  server.on("/calibrate", handleCalibrate);
  server.onNotFound(handleNotFound);
  
  // Iniciar servidor web
  server.begin();
  logInfo(F("Servidor web iniciado"));
}

void calibrateSensor() {
  if (!sensorAvailable) return;
  
  logInfo(F("Iniciando calibración (5 segundos)..."));
  logInfo(F("Mantén el sensor inmóvil"));
  
  const int samples = 1000;
  long acc_sum[3] = {0, 0, 0};
  long gyro_sum[3] = {0, 0, 0};
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    acc_sum[0] += ax;
    acc_sum[1] += ay;
    acc_sum[2] += az;
    gyro_sum[0] += gx;
    gyro_sum[1] += gy;
    gyro_sum[2] += gz;
    
    if (i % 200 == 0) Serial.print(".");
    delay(3);
  }
  Serial.println();
  
  // Calcular offsets
  accel_offset[0] = acc_sum[0] / samples;
  accel_offset[1] = acc_sum[1] / samples;
  accel_offset[2] = (acc_sum[2] / samples) - 8192; // Compensar gravedad (±4g: 1g = 8192)
  
  gyro_offset[0] = gyro_sum[0] / samples;
  gyro_offset[1] = gyro_sum[1] / samples;
  gyro_offset[2] = gyro_sum[2] / samples;
  
  calibrationComplete = true;
  logInfo(F("✓ Calibración completada"));
}

// ========== BUCLE PRINCIPAL OPTIMIZADO ==========
void loop() {
  unsigned long now = millis();
  unsigned long loopStart = micros();
  
  // Máquina de estados mejorada
  switch (currentState) {
    case STATE_INITIALIZING:
      if (sensorAvailable && calibrationComplete) {
        currentState = STATE_NORMAL;
        logInfo(F("Sistema iniciado correctamente"));
      }
      break;
      
    case STATE_CALIBRATING:
      // La calibración se maneja en la función específica
      break;
      
    case STATE_NORMAL:
      // Leer sensor a frecuencia fija
      if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
        if (sensorAvailable) {
          readSensorOptimized();
          addToHistory(sensor); // Agregar al historial
        } else {
          currentState = STATE_ERROR_RECOVERY;
        }
        lastSensorRead = now;
      }
      break;
      
    case STATE_ERROR_RECOVERY:
      attemptSensorRecovery();
      break;
      
    case STATE_LOW_POWER:
      // Modo de bajo consumo (futuras implementaciones)
      delay(100);
      break;
  }
  
  // Procesar clientes web
  server.handleClient();
  
  // Tareas periódicas con intervalos optimizados
  if (now - lastReport >= REPORT_INTERVAL) {
    updateSystemMetrics();
    optimizeMemoryUsage(); // Optimización de memoria
    lastReport = now;
  }
  
  if (now - lastWifiScan >= WIFI_SCAN_INTERVAL) {
    scanWiFiClients();
    lastWifiScan = now;
  }
  
  if (now - lastCleanup >= 30000) { // Limpieza cada 30s
    cleanupInactiveClients();
    lastCleanup = now;
  }
  
  // Calcular carga CPU
  unsigned long loopTime = micros() - loopStart;
  metrics.cpuLoad = (metrics.cpuLoad * 0.95f) + (loopTime / 1000.0f * 0.05f);
}

void readSensorOptimized() {
  if (!sensorAvailable) {
    // Intentar reconectar
    if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
      if (initializeMPU6050()) {
        logInfo(F("✓ Sensor reconectado"));
        sensorAvailable = true;
        reconnectAttempts = 0;
      } else {
        reconnectAttempts++;
      }
    }
    
    // Datos por defecto si no hay sensor
    sensor.valido = false;
    sensor.calidad = 0;
    return;
  }
  
  int16_t ax, ay, az, gx, gy, gz, temp;
  
  // Leer datos crudos
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  temp = mpu.getTemperature();
  
  // Verificar validez de datos
  if (ax == 0 && ay == 0 && az == 0) {
    metrics.sensorErrors++;
    sensor.valido = false;
    sensor.calidad = 0;
    sensorAvailable = false; // Marcar para reconexión
    return;
  }
  
  // Aplicar calibración
  if (calibrationComplete) {
    ax -= accel_offset[0];
    ay -= accel_offset[1];
    az -= accel_offset[2];
    gx -= gyro_offset[0];
    gy -= gyro_offset[1];
    gz -= gyro_offset[2];
  }
  
  // Convertir a unidades físicas (±4g: 1g = 8192 LSB)
  float accelX_raw = (float)ax / 8192.0f;
  float accelY_raw = (float)ay / 8192.0f;
  float accelZ_raw = (float)az / 8192.0f;
  
  // Convertir giroscopio (±500°/s: 1°/s = 65.5 LSB)
  float gyroX_raw = (float)gx / 65.5f;
  float gyroY_raw = (float)gy / 65.5f;
  float gyroZ_raw = (float)gz / 65.5f;
  
  // Aplicar filtros Kalman
  sensor.accelX = applyKalman(&kalman_ax, accelX_raw);
  sensor.accelY = applyKalman(&kalman_ay, accelY_raw);
  sensor.accelZ = accelZ_raw; // Z sin filtrar para detección de gravedad
  
  sensor.gyroX = applyKalman(&kalman_gx, gyroX_raw);
  sensor.gyroY = applyKalman(&kalman_gy, gyroY_raw);
  sensor.gyroZ = gyroZ_raw;
  
  // Convertir temperatura (fórmula del MPU6050)
  sensor.temperature = (float)temp / 340.0f + 36.53f;
  
  // Calcular ángulos (pitch y roll)
  sensor.pitch = atan2(sensor.accelY, sqrt(sensor.accelX * sensor.accelX + sensor.accelZ * sensor.accelZ)) * 180.0f / PI;
  sensor.roll = atan2(-sensor.accelX, sensor.accelZ) * 180.0f / PI;

  // Calcular calidad de la señal
  float accel_magnitude = sqrt(sensor.accelX * sensor.accelX + 
                              sensor.accelY * sensor.accelY + 
                              sensor.accelZ * sensor.accelZ);

  // Calidad basada en qué tan cerca está la magnitud total a 1g
  float quality_factor = 1.0f - abs(accel_magnitude - 1.0f);
  sensor.calidad = constrain(quality_factor * 100.0f, 0, 100);

  sensor.timestamp = millis();
  sensor.valido = true;
}

void attemptSensorRecovery() {
  static unsigned long lastRecoveryAttempt = 0;
  
  if (millis() - lastRecoveryAttempt > 5000) { // Intentar cada 5 segundos
    logInfo(F("Intentando recuperación del sensor..."));
    
    if (initializeMPU6050()) {
      logInfo(F("✓ Sensor recuperado"));
      sensorAvailable = true;
      currentState = STATE_NORMAL;
      reconnectAttempts = 0;
    } else {
      reconnectAttempts++;
      if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
        logError(F("✗ Fallo en recuperación"));
        delay(10000); // Esperar más tiempo antes del siguiente intento
        reconnectAttempts = 0;
      }
    }
    
    lastRecoveryAttempt = millis();
  }
}

// ========== PROCESAMIENTO WEB ==========
void handleRoot() {
  String html = buildMainPage();
  server.send(200, "text/html", html);
  updateClientInfo(server.client().remoteIP(), "web_browser");
  metrics.totalRequests++;
}

void handleSensorData() {
  JsonDocument doc;
  
  doc["accelX"] = round(sensor.accelX * 1000) / 1000.0;
  doc["accelY"] = round(sensor.accelY * 1000) / 1000.0;
  doc["accelZ"] = round(sensor.accelZ * 1000) / 1000.0;
  doc["gyroX"] = round(sensor.gyroX * 10) / 10.0;
  doc["gyroY"] = round(sensor.gyroY * 10) / 10.0;
  doc["gyroZ"] = round(sensor.gyroZ * 10) / 10.0;
  doc["pitch"] = round(sensor.pitch * 10) / 10.0;
  doc["roll"] = round(sensor.roll * 10) / 10.0;
  doc["temperature"] = round(sensor.temperature * 10) / 10.0;
  doc["timestamp"] = sensor.timestamp;
  doc["valido"] = sensor.valido;
  doc["calidad"] = sensor.calidad;
  doc["sensorConectado"] = sensorAvailable;
  
  String json;
  serializeJson(doc, json);
  
  server.send(200, "application/json", json);
  updateClientInfo(server.client().remoteIP(), "api_client");
  metrics.totalRequests++;
}

void handleStatus() {
  JsonDocument doc;
  
  doc["uptime"] = millis() - systemStart;
  doc["totalRequests"] = metrics.totalRequests;
  doc["errors404"] = metrics.errors404;
  doc["sensorErrors"] = metrics.sensorErrors;
  doc["cpuLoad"] = round(metrics.cpuLoad * 10) / 10.0;
  doc["freeMemory"] = ESP.getFreeHeap();
  doc["clientCount"] = getActiveClientCount();
  doc["sensorAvailable"] = sensorAvailable;
  doc["calibrationComplete"] = calibrationComplete;
  doc["macAddress"] = WiFi.softAPmacAddress();
  doc["chipModel"] = ESP.getChipModel();
  doc["chipCores"] = ESP.getChipCores();
  doc["flashSize"] = ESP.getFlashChipSize();
  
  // Información sobre clientes conectados
  JsonArray clientArray = doc.createNestedArray("clients");
  for (int i = 0; i < MAX_CLIENTES; i++) {
    if (clients[i].active) {
      JsonObject clientObj = clientArray.createNestedObject();
      clientObj["ip"] = clients[i].ip.toString();
      clientObj["requests"] = clients[i].requests;
      clientObj["lastSeen"] = millis() - clients[i].lastSeen;
      clientObj["device"] = clients[i].deviceName;
    }
  }
  
  String json;
  serializeJson(doc, json);
  
  server.send(200, "application/json", json);
  updateClientInfo(server.client().remoteIP(), "api_client");
  metrics.totalRequests++;
}

void handleCalibrate() {
  if (sensorAvailable) {
    calibrateSensor();
    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Calibración completada\"}");
  } else {
    server.send(503, "application/json", "{\"status\":\"error\",\"message\":\"Sensor no disponible\"}");
  }
  metrics.totalRequests++;
}

void handleNotFound() {
  String message = "Página no encontrada\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMétodo: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArgumentos: ";
  message += server.args();
  message += "\n";
  
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  
  server.send(404, "text/plain", message);
  metrics.errors404++;
}

// ========== PÁGINA WEB PRINCIPAL ==========
String buildMainPage() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>ESP32-S3 MPU6050 Monitor</title>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; min-height: 100vh; }";
  html += ".container { max-width: 1200px; margin: 0 auto; }";
  html += ".header { text-align: center; margin-bottom: 30px; }";
  html += ".header h1 { font-size: 2.5em; margin-bottom: 10px; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); }";
  html += ".status { background: rgba(255,255,255,0.1); border: 1px solid rgba(255,255,255,0.3); border-radius: 10px; padding: 15px; margin-bottom: 20px; backdrop-filter: blur(10px); }";
  html += ".grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }";
  html += ".card { background: rgba(255,255,255,0.1); border: 1px solid rgba(255,255,255,0.2); border-radius: 15px; padding: 20px; backdrop-filter: blur(10px); box-shadow: 0 8px 25px rgba(0,0,0,0.3); }";
  html += ".card h3 { margin-bottom: 15px; color: #fff; font-size: 1.3em; text-shadow: 0 2px 4px rgba(0,0,0,0.3); }";
  html += ".sensor-value { display: flex; justify-content: space-between; margin: 10px 0; padding: 8px; background: rgba(255,255,255,0.1); border: 1px solid rgba(255,255,255,0.2); border-radius: 5px; }";
  html += ".value { font-weight: bold; color: #4dd0e1; text-shadow: 0 1px 2px rgba(0,0,0,0.5); }";
  html += ".btn { background: linear-gradient(45deg, #1e88e5, #42a5f5); color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; margin: 5px; transition: all 0.3s; box-shadow: 0 4px 8px rgba(0,0,0,0.3); }";
  html += ".btn:hover { background: linear-gradient(45deg, #1976d2, #2196f3); transform: translateY(-2px); box-shadow: 0 6px 12px rgba(0,0,0,0.4); }";
  html += ".online { color: #4dd0e1; text-shadow: 0 1px 2px rgba(0,0,0,0.5); }";
  html += ".offline { color: #ff6b6b; text-shadow: 0 1px 2px rgba(0,0,0,0.5); }";
  html += "@media(max-width:768px) { .grid { grid-template-columns: 1fr; } .header h1 { font-size: 2em; } }";
  html += "</style></head><body>";
  
  html += "<div class='container'>";
  html += "<div class='header'>";
  html += "<h1> ESP32-S3 MPU6050 Monitor</h1>";
  html += "<p>Sistema de monitoreo optimizado v1.0</p>";
  html += "</div>";
  
  html += "<div class='status' id='status'>";
  html += "<span id='sensor-status'>🔄 Conectando...</span>";
  html += "<span style='float:right'>Calidad: <span id='quality'>--</span>%</span>";
  html += "</div>";
  
  html += "<div class='grid'>";
  
  // Card de Acelerómetro
  html += "<div class='card'>";
  html += "<h3>📊 Acelerómetro</h3>";
  html += "<div class='sensor-value'><span>X:</span><span class='value' id='accelX'>0.00 g</span></div>";
  html += "<div class='sensor-value'><span>Y:</span><span class='value' id='accelY'>0.00 g</span></div>";
  html += "<div class='sensor-value'><span>Z:</span><span class='value' id='accelZ'>0.00 g</span></div>";
  html += "</div>";
  
  // Card de Giroscopio
  html += "<div class='card'>";
  html += "<h3>🌀 Giroscopio</h3>";
  html += "<div class='sensor-value'><span>X:</span><span class='value' id='gyroX'>0.0 °/s</span></div>";
  html += "<div class='sensor-value'><span>Y:</span><span class='value' id='gyroY'>0.0 °/s</span></div>";
  html += "<div class='sensor-value'><span>Z:</span><span class='value' id='gyroZ'>0.0 °/s</span></div>";
  html += "</div>";
  
  // Card de Orientación
  html += "<div class='card'>";
  html += "<h3>🎯 Orientación</h3>";
  html += "<div class='sensor-value'><span>Pitch:</span><span class='value' id='pitch'>0.0°</span></div>";
  html += "<div class='sensor-value'><span>Roll:</span><span class='value' id='roll'>0.0°</span></div>";
  html += "<div class='sensor-value'><span>Temperatura:</span><span class='value' id='temperature'>0.0°C</span></div>";
  html += "</div>";
  
  // Card de Sistema
  html += "<div class='card'>";
  html += "<h3>⚙️ Sistema</h3>";
  html += "<div class='sensor-value'><span>CPU:</span><span class='value' id='cpu'>0.0%</span></div>";
  html += "<div class='sensor-value'><span>Memoria:</span><span class='value' id='memory'>0 KB</span></div>";
  html += "<div class='sensor-value'><span>Clientes:</span><span class='value' id='clients'>0</span></div>";
  html += "<div class='sensor-value'><span>Uptime:</span><span class='value' id='uptime'>0s</span></div>";
  html += "</div>";
  
  html += "</div>";
  
  // Botones de control
  html += "<div style='text-align: center; margin-top: 20px;'>";
  html += "<button class='btn' onclick='updateData()'>🔄 Actualizar</button>";
  html += "<button class='btn' onclick='calibrateSensor()'>⚖️ Calibrar</button>";
  html += "<button class='btn' onclick='window.location.href=\"/status\"'>📊 Estado</button>";
  html += "</div>";
  
  html += "<div style='margin-top: 20px; text-align: center; color: rgba(255,255,255,0.7);'>";
  html += "<p>ESP32-S3 N16R8 funcionando correctamente</p>";
  html += "</div>";
  
  html += "</div>";
  
  // JavaScript para actualización automática
  html += "<script>";
  html += "function updateData() {";
  html += "  fetch('/data')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      document.getElementById('accelX').textContent = data.accelX.toFixed(3) + ' g';";
  html += "      document.getElementById('accelY').textContent = data.accelY.toFixed(3) + ' g';";
  html += "      document.getElementById('accelZ').textContent = data.accelZ.toFixed(3) + ' g';";
  html += "      document.getElementById('gyroX').textContent = data.gyroX.toFixed(1) + ' °/s';";
  html += "      document.getElementById('gyroY').textContent = data.gyroY.toFixed(1) + ' °/s';";
  html += "      document.getElementById('gyroZ').textContent = data.gyroZ.toFixed(1) + ' °/s';";
  html += "      document.getElementById('pitch').textContent = data.pitch.toFixed(1) + '°';";
  html += "      document.getElementById('roll').textContent = data.roll.toFixed(1) + '°';";
  html += "      document.getElementById('temperature').textContent = data.temperature.toFixed(1) + '°C';";
  html += "      document.getElementById('quality').textContent = data.calidad;";
  html += "      ";
  html += "      let status = data.sensorConectado ? '🟢 Sensor Online' : '🔴 Sensor Offline';";
  html += "      document.getElementById('sensor-status').textContent = status;";
  html += "      document.getElementById('sensor-status').className = data.sensorConectado ? 'online' : 'offline';";
  html += "    })";
  html += "    .catch(error => {";
  html += "      document.getElementById('sensor-status').textContent = '❌ Error de conexión';";
  html += "      document.getElementById('sensor-status').className = 'offline';";
  html += "    });";
  html += "  ";
  html += "  fetch('/status')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      document.getElementById('cpu').textContent = data.cpuLoad.toFixed(1) + '%';";
  html += "      document.getElementById('memory').textContent = Math.round(data.freeMemory/1024) + ' KB';";
  html += "      document.getElementById('clients').textContent = data.clientCount;";
  html += "      document.getElementById('uptime').textContent = Math.round(data.uptime/1000) + 's';";
  html += "    });";
  html += "}";
  html += "";
  html += "function calibrateSensor() {";
  html += "  fetch('/calibrate')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      alert(data.message);";
  html += "      updateData();";
  html += "    })";
  html += "    .catch(error => alert('Error en calibración'));";
  html += "}";
  html += "";
  html += "// Actualizar datos cada segundo";
  html += "setInterval(updateData, 1000);";
  html += "updateData(); // Cargar datos iniciales";
  html += "</script>";
  
  html += "</body></html>";
  
  return html;
}

// ========== FUNCIONES DE UTILIDAD Y MÉTRICAS ==========
void updateSystemMetrics() {
  metrics.uptime = millis() - systemStart;
  metrics.freeMemory = ESP.getFreeHeap();
  metrics.clientCount = getActiveClientCount();
  
  // Log periódico optimizado
  static unsigned long lastFullReport = 0;
  if (millis() - lastFullReport > 30000) { // Cada 30 segundos
    logInfo("Estado: CPU=" + String(metrics.cpuLoad, 1) + 
           "% RAM=" + String(metrics.freeMemory/1024) + 
           "KB Clientes=" + String(metrics.clientCount) +
           " Sensor=" + (sensorAvailable ? "OK" : "ERROR"));
    lastFullReport = millis();
  }
}

void updateClientInfo(IPAddress ip, const String& deviceType) {
  unsigned long now = millis();
  
  // Buscar cliente existente
  for (int i = 0; i < MAX_CLIENTES; i++) {
    if (clients[i].active && clients[i].ip == ip) {
      clients[i].lastSeen = now;
      clients[i].requests++;
      return;
    }
  }
  
  // Buscar slot libre para nuevo cliente
  for (int i = 0; i < MAX_CLIENTES; i++) {
    if (!clients[i].active) {
      clients[i].ip = ip;
      clients[i].firstSeen = now;
      clients[i].lastSeen = now;
      clients[i].requests = 1;
      clients[i].deviceName = deviceType;
      clients[i].active = true;
      clients[i].type = (deviceType == "web_browser") ? HTTP_CLIENT : WIFI_ONLY;
      break;
    }
  }
}

uint8_t getActiveClientCount() {
  uint8_t count = 0;
  unsigned long now = millis();
  
  for (int i = 0; i < MAX_CLIENTES; i++) {
    if (clients[i].active && (now - clients[i].lastSeen < 60000)) { // Activo en último minuto
      count++;
    }
  }
  
  return count;
}

void scanWiFiClients() {
  // NOTA: Las APIs tcpip_adapter_* fueron deprecadas en ESP-IDF 4.1+
  // Ahora se usa esp_netif_* pero es más complejo
  
  // Solución simple: usar WiFi.softAPgetStationNum() para contar clientes
  uint8_t stationCount = WiFi.softAPgetStationNum();
  
  // Actualizar contador de clientes (método simplificado)
  static uint8_t lastStationCount = 0;
  if (stationCount != lastStationCount) {
    logInfo("Clientes WiFi conectados: " + String(stationCount));
    lastStationCount = stationCount;
    
    // Si hay más clientes que antes, crear entradas genéricas
    if (stationCount > lastStationCount) {
      for (int i = 0; i < MAX_CLIENTES; i++) {
        if (!clients[i].active) {
          // Crear cliente genérico (no podemos obtener IP específica fácilmente)
          clients[i].ip = IPAddress(192, 168, 4, 100 + i); // IP genérica
          clients[i].firstSeen = millis();
          clients[i].lastSeen = millis();
          clients[i].requests = 0;
          clients[i].deviceName = "wifi_device";
          clients[i].active = true;
          clients[i].type = WIFI_ONLY;
          break;
        }
      }
    }
  }
}

void cleanupInactiveClients() {
  unsigned long now = millis();
  int removed = 0;
  
  for (int i = 0; i < MAX_CLIENTES; i++) {
    if (clients[i].active && (now - clients[i].lastSeen > 300000)) { // 5 minutos
      memset(&clients[i], 0, sizeof(ClientInfo));
      removed++;
    }
  }
  
  if (removed > 0) {
    logInfo("Clientes inactivos removidos: " + String(removed));
  }
}

void optimizeMemoryUsage() {
  static unsigned long lastOptimization = 0;
  if (millis() - lastOptimization > 30000) { // Cada 30 segundos
    
    // Compactar array de clientes
    int activeCount = 0;
    for (int i = 0; i < MAX_CLIENTES; i++) {
      if (clients[i].active) {
        if (activeCount != i) {
          clients[activeCount] = clients[i];
          memset(&clients[i], 0, sizeof(ClientInfo));
        }
        activeCount++;
      }
    }
    
    lastOptimization = millis();
    logDebug("Memoria optimizada. Clientes activos: " + String(activeCount));
  }
}

// ========== HISTORIAL Y ANÁLISIS ==========
void addToHistory(const SensorData& data) {
  history.buffer[history.index] = data;
  history.index = (history.index + 1) % HISTORY_SIZE;
  if (history.count < HISTORY_SIZE) history.count++;
  
  // Actualizar promedios
  float accelSum = 0, gyroSum = 0;
  for (int i = 0; i < history.count; i++) {
    accelSum += sqrt(pow(history.buffer[i].accelX, 2) + 
                    pow(history.buffer[i].accelY, 2) + 
                    pow(history.buffer[i].accelZ, 2));
    gyroSum += sqrt(pow(history.buffer[i].gyroX, 2) + 
                   pow(history.buffer[i].gyroY, 2) + 
                   pow(history.buffer[i].gyroZ, 2));
  }
  
  history.avgAccelMagnitude = accelSum / history.count;
  history.avgGyroMagnitude = gyroSum / history.count;
  
  #ifdef ENABLE_DEEP_ANALYSIS
  analyzeMovementPatterns();
  #endif
}

void analyzeMovementPatterns() {
  if (history.count < HISTORY_SIZE) return;
  
  float accelVariance = 0;
  float gyroVariance = 0;
  
  // Calcular varianza para detección de movimiento
  for (int i = 0; i < HISTORY_SIZE; i++) {
    float accelMag = sqrt(pow(history.buffer[i].accelX, 2) + 
                         pow(history.buffer[i].accelY, 2) + 
                         pow(history.buffer[i].accelZ, 2));
    float gyroMag = sqrt(pow(history.buffer[i].gyroX, 2) + 
                        pow(history.buffer[i].gyroY, 2) + 
                        pow(history.buffer[i].gyroZ, 2));
    
    accelVariance += pow(accelMag - history.avgAccelMagnitude, 2);
    gyroVariance += pow(gyroMag - history.avgGyroMagnitude, 2);
  }
  
  accelVariance /= HISTORY_SIZE;
  gyroVariance /= HISTORY_SIZE;
  
  // Umbral para detección de movimiento
  history.movementDetected = (accelVariance > 0.01) || (gyroVariance > 5.0);
}

// ========== INFORMACIÓN DE ESTADO ==========
void printConnectionInfo() {
  Serial.println("=== ESTADO DEL HOTSPOT ===");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Clientes conectados: ");
  Serial.println(WiFi.softAPgetStationNum());
  Serial.print("Sensor MPU6050: ");
  Serial.println(sensorAvailable ? "OK" : "ERROR");
  Serial.print("Memoria libre: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("CPU Load: ");
  Serial.print(metrics.cpuLoad, 1);
  Serial.println("%");
  Serial.println("========================");
}
