#include <WiFiS3.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// Includes específicos para Arduino R4 WiFi (ARM Cortex-M4)
extern "C" char* sbrk(int incr);

// ========== CONFIGURACIÓN SISTEMA ==========
const char* ssid = "ArduinoR4-123456";
const char* password = "12345678";
WiFiServer server(80);
MPU6050 mpu;

// Constantes optimizadas
#define MAX_CLIENTES 10                  // Reducido para mejor rendimiento
#define TIMEOUT_HTTP 3000               // Timeout más corto
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
  uint16_t freeMemory;
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
#define ENABLE_WATCHDOG          // Habilitar watchdog para recuperación automática
#define ENABLE_DEEP_ANALYSIS     // Análisis avanzado de movimiento
#define ENABLE_DATA_LOGGING      // Logging de datos para análisis

// Buffer circular para datos históricos
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
  
  logInfo(F("=== Arduino R4 MPU6050 Monitor v2.0 ==="));
  logInfo(F("Iniciando sistema optimizado..."));
  
  systemStart = millis();
  
  // Inicializar filtros Kalman
  initKalman(&kalman_ax, KALMAN_Q, KALMAN_R);
  initKalman(&kalman_ay, KALMAN_Q, KALMAN_R);
  initKalman(&kalman_gx, KALMAN_Q * 10, KALMAN_R * 10); // Más ruido para gyro
  initKalman(&kalman_gy, KALMAN_Q * 10, KALMAN_R * 10);
  
  // Inicializar I2C y MPU6050
  Wire.begin();
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
  
  // Inicializar métricas
  memset(&metrics, 0, sizeof(metrics));
  memset(clients, 0, sizeof(clients));
  
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
  
  WiFi.config(IPAddress(192, 168, 4, 1));
  
  if (WiFi.beginAP(ssid, password) != WL_AP_LISTENING) {
    logError(F("Error al crear AP"));
    while (true) delay(1000);
  }
  
  delay(2000); // Estabilización
  server.begin();
  
  logInfo("AP creado: " + String(ssid));
  logInfo("IP: " + WiFi.localIP().toString());
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
  processWebClients();
  
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

void processWebClients() {
  WiFiClient client = server.available();
  if (!client) return;
  
  unsigned long clientStart = millis();
  String request = readHTTPRequest(client);
  
  if (request.length() > 0) {
    String userAgent = extractUserAgent(client);
    registerClient(client.remoteIP(), userAgent);
    
    bool processed = routeRequest(client, request);
    if (processed) {
      metrics.totalRequests++;
      updateResponseTime(client.remoteIP(), millis() - clientStart);
    }
  }
  
  client.stop();
}

String readHTTPRequest(WiFiClient& client) {
  String request = "";
  unsigned long timeout = millis() + 1000; // 1 segundo timeout
  
  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();
      
      if (line.length() == 0) break; // Fin de headers
      if (request == "") request = line; // Primera línea = request
    }
  }
  
  return request;
}

String extractUserAgent(WiFiClient& client) {
  // Implementación simplificada - en versión real habría que leer headers
  return "Unknown";
}

bool routeRequest(WiFiClient& client, const String& request) {
  if (request.indexOf("GET / ") >= 0) {
    sendMainPage(client);
    return true;
  } else if (request.indexOf("GET /data") >= 0) {
    sendSensorData(client);
    return true;
  } else if (request.indexOf("GET /status") >= 0) {
    sendSystemStatus(client);
    return true;
  } else if (request.indexOf("GET /calibrate") >= 0) {
    sendCalibrateResponse(client);
    return true;
  } else {
    send404(client);
    metrics.errors404++;
    return false;
  }
}

void sendSensorData(WiFiClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Access-Control-Allow-Origin: *"));
  client.println(F("Cache-Control: no-cache"));
  client.println();
    // Usar JsonDocument para compatibilidad con ArduinoJson v7
  JsonDocument doc;
    doc["accelX"] = round(sensor.accelX * 1000) / 1000.0;
  doc["accelY"] = round(sensor.accelY * 1000) / 1000.0;
  doc["accelZ"] = round(sensor.accelZ * 1000) / 1000.0;
  doc["gyroX"] = round(sensor.gyroX * 10) / 10.0;
  doc["gyroY"] = round(sensor.gyroY * 10) / 10.0;
  doc["gyroZ"] = round(sensor.gyroZ * 10) / 10.0;
  doc["pitch"] = round(sensor.pitch * 10) / 10.0;
  doc["roll"] = round(sensor.roll * 10) / 10.0;
  doc["timestamp"] = sensor.timestamp;
  doc["valido"] = sensor.valido;
  doc["calidad"] = sensor.calidad;
  doc["sensorConectado"] = sensorAvailable;
  
  String json;
  serializeJson(doc, json);
  client.print(json);
}

void sendSystemStatus(WiFiClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Access-Control-Allow-Origin: *"));
  client.println();

  JsonDocument doc;
  
  doc["uptime"] = millis() - systemStart;
  doc["totalRequests"] = metrics.totalRequests;
  doc["errors404"] = metrics.errors404;
  doc["sensorErrors"] = metrics.sensorErrors;
  doc["cpuLoad"] = round(metrics.cpuLoad * 10) / 10.0;
  doc["freeMemory"] = getFreeMemory();
  doc["clientCount"] = getActiveClientCount();
  doc["sensorAvailable"] = sensorAvailable;
  doc["calibrationComplete"] = calibrationComplete;
  doc["macAddress"] = getMacAddress();
  
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
  client.print(json);
}

void sendMainPage(WiFiClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html; charset=utf-8"));
  client.println();
  
  // Página HTML moderna y responsiva
  client.println(F("<!DOCTYPE html><html><head>"));
  client.println(F("<meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"));
  client.println(F("<title>Arduino R4 - MPU6050 Monitor v2.0</title>"));
  client.println(F("<style>"));

  client.println(F("*{margin:0;padding:0;box-sizing:border-box}"));
  client.println(F("body{font-family:'Segoe UI',Arial,sans-serif;background:linear-gradient(135deg,#1a2332 0%,#0f1419 50%,#2c3e50 100%);color:#e8f1ff;min-height:100vh;padding:20px}"));
  client.println(F(".container{max-width:1200px;margin:0 auto}"));
  client.println(F(".header{text-align:center;margin-bottom:30px}"));
  client.println(F(".header h1{font-size:2.5em;margin-bottom:10px;text-shadow:2px 2px 4px rgba(0,0,0,0.3)}"));  client.println(F(".status{background:rgba(56, 142, 191, 0.25);border:1px solid rgba(135, 206, 235, 0.3);border-radius:10px;padding:15px;margin-bottom:20px;backdrop-filter:blur(10px)}"));
  client.println(F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:20px}"));
  client.println(F(".card{background:rgba(30, 58, 93, 0.4);border:1px solid rgba(135, 206, 235, 0.2);border-radius:15px;padding:20px;backdrop-filter:blur(10px);box-shadow:0 8px 25px rgba(0,0,0,0.3)}"));
  client.println(F(".card h3{margin-bottom:15px;color:#87ceeb;font-size:1.3em;text-shadow:0 2px 4px rgba(0,0,0,0.3)}"));
  client.println(F(".sensor-value{display:flex;justify-content:space-between;margin:10px 0;padding:8px;background:rgba(30, 144, 255, 0.15);border:1px solid rgba(135, 206, 235, 0.2);border-radius:5px}"));
  client.println(F(".value{font-weight:bold;color:#4dd0e1;text-shadow:0 1px 2px rgba(0,0,0,0.5)}"));  client.println(F(".chart{height:200px;background:rgba(0,0,0,0.4);border:1px solid rgba(135, 206, 235, 0.2);border-radius:10px;margin-top:15px;position:relative;overflow:hidden}"));
  client.println(F(".btn{background:linear-gradient(45deg,#1e88e5,#42a5f5);color:white;border:none;padding:10px 20px;border-radius:5px;cursor:pointer;margin:5px;transition:all 0.3s;box-shadow:0 4px 8px rgba(0,0,0,0.3)}"));
  client.println(F(".btn:hover{background:linear-gradient(45deg,#1976d2,#2196f3);transform:translateY(-2px);box-shadow:0 6px 12px rgba(0,0,0,0.4)}"));
  client.println(F(".online{color:#4dd0e1;text-shadow:0 1px 2px rgba(0,0,0,0.5)}"));
  client.println(F(".offline{color:#ff6b6b;text-shadow:0 1px 2px rgba(0,0,0,0.5)}"));
  client.println(F("@media(max-width:768px){.grid{grid-template-columns:1fr}.header h1{font-size:2em}}"));
  client.println(F("/* Tabla de clientes conectados */"));
  client.println(F("#connected-clients {"));
  client.println(F("  max-height: 300px;"));
  client.println(F("  overflow-y: auto;"));
  client.println(F("  margin-top: 10px;"));
  client.println(F("}"));

  client.println(F("#connected-clients table {"));
  client.println(F("  width: 100%;"));
  client.println(F("  border-collapse: collapse;"));
  client.println(F("  font-size: 0.9em;"));
  client.println(F("}"));

  client.println(F("#connected-clients th {"));
  client.println(F("  text-align: left;"));
  client.println(F("  padding: 8px 5px;"));
  client.println(F("  border-bottom: 1px solid rgba(135, 206, 235, 0.3);"));
  client.println(F("  color: #87ceeb;"));
  client.println(F("  position: sticky;"));
  client.println(F("  top: 0;"));
  client.println(F("  background: rgba(30, 58, 93, 0.9);"));
  client.println(F("}"));

  client.println(F("#connected-clients td {"));
  client.println(F("  padding: 6px 5px;"));
  client.println(F("  border-bottom: 1px solid rgba(135, 206, 235, 0.1);"));
  client.println(F("}"));

  client.println(F("#connected-clients tr:last-child td {"));
  client.println(F("  border-bottom: none;"));
  client.println(F("}"));

  client.println(F("#connected-clients th:nth-child(2),"));
  client.println(F("#connected-clients th:nth-child(3),"));
  client.println(F("#connected-clients td:nth-child(2),"));
  client.println(F("#connected-clients td:nth-child(3) {"));
  client.println(F("  text-align: right;"));
  client.println(F("}"));


  client.println(F("</style></head><body>"));
  
  client.println(F("<div class='container'>"));
  client.println(F("<div class='header'>"));
  client.println(F("<h1>🚀 Arduino R4 MPU6050 Monitor</h1>"));
  client.println(F("<p>Sistema de monitoreo optimizado v2.0</p>"));
  client.println(F("</div>"));
  
  client.println(F("<div class='status' id='status'>"));
  client.println(F("<span id='sensor-status'>🔄 Conectando...</span>"));
  client.println(F("<span style='float:right'>Calidad: <span id='quality'>--</span>%</span>"));
  client.println(F("</div>"));
  
  client.println(F("<div class='grid'>"));
  
  // Card de Acelerómetro
  client.println(F("<div class='card'>"));
  client.println(F("<h3>📊 Acelerómetro</h3>"));
  client.println(F("<div class='sensor-value'><span>X:</span><span class='value' id='accelX'>0.00 g</span></div>"));
  client.println(F("<div class='sensor-value'><span>Y:</span><span class='value' id='accelY'>0.00 g</span></div>"));
  client.println(F("<div class='sensor-value'><span>Z:</span><span class='value' id='accelZ'>0.00 g</span></div>"));
  client.println(F("</div>"));
  
  // Card de Giroscopio
  client.println(F("<div class='card'>"));
  client.println(F("<h3>🌀 Giroscopio</h3>"));
  client.println(F("<div class='sensor-value'><span>X:</span><span class='value' id='gyroX'>0.0 °/s</span></div>"));
  client.println(F("<div class='sensor-value'><span>Y:</span><span class='value' id='gyroY'>0.0 °/s</span></div>"));
  client.println(F("<div class='sensor-value'><span>Z:</span><span class='value' id='gyroZ'>0.0 °/s</span></div>"));
  client.println(F("</div>"));
  
  // Card de Orientación
  client.println(F("<div class='card'>"));
  client.println(F("<h3>🎯 Orientación</h3>"));
  client.println(F("<div class='sensor-value'><span>Pitch:</span><span class='value' id='pitch'>0.0°</span></div>"));
  client.println(F("<div class='sensor-value'><span>Roll:</span><span class='value' id='roll'>0.0°</span></div>"));
  client.println(F("</div>"));

  // Card de clientes
  client.println(F("<div class='card'>"));
  client.println(F("<h3>📡 Información de Red</h3>"));
  client.println(F("<div class='sensor-value'><span>IP del Arduino:</span><span class='value' id='device-ip'>192.168.4.1</span></div>"));
  client.println(F("<div class='sensor-value'><span>MAC Address:</span><span class='value' id='device-mac'>--:--:--:--:--:--</span></div>"));
  client.println(F("<div class='sensor-value'><span>Clientes Conectados:</span><span class='value' id='client-count'>0</span></div>"));
  client.println(F("</div>"));
  
  // Card de Sistema
  client.println(F("<div class='card'>"));
  client.println(F("<h3>⚙️ Sistema</h3>"));
  client.println(F("<div class='sensor-value'><span>CPU:</span><span class='value' id='cpu'>0.0%</span></div>"));
  client.println(F("<div class='sensor-value'><span>Clientes:</span><span class='value' id='clients'>0</span></div>"));
  client.println(F("<div class='sensor-value'><span>Peticiones:</span><span class='value' id='requests'>0</span></div>"));
  client.println(F("<button class='btn' onclick='calibrate()'>🔧 Calibrar</button>"));
  client.println(F("<button class='btn' onclick='location.reload()'>🔄 Actualizar</button>"));
  client.println(F("</div>"));

  // Card de Clientes Conectados (añadir junto a las otras cards)
  client.println(F("<div class='card'>"));
  client.println(F("<h3>👥 Clientes Conectados</h3>"));
  client.println(F("<div id='connected-clients'>"));
  client.println(F("<table style='width:100%; border-collapse: collapse;'>"));
  client.println(F("<thead><tr>"));
  client.println(F("<th style='text-align:left; padding:8px 5px;'>IP</th>"));
  client.println(F("<th style='text-align:right; padding:8px 5px;'>Peticiones</th>"));
  client.println(F("<th style='text-align:right; padding:8px 5px;'>Tiempo</th>"));
  client.println(F("</tr></thead>"));
  client.println(F("<tbody id='client-list'></tbody>"));
  client.println(F("</table>"));
  client.println(F("</div>")); // cierre connected-clients
  client.println(F("</div>")); 
  
  client.println(F("</div></div>"));
  
  // JavaScript corregido
  client.println(F("<script>"));
  
  // Función para actualizar la lista de clientes (ya existente, mantener)
  client.println(F("function updateClientList(clients) {"));
  client.println(F("  const tbody = document.getElementById('client-list');"));
  client.println(F("  tbody.innerHTML = '';"));
  client.println(F("  clients.forEach(client => {"));
  client.println(F("    const row = document.createElement('tr');"));
  client.println(F("    row.style.borderBottom = '1px solid rgba(135,206,235,0.1)';"));
  client.println(F("    "));
  client.println(F("    // Calcular tiempo en minutos:segundos"));
  client.println(F("    const lastSeenSec = Math.floor(client.lastSeen / 1000);"));
  client.println(F("    const minutes = Math.floor(lastSeenSec / 60);"));
  client.println(F("    const seconds = lastSeenSec % 60;"));
  client.println(F("    const timeStr = `${minutes}:${seconds.toString().padStart(2, '0')}`;"));
  client.println(F("    "));
  client.println(F("    row.innerHTML = `"));
  client.println(F("      <td style='padding:6px 5px; text-align:left;'>${client.ip}</td>"));
  client.println(F("      <td style='padding:6px 5px; text-align:right;'>${client.requests}</td>"));
  client.println(F("      <td style='padding:6px 5px; text-align:right;'>${timeStr}</td>`;"));
  client.println(F("    tbody.appendChild(row);"));
  client.println(F("  });"));
  client.println(F("}"));
  
  // AGREGAR: JavaScript principal para actualizaciones automáticas
  client.println(F("let isUpdating = false;"));
  
  client.println(F("async function updateData() {"));
  client.println(F("  if (isUpdating) return;"));
  client.println(F("  isUpdating = true;"));
  client.println(F("  "));
  client.println(F("  try {"));
  client.println(F("    // Obtener datos del sensor"));
  client.println(F("    const sensorResponse = await fetch('/data');"));
  client.println(F("    const sensorData = await sensorResponse.json();"));
  client.println(F("    "));
  client.println(F("    // Actualizar valores del sensor"));
  client.println(F("    document.getElementById('accelX').textContent = sensorData.accelX.toFixed(3) + ' g';"));
  client.println(F("    document.getElementById('accelY').textContent = sensorData.accelY.toFixed(3) + ' g';"));
  client.println(F("    document.getElementById('accelZ').textContent = sensorData.accelZ.toFixed(3) + ' g';"));
  client.println(F("    document.getElementById('gyroX').textContent = sensorData.gyroX.toFixed(1) + ' °/s';"));
  client.println(F("    document.getElementById('gyroY').textContent = sensorData.gyroY.toFixed(1) + ' °/s';"));
  client.println(F("    document.getElementById('gyroZ').textContent = sensorData.gyroZ.toFixed(1) + ' °/s';"));
  client.println(F("    document.getElementById('pitch').textContent = sensorData.pitch.toFixed(1) + '°';"));
  client.println(F("    document.getElementById('roll').textContent = sensorData.roll.toFixed(1) + '°';"));
  client.println(F("    document.getElementById('quality').textContent = sensorData.calidad;"));
  client.println(F("    "));
  client.println(F("    // Actualizar estado del sensor"));
  client.println(F("    const statusElement = document.getElementById('sensor-status');"));
  client.println(F("    if (sensorData.sensorConectado && sensorData.valido) {"));
  client.println(F("      statusElement.innerHTML = '✅ Sensor Online';"));
  client.println(F("      statusElement.className = 'online';"));
  client.println(F("    } else {"));
  client.println(F("      statusElement.innerHTML = '❌ Sensor Offline';"));
  client.println(F("      statusElement.className = 'offline';"));
  client.println(F("    }"));
  client.println(F("    "));
  client.println(F("    // Obtener datos del sistema y clientes"));
  client.println(F("    const statusResponse = await fetch('/status');"));
  client.println(F("    const systemData = await statusResponse.json();"));
  client.println(F("    "));
  client.println(F("    // Actualizar métricas del sistema"));
  client.println(F("    document.getElementById('cpu').textContent = systemData.cpuLoad.toFixed(1) + '%';"));
  client.println(F("    document.getElementById('clients').textContent = systemData.clientCount;"));
  client.println(F("    document.getElementById('requests').textContent = systemData.totalRequests;"));
  client.println(F("    document.getElementById('client-count').textContent = systemData.clientCount;"));
  client.println(F("    document.getElementById('device-mac').textContent = systemData.macAddress;"));
  client.println(F("    "));
  client.println(F("    // Actualizar lista de clientes conectados"));
  client.println(F("    if (systemData.clients && systemData.clients.length > 0) {"));
  client.println(F("      updateClientList(systemData.clients);"));
  client.println(F("    } else {"));
  client.println(F("      document.getElementById('client-list').innerHTML = '<tr><td colspan=\\'3\\' style=\\'text-align:center; padding:15px; color:#87ceeb;\\'>No hay clientes conectados</td></tr>';"));
  client.println(F("    }"));
  client.println(F("    "));
  client.println(F("  } catch (error) {"));
  client.println(F("    console.error('Error actualizando datos:', error);"));
  client.println(F("    document.getElementById('sensor-status').innerHTML = '❌ Error de Conexión';"));
  client.println(F("    document.getElementById('sensor-status').className = 'offline';"));
  client.println(F("  }"));
  client.println(F("  "));
  client.println(F("  isUpdating = false;"));
  client.println(F("}"));
  
  client.println(F("function calibrate() {"));
  client.println(F("  if (confirm('¿Mantener el sensor inmóvil durante la calibración?')) {"));
  client.println(F("    fetch('/calibrate')"));
  client.println(F("      .then(response => response.json())"));
  client.println(F("      .then(data => alert(data.message))"));
  client.println(F("      .catch(error => alert('Error en calibración'));"));
  client.println(F("  }"));
  client.println(F("}"));
  
  client.println(F("// Iniciar actualizaciones automáticas"));
  client.println(F("document.addEventListener('DOMContentLoaded', function() {"));
  client.println(F("  updateData(); // Actualización inicial"));
  client.println(F("  setInterval(updateData, 2000); // Actualizar cada 2 segundos"));
  client.println(F("});"));
  
  client.println(F("</script></body></html>"));
}

void sendCalibrateResponse(WiFiClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println();
  
  if (sensorAvailable) {
    calibrateSensor();
    client.println(F("{\"status\":\"success\",\"message\":\"Calibración completada\"}"));
  } else {
    client.println(F("{\"status\":\"error\",\"message\":\"Sensor no disponible\"}"));
  }
}

void send404(WiFiClient& client) {
  client.println(F("HTTP/1.1 404 Not Found"));
  client.println(F("Content-Type: text/html"));
  client.println();
  client.println(F("<h1>404 - Página no encontrada</h1>"));
}

// ========== GESTIÓN DE CLIENTES ==========
void registerClient(IPAddress ip, const String& userAgent) {
  if (ip == IPAddress(0, 0, 0, 0)) return;
  
  unsigned long now = millis();
  
  // Buscar cliente existente
  for (int i = 0; i < MAX_CLIENTES; i++) {
    if (clients[i].active && clients[i].ip == ip) {
      clients[i].lastSeen = now;
      clients[i].requests++;
      return;
    }
  }
  
  // Buscar slot libre
  for (int i = 0; i < MAX_CLIENTES; i++) {
    if (!clients[i].active) {
      clients[i].ip = ip;
      clients[i].lastSeen = now;
      clients[i].firstSeen = now;
      clients[i].requests = 1;
      clients[i].active = true;
      clients[i].type = HTTP_CLIENT;
      clients[i].deviceName = detectDevice(userAgent);
      clients[i].avgResponseTime = 0;
      
      logInfo("Nuevo cliente: " + ip.toString());
      break;
    }
  }
}

String detectDevice(const String& userAgent) {
  // Detección simplificada
  return "Dispositivo Web";
}

void updateResponseTime(IPAddress ip, unsigned long responseTime) {
  for (int i = 0; i < MAX_CLIENTES; i++) {
    if (clients[i].active && clients[i].ip == ip) {
      if (clients[i].avgResponseTime == 0) {
        clients[i].avgResponseTime = responseTime;
      } else {
        clients[i].avgResponseTime = (clients[i].avgResponseTime * 0.8f) + (responseTime * 0.2f);
      }
      break;
    }
  }
}

void scanWiFiClients() {
  // Escaneo simplificado - en implementación real usaría ARP
  logInfo(F("Escaneando clientes WiFi..."));
}

void cleanupInactiveClients() {
  unsigned long now = millis();
  int removed = 0;
  
  for (int i = 0; i < MAX_CLIENTES; i++) {
    if (clients[i].active && (now - clients[i].lastSeen > 60000)) { // 1 minuto timeout
      clients[i].active = false;
      removed++;
    }
  }
  
  if (removed > 0) {
    logInfo("Clientes eliminados: " + String(removed));
  }
}

int getActiveClientCount() {
  int count = 0;
  for (int i = 0; i < MAX_CLIENTES; i++) {
    if (clients[i].active) count++;
  }
  return count;
}

// ========== MÉTRICAS Y UTILIDADES ==========
void updateSystemMetrics() {
  metrics.uptime = millis() - systemStart;
  metrics.freeMemory = getFreeMemory();
  metrics.clientCount = getActiveClientCount();
  
  // Log periódico optimizado
  static unsigned long lastFullReport = 0;
  if (millis() - lastFullReport > 30000) { // Cada 30 segundos
    logInfo("Estado: CPU=" + String(metrics.cpuLoad, 1) + 
           "% RAM=" + String(metrics.freeMemory) + 
           "B Clientes=" + String(metrics.clientCount) +
           " Sensor=" + (sensorAvailable ? "OK" : "ERROR"));
    lastFullReport = millis();
  }
}

uint16_t getFreeMemory() {
  // Implementación para ARM Cortex-M4 (Arduino R4)
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

String getMacAddress() {
  // Obtener la dirección MAC del Arduino R4 WiFi
  byte mac[6];
  WiFi.macAddress(mac);
  
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  return String(macStr);
}

// ========== FUNCIONES DE OPTIMIZACIÓN ==========

// Detectar patrones de movimiento
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

// Agregar datos al historial
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

// Recuperación automática de errores
void attemptSensorRecovery() {
  logInfo(F("Intentando recuperación del sensor..."));
    reconnectAttempts++;
  if (reconnectAttempts > MAX_RECONNECT_ATTEMPTS) {
    logError(F("Máximo de intentos alcanzado. Reiniciando sistema..."));
    delay(1000);
    // Reinicio por software para ARM Cortex-M4
    NVIC_SystemReset();
  }
  
  // Reinicializar I2C
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  
  // Reintentar inicialización
  if (initializeMPU6050()) {
    logInfo(F("✓ Sensor recuperado exitosamente"));
    sensorAvailable = true;
    reconnectAttempts = 0;
    currentState = STATE_NORMAL;
    calibrateSensor();
  } else {
    logError(F("✗ Fallo en recuperación"));
    currentState = STATE_ERROR_RECOVERY;
    delay(2000); // Esperar antes del siguiente intento
  }
}

// Optimización de memoria dinámica
void optimizeMemoryUsage() {
  // Limpiar buffers no utilizados
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

