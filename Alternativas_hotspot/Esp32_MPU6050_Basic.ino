/*
 * Código para ESP32 con sensor MPU6050
 * Incluye calibración, filtrado mejorado y manejo de deriva
 * Uso de la libreria de Electronic Cats
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

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

// Constantes de configuración
const float accelFactor = 9.81 / 16384.0; // Conversión a m/s² (±2g)
const int CALIBRATION_SAMPLES = 500;      // Muestras para calibración
const float noiseThreshold = 0.05;        // Umbral de ruido en m/s²
const float decayFactor = 0.9;            // Factor de decaimiento
const int sampleRate = 50;                // Tiempo entre muestras (ms)

// Variables de tiempo
unsigned long tiempoAnterior = 0;
unsigned long deltaT = 0;

void setup() {
  // Inicializar comunicación serial
  Serial.begin(115200);
  while (!Serial) {
    ; // Esperar conexión serial
  }

  Serial.println("Inicializando I2C...");
  Wire.begin();

  Serial.println("Inicializando MPU6050...");
  mpu.initialize();

  // Verificar conexión
  Serial.println("Probando conexión con el dispositivo...");
  Serial.print("MPU6050 conexión exitosa: ");
  Serial.println(mpu.testConnection() ? "OK" : "FALLO");

  // Configurar rango del acelerómetro a ±2g
  mpu.setFullScaleAccelRange(0);

  Serial.println("Calibrando sensor... Mantenga el dispositivo inmóvil.");
  
  // Calcular offsets
  calibrateSensor();
  
  Serial.println("=========================");
  Serial.println("Iniciando mediciones...");
  Serial.println("=========================");
  
  tiempoAnterior = millis();
}

void loop() {
  // Leer datos del sensor
  mpu.getAcceleration(&ax, &ay, &az);

  // Aplicar calibración
  ax -= offsetX;
  ay -= offsetY;

  // Convertir a m/s²
  accelX = ax * accelFactor;
  accelY = ay * accelFactor;

  // Calcular intervalo de tiempo
  unsigned long tiempoActual = millis();
  deltaT = tiempoActual - tiempoAnterior;
  float deltaTSegundos = deltaT / 1000.0;
  
  // Calcular velocidad con filtrado mejorado
  updateVelocity(deltaTSegundos);
  
  // Actualizar tiempo anterior
  tiempoAnterior = tiempoActual;

  // Mostrar resultados
  printData();

  delay(sampleRate); // Intervalo entre muestras
}

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

// Función para actualizar la velocidad
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

// Función para mostrar los datos
void printData() {
  Serial.println("=== Datos del MPU6050 ===");
  Serial.print("Aceleración: X = ");
  Serial.print(accelX, 4);  // 4 decimales
  Serial.print(" m/s², Y = ");
  Serial.print(accelY, 4);
  Serial.println(" m/s²");

  Serial.print("Velocidad: X = ");
  Serial.print(velocX, 4);
  Serial.print(" m/s, Y = ");
  Serial.print(velocY, 4);
  Serial.println(" m/s");

  Serial.print("Intervalo: ");
  Serial.print(deltaT);
  Serial.println(" ms");
  Serial.println("=======================");
}
