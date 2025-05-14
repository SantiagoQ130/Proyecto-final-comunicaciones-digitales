/*
 * Código para ESP32 conectado a un sensor MPU6050
 * Mide la velocidad y aceleración en los ejes X e Y
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Declaración del objeto MPU6050
MPU6050 mpu;

// Variables para almacenar los datos del sensor
int16_t ax, ay, az;
int16_t gx, gy, gz;

float accelX, accelY;      // Aceleración en los ejes X e Y (m/s²)
float velocX = 0, velocY = 0;  // Velocidad en los ejes X e Y (m/s)
unsigned long tiempoAnterior = 0;
unsigned long deltaT = 0;

// Constante para convertir las lecturas raw a m/s²
const float accelFactor = 9.81 / 16384.0; // Para rango ±2g

void setup() {
  // Inicializar comunicación serial
  Serial.begin(115200);
  while (!Serial) {
    ; // Esperar a que se establezca la conexión serial
  }

  Serial.println("Inicializando I2C...");
  Wire.begin();

  Serial.println("Inicializando MPU6050...");

  // Inicializar MPU6050
  mpu.initialize();

  // Verificar la conexión
  Serial.println("Probando conexión con el dispositivo...");
  Serial.print("MPU6050 conexión exitosa: ");
  Serial.println(mpu.testConnection() ? "OK" : "FALLO");

  // Configuración del rango de acelerómetro a ±2g
  mpu.setFullScaleAccelRange(0); // 0 = ±2g

  Serial.println("Calibrando sensor... Mantenga el dispositivo inmóvil.");
  delay(2000);  // Tiempo para estabilizar el sensor

  // No hay un método de calibración directo en esta librería, pero podrías
  // implementar uno midiendo y promediando el offset cuando el dispositivo está quieto

  tiempoAnterior = millis();
  Serial.println("=========================");
  Serial.println("Iniciando mediciones...");
  Serial.println("=========================");
}

void loop() {
  // Leer datos de aceleración
  mpu.getAcceleration(&ax, &ay, &az);

  // Convertir a m/s²
  accelX = ax * accelFactor;
  accelY = ay * accelFactor;

  // Calcular tiempo transcurrido desde la última medición
  unsigned long tiempoActual = millis();
  deltaT = tiempoActual - tiempoAnterior;  // Delta de tiempo en milisegundos
  float deltaTSegundos = deltaT / 1000.0;  // Convertir a segundos

  // Calcular velocidad (integración simple de la aceleración)
  // Aplicamos un filtro paso alto para eliminar la deriva
  if (abs(accelX) > 0.1) {  // Umbral para eliminar ruido
    velocX += accelX * deltaTSegundos;
  } else {
    // Reducir lentamente la velocidad cuando el acelerómetro está cerca de cero
    velocX *= 0.95;
  }

  if (abs(accelY) > 0.1) {  // Umbral para eliminar ruido
    velocY += accelY * deltaTSegundos;
  } else {
    // Reducir lentamente la velocidad cuando el acelerómetro está cerca de cero
    velocY *= 0.95;
  }

  // Actualizar tiempo anterior
  tiempoAnterior = tiempoActual;

  // Mostrar resultados en el Monitor Serial
  Serial.println("=== Datos del MPU6050 ===");
  Serial.print("Aceleración: X = ");
  Serial.print(accelX);
  Serial.print(" m/s², Y = ");
  Serial.print(accelY);
  Serial.println(" m/s²");

  Serial.print("Velocidad: X = ");
  Serial.print(velocX);
  Serial.print(" m/s, Y = ");
  Serial.print(velocY);
  Serial.println(" m/s");

  Serial.print("Tiempo delta: ");
  Serial.print(deltaT);
  Serial.println(" ms");
  Serial.println("=======================");

  delay(1000);  // Pequeña pausa entre lecturas
}
