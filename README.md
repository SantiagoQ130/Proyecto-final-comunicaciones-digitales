# Sistema de Localización Indoor con RSSI y Sensores IMU

## 📋 Descripción del Proyecto

Este proyecto implementa un sistema de localización en interiores que utiliza mediciones de RSSI (Received Signal Strength Indicator) para determinar la posición de objetos móviles en espacios cerrados. El sistema combina múltiples tecnologías de comunicación inalámbrica para crear una red de nodos que permite la trilateración y el seguimiento de dispositivos.

## 🎯 Objetivos

- **Objetivo Principal**: Desarrollar un sistema de localización preciso para ambientes interiores donde GPS no está disponible
- **Objetivos Específicos**:
  - Implementar una red de nodos transmisores con comunicación RF (NRF24L01)
  - Crear un concentrador que recopile datos de RSSI de múltiples transmisores
  - Integrar sensores IMU 
  - Desarrollar algoritmos de trilateración para cálculo de posición
  - Crear una interfaz web para monitoreo en tiempo real

## 🏗️ Arquitectura del Sistema

### Componentes Principales

1. **Nodos Transmisores (TX1-TX4)**
   - Microcontroladores Raspberry Pi Pico W
   - Módulos NRF24L01 para comunicación RF
   - Pantallas OLED para visualización local
   - Medición de RSSI con dispositivos WiFi cercanos

2. **Concentrador/Receptor (RX)**
   - Raspberry Pi Pico W central
   - Recepción de datos de todos los transmisores
   - Procesamiento y análisis de datos RSSI
   - Interfaz de comunicación serial

3. **Nodo Principal con IMU**
   - Arduino R4 WiFi o ESP32
   - Sensor MPU6050 (acelerómetro + giroscopio)
   - Punto de acceso WiFi (Hotspot)
   - Servidor web integrado
   - Filtrado Kalman para datos del sensor
   - Esp-S3-N16R8(ESP32 S3 cam)
   - Servidor web para la camara con diferentes configuraciones
   - Reconocimiento facial

4. **Sistema de Procesamiento**
   - Algoritmos de trilateración
   - Análisis de datos en Python
   - Visualización en tiempo real

### Diagrama de Comunicación

```
[TX1] ──┐
[TX2] ──┼── NRF24L01 ──→ [RX/Concentrador] ──→ [PC/Análisis]
[TX3] ──┤                       ↕
[TX4] ──┘                   Serial/USB

[Nodo IMU] ──→ WiFi Hotspot ──→ [Dispositivos Cliente]
    ↓
[MPU6050 Sensor]
```

## 📁 Estructura del Proyecto

```
Proyecto-final-comunicaciones-digitales-main/
├── Códigos/                    # Código principal del sistema
│   ├── ArduinoR4_hotspot_MPU6050.ino    # Nodo principal con IMU
│   ├── RX_Concentrador.py               # Receptor central
│   ├── TX1.py, TX2.py, TX3.py, TX4.py  # Nodos transmisores
│   └── PicoW_Plotter_concentrador.py    # Visualización en tiempo real
├── Alternativas_hotspot/       # Implementaciones alternativas
│   ├── Esp32_hostpot.ino      # Versión ESP32
│   ├── Esp32_MPU6050_Basic.ino # ESP32 con MPU6050 básico
│   └── Esp32-s3-hotspot-mpu6050.ino # ESP32-S3 avanzado
├── Progreso/                   # Desarrollos incrementales
│   ├── trilateracion.py       # Algoritmo de trilateración
│   ├── basic_serial_reader.py # Lector serial básico
│   └── simple_serial_reader.py # Lector serial simplificado
├── datos/                      # Datos experimentales y mediciones
│   ├── Measure 1.txt          # Mediciones RSSI vs distancia
│   ├── Measure 2.txt          # Segunda serie de mediciones
│   ├── Medidas RSSI.xlsx      # Análisis estadístico
│   └── serial_reader_data.py  # Procesador de datos serie
├── Diseño PCB/                 # Diseño de PCB personalizada
│   ├── NUEVO_NODO-*.gbr       # Archivos Gerber
│   ├── NUEVO_NODO-*.drl       # Archivos de perforación
│   └── Simulación/            # Archivos de simulación
└── README.md                   # Este archivo
```

## 🛠️ Requisitos del Sistema

### Hardware Necesario

#### Para Nodos Transmisores (TX1-TX4):
- 4x Raspberry Pi Pico W
- 4x Módulo NRF24L01+
- 4x Pantalla OLED SSD1306 (128x64)
- Cables jumper y protoboard
- Fuente de alimentación 3.3V

#### Para Concentrador (RX):
- 1x Raspberry Pi Pico W
- 1x Módulo NRF24L01+
- 1x Pantalla OLED SSD1306 (128x64)
- Cable USB para conexión serial

#### Para Nodo Principal:
- 1x Arduino R4 WiFi o ESP32 DevKit
- 1x Sensor MPU6050 (acelerómetro + giroscopio)
- Cables jumper
- 1x Esp32-S3-N16R8
- 1x Camara Ov2640

#### Opcional - PCB Personalizada:
- PCB diseñada (archivos en `/Diseño PCB/`)
- Componentes SMD según esquemático

### Software Necesario

#### Para Microcontroladores:
- **Arduino IDE** (versión 2.0 o superior)
- **MicroPython** (para Raspberry Pi Pico W)
- **Librerías Arduino**:
  - WiFiS3 (Arduino R4)
  - ArduinoJson
  - MPU6050
  - Wire

#### Para Raspberry Pi Pico W:
- **Librerías MicroPython**:
  - nrf24l01
  - ssd1306
  - machine

#### Para Análisis (PC):
- **Python 3.7+**
- **Librerías Python**:
  ```bash
  pip install numpy matplotlib pyserial
  ```

## 🚀 Instalación y Configuración

### 1. Configuración de Nodos Transmisores

1. **Preparar Raspberry Pi Pico W**:
   ```bash
   # Instalar MicroPython en cada Pico W
   # Copiar archivos TX1.py, TX2.py, TX3.py, TX4.py
   ```

2. **Conexiones Hardware TX**:
   ```
   NRF24L01    Pico W
   --------    ------
   VCC      →  3.3V
   GND      →  GND
   SCK      →  GP2
   MOSI     →  GP3
   MISO     →  GP4
   CSN      →  GP5
   CE       →  GP6
   
   OLED        Pico W
   ----        ------
   VCC      →  3.3V
   GND      →  GND
   SDA      →  GP14
   SCL      →  GP15
   ```

3. **Configuración de Direcciones**:
   - Cada transmisor tiene una dirección única
   - TX1: `\xe1\xf0\xf0\xf0\xf0`
   - TX2: `\xc3\xf0\xf0\xf0\xf0`
   - TX3: `\xd2\xf0\xf0\xf0\xf0`
   - TX4: `\xb1\xf0\xf0\xf0\xf0`

### 2. Configuración del Concentrador (RX)

1. **Hardware**: Mismo conexionado que transmisores
2. **Software**: Cargar `RX_Concentrador.py`
3. **Configuración**: 
   - Canal RF: 10
   - Direcciones sincronizadas con transmisores

### 3. Configuración del Nodo Principal

#### Para Arduino R4 WiFi:

1. **Conexiones MPU6050**:
   ```
   MPU6050     Arduino R4
   -------     ----------
   VCC      →  3.3V
   GND      →  GND
   SDA      →  SDA (pin 11)
   SCL      →  SCL (pin 12)
   ```

2. **Cargar código**: `ArduinoR4_hotspot_MPU6050.ino`

3. **Configuración WiFi**:
   - SSID: "ArduinoR4-123456"
   - Password: "12345678"
   - IP: 192.168.4.1

## 💻 Uso del Sistema

### 1. Inicialización

1. **Encender nodos en orden**:
   ```
   1. Concentrador (RX) primero
   2. Nodos transmisores (TX1-TX4)
   3. Nodo principal (Arduino/ESP32)
   ```

2. **Verificar conexiones**:
   - LEDs indicadores en cada nodo
   - Mensajes en pantallas OLED
   - Monitor serie para debug

### 2. Monitoreo en Tiempo Real

#### Interfaz Web (Nodo Principal):
1. Conectar dispositivo a WiFi "ArduinoR4-123456"
2. Abrir navegador en `http://192.168.4.1`
3. Ver datos de IMU en tiempo real

#### Monitor Serial (Concentrador):
```python
# Ejecutar visualizador
python PicoW_Plotter_concentrador.py
```

### 3. Análisis de Datos

#### Trilateración:
```python
# Ejecutar algoritmo de localización
python trilateracion.py
```

#### Procesamiento de Mediciones:
```python
# Analizar datos RSSI
python serial_reader_data.py
```

## 📊 Resultados Experimentales

### Precisión del Sistema
- **Rango de operación**: 0-30 metros
- **Precisión típica**: ±2-3 metros
- **Tiempo de actualización**: 50ms (20Hz)
- **Número de nodos**: 4 transmisores + 1 receptor

### Datos de Calibración RSSI vs Distancia

| Distancia (m) | TX1 (dBm) | TX2 (dBm) | TX3 (dBm) | TX4 (dBm) |
|---------------|-----------|-----------|-----------|-----------|
| 0             | -40       | -52       | -52       | -42       |
| 5             | -76       | -81       | -79       | -78       |
| 10            | -77       | -77       | -75       | -72       |
| 15            | -69       | -71       | -70       | -80       |
| 20            | -81       | -84       | -82       | -75       |

### Características del Sensor IMU
- **Acelerómetro**: ±2g, ±4g, ±8g, ±16g
- **Giroscopio**: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
- **Filtrado**: Kalman con Q=0.001, R=0.01
- **Frecuencia de muestreo**: 50Hz

## 📝 Notas

- Utilizamos power banks para energizar continuamente los sistemas.

## 🔧 Troubleshooting

### Problemas Comunes

#### 1. NRF24L01 no comunica:
```
Solución:
- Verificar alimentación 3.3V (no 5V)
- Revisar conexiones SPI
- Comprobar distancia entre módulos
- Verificar canal RF (debe ser igual en TX/RX)
```

#### 2. MPU6050 no responde:
```
Solución:
- Verificar conexiones I2C
- Comprobar dirección I2C (0x68 o 0x69)
- Realizar calibración inicial
- Verificar voltaje de alimentación
```

#### 3. WiFi no se conecta:
```
Solución:
- Verificar SSID y password
- Comprobar canal WiFi libre
- Reiniciar módulo WiFi
- Verificar antena conectada
```

### Comandos de Debug

```cpp
// En Arduino IDE - Monitor Serie
Serial.println("DEBUG: Estado del sistema");

// Verificar NRF24L01
nrf.printDetails();

// Estado MPU6050
Serial.print("MPU6050 ID: ");
Serial.println(mpu.getDeviceID());
```

## 🔬 Trabajo Futuro

### Mejoras Propuestas

1. **Algoritmos Avanzados**:
   - Implementar filtros de partículas
   - Fusión de sensores con filtro Kalman extendido
   - Machine Learning para calibración automática

2. **Hardware**:
   - PCB definitiva con componentes optimizados
   - Antenas direccionales para mejor RSSI
   - Batería recargable integrada

3. **Software**:
   - Interfaz web más avanzada
   - Base de datos para histórico
   - API REST para integración externa

4. **Escalabilidad**:
   - Soporte para más nodos
   - Mesh networking
   - Localización multi-piso

## 👥 Contribuidores

- **Desarrolladores**: [Rafael Garzon, Guillermo Hernandez, Sebastian Restrepo, Edgar Quiroz]
- **Proyecto**: Trilateracion indoor
- **Institución**: Universidad Militar Nueva Granada
- **Fecha**: Junio 2025

## 📚 Referencias

1. Zafari, F., Gkelias, A., & Leung, K. K. (2019). A survey of indoor localization systems and technologies. IEEE Communications Surveys & Tutorials.

2. Yasir, M., Ho, S. W., & Vellambi, B. N. (2016). Indoor positioning system using visible light and accelerometer. Journal of Lightwave Technology.

3. Arduino Reference Documentation: https://docs.arduino.cc/

4. Raspberry Pi Pico Documentation: https://datasheets.raspberrypi.org/pico/

---

**¡Gracias por usar nuestro Sistema de Localización Indoor!** 🎯📡
