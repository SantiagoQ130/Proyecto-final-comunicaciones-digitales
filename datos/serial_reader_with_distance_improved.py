import serial
import json
import time
import math
from datetime import datetime
from collections import deque

# Configuración del puerto serial
SERIAL_PORT = 'COM5'  # Cambia esto según tu sistema
BAUD_RATE = 115200

# Parámetros para cálculo de distancia
TX_POWER_DBM = -20  # Potencia de transmisión de referencia a 1 metro (dBm)
PATH_LOSS_EXPONENT = 2.0  # Exponente de pérdida de trayectoria (2.0 para espacio libre)

class SerialReaderWithDistance:
    def __init__(self, puerto_serial=SERIAL_PORT, baud_rate=BAUD_RATE, samples_to_average=20):
        """Inicializar el lector serial con cálculo de distancia."""
        self.puerto_serial = puerto_serial
        self.baud_rate = baud_rate
        self.samples_to_average = samples_to_average
        self.ser = None
        
        # Contadores para estadísticas
        self.total_lines = 0
        self.data_lines = 0
        self.error_lines = 0
        
        # Buffer circular para promediar muestras RSSI
        self.rssi_buffer = {
            'tx1': deque(maxlen=samples_to_average),
            'tx2': deque(maxlen=samples_to_average),
            'tx3': deque(maxlen=samples_to_average),
            'tx4': deque(maxlen=samples_to_average)
        }
        
        # Último RSSI promediado y distancia calculada
        self.averaged_rssi = {
            'tx1': None,
            'tx2': None,
            'tx3': None,
            'tx4': None
        }
        
        self.estimated_distance = {
            'tx1': None,
            'tx2': None,
            'tx3': None,
            'tx4': None
        }
        
        # Valores máximos y mínimos de RSSI
        self.rssi_max = {
            'tx1': None,
            'tx2': None,
            'tx3': None,
            'tx4': None
        }
        
        self.rssi_min = {
            'tx1': None,
            'tx2': None,
            'tx3': None,
            'tx4': None
        }
        
        # Inicializar conexión serial
        self.inicializar_serial()
    
    def inicializar_serial(self):
        """Inicializa la conexión serial."""
        try:
            self.ser = serial.Serial(self.puerto_serial, self.baud_rate, timeout=0.1)
            print(f"✅ Puerto serial {self.puerto_serial} conectado @ {self.baud_rate} bps")
            print(f"📡 Promediando {self.samples_to_average} muestras para cálculo de distancia")
            print(f"🎯 Parámetros: TX Power = {TX_POWER_DBM} dBm, Path Loss = {PATH_LOSS_EXPONENT}")
            print("-" * 80)
        except Exception as e:
            print(f"❌ Error conectando al puerto serial: {e}")
            print(f"💡 Verifica que el puerto {self.puerto_serial} esté disponible")
            raise
    
    def calcular_distancia(self, rssi_dbm):
        """
        Calcula la distancia estimada basada en el RSSI usando el modelo de pérdida de trayectoria.
        
        Fórmula: RSSI = TX_POWER - 10 * n * log10(d)
        Donde:
        - RSSI: Potencia recibida en dBm
        - TX_POWER: Potencia de transmisión a 1 metro en dBm
        - n: Exponente de pérdida de trayectoria
        - d: Distancia en metros
        
        Despejando d: d = 10^((TX_POWER - RSSI) / (10 * n))
        """
        if rssi_dbm is None:
            return None
        
        try:
            # Calcular distancia en metros
            distance = math.pow(10, (TX_POWER_DBM - rssi_dbm) / (10 * PATH_LOSS_EXPONENT))
            return round(distance, 2)
        except Exception as e:
            print(f"❌ Error calculando distancia: {e}")
            return None
    
    def actualizar_promedio_rssi(self, tx, rssi_value):
        """Actualiza el buffer de muestras y calcula el promedio."""
        if rssi_value is not None and rssi_value != -100:
            # Agregar nueva muestra al buffer circular
            self.rssi_buffer[tx].append(rssi_value)
            
            # Actualizar valores máximos y mínimos
            if self.rssi_max[tx] is None or rssi_value > self.rssi_max[tx]:
                self.rssi_max[tx] = rssi_value
            
            if self.rssi_min[tx] is None or rssi_value < self.rssi_min[tx]:
                self.rssi_min[tx] = rssi_value
            
            # Calcular promedio si tenemos suficientes muestras
            if len(self.rssi_buffer[tx]) > 0:
                promedio = sum(self.rssi_buffer[tx]) / len(self.rssi_buffer[tx])
                self.averaged_rssi[tx] = round(promedio, 1)
                
                # Calcular distancia estimada
                self.estimated_distance[tx] = self.calcular_distancia(promedio)
    
    def procesar_linea_data(self, line):
        """Procesa una línea que contiene datos JSON."""
        try:
            # Extraer JSON después de "DATA:"
            data_json = line[5:]  # Quitar "DATA:" del inicio
            data = json.loads(data_json)
            
            # Extraer datos RSSI
            rssi = data.get('rssi', {})
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            # Actualizar promedios para cada transmisor
            for tx in ['tx1', 'tx2', 'tx3', 'tx4']:
                if tx in rssi:
                    self.actualizar_promedio_rssi(tx, rssi[tx])
            
            # Mostrar datos en formato legible
            self.mostrar_datos_tiempo_real(timestamp)
            
            self.data_lines += 1
            return True
            
        except json.JSONDecodeError as e:
            print(f"❌ Error JSON: {e}")
            self.error_lines += 1
            return False
        except Exception as e:
            print(f"❌ Error procesando datos: {e}")
            self.error_lines += 1
            return False
    
    def mostrar_datos_tiempo_real(self, timestamp):
        """Muestra los datos RSSI promediados y distancias en tiempo real."""
        datos_linea = []
        
        for tx in ['tx1', 'tx2', 'tx3', 'tx4']:
            rssi_avg = self.averaged_rssi[tx]
            distancia = self.estimated_distance[tx]
            samples_count = len(self.rssi_buffer[tx])
            
            if rssi_avg is not None and distancia is not None:
                datos_linea.append(f"{tx.upper()}: {rssi_avg:5.1f}dBm ({distancia:5.2f}m) [{samples_count:2d}]")
            else:
                datos_linea.append(f"{tx.upper()}: ---- dBm (----m) [{samples_count:2d}]")
        
        print(f"[{timestamp}] {' | '.join(datos_linea)}")
    
    def procesar_linea_general(self, line):
        """Procesa otros tipos de líneas (SETUP, DEBUG, etc.)"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        if line.startswith("SETUP:"):
            print(f"🚀 [{timestamp}] {line}")
        elif line.startswith("DEBUG:"):
            print(f"🔧 [{timestamp}] {line}")
        elif line.startswith("ERROR:"):
            print(f"❌ [{timestamp}] {line}")
        elif line.strip():  # Si no está vacía
            print(f"ℹ️  [{timestamp}] {line}")
    
    def mostrar_estadisticas(self):
        """Muestra estadísticas detalladas de recepción y distancias."""
        print("\n" + "="*100)
        print("📊 ESTADÍSTICAS DE RECEPCIÓN Y DISTANCIAS")
        print("="*100)
        print(f"Total líneas leídas: {self.total_lines}")
        print(f"Líneas con datos: {self.data_lines}")
        print(f"Líneas con errores: {self.error_lines}")
        print(f"Tasa de éxito: {(self.data_lines/max(1,self.total_lines)*100):.1f}%")
        print()
        
        print("🎯 RSSI PROMEDIADO Y DISTANCIAS ESTIMADAS:")
        print("-" * 100)
        print("TX   | RSSI Avg (dBm) | Distancia (m) | Muestras | Desv. Estándar | RSSI Max | RSSI Min")
        print("-" * 100)
        
        for tx in ['tx1', 'tx2', 'tx3', 'tx4']:
            rssi_avg = self.averaged_rssi[tx]
            distancia = self.estimated_distance[tx]
            samples_count = len(self.rssi_buffer[tx])
            rssi_max = self.rssi_max[tx]
            rssi_min = self.rssi_min[tx]
            
            # Calcular desviación estándar si hay suficientes muestras
            desv_std = "N/A"
            if samples_count > 1:
                mean = sum(self.rssi_buffer[tx]) / samples_count
                variance = sum((x - mean) ** 2 for x in self.rssi_buffer[tx]) / samples_count
                desv_std = f"{math.sqrt(variance):.2f}"
            
            # Formatear valores máximos y mínimos
            max_str = f"{rssi_max:3d}" if rssi_max is not None else "---"
            min_str = f"{rssi_min:3d}" if rssi_min is not None else "---"
            
            if rssi_avg is not None and distancia is not None:
                print(f"{tx.upper():<4} | {rssi_avg:>12.1f} | {distancia:>11.2f} | {samples_count:>8d} | {desv_std:>12s} | {max_str:>8s} | {min_str:>8s}")
            else:
                print(f"{tx.upper():<4} | {'----':>12s} | {'----':>11s} | {samples_count:>8d} | {desv_std:>12s} | {max_str:>8s} | {min_str:>8s}")
        
        print("-" * 100)
        print(f"📐 Parámetros de cálculo:")
        print(f"   • Potencia TX a 1m: {TX_POWER_DBM} dBm")
        print(f"   • Exponente pérdida: {PATH_LOSS_EXPONENT}")
        print(f"   • Muestras promedio: {self.samples_to_average}")
        print(f"📊 Rango de RSSI observado:")
        for tx in ['tx1', 'tx2', 'tx3', 'tx4']:
            if self.rssi_max[tx] is not None and self.rssi_min[tx] is not None:
                rango = self.rssi_max[tx] - self.rssi_min[tx]
                print(f"   • {tx.upper()}: {rango:.1f} dBm de variación")
        print("="*100)
    
    def leer_serial_continuo(self):
        """Lee datos del puerto serial de forma continua."""
        try:
            print("🔄 Leyendo datos serial con cálculo de distancia (Ctrl+C para detener)...")
            print("📏 Formato: TX: RSSI_promedio (distancia_metros) [muestras_buffer]")
            print()
            
            # Tiempo para mostrar estadísticas cada 15 segundos
            last_stats_time = time.time()
            
            while True:
                try:
                    # Verificar si hay datos disponibles
                    if self.ser.in_waiting > 0:
                        # Leer línea completa
                        line = self.ser.readline().decode('utf-8').strip()
                        
                        if line:  # Si la línea no está vacía
                            self.total_lines += 1
                            
                            # Procesar según el tipo de línea
                            if line.startswith("DATA:"):
                                self.procesar_linea_data(line)
                            else:
                                self.procesar_linea_general(line)
                    
                    # Mostrar estadísticas cada 15 segundos
                    current_time = time.time()
                    if current_time - last_stats_time >= 15:
                        self.mostrar_estadisticas()
                        last_stats_time = current_time
                    
                    # Pequeña pausa para no saturar la CPU
                    time.sleep(0.01)
                    
                except UnicodeDecodeError:
                    print("⚠️  Error de codificación en línea recibida")
                    self.error_lines += 1
                
        except KeyboardInterrupt:
            print("\n⏹️  Lectura detenida por el usuario")
        except serial.SerialException as e:
            print(f"\n❌ Error de comunicación serial: {e}")
        except Exception as e:
            print(f"\n❌ Error inesperado: {e}")
        finally:
            self.mostrar_estadisticas()
    
    def cerrar(self):
        """Cierra la conexión serial."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("✅ Puerto serial cerrado correctamente")
        except Exception as e:
            print(f"⚠️  Error cerrando puerto serial: {e}")

def main():
    """Función principal."""
    print("="*80)
    print("🔌 LECTOR SERIAL CON CÁLCULO DE DISTANCIA MEJORADO")
    print("="*80)
    print(f"Puerto: {SERIAL_PORT}")
    print(f"Velocidad: {BAUD_RATE} bps")
    print(f"Muestras para promedio: 20")
    print("="*80)
    
    # Crear el lector serial
    try:
        reader = SerialReaderWithDistance(SERIAL_PORT, BAUD_RATE, samples_to_average=20)
        
        # Iniciar lectura continua
        reader.leer_serial_continuo()
        
    except Exception as e:
        print(f"❌ Error inicializando: {e}")
    finally:
        # Cerrar conexión
        try:
            reader.cerrar()
        except:
            pass
        print("🏁 Programa terminado")

if __name__ == "__main__":
    main()
