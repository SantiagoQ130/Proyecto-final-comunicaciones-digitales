import serial
import json
import time
from datetime import datetime

# Configuración del puerto serial
SERIAL_PORT = 'COM5'  # Cambia esto según tu sistema
BAUD_RATE = 115200

class SimpleSerialReader:
    def __init__(self, puerto_serial=SERIAL_PORT, baud_rate=BAUD_RATE):
        """Inicializar el lector serial simple."""
        self.puerto_serial = puerto_serial
        self.baud_rate = baud_rate
        self.ser = None
        
        # Contadores para estadísticas
        self.total_lines = 0
        self.data_lines = 0
        self.error_lines = 0
        
        # Último RSSI recibido de cada transmisor
        self.last_rssi = {
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
            print(f"📡 Esperando datos...")
            print("-" * 60)
        except Exception as e:
            print(f"❌ Error conectando al puerto serial: {e}")
            print(f"💡 Verifica que el puerto {self.puerto_serial} esté disponible")
            raise
    
    def procesar_linea_data(self, line):
        """Procesa una línea que contiene datos JSON."""
        try:
            # Extraer JSON después de "DATA:"
            data_json = line[5:]  # Quitar "DATA:" del inicio
            data = json.loads(data_json)
            
            # Extraer datos RSSI
            rssi = data.get('rssi', {})
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            # Actualizar últimos valores RSSI
            for tx in ['tx1', 'tx2', 'tx3', 'tx4']:
                if tx in rssi:
                    self.last_rssi[tx] = rssi[tx]
            
            # Mostrar datos en formato legible
            rssi_str = []
            for tx in ['tx1', 'tx2', 'tx3', 'tx4']:
                valor = rssi.get(tx, -100)
                if valor != -100:
                    rssi_str.append(f"{tx.upper()}: {valor:3d}dBm")
                else:
                    rssi_str.append(f"{tx.upper()}: ----")
            
            print(f"[{timestamp}] {' | '.join(rssi_str)}")
            
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
        """Muestra estadísticas de recepción."""
        print("\n" + "="*60)
        print("📊 ESTADÍSTICAS DE RECEPCIÓN")
        print("="*60)
        print(f"Total líneas leídas: {self.total_lines}")
        print(f"Líneas con datos: {self.data_lines}")
        print(f"Líneas con errores: {self.error_lines}")
        print(f"Tasa de éxito: {(self.data_lines/max(1,self.total_lines)*100):.1f}%")
        print()
        print("🎯 ÚLTIMO RSSI RECIBIDO:")
        for tx, rssi in self.last_rssi.items():
            if rssi is not None:
                print(f"  {tx.upper()}: {rssi:3d} dBm")
            else:
                print(f"  {tx.upper()}: Sin datos")
        print("="*60)
    
    def leer_serial_continuo(self):
        """Lee datos del puerto serial de forma continua."""
        try:
            print("🔄 Leyendo datos serial (Ctrl+C para detener)...")
            print()
            
            # Tiempo para mostrar estadísticas cada 10 segundos
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
                    
                    # Mostrar estadísticas cada 10 segundos
                    current_time = time.time()
                    if current_time - last_stats_time >= 10:
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
    print("="*60)
    print("🔌 LECTOR SERIAL SIMPLE")
    print("="*60)
    print(f"Puerto: {SERIAL_PORT}")
    print(f"Velocidad: {BAUD_RATE} bps")
    print("="*60)
    
    # Crear el lector serial
    try:
        reader = SimpleSerialReader(SERIAL_PORT, BAUD_RATE)
        
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
