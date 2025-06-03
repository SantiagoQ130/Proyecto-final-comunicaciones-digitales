import serial
import time
from datetime import datetime

# Configuración del puerto serial
SERIAL_PORT = 'COM5'  # Cambia esto según tu sistema
BAUD_RATE = 115200

def leer_serial_basico():
    """Función básica para leer puerto serial y mostrar todo lo que llega."""
    
    print("="*50)
    print("📡 LECTOR SERIAL BÁSICO")
    print("="*50)
    print(f"Puerto: {SERIAL_PORT}")
    print(f"Velocidad: {BAUD_RATE} bps")
    print("="*50)
    print("🔄 Conectando...")
    
    try:
        # Abrir puerto serial
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✅ Conectado a {SERIAL_PORT}")
        print("📝 Mostrando todo lo que se recibe (Ctrl+C para salir):")
        print("-" * 50)
        
        contador_lineas = 0
        
        while True:
            try:
                # Leer línea si hay datos disponibles
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    
                    if line:  # Si no está vacía
                        contador_lineas += 1
                        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        print(f"[{timestamp}] ({contador_lineas:04d}) {line}")
                
                # Pausa pequeña para no saturar
                time.sleep(0.01)
                
            except UnicodeDecodeError:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] ⚠️  Datos con codificación incorrecta")
            
    except serial.SerialException as e:
        print(f"❌ Error del puerto serial: {e}")
        print("💡 Verifica:")
        print(f"   - Que {SERIAL_PORT} sea el puerto correcto")
        print("   - Que el dispositivo esté conectado")
        print("   - Que no esté siendo usado por otro programa")
        
    except KeyboardInterrupt:
        print(f"\n⏹️  Detenido por el usuario")
        print(f"📊 Total de líneas recibidas: {contador_lineas}")
        
    except Exception as e:
        print(f"❌ Error inesperado: {e}")
        
    finally:
        try:
            ser.close()
            print("✅ Puerto serial cerrado")
        except:
            pass
        print("🏁 Programa terminado")

if __name__ == "__main__":
    leer_serial_basico()
