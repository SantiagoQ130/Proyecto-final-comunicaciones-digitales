import serial
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

# Configuración del puerto serial
SERIAL_PORT = 'COM22'  # Cambia esto según tu sistema
BAUD_RATE = 115200

# Tamaño máximo del historial de datos (ajustado para 200ms)
MAX_POINTS = 250  # 50 segundos de datos (250 * 0.2s)

# Crear colas de datos
timestamps = deque(maxlen=MAX_POINTS)
tx1_rssi = deque(maxlen=MAX_POINTS)
tx2_rssi = deque(maxlen=MAX_POINTS)
tx3_rssi = deque(maxlen=MAX_POINTS)
tx4_rssi = deque(maxlen=MAX_POINTS)

# Estadísticas en tiempo real
stats = {
    'tx1': {'count': 0, 'last_update': 0},
    'tx2': {'count': 0, 'last_update': 0},
    'tx3': {'count': 0, 'last_update': 0},
    'tx4': {'count': 0, 'last_update': 0}
}

# Inicializar la gráfica con mejor diseño
plt.style.use('dark_background')
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# Gráfica principal de RSSI
line1, = ax1.plot([], [], label='TX1', color='#FF6B6B', linewidth=2, marker='o', markersize=2)
line2, = ax1.plot([], [], label='TX2', color='#4ECDC4', linewidth=2, marker='s', markersize=2)
line3, = ax1.plot([], [], label='TX3', color='#45B7D1', linewidth=2, marker='^', markersize=2)
line4, = ax1.plot([], [], label='TX4', color='#FFA07A', linewidth=2, marker='d', markersize=2)

ax1.set_ylim(-110, -20)
ax1.set_xlim(0, MAX_POINTS)
ax1.set_ylabel("RSSI (dBm)", fontsize=12)
ax1.set_title("RSSI en Tiempo Real - Sistema de 4 Transmisores (200ms)", fontsize=14, fontweight='bold')
ax1.legend(loc='upper right', framealpha=0.9)
ax1.grid(True, alpha=0.3)

# Gráfica de estadísticas
bars = ax2.bar(['TX1', 'TX2', 'TX3', 'TX4'], [0, 0, 0, 0], 
               color=['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A'], alpha=0.7)
ax2.set_ylabel("Paquetes/seg", fontsize=12)
ax2.set_title("Tasa de Recepción de Datos", fontsize=12)
ax2.set_ylim(0, 10)

# Variables para cálculo de estadísticas
last_stats_update = time.time()
packet_counts = {'tx1': 0, 'tx2': 0, 'tx3': 0, 'tx4': 0}

def update_plot(frame):
    global last_stats_update, packet_counts
    
    try:
        # Leer múltiples líneas para no quedarse atrás
        lines_read = 0
        while ser.in_waiting > 0 and lines_read < 10:  # Limitar para evitar bloqueos
            line = ser.readline().decode('utf-8').strip()
            lines_read += 1
            
            if line.startswith("DATA:"):
                data_json = line[5:]
                data = json.loads(data_json)
                rssi = data['rssi']
                current_time = time.time()
                
                # Actualizar datos de RSSI
                timestamps.append(current_time)
                tx1_rssi.append(rssi.get('tx1', -100))
                tx2_rssi.append(rssi.get('tx2', -100))
                tx3_rssi.append(rssi.get('tx3', -100))
                tx4_rssi.append(rssi.get('tx4', -100))
                
                # Contar paquetes para estadísticas
                for tx in ['tx1', 'tx2', 'tx3', 'tx4']:
                    if rssi.get(tx, -100) != -100:
                        packet_counts[tx] += 1
                
                # Actualizar estadísticas cada segundo
                if current_time - last_stats_update >= 1.0:
                    for i, tx in enumerate(['tx1', 'tx2', 'tx3', 'tx4']):
                        rate = packet_counts[tx] / (current_time - last_stats_update)
                        bars[i].set_height(rate)
                        packet_counts[tx] = 0
                    last_stats_update = current_time
                
            elif line.startswith("SETUP:"):
                print(f"Sistema iniciado: {line}")
            elif line.startswith("DEBUG:"):
                print(f"Debug: {line}")
        
        # Actualizar gráfica principal solo si hay datos
        if len(timestamps) > 0:
            # Usar timestamps relativos para mejor visualización
            if len(timestamps) > 1:
                start_time = timestamps[0]
                x_vals = [(t - start_time) for t in timestamps]
            else:
                x_vals = [0]
            
            line1.set_data(x_vals, tx1_rssi)
            line2.set_data(x_vals, tx2_rssi)
            line3.set_data(x_vals, tx3_rssi)
            line4.set_data(x_vals, tx4_rssi)
            
            # Ajustar ventana de tiempo
            if len(x_vals) > 0:
                ax1.set_xlim(max(0, x_vals[-1] - 50), x_vals[-1] + 2)  # Ventana de 50 segundos
                ax1.set_xlabel(f"Tiempo (s) - Último: {x_vals[-1]:.1f}s", fontsize=10)
        
    except json.JSONDecodeError as e:
        print(f"Error JSON: {e}")
    except serial.SerialException as e:
        print(f"Error Serial: {e}")
    except Exception as e:
        print(f"Error general: {e}")
    
    return line1, line2, line3, line4

def print_system_info():
    print("="*60)
    print("🚀 SISTEMA DE MONITOREO RSSI INICIADO")
    print("="*60)
    print(f"📡 Puerto Serial: {SERIAL_PORT} @ {BAUD_RATE} bps")
    print(f"⏱️  Intervalo de muestreo: 200ms (5 Hz)")
    print(f"📊 Historial máximo: {MAX_POINTS} puntos ({MAX_POINTS * 0.2:.1f}s)")
    print(f"🎯 Transmisores monitoreados: TX1, TX2, TX3, TX4")
    print("="*60)
    print("💡 Comandos disponibles:")
    print("   - Cerrar ventana para terminar")
    print("   - Los datos se muestran en tiempo real")
    print("="*60)

# Inicialización
try:
    print_system_info()
    print("🔌 Conectando al puerto serial...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  # Timeout reducido
    print("✅ Conexión serial establecida")
    
    # Ejecutar la animación con intervalo ajustado
    print("🎬 Iniciando visualización...")
    ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=False)  # 100ms para suavidad
    
    plt.tight_layout()
    plt.show()
    
except serial.SerialException as e:
    print(f"❌ Error de conexión serial: {e}")
    print(f"💡 Verifica que el puerto {SERIAL_PORT} esté correcto y disponible")
except KeyboardInterrupt:
    print("\n⏹️  Deteniendo por interrupción del usuario...")
finally:
    try:
        ser.close()
        print("✅ Puerto serial cerrado correctamente")
    except:
        pass
    print("🏁 Programa terminado")
