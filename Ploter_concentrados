import re
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import serial
import threading
import sys

# Configuración inicial
PORT = "COM3"  # Cambiar al puerto correcto (en Windows: COM#, en Linux/Mac: /dev/tty...)
BAUD_RATE = 115200

# Variables para almacenar datos
rssi_data = {
    1: {"times": [], "values": []},
    2: {"times": [], "values": []},
    3: {"times": [], "values": []}
}

# Tiempo de inicio para calcular tiempos relativos
start_time = time.time()

# Variables de control
running = True
connected = False
max_points = 100  # Máximo número de puntos a mostrar

# Configuración de la gráfica
plt.style.use('dark_background')
fig, ax = plt.subplots(figsize=(12, 6))
fig.canvas.manager.set_window_title('RSSI Monitor en Tiempo Real')

# Líneas para cada transmisor
line1, = ax.plot([], [], 'r-', label='TX1', linewidth=2)
line2, = ax.plot([], [], 'g-', label='TX2', linewidth=2)
line3, = ax.plot([], [], 'b-', label='TX3', linewidth=2)

# Configuración de ejes
ax.set_xlim(0, 30)  # 30 segundos de ventana
ax.set_ylim(-100, -30)
ax.set_title('Monitoreo de RSSI en Tiempo Real')
ax.set_xlabel('Tiempo (segundos)')
ax.set_ylabel('RSSI (dBm)')
ax.grid(True, linestyle='--', alpha=0.7)
ax.legend(loc='upper right')

# Texto para mostrar valores actuales
current_values_text = ax.text(0.02, 0.02, "", transform=ax.transAxes, 
                           bbox=dict(facecolor='black', alpha=0.7, boxstyle='round'))

def serial_reader():
    """Lee datos del puerto serial en un hilo separado."""
    global connected
    
    try:
        # Configurar puerto serial
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        connected = True
        print(f"Conectado a {PORT} a {BAUD_RATE} baudios")
        
        while running:
            try:
                # Leer una línea del puerto serial
                line = ser.readline().decode('utf-8').strip()
                
                # Procesar la línea si contiene datos
                if line.startswith("DATA:"):
                    # Extraer JSON de la línea
                    json_str = line[5:]  # Quitar el prefijo "DATA:"
                    data = json.loads(json_str)
                    
                    # Obtener el tiempo relativo (segundos desde el inicio)
                    current_time = time.time() - start_time
                    
                    # Guardar datos para cada transmisor
                    rssi_data[1]["times"].append(current_time)
                    rssi_data[1]["values"].append(data["rssi"]["tx1"])
                    
                    rssi_data[2]["times"].append(current_time)
                    rssi_data[2]["values"].append(data["rssi"]["tx2"])
                    
                    rssi_data[3]["times"].append(current_time)
                    rssi_data[3]["values"].append(data["rssi"]["tx3"])
                    
                    # Limitar el número de puntos
                    if len(rssi_data[1]["times"]) > max_points:
                        for tx in range(1, 4):
                            rssi_data[tx]["times"] = rssi_data[tx]["times"][-max_points:]
                            rssi_data[tx]["values"] = rssi_data[tx]["values"][-max_points:]
                
                elif line.startswith("SETUP:"):
                    print("Dispositivo iniciado correctamente")
                
                elif line.startswith("CONFIG:"):
                    print("Configuración recibida")
                
                elif line.startswith("DEBUG:"):
                    print(f"Debug: {line[6:]}")
                    
            except Exception as e:
                print(f"Error al procesar datos: {e}")
                time.sleep(0.1)
        
        # Cerrar puerto serial al terminar
        ser.close()
        print("Puerto serial cerrado")
        
    except Exception as e:
        print(f"Error de conexión: {e}")
        print(f"Asegúrate de que el dispositivo está conectado al puerto {PORT}")
        connected = False

def update_plot(frame):
    """Actualiza la gráfica con los nuevos datos."""
    # Actualizar datos de las líneas
    line1.set_data(rssi_data[1]["times"], rssi_data[1]["values"])
    line2.set_data(rssi_data[2]["times"], rssi_data[2]["values"])
    line3.set_data(rssi_data[3]["times"], rssi_data[3]["values"])
    
    # Ajustar límites del eje X si hay datos
    if rssi_data[1]["times"]:
        current_max = max(rssi_data[1]["times"][-1], 30)  # Al menos 30 segundos
        ax.set_xlim(max(0, current_max - 30), current_max)  # Ventana móvil de 30 segundos
    
    # Actualizar texto con valores actuales
    current_values = "Valores actuales:\n"
    for tx in range(1, 4):
        if rssi_data[tx]["values"]:
            current_values += f"TX{tx}: {rssi_data[tx]['values'][-1]} dBm\n"
        else:
            current_values += f"TX{tx}: -- dBm\n"
    current_values_text.set_text(current_values)
    
    return line1, line2, line3, current_values_text

def on_close(event):
    """Maneja el evento de cierre de la figura."""
    global running
    running = False
    print("Cerrando aplicación...")

def main():
    """Función principal."""
    # Configurar evento al cerrar la ventana
    fig.canvas.mpl_connect('close_event', on_close)
    
    # Iniciar hilo de lectura serial
    serial_thread = threading.Thread(target=serial_reader)
    serial_thread.daemon = True
    serial_thread.start()
    
    # Esperar a que se establezca la conexión
    connection_timeout = 5  # 5 segundos para conectar
    start_wait = time.time()
    while not connected and time.time() - start_wait < connection_timeout:
        time.sleep(0.1)
        
    if not connected:
        print(f"No se pudo conectar al puerto {PORT}. Verifica la conexión y el puerto.")
        print("La visualización continuará pero no recibirá datos.")
    
    # Iniciar animación
    ani = FuncAnimation(fig, update_plot, interval=100, blit=True)
    plt.tight_layout()
    plt.show()
    
    # Esperar a que termine el hilo de lectura
    running = False
    if serial_thread.is_alive():
        serial_thread.join(timeout=1.0)

if __name__ == "__main__":
    print("Iniciando monitor RSSI...")
    print("Presiona Ctrl+C para salir")
    try:
        main()
    except KeyboardInterrupt:
        print("Programa terminado por el usuario")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        running = False
        print("Programa finalizado")
        sys.exit(0)
