import serial
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Configuración del puerto serial
SERIAL_PORT = 'COM22'  # Cambia esto según tu sistema
BAUD_RATE = 115200

# Tamaño máximo del historial de datos
MAX_POINTS = 100

# Crear colas de datos
timestamps = deque(maxlen=MAX_POINTS)
tx1_rssi = deque(maxlen=MAX_POINTS)
tx2_rssi = deque(maxlen=MAX_POINTS)
tx3_rssi = deque(maxlen=MAX_POINTS)
tx4_rssi = deque(maxlen=MAX_POINTS)

# Inicializar la gráfica
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='TX1', color='red')
line2, = ax.plot([], [], label='TX2', color='green')
line3, = ax.plot([], [], label='TX3', color='blue')
line4, = ax.plot([], [], label='TX4', color='orange')

ax.set_ylim(-110, -20)
ax.set_xlim(0, MAX_POINTS)
ax.set_ylabel("RSSI (dBm)")
ax.set_xlabel("Muestras")
ax.legend(loc='upper right')
plt.title("RSSI en Tiempo Real - TX1 a TX4")

def update_plot(frame):
    try:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith("DATA:"):
            data_json = line[5:]
            data = json.loads(data_json)
            rssi = data['rssi']
            timestamps.append(data['timestamp'])
            tx1_rssi.append(rssi.get('tx1', -100))
            tx2_rssi.append(rssi.get('tx2', -100))
            tx3_rssi.append(rssi.get('tx3', -100))
            tx4_rssi.append(rssi.get('tx4', -100))

            # Actualizar datos de las líneas
            x_vals = range(len(timestamps))
            line1.set_data(x_vals, tx1_rssi)
            line2.set_data(x_vals, tx2_rssi)
            line3.set_data(x_vals, tx3_rssi)
            line4.set_data(x_vals, tx4_rssi)
            ax.set_xlim(max(0, len(timestamps) - MAX_POINTS), len(timestamps))
    except Exception as e:
        print("Error leyendo línea:", e)

    return line1, line2, line3, line4

# Abrir el puerto serial
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Ejecutar la animación
ani = animation.FuncAnimation(fig, update_plot, interval=200)
plt.tight_layout()
plt.show()

# Cerrar el puerto serial al salir (opcional)
ser.close()
