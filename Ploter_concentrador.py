import serial
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Configura el puerto serial (ajusta el nombre del puerto y velocidad según tu caso)
SERIAL_PORT = 'COM22'  # Cambia a tu puerto, por ejemplo '/dev/ttyUSB0' en Linux
BAUD_RATE = 115200

# Historial limitado de datos
MAX_POINTS = 100
timestamps = deque(maxlen=MAX_POINTS)
tx1_rssi = deque(maxlen=MAX_POINTS)
tx2_rssi = deque(maxlen=MAX_POINTS)
tx3_rssi = deque(maxlen=MAX_POINTS)

# Inicializar gráfico
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='TX1', color='red')
line2, = ax.plot([], [], label='TX2', color='green')
line3, = ax.plot([], [], label='TX3', color='blue')

ax.set_ylim(-110, -20)
ax.set_xlim(0, MAX_POINTS)
ax.set_ylabel("RSSI (dBm)")
ax.set_xlabel("Muestras")
ax.legend()
plt.title("RSSI en Tiempo Real")

def update_plot(frame):
    """Función de animación que actualiza la gráfica en tiempo real."""
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

            # Actualiza las líneas de la gráfica
            line1.set_data(range(len(tx1_rssi)), tx1_rssi)
            line2.set_data(range(len(tx2_rssi)), tx2_rssi)
            line3.set_data(range(len(tx3_rssi)), tx3_rssi)

            ax.set_xlim(max(0, len(tx1_rssi) - MAX_POINTS), len(tx1_rssi))
    except Exception as e:
        print("Error leyendo línea:", e)

    return line1, line2, line3

# Iniciar conexión serial
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Animación en tiempo real
ani = animation.FuncAnimation(fig, update_plot, interval=200)
plt.show()

# Cerrar serial al salir (opcional, si necesitas)
ser.close()
