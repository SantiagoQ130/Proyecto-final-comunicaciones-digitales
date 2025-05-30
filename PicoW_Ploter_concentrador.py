import serial
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
from collections import deque
import time
import numpy as np
from scipy.optimize import minimize

# Configuración del puerto serial
SERIAL_PORT = 'COM22'  # Cambia esto según tu sistema
BAUD_RATE = 115200

# Tamaño máximo del historial de datos
MAX_POINTS = 250  # 50 segundos de datos (250 * 0.2s)

class MonitorRSSIConMapa:
    def __init__(self, puerto_serial=SERIAL_PORT, baud_rate=BAUD_RATE):
        # Configuración del puerto serial
        self.puerto_serial = puerto_serial
        self.baud_rate = baud_rate
        self.ser = None
        
        # Dimensiones del espacio interior (en metros)
        self.ancho = 20
        self.alto = 15
        
        # Crear colas de datos para RSSI
        self.timestamps = deque(maxlen=MAX_POINTS)
        self.tx1_rssi = deque(maxlen=MAX_POINTS)
        self.tx2_rssi = deque(maxlen=MAX_POINTS)
        self.tx3_rssi = deque(maxlen=MAX_POINTS)
        self.tx4_rssi = deque(maxlen=MAX_POINTS)
        
        # Estadísticas en tiempo real
        self.stats = {
            'tx1': {'count': 0, 'last_update': 0},
            'tx2': {'count': 0, 'last_update': 0},
            'tx3': {'count': 0, 'last_update': 0},
            'tx4': {'count': 0, 'last_update': 0}
        }
        
        # Variables para cálculo de estadísticas
        self.last_stats_update = time.time()
        self.packet_counts = {'tx1': 0, 'tx2': 0, 'tx3': 0, 'tx4': 0}
        
        # Definir paredes y habitaciones
        self.paredes = [
            # Paredes exteriores
            Rectangle((0, 0), self.ancho, 0.2, color='gray'),  # Pared inferior
            Rectangle((0, 0), 0.2, self.alto, color='gray'),   # Pared izquierda
            Rectangle((0, self.alto-0.2), self.ancho, 0.2, color='gray'),  # Pared superior
            Rectangle((self.ancho-0.2, 0), 0.2, self.alto, color='gray'),  # Pared derecha
            
            # Paredes interiores (habitaciones)
            Rectangle((2, 2), 5, 4, color='lightgray'),  # Habitación 1
            Rectangle((9, 2), 5, 4, color='lightgray'),  # Habitación 2
            Rectangle((2, 9), 5, 4, color='lightgray'),  # Habitación 3
            Rectangle((9, 9), 5, 4, color='lightgray'),  # Habitación 4
            
            # Pared central
            Rectangle((7, 2), 0.2, 11, color='gray'),  # Pared vertical central
            Rectangle((2, 7), 16, 0.2, color='gray'),  # Pared horizontal central
        ]
        
        # Posiciones de los transmisores (nodos fijos)
        self.nodos_fijos = np.array([
            [1, 1],                    # TX1 - Esquina inferior izquierda
            [self.ancho-1, 1],         # TX2 - Esquina inferior derecha
            [self.ancho-1, self.alto-1], # TX3 - Esquina superior derecha
            [1, self.alto-1]           # TX4 - Esquina superior izquierda
        ])
        
        # Etiquetas para los nodos
        self.etiquetas_nodos = ['TX1', 'TX2', 'TX3', 'TX4']
        
        # Parámetros para conversión RSSI a distancia
        self.rssi_ref = -40  # RSSI de referencia a 1 metro (dBm)
        self.n = 2.5  # Exponente de pérdida de propagación para interiores
        
        # Posición estimada del dispositivo móvil
        self.posicion_estimada = np.array([self.ancho/2, self.alto/2])
        self.trayectoria_x = []
        self.trayectoria_y = []
        
        # Inicializar puerto serial
        self.inicializar_serial()
        
        # Crear la interfaz
        self.crear_interfaz()
    
    def inicializar_serial(self):
        """Inicializa la conexión serial."""
        try:
            self.ser = serial.Serial(self.puerto_serial, self.baud_rate, timeout=0.1)
            print(f"✅ Puerto serial {self.puerto_serial} abierto correctamente.")
        except Exception as e:
            print(f"❌ Error abriendo puerto serial: {e}")
            raise
    
    def rssi_a_distancia(self, rssi):
        """Convierte RSSI a distancia usando el modelo de propagación."""
        if rssi >= 0 or rssi == -100:  # Valores inválidos
            return 25.0  # Distancia por defecto
        
        # Fórmula: d = 10^((RSSI_ref - RSSI) / (10 * n))
        distancia = 10 ** ((self.rssi_ref - rssi) / (10 * self.n))
        
        # Limitar distancia mínima y máxima
        distancia = max(0.5, min(distancia, 50.0))
        return distancia
    
    def trilateracion(self, distancias):
        """Calcula la posición usando trilateración."""
        if len([d for d in distancias if d < 25.0]) < 3:  # Necesitamos al menos 3 distancias válidas
            return self.posicion_estimada
        
        def error_function(pos):
            """Función de error para minimizar."""
            x, y = pos
            error = 0
            for i, dist in enumerate(distancias):
                if dist < 25.0:  # Solo usar distancias válidas
                    dist_calculada = np.sqrt((x - self.nodos_fijos[i][0])**2 + 
                                           (y - self.nodos_fijos[i][1])**2)
                    error += (dist_calculada - dist)**2
            return error
        
        # Posición inicial (centro del edificio)
        pos_inicial = [self.ancho/2, self.alto/2]
        
        # Restricciones de posición (dentro del edificio)
        bounds = [(0.5, self.ancho-0.5), (0.5, self.alto-0.5)]
        
        try:
            resultado = minimize(error_function, pos_inicial, bounds=bounds, method='L-BFGS-B')
            if resultado.success:
                return np.array(resultado.x)
        except:
            pass
        
        return self.posicion_estimada
    
    def crear_interfaz(self):
        """Crea la interfaz gráfica completa."""
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(20, 12))
        
        # Crear grid de subplots
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # Subplot principal - Mapa (ocupa 2x2)
        self.ax_mapa = self.fig.add_subplot(gs[:2, :2])
        
        # Subplot de RSSI en tiempo real
        self.ax_rssi = self.fig.add_subplot(gs[0, 2])
        
        # Subplot de estadísticas
        self.ax_stats = self.fig.add_subplot(gs[1, 2])
        
        # Subplot de distancias
        self.ax_distancias = self.fig.add_subplot(gs[2, :])
        
        self.configurar_mapa()
        self.configurar_graficas_rssi()
        self.configurar_estadisticas()
        self.configurar_distancias()
    
    def configurar_mapa(self):
        """Configura el mapa interior."""
        self.ax_mapa.set_xlim(0, self.ancho)
        self.ax_mapa.set_ylim(0, self.alto)
        self.ax_mapa.set_xlabel('X (metros)', fontsize=12)
        self.ax_mapa.set_ylabel('Y (metros)', fontsize=12)
        self.ax_mapa.set_title('🏢 Mapa Interior - Localización en Tiempo Real', fontsize=14, fontweight='bold')
        
        # Dibujar paredes y habitaciones
        for pared in self.paredes:
            self.ax_mapa.add_patch(pared)
        
        # Graficar los transmisores
        self.ax_mapa.scatter(self.nodos_fijos[:, 0], self.nodos_fijos[:, 1], 
                           color=['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A'], 
                           s=200, zorder=5, edgecolors='white', linewidth=2)
        
        # Etiquetas para los transmisores
        for i, nodo in enumerate(self.nodos_fijos):
            self.ax_mapa.text(nodo[0] + 0.3, nodo[1] + 0.3, 
                            self.etiquetas_nodos[i], fontsize=12, weight='bold', color='white')
        
        # Inicializar elementos móviles
        self.nodo_movil_plot, = self.ax_mapa.plot([], [], 'yo', 
                                                 markersize=15, zorder=6, 
                                                 markeredgecolor='black', markeredgewidth=2)
        self.trayectoria, = self.ax_mapa.plot([], [], 'yellow', alpha=0.7, linewidth=3, 
                                             zorder=4)
        
        # Círculos de distancia
        self.circulos_distancia = []
        colores_circulos = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A']
        for i in range(len(self.nodos_fijos)):
            circulo = Circle((0, 0), 0, fill=False, color=colores_circulos[i], 
                           alpha=0.4, linewidth=2, zorder=3)
            self.ax_mapa.add_patch(circulo)
            self.circulos_distancia.append(circulo)
        
        self.ax_mapa.grid(True, alpha=0.3)
        self.ax_mapa.set_aspect('equal')
    
    def configurar_graficas_rssi(self):
        """Configura las gráficas de RSSI."""
        # Gráfica de RSSI en tiempo real
        self.line1, = self.ax_rssi.plot([], [], label='TX1', color='#FF6B6B', linewidth=2, marker='o', markersize=2)
        self.line2, = self.ax_rssi.plot([], [], label='TX2', color='#4ECDC4', linewidth=2, marker='s', markersize=2)
        self.line3, = self.ax_rssi.plot([], [], label='TX3', color='#45B7D1', linewidth=2, marker='^', markersize=2)
        self.line4, = self.ax_rssi.plot([], [], label='TX4', color='#FFA07A', linewidth=2, marker='d', markersize=2)
        
        self.ax_rssi.set_ylim(-110, -20)
        self.ax_rssi.set_xlim(0, MAX_POINTS)
        self.ax_rssi.set_ylabel("RSSI (dBm)", fontsize=10)
        self.ax_rssi.set_title("📡 RSSI en Tiempo Real", fontsize=12, fontweight='bold')
        self.ax_rssi.legend(loc='upper right', framealpha=0.9)
        self.ax_rssi.grid(True, alpha=0.3)
    
    def configurar_estadisticas(self):
        """Configura las gráficas de estadísticas."""
        self.bars = self.ax_stats.bar(['TX1', 'TX2', 'TX3', 'TX4'], [0, 0, 0, 0], 
                                     color=['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A'], alpha=0.7)
        self.ax_stats.set_ylabel("Paquetes/seg", fontsize=10)
        self.ax_stats.set_title("📊 Tasa de Recepción", fontsize=12)
        self.ax_stats.set_ylim(0, 10)
        self.ax_stats.grid(True, alpha=0.3)
    
    def configurar_distancias(self):
        """Configura la gráfica de distancias."""
        self.dist_line1, = self.ax_distancias.plot([], [], label='Dist TX1', color='#FF6B6B', linewidth=2)
        self.dist_line2, = self.ax_distancias.plot([], [], label='Dist TX2', color='#4ECDC4', linewidth=2)
        self.dist_line3, = self.ax_distancias.plot([], [], label='Dist TX3', color='#45B7D1', linewidth=2)
        self.dist_line4, = self.ax_distancias.plot([], [], label='Dist TX4', color='#FFA07A', linewidth=2)
        
        self.ax_distancias.set_ylim(0, 30)
        self.ax_distancias.set_xlim(0, MAX_POINTS)
        self.ax_distancias.set_ylabel("Distancia (m)", fontsize=10)
        self.ax_distancias.set_xlabel("Tiempo", fontsize=10)
        self.ax_distancias.set_title("📏 Distancias Calculadas", fontsize=12, fontweight='bold')
        self.ax_distancias.legend(loc='upper right', framealpha=0.9)
        self.ax_distancias.grid(True, alpha=0.3)
        
        # Deques para distancias
        self.dist1 = deque(maxlen=MAX_POINTS)
        self.dist2 = deque(maxlen=MAX_POINTS)
        self.dist3 = deque(maxlen=MAX_POINTS)
        self.dist4 = deque(maxlen=MAX_POINTS)
    
    def update_plot(self, frame):
        """Actualiza todas las gráficas."""
        try:
            # Leer múltiples líneas para no quedarse atrás
            lines_read = 0
            while self.ser.in_waiting > 0 and lines_read < 10:
                line = self.ser.readline().decode('utf-8').strip()
                lines_read += 1
                
                if line.startswith("DATA:"):
                    data_json = line[5:]
                    data = json.loads(data_json)
                    rssi = data['rssi']
                    current_time = time.time()
                    
                    # Actualizar datos de RSSI
                    self.timestamps.append(current_time)
                    rssi_tx1 = rssi.get('tx1', -100)
                    rssi_tx2 = rssi.get('tx2', -100)
                    rssi_tx3 = rssi.get('tx3', -100)
                    rssi_tx4 = rssi.get('tx4', -100)
                    
                    self.tx1_rssi.append(rssi_tx1)
                    self.tx2_rssi.append(rssi_tx2)
                    self.tx3_rssi.append(rssi_tx3)
                    self.tx4_rssi.append(rssi_tx4)
                    
                    # Calcular distancias
                    dist1 = self.rssi_a_distancia(rssi_tx1)
                    dist2 = self.rssi_a_distancia(rssi_tx2)
                    dist3 = self.rssi_a_distancia(rssi_tx3)
                    dist4 = self.rssi_a_distancia(rssi_tx4)
                    
                    self.dist1.append(dist1)
                    self.dist2.append(dist2)
                    self.dist3.append(dist3)
                    self.dist4.append(dist4)
                    
                    # Contar paquetes para estadísticas
                    for tx in ['tx1', 'tx2', 'tx3', 'tx4']:
                        if rssi.get(tx, -100) != -100:
                            self.packet_counts[tx] += 1
                    
                    # Actualizar estadísticas cada segundo
                    if current_time - self.last_stats_update >= 1.0:
                        for i, tx in enumerate(['tx1', 'tx2', 'tx3', 'tx4']):
                            rate = self.packet_counts[tx] / (current_time - self.last_stats_update)
                            self.bars[i].set_height(rate)
                            self.packet_counts[tx] = 0
                        self.last_stats_update = current_time
                    
                    # Calcular posición usando trilateración
                    distancias = [dist1, dist2, dist3, dist4]
                    nueva_posicion = self.trilateracion(distancias)
                    self.posicion_estimada = nueva_posicion
                    
                    # Actualizar posición en el mapa
                    self.nodo_movil_plot.set_data([self.posicion_estimada[0]], [self.posicion_estimada[1]])
                    
                    # Actualizar trayectoria
                    self.trayectoria_x.append(self.posicion_estimada[0])
                    self.trayectoria_y.append(self.posicion_estimada[1])
                    
                    # Mantener solo los últimos puntos de la trayectoria
                    if len(self.trayectoria_x) > MAX_POINTS:
                        self.trayectoria_x.pop(0)
                        self.trayectoria_y.pop(0)
                    
                    self.trayectoria.set_data(self.trayectoria_x, self.trayectoria_y)
                    
                    # Actualizar círculos de distancia en el mapa
                    for i, (dist, nodo) in enumerate(zip(distancias, self.nodos_fijos)):
                        self.circulos_distancia[i].set_center(nodo)
                        self.circulos_distancia[i].set_radius(dist)
                    
                    # Mostrar información en consola
                    print(f"Pos: X={self.posicion_estimada[0]:.1f}m, Y={self.posicion_estimada[1]:.1f}m | "
                          f"RSSI: [{rssi_tx1}, {rssi_tx2}, {rssi_tx3}, {rssi_tx4}] | "
                          f"Dist: [{dist1:.1f}, {dist2:.1f}, {dist3:.1f}, {dist4:.1f}]m")
                
                elif line.startswith("SETUP:"):
                    print(f"🚀 {line}")
                elif line.startswith("DEBUG:"):
                    print(f"🔧 {line}")
            
            # Actualizar gráficas solo si hay datos
            if len(self.timestamps) > 0:
                # Usar índices relativos para mejor visualización
                x_vals = list(range(len(self.tx1_rssi)))
                
                # Actualizar líneas RSSI
                self.line1.set_data(x_vals, self.tx1_rssi)
                self.line2.set_data(x_vals, self.tx2_rssi)
                self.line3.set_data(x_vals, self.tx3_rssi)
                self.line4.set_data(x_vals, self.tx4_rssi)
                
                # Actualizar líneas de distancia
                self.dist_line1.set_data(x_vals, self.dist1)
                self.dist_line2.set_data(x_vals, self.dist2)
                self.dist_line3.set_data(x_vals, self.dist3)
                self.dist_line4.set_data(x_vals, self.dist4)
                
                # Ajustar ventanas de tiempo
                if len(x_vals) > 0:
                    window_start = max(0, len(x_vals) - MAX_POINTS)
                    self.ax_rssi.set_xlim(window_start, len(x_vals))
                    self.ax_distancias.set_xlim(window_start, len(x_vals))
        
        except json.JSONDecodeError as e:
            print(f"❌ Error JSON: {e}")
        except serial.SerialException as e:
            print(f"❌ Error Serial: {e}")
        except Exception as e:
            print(f"❌ Error general: {e}")
        
        return [self.line1, self.line2, self.line3, self.line4, 
                self.nodo_movil_plot, self.trayectoria,
                self.dist_line1, self.dist_line2, self.dist_line3, self.dist_line4]
    
    def print_system_info(self):
        """Imprime información del sistema."""
        print("="*80)
        print("🏢 SISTEMA DE MONITOREO RSSI CON LOCALIZACIÓN INTERIOR")
        print("="*80)
        print(f"📡 Puerto Serial: {self.puerto_serial} @ {self.baud_rate} bps")
        print(f"⏱️  Intervalo de muestreo: 200ms (5 Hz)")
        print(f"📊 Historial máximo: {MAX_POINTS} puntos ({MAX_POINTS * 0.2:.1f}s)")
        print(f"🎯 Transmisores: TX1, TX2, TX3, TX4")
        print(f"🏢 Dimensiones del edificio: {self.ancho}m x {self.alto}m")
        print(f"📏 Parámetros RSSI→Distancia: RSSI_ref={self.rssi_ref}dBm, n={self.n}")
        print("="*80)
        print("💡 Funcionalidades:")
        print("   ✅ Monitoreo RSSI en tiempo real")
        print("   ✅ Cálculo automático de distancias")
        print("   ✅ Localización por trilateración")
        print("   ✅ Mapa interior con trayectoria")
        print("   ✅ Estadísticas de recepción")
        print("="*80)
    
    def iniciar(self):
        """Inicia el monitor completo."""
        try:
            self.print_system_info()
            print("🔌 Iniciando conexión serial...")
            print("🎬 Iniciando visualización...")
            
            # Ejecutar la animación
            ani = animation.FuncAnimation(self.fig, self.update_plot, 
                                        interval=100, blit=False)
            
            plt.tight_layout()
            plt.show()
            
        except serial.SerialException as e:
            print(f"❌ Error de conexión serial: {e}")
            print(f"💡 Verifica que el puerto {self.puerto_serial} esté correcto y disponible")
        except KeyboardInterrupt:
            print("\n⏹️  Deteniendo por interrupción del usuario...")
        finally:
            try:
                if self.ser:
                    self.ser.close()
                    print("✅ Puerto serial cerrado correctamente")
            except:
                pass
            print("🏁 Programa terminado")

def main():
    """Función principal."""
    # Crear y ejecutar el monitor
    try:
        monitor = MonitorRSSIConMapa(
            puerto_serial=SERIAL_PORT,
            baud_rate=BAUD_RATE
        )
        monitor.iniciar()
    except Exception as e:
        print(f"❌ Error inicializando el monitor: {e}")

if __name__ == "__main__":
    main()
