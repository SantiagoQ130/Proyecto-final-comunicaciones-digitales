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
        self.ancho = 4.80
        self.alto = 15.40
        
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
        
        # Parámetros calibrados basados en los datos reales
        # Análisis de los datos: RSSI a 0m varía entre -40 y -53 dBm
        self.rssi_ref = -45  # RSSI promedio de referencia a 1 metro
        self.n = 2.0  # Exponente de pérdida de propagación reducido
        self.offset_distancia = 0.8  # Offset para compensar el modelo
        
        # Filtro de medias móviles para suavizar RSSI
        self.ventana_filtro = 5
        self.historial_rssi = {
            'tx1': deque(maxlen=self.ventana_filtro),
            'tx2': deque(maxlen=self.ventana_filtro),
            'tx3': deque(maxlen=self.ventana_filtro),
            'tx4': deque(maxlen=self.ventana_filtro)
        }
        
        # Posición estimada del dispositivo móvil
        self.posicion_estimada = np.array([self.ancho/2, self.alto/2])
        self.posicion_anterior = np.array([self.ancho/2, self.alto/2])
        self.trayectoria_x = []
        self.trayectoria_y = []
        
        # Parámetros para filtro de movimiento
        self.max_velocidad = 2.0  # m/s máxima velocidad esperada
        self.alpha_filtro = 0.3  # Factor de suavizado (0-1)
        
        # Añadir buffers para promedios
        self.buffer_distancias = {
            'tx1': deque(maxlen=5),
            'tx2': deque(maxlen=5),
            'tx3': deque(maxlen=5),
            'tx4': deque(maxlen=5)
        }
        self.buffer_posicion = deque(maxlen=5)
        self.ultima_posicion_valida = np.array([self.ancho/2, self.alto/2])

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
    
    def filtrar_rssi(self, rssi_values):
        """Aplica filtro de media móvil a los valores RSSI y retorna el promedio."""
        rssi_filtrado = {}
        rssi_promedio = {}
        for tx, valor in rssi_values.items():
            if valor != -100:  # Solo filtrar valores válidos
                self.historial_rssi[tx].append(valor)
                # Calcular media móvil
                if len(self.historial_rssi[tx]) > 0:
                    rssi_filtrado[tx] = np.mean(list(self.historial_rssi[tx]))
                else:
                    rssi_filtrado[tx] = valor
            else:
                rssi_filtrado[tx] = valor
            # Guardar el promedio para cada TX
            rssi_promedio[tx] = np.mean(self.historial_rssi[tx]) if len(self.historial_rssi[tx]) > 0 else valor
        return rssi_filtrado, rssi_promedio
    
    def rssi_a_distancia(self, rssi):
        """
        Convierte RSSI a distancia usando la tabla de rangos proporcionada.
        Si el RSSI está fuera del rango, devuelve 30.0.
        """
        if rssi >= -28 or rssi <= -72 or rssi == -100:
            return 30.0

        # Rango basado en la tabla proporcionada
        tabla = [
            (-38, -45, 0),
            (-45, -55, 1),
            (-55, -62, 2),
            (-62, -69, 3),
            (-69, -76, 4),
            (-76, -79, 5),
            (-79, -82, 6)
        ]
        for min_rssi, max_rssi, distancia in tabla:
            if min_rssi >= rssi > max_rssi:
                return distancia

        # Si no cae en ningún rango, devolver 30.0 como distancia inválida
        return 30.0
    
    def interseccion_circulos(self, c1, r1, c2, r2):
        """Calcula los puntos de intersección entre dos círculos."""
        x0, y0 = c1
        x1, y1 = c2
        d = np.hypot(x1 - x0, y1 - y0)
        if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
            # No hay intersección o círculos concéntricos
            return []
        # Distancia desde c1 al punto medio entre intersecciones
        a = (r1**2 - r2**2 + d**2) / (2 * d)
        h = np.sqrt(max(0, r1**2 - a**2))
        xm = x0 + a * (x1 - x0) / d
        ym = y0 + a * (y1 - y0) / d
        xs1 = xm + h * (y1 - y0) / d
        ys1 = ym - h * (x1 - x0) / d
        xs2 = xm - h * (y1 - y0) / d
        ys2 = ym + h * (x1 - x0) / d
        return [(xs1, ys1), (xs2, ys2)]

    def centroide_intersecciones(self, nodos, radios):
        """Calcula el centroide de todas las intersecciones de los círculos."""
        puntos = []
        n = len(nodos)
        for i in range(n):
            for j in range(i+1, n):
                pts = self.interseccion_circulos(nodos[i], radios[i], nodos[j], radios[j])
                puntos.extend(pts)
        if puntos:
            puntos = np.array(puntos)
            return np.mean(puntos, axis=0)
        else:
            # Si no hay intersección, usar centroide de los centros de los círculos
            return np.mean(nodos, axis=0)

    def trilateracion_mejorada(self, distancias):
        """Trilateración basada en intersección de círculos (modelo geométrico)."""
        nodos = self.nodos_fijos
        radios = np.array([d if d < 30.0 else 30.0 for d in distancias])
        # Solo usar nodos con radios válidos
        nodos_validos = []
        radios_validos = []
        for i, r in enumerate(radios):
            if r < 30.0:
                nodos_validos.append(nodos[i])
                radios_validos.append(r)
        if len(nodos_validos) < 2:
            # No se puede estimar, mantener posición anterior
            return self.posicion_anterior
        # Calcular centroide de intersecciones
        pos = self.centroide_intersecciones(nodos_validos, radios_validos)
        # Aplicar filtro de movimiento
        return self.aplicar_filtro_movimiento(pos)

    def aplicar_filtro_movimiento(self, nueva_posicion):
        """Aplica filtro de movimiento para evitar saltos bruscos."""
        if self.posicion_anterior is None:
            return nueva_posicion
        
        # Calcular distancia del movimiento
        distancia_movimiento = np.linalg.norm(nueva_posicion - self.posicion_anterior)
        
        # Si el movimiento es muy grande, aplicar filtro más fuerte
        if distancia_movimiento > self.max_velocidad * 0.2:  # 0.2s es el intervalo de muestreo
            # Filtro exponencial más agresivo para movimientos grandes
            factor_filtro = self.alpha_filtro * 0.5
        else:
            factor_filtro = self.alpha_filtro
        
        # Aplicar filtro exponencial
        posicion_filtrada = (factor_filtro * nueva_posicion + 
                           (1 - factor_filtro) * self.posicion_anterior)
        
        # Asegurar que la posición esté dentro de los límites
        posicion_filtrada[0] = np.clip(posicion_filtrada[0], 0.3, self.ancho - 0.3)
        posicion_filtrada[1] = np.clip(posicion_filtrada[1], 0.3, self.alto - 0.3)
        
        return posicion_filtrada
    
    def promedio_distancias(self, nueva_distancia, tx):
        """Calcula promedio móvil de distancias"""
        self.buffer_distancias[tx].append(nueva_distancia)
        return np.mean(self.buffer_distancias[tx])

    def suavizar_trayectoria(self, nueva_pos):
        """Suaviza la trayectoria usando promedio móvil y limitación de velocidad"""
        self.buffer_posicion.append(nueva_pos)
        pos_promedio = np.mean(self.buffer_posicion, axis=0)
        
        # Limitar velocidad máxima
        delta = pos_promedio - self.ultima_posicion_valida
        dist = np.linalg.norm(delta)
        if dist > self.max_velocidad * 0.1:  # 0.1s entre actualizaciones
            pos_promedio = self.ultima_posicion_valida + (delta/dist) * self.max_velocidad * 0.1
        
        self.ultima_posicion_valida = pos_promedio
        return pos_promedio

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
        self.ax_mapa.set_title('🏢 Mapa Interior - Localización en Tiempo Real (Calibrado)', fontsize=14, fontweight='bold')
        
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
                           alpha=0.3, linewidth=2, zorder=3)
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
        self.ax_rssi.set_title("📡 RSSI Filtrado en Tiempo Real", fontsize=12, fontweight='bold')
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
        # Agregar etiquetas de texto para cada barra
        self.bar_labels = []
        for bar in self.bars:
            label = self.ax_stats.text(
                bar.get_x() + bar.get_width() / 2, 0, "0", 
                ha='center', va='bottom', fontsize=10, color='white', fontweight='bold'
            )
            self.bar_labels.append(label)
        # Eliminar el promedio de la tasa de recepción
        # (No se crea ni usa self.rx_rate_history)

    def configurar_distancias(self):
        """Configura la gráfica de distancias."""
        self.dist_line1, = self.ax_distancias.plot([], [], label='Dist TX1', color='#FF6B6B', linewidth=2)
        self.dist_line2, = self.ax_distancias.plot([], [], label='Dist TX2', color='#4ECDC4', linewidth=2)
        self.dist_line3, = self.ax_distancias.plot([], [], label='Dist TX3', color='#45B7D1', linewidth=2)
        self.dist_line4, = self.ax_distancias.plot([], [], label='Dist TX4', color='#FFA07A', linewidth=2)
        
        self.ax_distancias.set_ylim(0, 35)
        self.ax_distancias.set_xlim(0, MAX_POINTS)
        self.ax_distancias.set_ylabel("Distancia (m)", fontsize=10)
        self.ax_distancias.set_xlabel("Tiempo", fontsize=10)
        self.ax_distancias.set_title("📏 Distancias Calculadas (Calibradas)", fontsize=12, fontweight='bold')
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
                    
                    # Aplicar filtro de media móvil a RSSI
                    rssi_raw = {
                        'tx1': rssi.get('tx1', -100),
                        'tx2': rssi.get('tx2', -100),
                        'tx3': rssi.get('tx3', -100),
                        'tx4': rssi.get('tx4', -100)
                    }
                    
                    rssi_filtrado, rssi_promedio = self.filtrar_rssi(rssi_raw)
                    
                    # Actualizar datos de RSSI con valores filtrados
                    self.timestamps.append(current_time)
                    self.tx1_rssi.append(rssi_filtrado['tx1'])
                    self.tx2_rssi.append(rssi_filtrado['tx2'])
                    self.tx3_rssi.append(rssi_filtrado['tx3'])
                    self.tx4_rssi.append(rssi_filtrado['tx4'])
                    
                    # Calcular distancias usando el promedio de RSSI
                    dist1 = self.rssi_a_distancia(rssi_promedio['tx1'])
                    dist2 = self.rssi_a_distancia(rssi_promedio['tx2'])
                    dist3 = self.rssi_a_distancia(rssi_promedio['tx3'])
                    dist4 = self.rssi_a_distancia(rssi_promedio['tx4'])
                    
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
                            self.bar_labels[i].set_text(str(self.packet_counts[tx]))
                            self.bar_labels[i].set_x(self.bars[i].get_x() + self.bars[i].get_width() / 2)
                            self.bar_labels[i].set_y(rate + 0.2)
                            self.packet_counts[tx] = 0
                        self.last_stats_update = current_time
                    
                    # Calcular distancias promediadas
                    distancias = [
                        self.promedio_distancias(dist1, 'tx1'),
                        self.promedio_distancias(dist2, 'tx2'),
                        self.promedio_distancias(dist3, 'tx3'),
                        self.promedio_distancias(dist4, 'tx4')
                    ]
                    
                    # Calcular posición y suavizar trayectoria
                    nueva_posicion = self.trilateracion_mejorada(distancias)
                    pos_suavizada = self.suavizar_trayectoria(nueva_posicion)
                    
                    # Actualizar posiciones con valores suavizados
                    self.posicion_anterior = self.posicion_estimada.copy()
                    self.posicion_estimada = pos_suavizada
                    
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
                        if dist < 30:  # Solo mostrar círculos para distancias válidas
                            self.circulos_distancia[i].set_center(nodo)
                            self.circulos_distancia[i].set_radius(dist)
                            self.circulos_distancia[i].set_alpha(0.3)
                        else:
                            self.circulos_distancia[i].set_alpha(0.0)  # Ocultar círculos inválidos
                    
                    # Mostrar información en consola con valores filtrados
                    print(f"Pos: X={self.posicion_estimada[0]:.2f}m, Y={self.posicion_estimada[1]:.2f}m | "
                          f"RSSI: [{rssi_filtrado['tx1']:.1f}, {rssi_filtrado['tx2']:.1f}, {rssi_filtrado['tx3']:.1f}, {rssi_filtrado['tx4']:.1f}] | "
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
        print("🏢 SISTEMA DE MONITOREO RSSI CON LOCALIZACIÓN INTERIOR - VERSIÓN CALIBRADA")
        print("="*80)
        print(f"📡 Puerto Serial: {self.puerto_serial} @ {self.baud_rate} bps")
        print(f"⏱️  Intervalo de muestreo: 200ms (5 Hz)")
        print(f"📊 Historial máximo: {MAX_POINTS} puntos ({MAX_POINTS * 0.2:.1f}s)")
        print(f"🎯 Transmisores: TX1, TX2, TX3, TX4")
        print(f"🏢 Dimensiones del edificio: {self.ancho}m x {self.alto}m")
        print(f"📏 Parámetros calibrados: RSSI_ref={self.rssi_ref}dBm, n={self.n}, offset={self.offset_distancia}m")
        print(f"🔧 Filtro RSSI: Media móvil de {self.ventana_filtro} muestras")
        print(f"🚀 Filtro movimiento: α={self.alpha_filtro}, v_max={self.max_velocidad}m/s")
        print("="*80)
        print("💡 Mejoras implementadas:")
        print("   ✅ Modelo RSSI→Distancia calibrado con datos reales")
        print("   ✅ Filtro de media móvil para suavizar RSSI")
        print("   ✅ Trilateración mejorada con múltiples puntos de inicio")
        print("   ✅ Filtro de movimiento adaptativo")
        print("   ✅ Manejo robusto de errores y desconexiones")
        print("="*80)
        print("⏹️  Presiona Ctrl+C para detener el sistema\n")

def main():
    """Función principal del programa."""
    monitor = None
    try:
        monitor = MonitorRSSIConMapa()
        monitor.print_system_info()
        ani = animation.FuncAnimation(
            monitor.fig, monitor.update_plot, interval=100,
            blit=False, cache_frame_data=False
        )
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        print("\n🛑 Detenido por el usuario")
    except Exception as e:
        print(f"\n❌ Error crítico: {e}")
        print("🔧 Verifica el puerto serial, conexión y dependencias.")
    finally:
        if monitor and hasattr(monitor, 'ser') and monitor.ser and monitor.ser.is_open:
            monitor.ser.close()
            print("🔌 Puerto serial cerrado")

if __name__ == "__main__":
    main()
