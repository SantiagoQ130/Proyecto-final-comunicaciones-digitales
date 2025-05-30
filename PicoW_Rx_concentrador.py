import struct
import utime
import json
from machine import Pin, SPI, I2C
from nrf24l01 import NRF24L01
from ssd1306 import SSD1306_I2C

# Configuración de pines
SPI_ID = 0
SCK_PIN = 2
MOSI_PIN = 3
MISO_PIN = 4
CSN_PIN = 5
CE_PIN = 6
SDA = 14
SCL = 15

# Configuración de pantalla OLED
WIDTH = 128
HEIGHT = 64

# Configuración de canal y direcciones (sincronizadas con transmisores)
CANAL_RF = 10

# Direcciones de transmisores (sincronizadas)
TX_ADDRESSES = {
    1: b"\xe1\xf0\xf0\xf0\xf0",
    2: b"\xc3\xf0\xf0\xf0\xf0", 
    3: b"\xd2\xf0\xf0\xf0\xf0",
    4: b"\xb1\xf0\xf0\xf0\xf0"  # Coincide con TX4
}

# Variables de control de rendimiento
last_rssi = {1: -100, 2: -100, 3: -100, 4: -100}
rssi_buffer = {1: [], 2: [], 3: [], 4: []}  # Buffer para promedio móvil
last_received_time = {1: 0, 2: 0, 3: 0, 4: 0}
packet_count = {1: 0, 2: 0, 3: 0, 4: 0}  # Contador de paquetes

# Configuración de buffers y timeouts (optimizados)
BUFFER_SIZE = 10  # 10 segundos de buffer como los TX
RSSI_TIMEOUT_MS = 15000  # 15 segundos timeout (más agresivo)
UPDATE_INTERVAL_MS = 200  # 200ms = 5Hz (sincronizado con monitor)

# Variables de control de tiempo
last_update_time = 0
last_stats_time = 0

def setup_nrf24l01():
    """Configuración optimizada del NRF24L01 (igual que transmisores)"""
    spi = SPI(SPI_ID, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    csn = Pin(CSN_PIN, mode=Pin.OUT, value=1)
    ce = Pin(CE_PIN, mode=Pin.OUT, value=0)
    
    nrf = NRF24L01(spi, csn, ce, payload_size=8)  # 4 bytes ID + 4 bytes RSSI
    nrf.set_channel(CANAL_RF)
    
    # Configuración idéntica a transmisores para máximo rendimiento
    nrf.reg_write(0x06, 0x0E)  # 2Mbps, máxima potencia (+0dBm)
    nrf.reg_write(0x1C, 0x00)  # Deshabilitar auto-ACK para mayor velocidad
    nrf.reg_write(0x1D, 0x00)  # Sin retransmisiones automáticas
    
    # Abrir pipes de recepción para todos los transmisores
    for pipe, addr in TX_ADDRESSES.items():
        nrf.open_rx_pipe(pipe, addr)
    
    # Verificar comunicación con NRF24L01
    try:
        config_reg = nrf.reg_read(0x00)
        print(f"DEBUG:NRF24L01 RX configurado - Config: 0x{config_reg:02x}")
        return nrf
    except Exception as e:
        print(f"DEBUG:Error NRF24L01 - {e}")
        return None

def setup_oled():
    """Configuración de OLED con manejo de errores"""
    try:
        i2c = I2C(1, scl=Pin(SCL), sda=Pin(SDA))
        oled = SSD1306_I2C(WIDTH, HEIGHT, i2c)
        oled.fill(0)
        oled.text("Iniciando RX...", 0, 20)
        oled.text("NRF24L01", 20, 35)
        oled.show()
        print("DEBUG:OLED inicializado correctamente")
        return oled
    except Exception as e:
        print(f"DEBUG:Error OLED - {e}")
        return None

def update_rssi_buffer(tx_id, rssi):
    """Actualiza buffer de RSSI para promedio móvil (como TX)"""
    global last_rssi, rssi_buffer, last_received_time, packet_count
    
    current_time = utime.ticks_ms()
    last_received_time[tx_id] = current_time
    packet_count[tx_id] += 1
    
    # Mantener buffer de últimos valores para promedio
    rssi_buffer[tx_id].append(rssi)
    if len(rssi_buffer[tx_id]) > BUFFER_SIZE:
        rssi_buffer[tx_id].pop(0)
    
    # Calcular promedio móvil
    if rssi_buffer[tx_id]:
        promedio = sum(rssi_buffer[tx_id]) / len(rssi_buffer[tx_id])
        last_rssi[tx_id] = int(promedio)

def check_timeouts():
    """Verifica timeouts y resetea valores (optimizado)"""
    current_time = utime.ticks_ms()
    
    for tx_id in TX_ADDRESSES:
        time_diff = utime.ticks_diff(current_time, last_received_time[tx_id])
        if time_diff > RSSI_TIMEOUT_MS:
            if last_rssi[tx_id] != -100:
                last_rssi[tx_id] = -100
                rssi_buffer[tx_id].clear()  # Limpiar buffer
                print(f"TIMEOUT:TX{tx_id} - {time_diff}ms sin respuesta")

def send_data_to_computer():
    """Envía datos al PC en formato JSON optimizado"""
    data = {
        "timestamp": utime.ticks_ms(),
        "rssi": {
            "tx1": last_rssi[1],
            "tx2": last_rssi[2], 
            "tx3": last_rssi[3],
            "tx4": last_rssi[4]
        }
    }
    print(f"DATA:{json.dumps(data)}")

def show_oled_status(oled):
    """Muestra estado en OLED con diseño mejorado"""
    if not oled:
        return
    
    try:
        oled.fill(0)
        oled.text("NRF24L01 RX", 20, 0)
        
        # Mostrar RSSI con indicadores de estado
        for i, tx_id in enumerate([1, 2, 3, 4]):
            rssi = last_rssi[tx_id]
            y_pos = 15 + (i * 10)
            
            # Indicador de estado: OK/TIMEOUT
            status = "OK" if rssi > -100 else "TO"
            oled.text(f"TX{tx_id}:{rssi:4d} {status}", 0, y_pos)
        
        # Mostrar canal
        oled.text(f"CH:{CANAL_RF}", 95, 55)
        oled.show()
        
    except Exception as e:
        print(f"DEBUG:Error OLED update - {e}")

def mostrar_error_nrf(oled):
    """Muestra error del NRF24L01 en pantalla"""
    if oled:
        oled.fill(0)
        oled.text("RX - ERROR", 30, 15)
        oled.text("NRF24L01 no", 25, 30)
        oled.text("responde!", 35, 40)
        oled.show()

def print_startup_info():
    """Información de inicio del sistema"""
    print("SETUP:NRF24L01_RECEIVER_OPTIMIZED")
    print("CONFIG:{\"transmitters\":[1,2,3,4], \"channel\":10, \"rate\":\"2Mbps\"}")
    print(f"DEBUG:Buffer size: {BUFFER_SIZE}, Update rate: {1000//UPDATE_INTERVAL_MS}Hz")

def receiver_loop(nrf, oled):
    """Bucle principal optimizado del receptor"""
    global last_update_time, last_stats_time
    
    nrf.start_listening()
    led = Pin("LED", Pin.OUT)
    
    print_startup_info()
    
    # Contadores para estadísticas
    total_packets = 0
    packets_per_second = 0
    
    while True:
        current_time = utime.ticks_ms()
        
        # Procesar mensajes RF con máxima eficiencia
        messages_processed = 0
        while nrf.any() and messages_processed < 10:  # Limitar para evitar bloqueos
            led.on()
            try:
                buf = nrf.recv()
                if len(buf) == 8:
                    tx_id, rssi = struct.unpack("ii", buf)
                    if tx_id in TX_ADDRESSES:
                        update_rssi_buffer(tx_id, rssi)
                        total_packets += 1
                        packets_per_second += 1
                        # print(f"DEBUG:RX TX{tx_id} = {rssi} dBm")  # Comentado para rendimiento
                    else:
                        print(f"DEBUG:ID desconocido: {tx_id}")
                else:
                    print(f"DEBUG:Payload incorrecto: {len(buf)} bytes")
                    
            except Exception as e:
                print(f"DEBUG:Error recepción: {e}")
            
            led.off()
            messages_processed += 1
        
        # Actualizar sistema cada 200ms (5Hz)
        if utime.ticks_diff(current_time, last_update_time) >= UPDATE_INTERVAL_MS:
            check_timeouts()
            send_data_to_computer()
            show_oled_status(oled)
            last_update_time = current_time
        
        # Estadísticas cada segundo
        if utime.ticks_diff(current_time, last_stats_time) >= 1000:
            if packets_per_second > 0:
                print(f"DEBUG:Paquetes/seg: {packets_per_second}, Total: {total_packets}")
            packets_per_second = 0
            last_stats_time = current_time
        
        # Pequeña pausa para no saturar CPU
        utime.sleep_ms(5)

def main():
    """Función principal con manejo robusto de errores"""
    print("DEBUG:Iniciando receptor NRF24L01 optimizado...")
    
    # Configurar OLED primero
    oled = setup_oled()
    utime.sleep_ms(1000)
    
    # Configurar NRF24L01
    nrf = setup_nrf24l01()
    if not nrf:
        mostrar_error_nrf(oled)
        print("ERROR:NRF24L01 no funciona. Deteniendo programa.")
        return
    
    try:
        # Iniciar bucle principal
        receiver_loop(nrf, oled)
        
    except KeyboardInterrupt:
        print("DEBUG:Programa interrumpido por usuario")
    except Exception as e:
        print(f"ERROR:Error crítico en main: {e}")
        if oled:
            oled.fill(0)
            oled.text("ERROR CRITICO", 15, 20)
            oled.text(str(e)[:16], 0, 35)
            oled.show()
    finally:
        print("DEBUG:Cerrando receptor...")

if __name__ == "__main__":
    main()
