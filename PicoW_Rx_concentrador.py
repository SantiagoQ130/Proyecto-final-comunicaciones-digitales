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
    4: b"\xb1\xf0\xf0\xf0\xf0"
}

# Variables de control simplificadas (sin buffers de promedio)
last_rssi = {1: -100, 2: -100, 3: -100, 4: -100}
last_received_time = {1: 0, 2: 0, 3: 0, 4: 0}
packet_count = {1: 0, 2: 0, 3: 0, 4: 0}

# Configuración optimizada para recepción continua
RSSI_TIMEOUT_MS = 10000  # 10 segundos timeout (más agresivo)
UPDATE_INTERVAL_MS = 50   # 50ms = 20Hz (más frecuente para datos continuos)
MAX_PACKETS_PER_CYCLE = 20  # Procesar hasta 20 paquetes por ciclo

# Variables de control de tiempo
last_update_time = 0
last_stats_time = 0

def setup_nrf24l01():
    """Configuración ultra optimizada del NRF24L01 para recepción continua"""
    spi = SPI(SPI_ID, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    csn = Pin(CSN_PIN, mode=Pin.OUT, value=1)
    ce = Pin(CE_PIN, mode=Pin.OUT, value=0)
    
    nrf = NRF24L01(spi, csn, ce, payload_size=8)
    nrf.set_channel(CANAL_RF)
    
    # Configuración para máximo throughput y mínima latencia
    nrf.reg_write(0x06, 0x0E)  # 2Mbps, máxima potencia
    nrf.reg_write(0x1C, 0x00)  # Sin auto-ACK (máxima velocidad)
    nrf.reg_write(0x1D, 0x00)  # Sin retransmisiones
    nrf.reg_write(0x11, 0x00)  # Vaciar TX FIFO
    nrf.reg_write(0x17, 0x70)  # Limpiar flags de estado
    
    # Configurar todos los pipes de recepción
    for pipe, addr in TX_ADDRESSES.items():
        nrf.open_rx_pipe(pipe, addr)
    
    # Verificar comunicación
    try:
        config_reg = nrf.reg_read(0x00)
        print(f"DEBUG:NRF24L01 RX Ultra configurado - Config: 0x{config_reg:02x}")
        return nrf
    except Exception as e:
        print(f"DEBUG:Error NRF24L01 - {e}")
        return None

def setup_oled():
    """Configuración de OLED optimizada"""
    try:
        i2c = I2C(1, scl=Pin(SCL), sda=Pin(SDA))
        oled = SSD1306_I2C(WIDTH, HEIGHT, i2c)
        oled.fill(0)
        oled.text("RX Continuo", 25, 20)
        oled.text("Preparando...", 20, 35)
        oled.show()
        print("DEBUG:OLED inicializado")
        return oled
    except Exception as e:
        print(f"DEBUG:Error OLED - {e}")
        return None

def update_rssi_direct(tx_id, rssi):
    """Actualización directa de RSSI sin promedios"""
    global last_rssi, last_received_time, packet_count
    
    current_time = utime.ticks_ms()
    last_received_time[tx_id] = current_time
    packet_count[tx_id] += 1
    
    # Actualización directa sin buffer
    last_rssi[tx_id] = rssi

def check_timeouts():
    """Verificación rápida de timeouts"""
    current_time = utime.ticks_ms()
    
    for tx_id in TX_ADDRESSES:
        time_diff = utime.ticks_diff(current_time, last_received_time[tx_id])
        if time_diff > RSSI_TIMEOUT_MS and last_rssi[tx_id] != -100:
            last_rssi[tx_id] = -100
            print(f"TIMEOUT:TX{tx_id}")

def send_data_to_computer():
    """Envío optimizado de datos al PC"""
    data = {
        "t": utime.ticks_ms(),  # Clave acortada
        "rssi": {
            "tx1": last_rssi[1],
            "tx2": last_rssi[2], 
            "tx3": last_rssi[3],
            "tx4": last_rssi[4]
        }
    }
    print(f"DATA:{json.dumps(data)}")

def show_oled_status(oled):
    """Display OLED optimizado para datos continuos"""
    if not oled:
        return
    
    try:
        oled.fill(0)
        oled.text("RX CONTINUO", 25, 0)
        
        # Mostrar RSSI con indicadores más compactos
        for i, tx_id in enumerate([1, 2, 3, 4]):
            rssi = last_rssi[tx_id]
            y_pos = 12 + (i * 12)
            
            # Estado: "OK" si recibe datos, "TO" si timeout
            status = "OK" if rssi > -100 else "X"
            oled.text(f"TX{tx_id} {rssi:3d} {status}", 0, y_pos)
        
        # Info de canal y frecuencia de actualización
        oled.text(f"CH{CANAL_RF} 20Hz", 75, 55)
        oled.show()
        
    except Exception as e:
        print(f"DEBUG:Error OLED - {e}")

def mostrar_error_nrf(oled):
    """Error display para NRF24L01"""
    if oled:
        oled.fill(0)
        oled.text("ERROR NRF", 35, 20)
        oled.text("Revisar SPI", 25, 35)
        oled.show()

def print_startup_info():
    """Información de inicio optimizada"""
    print("SETUP:NRF24L01_RX_CONTINUO_OPTIMIZED")
    print("CONFIG:{\"tx\":[1,2,3,4], \"ch\":10, \"rate\":\"2Mbps\", \"mode\":\"continuous\"}")
    print(f"DEBUG:Update rate: 20Hz, Max packets/cycle: {MAX_PACKETS_PER_CYCLE}")

def receiver_loop(nrf, oled):
    """Bucle principal ultra optimizado para recepción continua"""
    global last_update_time, last_stats_time
    
    nrf.start_listening()
    led = Pin("LED", Pin.OUT)
    
    print_startup_info()
    
    # Contadores para estadísticas
    total_packets = 0
    packets_per_second = 0
    rx_errors = 0
    
    while True:
        current_time = utime.ticks_ms()
        
        # Procesar todos los mensajes RF disponibles (sin límite de tiempo)
        messages_this_cycle = 0
        
        while nrf.any() and messages_this_cycle < MAX_PACKETS_PER_CYCLE:
            led.on()
            try:
                buf = nrf.recv()
                if len(buf) == 8:
                    tx_id, rssi = struct.unpack("ii", buf)
                    if tx_id in TX_ADDRESSES:
                        update_rssi_direct(tx_id, rssi)
                        total_packets += 1
                        packets_per_second += 1
                    else:
                        rx_errors += 1
                else:
                    rx_errors += 1
                    
            except Exception as e:
                rx_errors += 1
                if rx_errors % 100 == 0:  # Log cada 100 errores
                    print(f"DEBUG:RX errors: {rx_errors}")
            
            led.off()
            messages_this_cycle += 1
        
        # Actualizar sistema cada 50ms (20Hz) para datos continuos
        if utime.ticks_diff(current_time, last_update_time) >= UPDATE_INTERVAL_MS:
            check_timeouts()
            send_data_to_computer()
            show_oled_status(oled)
            last_update_time = current_time
        
        # Estadísticas cada segundo
        if utime.ticks_diff(current_time, last_stats_time) >= 1000:
            if packets_per_second > 0 or rx_errors > 0:
                print(f"STATS:PPS:{packets_per_second} Total:{total_packets} Errors:{rx_errors}")
            packets_per_second = 0
            last_stats_time = current_time
        
        # Mínima pausa para no saturar CPU pero mantener responsividad
        utime.sleep_ms(1)

def main():
    """Función principal con manejo de errores"""
    print("DEBUG:Iniciando receptor continuo NRF24L01...")
    
    # Configurar OLED
    oled = setup_oled()
    utime.sleep_ms(500)  # Pausa reducida
    
    # Configurar NRF24L01
    nrf = setup_nrf24l01()
    if not nrf:
        mostrar_error_nrf(oled)
        print("ERROR:NRF24L01 fallo. Programa detenido.")
        return
    
    try:
        # Iniciar bucle de recepción continua
        receiver_loop(nrf, oled)
        
    except KeyboardInterrupt:
        print("DEBUG:Programa interrumpido")
    except Exception as e:
        print(f"ERROR:Error crítico: {e}")
        if oled:
            oled.fill(0)
            oled.text("ERROR CRITICO", 15, 20)
            oled.text(str(e)[:16], 0, 35)
            oled.show()
    finally:
        print("DEBUG:Cerrando receptor...")

if __name__ == "__main__":
    main()
