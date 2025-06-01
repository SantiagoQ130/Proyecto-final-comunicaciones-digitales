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

# Variables de control de rendimiento (OPTIMIZADAS PARA VELOCIDAD)
last_rssi = {1: -100, 2: -100, 3: -100, 4: -100}
rssi_buffer = {1: [], 2: [], 3: [], 4: []}
last_received_time = {1: 0, 2: 0, 3: 0, 4: 0}
packet_count = {1: 0, 2: 0, 3: 0, 4: 0}

# Configuración ultra-rápida
BUFFER_SIZE = 5  # Reducido para menos procesamiento
RSSI_TIMEOUT_MS = 10000  # 10 segundos (más agresivo)
UPDATE_INTERVAL_MS = 50   # 50ms = 20Hz (4x más rápido)
OLED_UPDATE_MS = 250     # OLED se actualiza menos frecuente (recursos)
MAX_PACKETS_PER_CYCLE = 20  # Procesar más paquetes por ciclo

# Variables de control de tiempo
last_update_time = 0
last_oled_time = 0
last_stats_time = 0

def setup_nrf24l01_ultra_fast():
    """Configuración ultra-rápida del NRF24L01"""
    spi = SPI(SPI_ID, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN), 
              baudrate=8000000)  # SPI a 8MHz (máximo para RP2040)
    csn = Pin(CSN_PIN, mode=Pin.OUT, value=1)
    ce = Pin(CE_PIN, mode=Pin.OUT, value=0)
    
    nrf = NRF24L01(spi, csn, ce, payload_size=8)
    nrf.set_channel(CANAL_RF)
    
    # Configuración ultra-optimizada
    nrf.reg_write(0x00, 0x0F)  # PWR_UP + PRIM_RX + EN_CRC (CRC de 1 byte)
    nrf.reg_write(0x01, 0x00)  # Sin auto-ACK para máxima velocidad
    nrf.reg_write(0x02, 0x0F)  # Habilitar todos los pipes (1-4)
    nrf.reg_write(0x03, 0x03)  # Dirección de 5 bytes
    nrf.reg_write(0x04, 0x00)  # Sin retransmisiones
    nrf.reg_write(0x05, CANAL_RF)  # Canal RF
    nrf.reg_write(0x06, 0x0E)  # 2Mbps, máxima potencia
    nrf.reg_write(0x07, 0x70)  # Limpiar todas las interrupciones
    nrf.reg_write(0x11, 0x08)  # Payload fijo de 8 bytes para todos los pipes
    nrf.reg_write(0x12, 0x08)
    nrf.reg_write(0x13, 0x08)
    nrf.reg_write(0x14, 0x08)
    nrf.reg_write(0x1C, 0x00)  # Sin dynamic payloads
    nrf.reg_write(0x1D, 0x00)  # Sin features especiales
    
    # Configurar direcciones de pipes
    for pipe, addr in TX_ADDRESSES.items():
        nrf.open_rx_pipe(pipe, addr)
    
    # Verificar comunicación
    try:
        config_reg = nrf.reg_read(0x00)
        print(f"DEBUG:NRF24L01 ULTRA-FAST configurado - Config: 0x{config_reg:02x}")
        print(f"DEBUG:Velocidad SPI: 8MHz, RF: 2Mbps, Update: {1000//UPDATE_INTERVAL_MS}Hz")
        return nrf
    except Exception as e:
        print(f"DEBUG:Error NRF24L01 - {e}")
        return None

def setup_oled():
    """Configuración de OLED optimizada"""
    try:
        i2c = I2C(1, scl=Pin(SCL), sda=Pin(SDA), freq=800000)  # I2C más rápido
        oled = SSD1306_I2C(WIDTH, HEIGHT, i2c)
        oled.fill(0)
        oled.text("ULTRA-FAST RX", 15, 20)
        oled.text("NRF24L01", 25, 35)
        oled.show()
        print("DEBUG:OLED inicializado (800kHz)")
        return oled
    except Exception as e:
        print(f"DEBUG:Error OLED - {e}")
        return None

def fast_rssi_update(tx_id, rssi):
    """Actualización ultra-rápida de RSSI (sin promedio complejo)"""
    global last_rssi, rssi_buffer, last_received_time, packet_count
    
    current_time = utime.ticks_ms()
    last_received_time[tx_id] = current_time
    packet_count[tx_id] += 1
    
    # Buffer más pequeño y actualización más directa
    rssi_buffer[tx_id].append(rssi)
    if len(rssi_buffer[tx_id]) > BUFFER_SIZE:
        rssi_buffer[tx_id].pop(0)
    
    # Promedio rápido (solo últimos 3 valores para menos cálculo)
    if len(rssi_buffer[tx_id]) >= 3:
        last_rssi[tx_id] = int(sum(rssi_buffer[tx_id][-3:]) / 3)
    else:
        last_rssi[tx_id] = rssi

def quick_timeout_check():
    """Verificación rápida de timeouts (optimizada)"""
    current_time = utime.ticks_ms()
    
    for tx_id in TX_ADDRESSES:
        if utime.ticks_diff(current_time, last_received_time[tx_id]) > RSSI_TIMEOUT_MS:
            if last_rssi[tx_id] != -100:
                last_rssi[tx_id] = -100
                rssi_buffer[tx_id].clear()

def send_data_ultra_fast():
    """Envío ultra-rápido de datos (formato mínimo)"""
    # Formato JSON mínimo para máxima velocidad
    print(f"DATA:{{{last_rssi[1]},{last_rssi[2]},{last_rssi[3]},{last_rssi[4]}}}")

def show_oled_fast(oled):
    """Actualización rápida de OLED (menos procesamiento)"""
    if not oled:
        return
    
    try:
        oled.fill(0)
        oled.text("ULTRA-FAST", 30, 0)
        
        # Solo mostrar RSSI, sin texto extra
        for i, tx_id in enumerate([1, 2, 3, 4]):
            rssi = last_rssi[tx_id]
            y_pos = 12 + (i * 12)
            status = "+" if rssi > -100 else "-"
            oled.text(f"{tx_id}:{rssi:4d}{status}", 0, y_pos)
        
        oled.show()
        
    except Exception as e:
        print(f"DEBUG:OLED error - {e}")

def ultra_fast_receiver_loop(nrf, oled):
    """Bucle principal ULTRA-OPTIMIZADO"""
    global last_update_time, last_oled_time, last_stats_time
    
    nrf.start_listening()
    led = Pin("LED", Pin.OUT)
    
    print("SETUP:ULTRA_FAST_NRF24L01_RECEIVER")
    print(f"CONFIG:{{\"rate\":\"20Hz\",\"spi\":\"8MHz\",\"rf\":\"2Mbps\",\"buffer\":{BUFFER_SIZE}}}")
    
    packets_per_second = 0
    cycle_count = 0
    
    while True:
        current_time = utime.ticks_ms()
        
        # PROCESAR RF CON MÁXIMA PRIORIDAD Y VELOCIDAD
        packets_in_cycle = 0
        
        # Ciclo optimizado de recepción - procesar TODOS los paquetes disponibles
        while nrf.any() and packets_in_cycle < MAX_PACKETS_PER_CYCLE:
            led.on()
            try:
                buf = nrf.recv()
                if len(buf) == 8:
                    tx_id, rssi = struct.unpack("ii", buf)
                    if tx_id in TX_ADDRESSES:
                        fast_rssi_update(tx_id, rssi)
                        packets_per_second += 1
                        packets_in_cycle += 1
                    
            except Exception as e:
                print(f"RX_ERR:{e}")
            
            led.off()
        
        # Actualización ultra-rápida cada 50ms (20Hz)
        if utime.ticks_diff(current_time, last_update_time) >= UPDATE_INTERVAL_MS:
            quick_timeout_check()
            send_data_ultra_fast()
            last_update_time = current_time
            cycle_count += 1
        
        # OLED menos frecuente para no afectar velocidad de RF
        if utime.ticks_diff(current_time, last_oled_time) >= OLED_UPDATE_MS:
            show_oled_fast(oled)
            last_oled_time = current_time
        
        # Estadísticas cada segundo
        if utime.ticks_diff(current_time, last_stats_time) >= 1000:
            if packets_per_second > 0:
                print(f"STATS:PPS:{packets_per_second} Cycles:{cycle_count}")
            packets_per_second = 0
            cycle_count = 0
            last_stats_time = current_time
        
        # MÍNIMA pausa - solo 1ms para no perder paquetes
        utime.sleep_ms(1)

def main():
    """Función principal ultra-optimizada"""
    print("DEBUG:Iniciando RECEPTOR ULTRA-RÁPIDO...")
    
    # OLED primero
    oled = setup_oled()
    utime.sleep_ms(500)  # Menos tiempo de espera
    
    # NRF24L01 ultra-rápido
    nrf = setup_nrf24l01_ultra_fast()
    if not nrf:
        print("ERROR:NRF24L01 falló. Abortando.")
        return
    
    try:
        print("DEBUG:Iniciando bucle ultra-rápido...")
        ultra_fast_receiver_loop(nrf, oled)
        
    except KeyboardInterrupt:
        print("DEBUG:Interrumpido por usuario")
    except Exception as e:
        print(f"ERROR:Crítico: {e}")
    finally:
        print("DEBUG:Cerrando receptor ultra-rápido...")

if __name__ == "__main__":
    main()
