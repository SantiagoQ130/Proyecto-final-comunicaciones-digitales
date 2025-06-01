import struct
import time
import network
from machine import Pin, SPI, I2C, freq
from nrf24l01 import NRF24L01
from ssd1306 import SSD1306_I2C
import _thread
import gc

# OVERCLOCK PARA MÁXIMA PERFORMANCE
freq(250000000)  # 250MHz (máximo seguro para Pico W)

# Configuración de pines optimizada para Pico W
SPI_ID = 0
SCK_PIN = 2
MOSI_PIN = 3
MISO_PIN = 4
CSN_PIN = 5
CE_PIN = 6

# I2C para OLED
SDA = 14
SCL = 15
WIDTH = 128
HEIGHT = 64

# LED onboard Pico W
led = Pin("LED", Pin.OUT)

# Configuración RF optimizada
CANAL_RF = 10  # Canal libre de WiFi (evita 2.4GHz WiFi)
TX_ADDRESS = b"\xb1\xf0\xf0\xf0\xf0"
TX_ID = 1

# WiFi
SSID = "ArduinoR4-123456"
PASSWORD = "12345678"

# Variables globales para threading
rssi_actual = -100
transmision_activa = True
contador_envios = 0

def setup_nrf24l01_ultra_fast():
    """Configuración NRF24L01 para ULTRA baja latencia en Pico W"""
    # SPI a máxima velocidad soportada por NRF24L01
    spi = SPI(SPI_ID, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN), 
              baudrate=10000000)  # 10MHz - máximo para NRF24L01
    
    csn = Pin(CSN_PIN, mode=Pin.OUT, value=1)
    ce = Pin(CE_PIN, mode=Pin.OUT, value=0)
    
    # Payload ultra mínimo para velocidad máxima
    nrf = NRF24L01(spi, csn, ce, payload_size=3)  # Solo 3 bytes: 1 ID + 2 RSSI
    
    # Configuración CRÍTICA para latencia mínima
    nrf.set_channel(CANAL_RF)
    
    # Registro RF_SETUP (0x06): 2Mbps + PA máxima
    nrf.reg_write(0x06, 0x0F)  # 2Mbps, +0dBm, LNA habilitado
    
    # Deshabilitar COMPLETAMENTE auto-ACK (crítico)
    nrf.reg_write(0x1C, 0x00)  # EN_AA: Sin auto-acknowledgment
    
    # Sin retransmisiones automáticas
    nrf.reg_write(0x1D, 0x00)  # EN_RXADDR: Solo pipe 0
    nrf.reg_write(0x04, 0x00)  # SETUP_RETR: Sin delays ni reintentos
    
    # Configuración CONFIG optimizada
    nrf.reg_write(0x00, 0x0A)  # PWR_UP=1, PRIM_RX=0, EN_CRC=0 (sin CRC para velocidad)
    
    # FIFO y timing
    nrf.reg_write(0x11, 0x00)  # RX_PW_P0: 0 bytes (no RX)
    nrf.reg_write(0x1F, 0x00)  # DYNPD: Sin dynamic payload
    nrf.reg_write(0x1E, 0x00)  # FEATURE: Características extras deshabilitadas
    
    nrf.open_tx_pipe(TX_ADDRESS)
    
    # Verificar configuración
    try:
        config_reg = nrf.reg_read(0x00)
        rf_setup = nrf.reg_read(0x06)
        print(f"NRF24L01 ULTRA-FAST - Config: 0x{config_reg:02x}, RF: 0x{rf_setup:02x}")
        
        # Flush TX FIFO para empezar limpio
        nrf.reg_write_bytes(0xE1, b'')  # FLUSH_TX
        
    except Exception as e:
        print(f"Error NRF24L01: {e}")
        return None
    
    return nrf

def wifi_thread():
    """Thread dedicado para WiFi - no bloquea transmisión NRF"""
    global rssi_actual, transmision_activa
    
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    
    # Configuración WiFi para baja latencia
    wifi.config(pm=0xa11140)  # Power management OFF
    
    print("Conectando WiFi en thread separado...")
    wifi.connect(SSID, PASSWORD)
    
    # Esperar conexión inicial
    timeout = 20
    while not wifi.isconnected() and timeout > 0:
        time.sleep(0.1)
        timeout -= 1
    
    if wifi.isconnected():
        print(f"WiFi conectado: {wifi.ifconfig()[0]}")
    
    # Loop de monitoreo WiFi
    reconexion_contador = 0
    while transmision_activa:
        try:
            if wifi.isconnected():
                rssi_actual = wifi.status('rssi')
                reconexion_contador = 0
            else:
                rssi_actual = -100
                reconexion_contador += 1
                
                # Reintentar conexión cada 5 segundos
                if reconexion_contador >= 50:  # 50 * 0.1s = 5s
                    print("Reconectando WiFi...")
                    wifi.connect(SSID, PASSWORD)
                    reconexion_contador = 0
                    
        except Exception as e:
            rssi_actual = -100
            print(f"Error WiFi: {e}")
        
        time.sleep(0.1)  # Actualizar RSSI cada 100ms

def setup_oled_fast():
    """OLED optimizado para Pico W"""
    try:
        i2c = I2C(1, scl=Pin(SCL), sda=Pin(SDA), freq=1000000)  # I2C a 1MHz
        oled = SSD1306_I2C(WIDTH, HEIGHT, i2c)
        oled.fill(0)
        oled.text("PICO W TX1", 25, 0)
        oled.text("NRF Ultra Fast", 5, 15)
        oled.text("Freq: 250MHz", 10, 30)
        oled.show()
        return oled
    except Exception as e:
        print(f"Error OLED: {e}")
        return None

def transmit_ultra_fast(nrf, rssi):
    """Transmisión ultra optimizada - sin delays internos"""
    led.on()
    
    try:
        # Payload mínimo: 1 byte ID + 2 bytes RSSI (signed short)
        mensaje = struct.pack("Bh", TX_ID, rssi)
        
        # Envío directo sin verificaciones adicionales
        nrf.send(mensaje)
        
        led.off()
        return True
        
    except Exception as e:
        led.off()
        return False

def main_ultra_fast():
    """Loop principal ultra optimizado"""
    global rssi_actual, transmision_activa, contador_envios
    
    print("=== RASPBERRY PI PICO W - NRF24L01 ULTRA FAST ===")
    print(f"CPU Frequency: {freq()} Hz")
    
    # Configurar hardware
    oled = setup_oled_fast()
    time.sleep(0.2)
    
    nrf = setup_nrf24l01_ultra_fast()
    if not nrf:
        print("ERROR CRÍTICO: NRF24L01 no disponible")
        if oled:
            oled.fill(0)
            oled.text("ERROR NRF24L01", 10, 20)
            oled.show()
        return
    
    # Iniciar thread de WiFi
    _thread.start_new_thread(wifi_thread, ())
    time.sleep(1)  # Dar tiempo al thread de WiFi
    
    print("¡MODO ULTRA RÁPIDO ACTIVO!")
    print("Frecuencia objetivo: 200Hz (5ms entre envíos)")
    
    # Variables de timing
    ultima_pantalla = time.ticks_ms()
    errores = 0
    inicio = time.ticks_ms()
    
    # LOOP PRINCIPAL - CRÍTICO PARA LATENCIA
    while True:
        # Timestamp preciso
        timestamp = time.ticks_us()
        
        # Transmisión INMEDIATA
        if transmit_ultra_fast(nrf, rssi_actual):
            contador_envios += 1
        else:
            errores += 1
        
        # Actualizar pantalla solo cada 1 segundo (no cada envío)
        ahora = time.ticks_ms()
        if oled and time.ticks_diff(ahora, ultima_pantalla) >= 1000:
            tiempo_total = time.ticks_diff(ahora, inicio) / 1000
            freq_real = contador_envios / tiempo_total if tiempo_total > 0 else 0
            
            oled.fill(0)
            oled.text("PICO W ULTRA", 20, 0)
            oled.text(f"RSSI: {rssi_actual}", 0, 12)
            oled.text(f"Enviados: {contador_envios}", 0, 22)
            oled.text(f"Errores: {errores}", 0, 32)
            oled.text(f"Freq: {freq_real:.1f}Hz", 0, 42)
            oled.text(f"CPU: 250MHz", 0, 52)
            oled.show()
            
            ultima_pantalla = ahora
            
            # Garbage collection periódico para evitar pausas
            if contador_envios % 1000 == 0:
                gc.collect()
        
        # DELAY CRÍTICO - Ajustar según necesidades
        # 0.005 = 200Hz, 0.002 = 500Hz, 0.001 = 1000Hz
        time.sleep(0.002)  # 500Hz - ULTRA RÁPIDO

def cleanup():
    """Limpieza al finalizar"""
    global transmision_activa
    transmision_activa = False
    led.off()
    print("Transmisión finalizada")

if __name__ == "__main__":
    try:
        main_ultra_fast()
    except KeyboardInterrupt:
        cleanup()
    except Exception as e:
        print(f"Error crítico: {e}")
        cleanup()
