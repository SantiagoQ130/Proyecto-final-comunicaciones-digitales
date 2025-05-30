import struct
import utime
import network
from machine import Pin, SPI, I2C
from nrf24l01 import NRF24L01
from ssd1306 import SSD1306_I2C

# Pines SPI y NRF
SPI_ID = 0
SCK_PIN = 2
MOSI_PIN = 3
MISO_PIN = 4
CSN_PIN = 5
CE_PIN = 6

# Pines I2C para OLED
SDA = 14
SCL = 15
WIDTH = 128
HEIGHT = 64

# LED indicador
led = Pin("LED", Pin.OUT)

# Configuración de canal y dirección
CANAL_RF = 10
TX_ADDRESS = b"\xb1\xf0\xf0\xf0\xf0"  # Asegúrate que coincida con el RX

# Config WiFi
SSID = "ArduinoR4-123456"
PASSWORD = "12345678"

# ID del transmisor
TX_ID = 1

def setup_nrf24l01():
    spi = SPI(SPI_ID, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    csn = Pin(CSN_PIN, mode=Pin.OUT, value=1)
    ce = Pin(CE_PIN, mode=Pin.OUT, value=0)
    nrf = NRF24L01(spi, csn, ce, payload_size=8)  # 4 bytes ID + 4 bytes RSSI
    nrf.set_channel(CANAL_RF)
    nrf.reg_write(0x06, 0x0E)  # 2Mbps, máxima potencia (+0dBm)
    nrf.reg_write(0x1C, 0x00)  # Deshabilitar auto-ACK para mayor velocidad
    nrf.reg_write(0x1D, 0x00)  # Sin retransmisiones automáticas
    nrf.open_tx_pipe(TX_ADDRESS)
    
    # Verificar comunicación con NRF24L01
    try:
        config_reg = nrf.reg_read(0x00)  # Leer registro de configuración
        print(f"NRF24L01 configurado correctamente - Config: 0x{config_reg:02x}")
    except Exception as e:
        print(f"Error: NRF24L01 no responde - {e}")
        return None
    return nrf

def conectar_wifi():
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    return wifi

def intentar_conexion_wifi(wifi, oled, max_intentos=10):
    """Intenta conectar al WiFi mostrando el estado en pantalla"""
    if wifi.isconnected():
        return True
    
    print(f"Conectando a {SSID}...")
    wifi.connect(SSID, PASSWORD)
    
    for intento in range(max_intentos):
        # Mostrar estado de conexión en OLED
        mostrar_conectando_wifi(oled, intento + 1, max_intentos)
        
        if wifi.isconnected():
            print("¡Conectado al WiFi!")
            return True
        
        utime.sleep(0.2)
    
    print("No se pudo conectar al WiFi en este intento")
    return False

def mostrar_conectando_wifi(oled, intento, max_intentos):
    """Muestra el estado de conexión WiFi en la pantalla"""
    oled.fill(0)
    oled.text("TX1", 45, 0)
    oled.text("Conectando WiFi...", 0, 15)
    oled.text(f"Intento: {intento}/{max_intentos}", 0, 25)
    oled.text(f"SSID: {SSID[:12]}", 0, 35)
    
    # Barra de progreso simple
    barra_ancho = 80
    progreso = int((intento / max_intentos) * barra_ancho)
    oled.rect(20, 45, barra_ancho, 6, 1)
    oled.fill_rect(20, 45, progreso, 6, 1)
    
    oled.show()

def get_wifi_strength(wifi):
    """Obtiene la intensidad de señal WiFi con manejo de errores mejorado"""
    try:
        if wifi and wifi.isconnected():
            return wifi.status('rssi')
        else:
            return -100
    except Exception as e:
        print(f"Error obteniendo RSSI: {e}")
        return -100

def setup_oled():
    i2c = I2C(1, scl=Pin(SCL), sda=Pin(SDA))  # Usamos I2C(1)
    oled = SSD1306_I2C(WIDTH, HEIGHT, i2c)
    oled.fill(0)
    oled.text("Iniciando TX1...", 0, 0)
    oled.show()
    return oled

def mostrar_en_oled(oled, rssi, ssid, ip, conectado=True):
    """Muestra información en OLED diferenciando si está conectado o no"""
    oled.fill(0)
    oled.text("TX1", 50, 0)
    
    if conectado:
        oled.text(f"SSID: {ssid[:12]}", 0, 15)
        oled.text(f"IP: {ip[:15]}", 0, 25)
        oled.text(f"RSSI: {rssi} dBm", 0, 35)
        oled.text("Estado: CONECTADO", 0, 45)
    else:
        oled.text("WiFi DESCONECTADO", 0, 15)
        oled.text("Reintentando...", 0, 25)
        oled.text(f"SSID: {SSID[:12]}", 0, 35)
        oled.text("Estado: ERROR", 0, 45)
    
    oled.show()

def mostrar_error_nrf(oled):
    """Muestra error del NRF24L01 en pantalla"""
    oled.fill(0)
    oled.text("TX1 - ERROR", 30, 15)
    oled.text("NRF24L01 no", 25, 30)
    oled.text("responde!", 35, 40)
    oled.show()

def transmit_message(nrf, rssi):
    """Transmite mensaje con manejo de errores mejorado"""
    if not nrf:
        return False
    
    led.on()
    try:
        mensaje = struct.pack("ii", TX_ID, rssi)
        nrf.send(mensaje)
        print(f"Enviado -> TX{TX_ID} = {rssi} dBm")
        led.off()
        return True
    except Exception as e:
        print(f"Error al enviar: {e}")
        led.off()
        return False

def main():
    print("Iniciando TX NRF24L01...")
    
    # Configurar OLED primero para mostrar estados
    oled = setup_oled()
    utime.sleep(1)
    
    # Configurar NRF24L01
    nrf = setup_nrf24l01()
    if not nrf:
        mostrar_error_nrf(oled)
        print("Error crítico: NRF24L01 no funciona. Deteniendo programa.")
        return
    
    # Configurar WiFi
    wifi = conectar_wifi()
    # Buffer para mantener 10 segundos de datos con frecuencia de 0.2s
    BUFFER_SIZE = int(10 / 0.2)  # 50 muestras para 10 segundos
    buffer = []
    
    # Intentar conexión inicial
    conectado = intentar_conexion_wifi(wifi, oled)
    
    while True:
        # Verificar conexión WiFi
        if not wifi.isconnected():
            conectado = False
            print("WiFi desconectado, reintentando...")
            mostrar_en_oled(oled, -100, "Desconectado", "0.0.0.0", False)
            utime.sleep(2)
            
            # Intentar reconectar
            conectado = intentar_conexion_wifi(wifi, oled, max_intentos=5)
            
            if not conectado:
                # Si no se pudo conectar, esperar un poco más antes del siguiente intento
                utime.sleep(5)
                continue
        
        # Si llegamos aquí, estamos conectados
        if not conectado:
            conectado = True
            print("Reconexión WiFi exitosa!")
        
        # Obtener datos de red
        rssi = get_wifi_strength(wifi)
        ssid = wifi.config('essid') if wifi.isconnected() else "Desconocido"
        ip = wifi.ifconfig()[0] if wifi.isconnected() else "0.0.0.0"
        
        # Mantener buffer de RSSI para promedio móvil (últimos 10 segundos)
        buffer.append(rssi)
        if len(buffer) > BUFFER_SIZE:
            buffer.pop(0)
        
        # Calcular promedio con división flotante mejorada
        promedio = sum(buffer) / len(buffer)
        promedio_int = int(promedio)
        
        # Mostrar información en OLED
        mostrar_en_oled(oled, promedio_int, ssid, ip, conectado)
        
        # Transmitir mensaje
        if not transmit_message(nrf, promedio_int):
            print("Error en transmisión, continuando...")
        
        utime.sleep(1)

if __name__ == "__main__":
    main()
