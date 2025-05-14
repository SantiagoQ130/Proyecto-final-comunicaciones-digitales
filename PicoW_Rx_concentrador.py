import struct
import utime
import json
from machine import Pin, SPI, I2C
from nrf24l01 import NRF24L01
import ssd1306

# Configuración de pines
SPI_ID = 0
SCK = 2
MOSI = 3
MISO = 4
CSN = 5
CE = 6
SDA = 14
SCL = 15

# Configuración de pantalla OLED
WIDTH = 128
HEIGHT = 64

# Direcciones de transmisores
TX_ADDRESSES = {
    1: b"\xe1\xf0\xf0\xf0\xf0",
    2: b"\xc3\xf0\xf0\xf0\xf0",
    3: b"\xd2\xf0\xf0\xf0\xf0"
}

# Inicialización de variables para el historial de RSSI
last_rssi = {1: -100, 2: -100, 3: -100}
rssi_history = {1: [], 2: [], 3: []}
MAX_HISTORY_SIZE = 100  # Limitar el tamaño del historial para evitar problemas de memoria

# Variables de tiempo para control de actualizaciones
last_update_time = 0
update_interval_ms = 200  # Enviar datos al PC cada 200ms

# Definición de funciones
def setup_nrf():
    """Configura y devuelve un objeto NRF24L01."""
    spi = SPI(SPI_ID, sck=Pin(SCK), mosi=Pin(MOSI), miso=Pin(MISO))
    csn = Pin(CSN, Pin.OUT, value=1)
    ce = Pin(CE, Pin.OUT, value=0)
    nrf = NRF24L01(spi, csn, ce, payload_size=8)
    nrf.set_channel(21)
    for pipe, addr in TX_ADDRESSES.items():
        nrf.open_rx_pipe(pipe, addr)
    return nrf

def update_history(tx_id, rssi):
    """Actualiza el historial de RSSI para un transmisor específico."""
    global last_rssi, rssi_history
    
    # Actualizar el último valor recibido
    last_rssi[tx_id] = rssi
    
    # Actualizar historial con timestamp
    timestamp = utime.ticks_ms()
    rssi_history[tx_id].append((timestamp, rssi))
    
    # Mantener el historial dentro del límite de tamaño
    if len(rssi_history[tx_id]) > MAX_HISTORY_SIZE:
        rssi_history[tx_id].pop(0)

def send_data_to_computer():
    """Envía los datos de RSSI al computador en formato JSON para visualización."""
    # Crear un diccionario con los datos actuales
    data = {
        "timestamp": utime.ticks_ms(),
        "rssi": {
            "tx1": last_rssi[1],
            "tx2": last_rssi[2],
            "tx3": last_rssi[3]
        }
    }
    
    # Enviar datos al computador (serán visualizados por el script en Thonny)
    print(f"DATA:{json.dumps(data)}")

def show_oled_status():
    """Muestra el estado actual en la pantalla OLED."""
    oled.fill(0)
    oled.text("RSSI Monitor", 0, 0)
    oled.text(f"TX1: {last_rssi[1]} dBm", 0, 15)
    oled.text(f"TX2: {last_rssi[2]} dBm", 0, 30)
    oled.text(f"TX3: {last_rssi[3]} dBm", 0, 45)
    oled.text("-> PC Display ON", 0, 55)
    oled.show()

def receiver_loop(nrf):
    """Bucle principal del receptor."""
    global last_update_time
    
    # Iniciar recepción
    nrf.start_listening()
    led = Pin("LED", Pin.OUT)
    print("SETUP:NRF24L01_RECEIVER")  # Señal para que el script en Thonny sepa que iniciamos
    
    # Mandar configuración inicial
    print("CONFIG:{\"transmitters\":[1,2,3], \"min_rssi\":-100, \"max_rssi\":-30}")
    
    while True:
        # Comprobar recepción de datos
        if nrf.any():
            led.on()
            try:
                buf = nrf.recv()
                if len(buf) == 8:
                    message_id, rssi = struct.unpack("ii", buf)
                    if message_id in TX_ADDRESSES:
                        update_history(message_id, rssi)
                        # No usar print normal para evitar interferir con los datos JSON
                        # para debugging interno
                    else:
                        # Mensaje para debugging en la consola serial
                        print(f"DEBUG:ID desconocido: {message_id}")
                else:
                    print(f"DEBUG:Payload inesperado: {len(buf)} bytes")
            except Exception as e:
                print(f"DEBUG:Error al leer: {e}")
            led.off()
        
        # Enviar datos al PC periódicamente
        current_time = utime.ticks_ms()
        if utime.ticks_diff(current_time, last_update_time) > update_interval_ms:
            send_data_to_computer()
            show_oled_status()
            last_update_time = current_time
        
        # Pequeña pausa para reducir consumo de CPU
        utime.sleep_ms(5)

def main():
    """Función principal."""
    global oled
    
    try:
        # Inicializar I2C y OLED
        i2c = I2C(1, scl=Pin(SCL), sda=Pin(SDA))
        oled = ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c)
        
        # Mensaje de inicio
        oled.fill(0)
        oled.text("Iniciando...", 0, 20)
        oled.text("NRF24L01 RX", 0, 30)
        oled.show()
        utime.sleep_ms(1000)
        
        # Configurar NRF24L01
        nrf = setup_nrf()
        
        # Iniciar bucle de recepción
        receiver_loop(nrf)
        
    except Exception as e:
        if 'oled' in globals():
            oled.fill(0)
            oled.text("ERROR:", 0, 0)
            oled.text(str(e)[:16], 0, 10)
            oled.text(str(e)[16:32], 0, 20)
            oled.show()
        print(f"DEBUG:Error en el receptor: {e}")

if __name__ == "__main__":
    main()
