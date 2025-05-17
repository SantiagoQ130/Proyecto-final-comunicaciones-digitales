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
    3: b"\xd2\xf0\xf0\xf0\xf0",
    4: b"\xb1\xf0\xf0\xf0\xf0"  # TX4 agregado
}

# Inicialización de variables para el historial de RSSI
last_rssi = {1: -100, 2: -100, 3: -100, 4: -100}
rssi_history = {1: [], 2: [], 3: [], 4: []}
last_received_time = {1: 0, 2: 0, 3: 0, 4: 0}
MAX_HISTORY_SIZE = 100
RSSI_TIMEOUT_MS = 40000

# Variables de tiempo para control de actualizaciones
last_update_time = 0
update_interval_ms = 200

# Definición de funciones
def setup_nrf():
    spi = SPI(SPI_ID, sck=Pin(SCK), mosi=Pin(MOSI), miso=Pin(MISO))
    csn = Pin(CSN, Pin.OUT, value=1)
    ce = Pin(CE, Pin.OUT, value=0)
    nrf = NRF24L01(spi, csn, ce, payload_size=8)
    nrf.set_channel(10)
    for pipe, addr in TX_ADDRESSES.items():
        nrf.open_rx_pipe(pipe, addr)
    return nrf

def update_history(tx_id, rssi):
    global last_rssi, rssi_history, last_received_time
    last_rssi[tx_id] = rssi
    last_received_time[tx_id] = utime.ticks_ms()
    rssi_history[tx_id].append((last_received_time[tx_id], rssi))
    if len(rssi_history[tx_id]) > MAX_HISTORY_SIZE:
        rssi_history[tx_id].pop(0)

def check_timeouts():
    now = utime.ticks_ms()
    for tx_id in TX_ADDRESSES:
        if utime.ticks_diff(now, last_received_time[tx_id]) > RSSI_TIMEOUT_MS:
            if last_rssi[tx_id] != -100:
                last_rssi[tx_id] = -100  # Restablecer RSSI a -100
                print(f"TIMEOUT:TX{tx_id} no ha respondido en 40s, RSSI reiniciado.")

def send_data_to_computer():
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

def show_oled_status():
    oled.fill(0)
    oled.text("RSSI", 45, 0)
    oled.text(f"TX1: {last_rssi[1]} dBm", 0, 20)
    oled.text(f"TX2: {last_rssi[2]} dBm", 0, 30)
    oled.text(f"TX3: {last_rssi[3]} dBm", 0, 40)
    oled.text(f"TX4: {last_rssi[4]} dBm", 0, 50)
    oled.show()

def receiver_loop(nrf):
    global last_update_time
    nrf.start_listening()
    led = Pin("LED", Pin.OUT)
    print("SETUP:NRF24L01_RECEIVER")
    print("CONFIG:{\"transmitters\":[1,2,3,4], \"min_rssi\":-100, \"max_rssi\":-30}")

    while True:
        if nrf.any():
            led.on()
            try:
                buf = nrf.recv()
                if len(buf) == 8:
                    message_id, rssi = struct.unpack("ii", buf)
                    if message_id in TX_ADDRESSES:
                        update_history(message_id, rssi)
                    else:
                        print(f"DEBUG:ID desconocido: {message_id}")
                else:
                    print(f"DEBUG:Payload inesperado: {len(buf)} bytes")
            except Exception as e:
                print(f"DEBUG:Error al leer: {e}")
            led.off()

        current_time = utime.ticks_ms()
        if utime.ticks_diff(current_time, last_update_time) > update_interval_ms:
            check_timeouts()  # Verifica si algún TX no ha enviado en 40s
            send_data_to_computer()
            show_oled_status()
            last_update_time = current_time

        utime.sleep_ms(5)

def main():
    global oled
    try:
        i2c = I2C(1, scl=Pin(SCL), sda=Pin(SDA))
        oled = ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c)
        oled.fill(0)
        oled.text("Iniciando...", 0, 20)
        oled.text("NRF24L01 RX", 0, 30)
        oled.show()
        utime.sleep_ms(1000)
        nrf = setup_nrf()
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
