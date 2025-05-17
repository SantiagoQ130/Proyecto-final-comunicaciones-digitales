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
TX_ADDRESS = b"\xc3\xf0\xf0\xf0\xf0"  # Asegúrate que coincida con el RX

# Config WiFi
SSID = "ESP32-123456"
PASSWORD = "12345678"

# ID del transmisor
TX_ID = 2

def setup_nrf24l01():
    spi = SPI(SPI_ID, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    csn = Pin(CSN_PIN, mode=Pin.OUT, value=1)
    ce = Pin(CE_PIN, mode=Pin.OUT, value=0)

    nrf = NRF24L01(spi, csn, ce, payload_size=8)  # 4 bytes ID + 4 bytes RSSI
    nrf.set_channel(CANAL_RF)
    nrf.reg_write(0x06, 0x26)  # 250kbps, máxima potencia
    nrf.open_tx_pipe(TX_ADDRESS)
    print("NRF24L01 configurado correctamente")
    return nrf

def conectar_wifi():
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    if not wifi.isconnected():
        print(f"Conectando a {SSID}...")
        wifi.connect(SSID, PASSWORD)
        for _ in range(10):
            if wifi.isconnected():
                break
            utime.sleep(1)
    if wifi.isconnected():
        print("Conectado al WiFi!")
        return wifi
    print("No se pudo conectar al WiFi")
    return None

def get_wifi_strength(wifi):
    try:
        return wifi.status('rssi') if wifi and wifi.isconnected() else -100
    except:
        return -100

def setup_oled():
    i2c = I2C(1, scl=Pin(SCL), sda=Pin(SDA))  # Usamos I2C(1)
    oled = SSD1306_I2C(WIDTH, HEIGHT, i2c)
    oled.fill(0)
    oled.text("Iniciando...", 0, 0)
    oled.show()
    return oled

def mostrar_en_oled(oled, rssi, ssid, ip):
    oled.fill(0)
    oled.text("TX2", 45, 10)
    oled.text(f"SSID: {ssid}", 0, 20)
    oled.text(f"IP: {ip}", 0, 30)
    oled.text(f"RSSI: {rssi} dBm", 0, 40)
    oled.show()

def transmit_message(nrf, rssi):
    led.on()
    try:
        mensaje = struct.pack("ii", TX_ID, rssi)
        nrf.send(mensaje)
        print(f"Enviado -> TX{TX_ID} = {rssi} dBm")
    except Exception as e:
        print(f"Error al enviar: {e}")
    led.off()

def main():
    print("Iniciando TX NRF24L01...")
    wifi = conectar_wifi()
    nrf = setup_nrf24l01()
    oled = setup_oled()
    buffer = []

    # Obtener info de red
    ssid = wifi.config('essid') if wifi and wifi.isconnected() else "Desconocido"
    ip = wifi.ifconfig()[0] if wifi and wifi.isconnected() else "0.0.0.0"

    while True:
        rssi = get_wifi_strength(wifi)
        buffer.append(rssi)
        if len(buffer) > 10:
            buffer.pop(0)
        promedio = sum(buffer) // len(buffer)

        mostrar_en_oled(oled, promedio, ssid, ip)
        transmit_message(nrf, promedio)

        utime.sleep(1)

if __name__ == "__main__":
    main()
