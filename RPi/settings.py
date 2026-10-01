# STM32 BOARD SERIAL CONNECTION
# The by-id name embeds the USB adapter's serial number. This board has two
# CH34x bridges; only ...8380 is wired to the STM's UART. If connect() fails,
# run `ls /dev/serial/by-id/` and paste the real name here.
SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C068380-if00"  # stm32 (CH34x adapter)
BAUD_RATE = 115200

# API DETAILS
# Path planning: the Algo server (Flask) running on the laptop.
API_IP = "192.168.50.147"  # laptop running the Algo server
API_PORT = 8000

# IMAGE API
# Recognition: image-api (best_v11i_m.onnx) on the same laptop. Algo keeps 8000.
API_IP_IMAGE = "192.168.50.147"
API_PORT_IMAGE = 8001

# ROBOT SETTINGS
OUTDOOR_BIG_TURN = False
