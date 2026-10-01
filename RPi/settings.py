# STM32 BOARD SERIAL CONNECTION
SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C068380-if00"  # stm32 (CH34x adapter)
BAUD_RATE = 115200

# API DETAILS
API_IP = "192.168.50.181"  # laptop running the Algo server
API_PORT = 8000

#IMAGE API
API_IP_IMAGE="192.168.50.181"
API_PORT_IMAGE=8001  # image-api (best_v11i_m.onnx); algo keeps 8000

# ROBOT SETTINGS
OUTDOOR_BIG_TURN = False
