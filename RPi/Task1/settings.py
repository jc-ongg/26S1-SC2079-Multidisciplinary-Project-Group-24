import os

# Override per session; do not edit a teammate's address into source control.
SERIAL_PORT = os.environ.get("STM_SERIAL_PORT", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C068380-if00")
BAUD_RATE = 115200

# API DETAILS
API_IP = os.environ.get("LAPTOP_IP", "192.168.50.181")
API_PORT = int(os.environ.get("ALGO_PORT", "8000"))

#IMAGE API
API_IP_IMAGE = os.environ.get("CV_IP", API_IP)
API_PORT_IMAGE = int(os.environ.get("IMAGE_API_PORT", "8001"))

# ROBOT SETTINGS
OUTDOOR_BIG_TURN = False
