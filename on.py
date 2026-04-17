from raspberry.raspberry_service import RaspberryService


rpi = RaspberryService()

rpi.configure_ap(ssid="Ananasiki", password="123456789")
rpi.enable_ap()