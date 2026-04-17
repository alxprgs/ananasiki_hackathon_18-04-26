from raspberry.raspberry_service import RaspberryService


rpi = RaspberryService()

rpi.configure_ap(ssid="HighSchool777", password="Vf65gght")
rpi.enable_ap()