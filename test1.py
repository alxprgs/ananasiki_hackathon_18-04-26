from raspberry.raspberry_service import RaspberryService
from raspberry.arduino_service import ArduinoService


def main():
    with ArduinoService() as robot:
        robot.servo.set(0)

if __name__ == "__main__":
    main()