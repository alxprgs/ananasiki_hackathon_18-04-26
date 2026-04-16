from raspberry.raspberry_service import RaspberryService
from raspberry.arduino_service import ArduinoService


def main():
    with ArduinoService(retry_count=3) as robot:
        robot.set_led(0)


if __name__ == "__main__":
    main()