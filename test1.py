from raspberry.raspberry_service import RaspberryService
from raspberry.arduino_service import ArduinoService


def main():
    with ArduinoService(retry_count=3) as robot:
        print(robot.button_status())


if __name__ == "__main__":
    main()