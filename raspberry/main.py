from __future__ import annotations

import time

try:
    from .arduino_service import ArduinoService
    from .raspberry_service import RaspberryService
except ImportError:  # pragma: no cover - позволяет запускать файл напрямую
    from arduino_service import ArduinoService  # type: ignore[no-redef]
    from raspberry_service import RaspberryService  # type: ignore[no-redef]

def main():
    with ArduinoService(port="COM3") as robot:
        robot.start_activity_session()
        while True:
            print(f"Растояние первого датчика в мм: {robot.distance_sensor.get(1)}")
            print(f"Растояние второго датчика в мм: {robot.distance_sensor.get(2)}")
            time.sleep(1)



if __name__ == "__main__":
    main()
