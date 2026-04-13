from __future__ import annotations


try:
    from .arduino_service import ArduinoService
    from .raspberry_service import RaspberryService
except ImportError:  # pragma: no cover - позволяет запускать файл напрямую
    from arduino_service import ArduinoService  # type: ignore[no-redef]
    from raspberry_service import RaspberryService  # type: ignore[no-redef]

def main():
    with ArduinoService(port="COM3") as robot:
        robot.start_activity_session(include_map=True)
        robot.eng_r.pwm(60).time(1)

if __name__ == "__main__":  
    main()
