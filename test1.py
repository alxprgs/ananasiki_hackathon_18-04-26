from raspberry.raspberry_service import RaspberryService
from raspberry.arduino_service import ArduinoService


def main():
    with ArduinoService(retry_count=3) as robot:
        print(f"Попытка получения данных с первого сенсора: {robot.distance_sensor.info(1)}")
        try:
            print(f"Данные с первого сенсора успешно получены! Растояние в ММ: {robot.distance_sensor.get(1)}")
        except Exception as e:
            print(f"Ошибка получения данных с первого сенсора: {e}")
        print(f"Попытка получения данных со второго сенсора: {robot.distance_sensor.info(2)}")
        try:
            print(f"Данные со второго успешно получены! Растояние в ММ: {robot.distance_sensor.get(2)}")
        except Exception as e:
            print(f"Ошибка получения данных со второго сенсора: {e}")



if __name__ == "__main__":
    main()