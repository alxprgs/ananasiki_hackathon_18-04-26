# Rescue Maze Low-Level Stack

Низкоуровневый стек для робота RoboCupJunior Rescue Maze с разделением ролей между `Arduino` и `Raspberry Pi 4`.

## Идея проекта

- `Arduino` управляет железом в реальном времени: моторы, датчики, кнопка и опциональные исполнительные модули.
- `Raspberry Pi 4` работает как управляющий уровень: отправляет атомарные команды в Arduino по USB serial и выполняет системные задачи, которые не относятся к микроконтроллеру.
- Обмен между устройствами идёт через построчный JSON-протокол: один запрос в строке, один ответ в строке.

## Структура репозитория

- [arduino/arduino.cpp](arduino/arduino.cpp) — прошивка Arduino.
- [raspberry/arduino_service.py](raspberry/arduino_service.py) — Python API для работы с Arduino по serial.
- [raspberry/raspberry_service.py](raspberry/raspberry_service.py) — системные команды Raspberry Pi для AP-режима и SSH.
- [raspberry/main.py](raspberry/main.py) — демонстрационная точка входа.
- [tests/test_arduino_service.py](tests/test_arduino_service.py) — unit-тесты serial API.
- [tests/test_raspberry_service.py](tests/test_raspberry_service.py) — unit-тесты системного сервиса Raspberry Pi.

## Архитектура

### Arduino

Прошивка реализует:

- управление двумя моторами через Motor Shield;
- чтение двух ультразвуковых датчиков без блокировки `loop()`;
- чтение кнопки;
- watchdog по времени отсутствия команд;
- timed-команды для моторов;
- поддержку опциональных модулей: `servo`, `relay`, `stepper`.

### Raspberry Pi

Python-слой разделён на два сервиса:

- `ArduinoService`:
  - ищет Arduino по serial;
  - проверяет соединение через `ping`;
  - предоставляет удобный Python API поверх протокола;
  - конвертирует расстояния в `mm`, `cm`, `m`.
- `RaspberryService`:
  - создаёт и настраивает AP-профиль через `nmcli`;
  - включает и отключает AP-режим;
  - восстанавливает прошлый Wi‑Fi-клиент;
  - завершает все активные SSH-сессии без остановки master `sshd`.

## Serial-протокол

Формат запроса:

```json
{"id": 1, "op": "set_motor", "args": {"target": "all", "pwm": 50, "duration_ms": 1000}}
```

Формат успешного ответа:

```json
{"id": 1, "ok": true, "data": {"target": "all", "pwm": 50, "duration_ms": 1000}}
```

Формат ответа с ошибкой:

```json
{"id": 1, "ok": false, "error": {"code": "bad_request", "message": "поле pwm обязательно"}}
```

## Доступные команды Arduino

- `ping`
- `get_status`
- `get_distance`
- `get_button`
- `set_motor`
- `stop_all`
- `set_servo`
- `set_relay`
- `stepper_move`
- `stepper_stop`

## Публичный Python API

### ArduinoService

Примеры:

```python
from raspberry.arduino_service import ArduinoService

with ArduinoService() as arduino:
    print(arduino.ping())
    print(arduino.status())
    print(arduino.distance_sensor.get(1, unit="cm"))
    print(arduino.button_status())

    arduino.eng_all.pwm(40).time(1.5)
    arduino.eng_l.pwm(25).now()
    arduino.stop_all()

    arduino.servo.set(90)
    arduino.relay.on()
    arduino.stepper.move(steps=200, rpm=30, direction="forward")
    arduino.stepper.stop()
```

Основные фасады:

- `distance_sensor.get(sensor_id, unit="mm")`
- `button_status()`
- `eng_all.pwm(percent).time(seconds)`
- `eng_l.pwm(percent).now()`
- `eng_r.pwm(percent).now()`
- `servo.set(angle_deg)`
- `relay.on()`, `relay.off()`, `relay.set(enabled)`
- `stepper.move(...)`, `stepper.stop()`

### RaspberryService

Примеры:

```python
from raspberry.raspberry_service import RaspberryService

service = RaspberryService()
service.configure_ap("RescueMazeRobot", "RescueMaze123", channel=6)
service.enable_ap()
service.disable_ap()
service.disconnect_all_ssh()
```

Доступные методы:

- `configure_ap(ssid, password, channel=1, ipv4_cidr="192.168.4.1/24")`
- `enable_ap()`
- `disable_ap()`
- `disconnect_all_ssh()`

## Распиновка по умолчанию

В прошивке сейчас заложена следующая базовая схема:

- ультразвуковой датчик 1: `TRIG D2`, `ECHO D3`
- левый мотор: `DIR D4`, `PWM D5`
- правый мотор: `PWM D6`, `DIR D7`
- ультразвуковой датчик 2: `TRIG D8`, `ECHO D9`
- сервопривод: `D10`
- реле: `D11`
- шаговый двигатель: `STEP D12`, `DIR D13`, `ENABLE A0`
- кнопка: `A1`

При необходимости это меняется в конфигурационном блоке в начале [arduino/arduino.cpp](arduino/arduino.cpp).

## Установка зависимостей

Для Raspberry Pi:

```bash
pip install -r raspberry/requirements.txt
```

Для AP-режима нужен `NetworkManager` и утилита `nmcli`.

## Демонстрационный запуск

```bash
python -m raspberry.main --motion-percent 30 --motion-seconds 1.0
```

Пример с включением точки доступа:

```bash
python -m raspberry.main --enable-ap --ap-ssid RescueMazeRobot --ap-password RescueMaze123
```

## Тесты

Локальные тесты Python запускаются так:

```bash
python -m unittest discover -s tests -v
```

Они проверяют:

- поиск и проверку Arduino по serial;
- согласование request/response по `id`;
- конвертацию единиц расстояния;
- fluent API моторов;
- формирование команд для `nmcli`;
- выбор процессов для завершения SSH-сессий.

## Важные замечания

- AP-режим нужен для настройки и отладки, а не для официального заезда.
- Движение сейчас open-loop: без энкодеров время и PWM не дают точной одометрии.
- Некоторые команды Raspberry Pi требуют повышенных прав.
  Проверка этих прав встроена в `RaspberryService` и срабатывает только на реальной Raspberry Pi.
- Опциональные модули `servo`, `relay`, `stepper` можно отключать в конфигурации прошивки; в этом случае Arduino будет возвращать явную ошибку `unsupported`.
