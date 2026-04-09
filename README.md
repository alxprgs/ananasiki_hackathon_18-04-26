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
    print(arduino.align_parallel_to_wall(wall_side="right", tolerance_mm=8.0))

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
- `align_parallel_to_wall(front_sensor_id=1, rear_sensor_id=2, wall_side="right", ...)`
- `eng_all.pwm(percent).time(seconds)`
- `eng_l.pwm(percent).now()`
- `eng_r.pwm(percent).now()`
- `servo.set(angle_deg)`
- `relay.on()`, `relay.off()`, `relay.set(enabled)`
- `stepper.move(...)`, `stepper.stop()`

#### Как создавать и закрывать `ArduinoService`

`ArduinoService(port=None, baudrate=115200, timeout=1.0, retry_count=1)` создаёт соединение с Arduino сразу в момент инициализации.

Параметры:

- `port` — явный serial-порт, например `"/dev/ttyACM0"` или `"COM4"`. Если не указан, сервис попытается найти Arduino автоматически и проверит кандидата через `ping()`.
- `baudrate` — скорость serial-соединения. По умолчанию `115200`, она должна совпадать с прошивкой Arduino.
- `timeout` — таймаут чтения и записи в секундах. Если Arduino не отвечает дольше этого времени, будет выброшена ошибка таймаута.
- `retry_count` — сколько раз повторять идемпотентные команды чтения, например `ping()` или `distance_sensor.get()`, если произошёл таймаут или протокольная ошибка.

Рекомендуемый способ использования:

```python
from raspberry.arduino_service import ArduinoService

with ArduinoService(port="/dev/ttyACM0") as arduino:
    print(arduino.ping())
```

Полезные свойства и методы жизненного цикла:

- `arduino.is_closed` — показывает, закрыт ли сервис.
- `arduino.close()` — вручную закрывает serial-соединение.
- `with ArduinoService(...) as arduino:` — предпочтительный способ, потому что соединение закроется автоматически.

#### `ping()`

Назначение:

- проверяет, что Arduino отвечает по протоколу;
- удобно использовать как первый health-check после запуска;
- вызывается автоматически во время автоопределения порта.

Что возвращает:

- словарь `data` из ответа Arduino;
- обычно там есть `pong=True` и строка версии прошивки.

Пример:

```python
reply = arduino.ping()
if reply.get("pong"):
    print("Связь с Arduino есть")
```

Когда использовать:

- сразу после старта программы;
- перед началом движения;
- при диагностике проблем со связью.

#### `status()`

Назначение:

- получает общий снимок состояния Arduino и подключённой периферии;
- позволяет быстро понять, что происходит с моторами, кнопкой, watchdog и датчиками.

Что возвращает:

- словарь `data` с текущими полями состояния, которые отдаёт прошивка;
- обычно это статус кнопки, текущие значения PWM, последние измеренные расстояния и другие служебные поля.

Пример:

```python
state = arduino.status()
print(state)
```

Когда использовать:

- для отладки;
- при выводе телеметрии;
- перед выполнением сложной последовательности команд.

#### `button_status()`

Назначение:

- читает текущее состояние пользовательской кнопки на Arduino.

Что возвращает:

- `True`, если кнопка нажата;
- `False`, если кнопка отпущена.

Пример:

```python
if arduino.button_status():
    print("Кнопка нажата")
```

Когда использовать:

- как локальный старт миссии;
- как аварийный или пользовательский триггер;
- для простого ручного режима.

#### `distance_sensor.get(sensor_id, unit="mm")`

Назначение:

- читает расстояние с одного ультразвукового датчика;
- умеет автоматически конвертировать миллиметры в сантиметры или метры.

Параметры:

- `sensor_id` — номер датчика, только `1` или `2`.
- `unit` — единица измерения: `"mm"`, `"cm"` или `"m"`.

Что возвращает:

- число `float` в выбранной единице измерения.

Примеры:

```python
distance_mm = arduino.distance_sensor.get(1)
distance_cm = arduino.distance_sensor.get(2, unit="cm")
```

Когда использовать:

- для навигации вдоль стены;
- для проверки препятствия;
- как базовое измерение перед принятием решения.

#### `align_parallel_to_wall(...)`

Назначение:

- пытается развернуть робота так, чтобы он стоял примерно параллельно стене;
- рассчитан на конфигурацию, где два датчика смотрят в одну сторону робота: один ближе к переду, второй ближе к задней части;
- сравнивает показания датчиков и делает короткие импульсы поворота на месте.

Основные параметры:

- `front_sensor_id` — датчик ближе к носу робота.
- `rear_sensor_id` — датчик ближе к хвосту робота.
- `wall_side` — сторона, с которой находится стена: `"right"` или `"left"`.
- `tolerance_mm` — допустимая разница между показаниями датчиков.
- `turn_power` — мощность корректирующего поворота в процентах.
- `pulse_seconds` — длительность одного импульса поворота.
- `settle_seconds` — пауза после остановки, чтобы показания стабилизировались.
- `max_iterations` — максимальное число циклов измерения и коррекции.

Что возвращает:

- словарь с результатом выравнивания;
- ключ `aligned` показывает, удалось ли уложиться в допуск;
- ключ `delta_mm` содержит финальную разницу `front - rear`;
- ключ `last_turn_direction` показывает последнее направление коррекции.

Пример:

```python
result = arduino.align_parallel_to_wall(
    front_sensor_id=1,
    rear_sensor_id=2,
    wall_side="right",
    tolerance_mm=8.0,
    turn_power=18,
    pulse_seconds=0.12,
    settle_seconds=0.05,
    max_iterations=10,
)

if result["aligned"]:
    print("Робот выровнен относительно стены")
else:
    print("Не удалось уложиться в допуск")
```

Когда использовать:

- перед движением вдоль стены;
- после неаккуратного поворота;
- когда нужно подровнять корпус перед точным манёвром.

Важно:

- метод работает на стороне Raspberry Pi поверх уже существующих команд `get_distance`, `set_motor` и `stop_all`;
- отдельная команда в прошивке Arduino для него не нужна;
- это приближённое выравнивание, зависящее от реальных датчиков, люфтов, покрытия и подобранных параметров импульса.

#### Управление моторами: `eng_all`, `eng_l`, `eng_r`

Назначение:

- `eng_all` управляет сразу обоими моторами;
- `eng_l` управляет только левым мотором;
- `eng_r` управляет только правым мотором.

Все три фасада работают одинаково и начинаются с вызова `.pwm(percent)`.

##### `pwm(percent)`

Назначение:

- задаёт мощность мотора или группы моторов;
- положительные значения означают движение вперёд;
- отрицательные — движение назад;
- диапазон значений ограничен интервалом от `-100` до `100`.

Что возвращает:

- объект команды, у которого потом нужно вызвать `.time(seconds)` или `.now()`.

Примеры:

```python
arduino.eng_all.pwm(40).time(1.5)
arduino.eng_l.pwm(25).now()
arduino.eng_r.pwm(-30).time(0.3)
```

##### `.time(seconds)`

Назначение:

- отправляет одну атомарную команду на Arduino;
- моторы будут работать с заданной мощностью указанное количество секунд, после чего Arduino остановит их сама.

Параметры:

- `seconds` — длительность работы в секундах, должна быть больше нуля.

Что возвращает:

- словарь `data` из ответа Arduino.

Когда использовать:

- для коротких точных импульсов движения;
- для разворотов по времени;
- в сценариях, где не хочется отдельно отправлять `stop_all()`.

##### `.now()`

Назначение:

- отправляет команду без ограничения по времени;
- моторы будут работать, пока не придёт новая команда, `stop_all()` или не сработает watchdog Arduino.

Что возвращает:

- словарь `data` из ответа Arduino.

Когда использовать:

- для непрерывного движения;
- в циклах ручного управления;
- когда остановка определяется внешней логикой.

#### `stop_all()`

Назначение:

- немедленно останавливает оба моторных канала;
- также останавливает шаговый двигатель, если он был активен.

Что возвращает:

- словарь `data` из ответа Arduino с подтверждением остановки.

Пример:

```python
arduino.stop_all()
```

Когда использовать:

- как аварийный стоп;
- между манёврами;
- перед закрытием программы.

#### Управление сервоприводом: `servo.set(angle_deg)` и `set_servo(angle_deg)`

Назначение:

- `servo.set(angle_deg)` — удобный фасад для прикладного кода;
- `set_servo(angle_deg)` — прямой метод `ArduinoService`, который делает то же самое.

Параметры:

- `angle_deg` — желаемый угол в градусах.

Что возвращает:

- словарь `data` с подтверждённым углом, который приняла Arduino.

Пример:

```python
arduino.servo.set(90)
```

Когда использовать:

- для управления механизмом сброса;
- для поворота дополнительного узла;
- для тестирования сервы.

Важно:

- если сервопривод отключён в конфигурации прошивки, будет выброшена ошибка `UnsupportedHardwareError`;
- на стороне Arduino угол дополнительно ограничивается допустимым диапазоном.

#### Управление реле: `relay.on()`, `relay.off()`, `relay.set(enabled)` и `set_relay(enabled)`

Назначение:

- `relay.on()` — включает реле;
- `relay.off()` — выключает реле;
- `relay.set(enabled)` — устанавливает произвольное состояние;
- `set_relay(enabled)` — прямой низкоуровневый метод `ArduinoService`.

Параметры:

- `enabled` — `True`, если реле нужно включить, и `False`, если выключить.

Что возвращает:

- словарь `data` с финальным состоянием реле.

Примеры:

```python
arduino.relay.on()
arduino.relay.off()
arduino.relay.set(True)
```

Когда использовать:

- для включения дополнительной нагрузки;
- для активации механизма через силовой канал;
- для ручной проверки релейного выхода.

#### Управление шаговым двигателем: `stepper.move(...)`, `stepper.stop()`, `move_stepper(...)`, `stop_stepper()`

Назначение:

- `stepper.move(...)` и `stepper.stop()` — удобные фасады для прикладного кода;
- `move_stepper(...)` и `stop_stepper()` — прямые методы `ArduinoService`.

Параметры `stepper.move(...)`:

- `steps` — число шагов или `None`, если нужно непрерывное движение.
- `rpm` — скорость в оборотах в минуту.
- `direction` — направление `"forward"` или `"reverse"`.
- `duration` — ограничение по времени в секундах или `None`.

Параметры `move_stepper(...)`:

- `steps` — число шагов или `None`.
- `rpm` — скорость в оборотах в минуту.
- `direction` — `"forward"` или `"reverse"`.
- `duration_ms` — ограничение времени в миллисекундах.

Что возвращает:

- словарь `data` с подтверждением запуска или остановки.

Примеры:

```python
arduino.stepper.move(steps=200, rpm=30, direction="forward")
arduino.stepper.move(rpm=20, direction="reverse", duration=1.2)
arduino.stepper.stop()
```

Когда использовать:

- для линейного механизма;
- для вращения захвата или сброса;
- для любых точных повторяемых движений, где обычного DC-мотора недостаточно.

Важно:

- если шаговый двигатель отключён в конфигурации прошивки, сервис выбросит `UnsupportedHardwareError`;
- `stepper.move(duration=...)` сам переводит секунды в миллисекунды и отправляет уже готовую команду в Arduino.

#### Основные ошибки `arduino_service.py`

В прикладном коде полезно понимать, какие исключения может выбросить сервис:

- `ArduinoDependencyError` — в системе нет `pyserial`.
- `ArduinoUnavailableError` — Arduino не найдена или соединение не удалось установить.
- `SerialTimeoutError` — Arduino не ответила в пределах таймаута.
- `ArduinoProtocolError` — ответ повреждён, не совпал `id` или прошивка вернула ошибку.
- `UnsupportedHardwareError` — вызвана команда для опционального модуля, который отключён в прошивке.

Пример обработки:

```python
from raspberry.arduino_service import (
    ArduinoProtocolError,
    ArduinoService,
    ArduinoUnavailableError,
    SerialTimeoutError,
)

try:
    with ArduinoService() as arduino:
        arduino.eng_all.pwm(35).time(0.5)
except ArduinoUnavailableError:
    print("Arduino не найдена")
except SerialTimeoutError:
    print("Arduino не отвечает вовремя")
except ArduinoProtocolError as exc:
    print(f"Ошибка протокола: {exc}")
```

### RaspberryService

Примеры:

```python
from raspberry.raspberry_service import RaspberryService

service = RaspberryService()
service.configure_ap("RescueMazeRobot", "RescueMaze123", channel=6)
service.configure_ap("RescueMazeRobot", "RescueMaze123", channel="auto")
service.enable_ap()
service.disable_ap()
service.disconnect_all_ssh()
print(service.get_temperature_telemetry())
print(service.get_power_telemetry())
print(service.get_board_telemetry())
```

Доступные методы:

- `configure_ap(ssid, password, channel=1, ipv4_cidr="192.168.4.1/24")`
- `enable_ap()`
- `disable_ap()`
- `disconnect_all_ssh()`
- `select_ap_channel()`
- `get_temperature_telemetry()`
- `get_power_telemetry()`
- `get_board_telemetry()`

#### Как использовать `RaspberryService`

`RaspberryService()` не открывает постоянное соединение и не держит фоновых потоков. Это обычный объект-обёртка над системными командами Linux и NetworkManager.

Что важно понимать:

- методы AP и управления SSH работают через системные утилиты вроде `nmcli`, `ps` и `vcgencmd`;
- часть методов на реальной Raspberry Pi требует повышенных прав;
- проверки прав встроены в сервис и выполняются только на самой Raspberry Pi, а не на обычном ПК разработчика.

##### `configure_ap(ssid, password, channel=1, ipv4_cidr="192.168.4.1/24")`

Назначение:

- создаёт профиль точки доступа в `NetworkManager`, если его ещё нет;
- обновляет параметры существующего AP-профиля;
- подготавливает Raspberry Pi к последующему вызову `enable_ap()`.

Основные параметры:

- `ssid` — имя будущей Wi‑Fi сети;
- `password` — пароль WPA-PSK, минимум 8 символов;
- `channel` — номер канала `1..13` или строка `"auto"` для автоматического выбора;
- `ipv4_cidr` — IP-адрес и маска интерфейса точки доступа, например `"192.168.4.1/24"`.

Пример:

```python
service.configure_ap(
    ssid="RescueMazeRobot",
    password="RescueMaze123",
    channel="auto",
    ipv4_cidr="192.168.4.1/24",
)
```

Когда использовать:

- один раз при начальной настройке;
- при изменении имени, пароля или адреса AP;
- если нужно пересобрать профиль без ручной работы в `nmcli`.

##### `select_ap_channel()`

Назначение:

- подбирает канал для точки доступа автоматически;
- анализирует, какие 2.4 ГГц каналы поддерживает адаптер;
- оценивает занятость эфира по соседним сетям;
- старается выбирать наименее загруженный непересекающийся канал.

Пример:

```python
best_channel = service.select_ap_channel()
print(f"Лучший канал для AP: {best_channel}")
```

Когда использовать:

- если не хочется фиксировать канал вручную;
- когда робот запускается в разных помещениях с разной загрузкой эфира;
- как подготовительный шаг перед `configure_ap(..., channel="auto")`.

##### `enable_ap()`

Назначение:

- включает ранее настроенный профиль точки доступа;
- отключает конфликтующее клиентское Wi‑Fi-подключение;
- запоминает прошлый клиентский профиль, чтобы потом попробовать восстановить его.

Пример:

```python
service.enable_ap()
```

Когда использовать:

- перед подключением к Raspberry Pi по собственному AP;
- в сценарии настройки робота без внешнего роутера;
- при полевой отладке.

##### `disable_ap()`

Назначение:

- выключает AP-профиль;
- пытается вернуть прошлое клиентское Wi‑Fi-подключение, если оно было сохранено при `enable_ap()`.

Пример:

```python
service.disable_ap()
```

Когда использовать:

- после завершения настройки через AP;
- чтобы вернуть Raspberry Pi в обычный клиентский режим;
- перед штатным использованием робота без точки доступа.

##### `disconnect_all_ssh()`

Назначение:

- завершает все активные SSH-сессии;
- не трогает основной master-процесс `sshd`, чтобы сама служба SSH не падала.

Что возвращает:

- список PID, которым был отправлен `SIGTERM`.

Пример:

```python
terminated_pids = service.disconnect_all_ssh()
print(terminated_pids)
```

Когда использовать:

- перед переходом робота в автономный режим;
- если нужно гарантированно разорвать внешние SSH-подключения;
- в сценариях безопасности, где нельзя оставлять открытые интерактивные сессии.

#### Телеметрия Raspberry Pi

`RaspberryService` теперь умеет читать встроенную телеметрию самой платы.

Важно:

- это телеметрия именно Raspberry Pi, а не аккумулятора робота;
- без внешнего датчика питания сервис не может честно измерить заряд батареи, ток потребления или напряжение на входе всего робота;
- зато он может показать температуру SoC и штатные флаги проблем с питанием самой Raspberry Pi.

##### `get_temperature_telemetry()`

Назначение:

- читает температуру процессора/SoC Raspberry Pi;
- сначала пытается использовать Linux sysfs;
- если sysfs недоступен, делает резервную попытку через `vcgencmd measure_temp`.

Что возвращает:

- `celsius` — температура в градусах Цельсия;
- `fahrenheit` — та же температура в градусах Фаренгейта;
- `state` — грубая оценка нагрева: `normal`, `warm`, `hot`, `critical`;
- `source` — источник, из которого удалось взять данные.

Пример:

```python
temperature = service.get_temperature_telemetry()
print(f"Температура: {temperature['celsius']} C")
if temperature["state"] in {"hot", "critical"}:
    print("Нужно снизить нагрузку или улучшить охлаждение")
```

Когда использовать:

- перед запуском тяжёлой логики на Raspberry Pi;
- периодически во время заезда или отладки;
- для логирования перегрева.

##### `get_power_telemetry()`

Назначение:

- читает встроенные флаги состояния питания Raspberry Pi;
- показывает, не было ли признаков просадки питания, ограничения частоты или throttling;
- дополнительно пытается считать внутреннее напряжение ядра через `vcgencmd measure_volts core`.

Что возвращает:

- `throttled_raw` — исходная строка из `vcgencmd get_throttled`;
- `throttled_mask` — та же информация в виде числа;
- `core_voltage_volts` — напряжение ядра, если его удалось получить;
- `voltage_source` — источник напряжения или `None`;
- `undervoltage_now` — есть ли прямо сейчас признак просадки питания;
- `undervoltage_occurred` — был ли такой признак с момента загрузки;
- `frequency_capped_now` и `frequency_capped_occurred` — ограничивается ли частота сейчас или ограничивалась раньше;
- `throttled_now` и `throttling_occurred` — находится ли плата в throttling сейчас или попадала туда раньше;
- `soft_temperature_limit_now` и `soft_temperature_limit_occurred` — достигнут ли мягкий температурный лимит;
- `power_good_now` — нет ли сейчас признака просадки питания;
- `performance_limited_now` — ограничена ли производительность платы в текущий момент.

Пример:

```python
power = service.get_power_telemetry()
if not power["power_good_now"]:
    print("У Raspberry Pi сейчас есть признак просадки питания")
if power["throttling_occurred"]:
    print("Плата уже сталкивалась с throttling после загрузки")
```

Когда использовать:

- при диагностике случайных перезагрузок и подвисаний;
- если камера, Wi‑Fi или USB-нагрузка делают систему нестабильной;
- чтобы понимать, хватает ли питания Raspberry Pi в реальном роботе.

##### `get_board_telemetry()`

Назначение:

- собирает оба блока телеметрии одним вызовом;
- удобно для логирования, отладочного API и периодического мониторинга.

Что возвращает:

- словарь с ключами `temperature` и `power`.

Пример:

```python
telemetry = service.get_board_telemetry()
print(telemetry["temperature"]["celsius"])
print(telemetry["power"]["power_good_now"])
```

Автовыбор канала:

- доступен через `configure_ap(..., channel="auto")` или отдельный вызов `select_ap_channel()`;
- учитывает только каналы, которые реально поддерживает Wi‑Fi адаптер Raspberry Pi;
- старается выбрать наименее загруженный канал в диапазоне 2.4 ГГц;
- приоритетно использует непересекающиеся каналы `1`, `6` и `11`.

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
- выбор процессов для завершения SSH-сессий;
- разбор температурной и энергетической телеметрии Raspberry Pi.

## Важные замечания

- AP-режим нужен для настройки и отладки, а не для официального заезда.
- Движение сейчас open-loop: без энкодеров время и PWM не дают точной одометрии.
- Некоторые команды Raspberry Pi требуют повышенных прав.
  Проверка этих прав встроена в `RaspberryService` и срабатывает только на реальной Raspberry Pi.
- Опциональные модули `servo`, `relay`, `stepper` можно отключать в конфигурации прошивки; в этом случае Arduino будет возвращать явную ошибку `unsupported`.
