"""Клиентский API для управления Arduino по serial-протоколу Rescue Maze.

Модуль инкапсулирует:
- поиск и валидацию Arduino по USB serial;
- обмен newline-delimited JSON сообщениями;
- удобные Python-объекты для моторов, датчиков и опциональных актуаторов.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass
from itertools import count
from typing import Any, Callable, Literal

try:
    import serial  # type: ignore[import-not-found]
    from serial.tools import list_ports  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - проверяется через внедрение зависимостей в тестах
    serial = None
    list_ports = None


LOGGER = logging.getLogger(__name__)
UnitName = Literal["mm", "cm", "m"]
MotorTarget = Literal["all", "left", "right"]
WallSide = Literal["left", "right"]
RotationDirection = Literal["left", "right"]


class ArduinoServiceError(RuntimeError):
    """Базовое исключение для ошибок интеграции Raspberry Pi <-> Arduino."""


class ArduinoDependencyError(ArduinoServiceError):
    """Выбрасывается, когда отсутствуют runtime-зависимости, например pyserial."""


class ArduinoUnavailableError(ArduinoServiceError):
    """Выбрасывается, когда Arduino недоступна или не проходит проверку."""


class ArduinoProtocolError(ArduinoServiceError):
    """Выбрасывается, когда serial-протокол повреждён или вернул ошибку."""


class UnsupportedHardwareError(ArduinoProtocolError):
    """Выбрасывается, когда команда обращается к отключённому опциональному модулю."""


class SerialTimeoutError(ArduinoProtocolError):
    """Выбрасывается, когда Arduino не отвечает в пределах заданного таймаута."""


def _default_serial_factory(**kwargs: Any) -> Any:
    if serial is None:
        raise ArduinoDependencyError(
            "Для обмена с Arduino нужен pyserial. Установите его командой "
            "`pip install -r raspberry/requirements.txt`."
        )
    return serial.Serial(**kwargs)


def _default_port_enumerator() -> list[Any]:
    if list_ports is None:
        raise ArduinoDependencyError(
            "Для поиска serial-портов нужен pyserial. Установите его командой "
            "`pip install -r raspberry/requirements.txt`."
        )
    return list(list_ports.comports())


def _clamp_pwm(percent: float) -> int:
    bounded = max(-100.0, min(100.0, float(percent)))
    return int(round(bounded))


def _seconds_to_ms(seconds: float) -> int:
    if seconds <= 0:
        raise ValueError("seconds должно быть больше нуля")
    milliseconds = int(round(seconds * 1000))
    if milliseconds <= 0:
        raise ValueError("seconds должно давать положительную длительность")
    return milliseconds


def _convert_distance(distance_mm: float, unit: UnitName) -> float:
    if unit == "mm":
        return distance_mm
    if unit == "cm":
        return distance_mm / 10.0
    if unit == "m":
        return distance_mm / 1000.0
    raise ValueError(f"Неподдерживаемая единица измерения: {unit}")


@dataclass(slots=True)
class _StepperMoveOptions:
    direction: Literal["forward", "reverse"] = "forward"
    rpm: float = 60.0
    steps: int | None = None
    duration_ms: int | None = None


class ArduinoService:
    """Высокоуровневый API управления поверх построчного JSON serial-протокола.

    Экземпляр класса устанавливает соединение с Arduino, проверяет его через
    команду ``ping`` и затем предоставляет готовые команды для:
    - чтения расстояния с ультразвуковых датчиков;
    - чтения состояния кнопки;
    - управления моторами гусениц;
    - управления сервоприводом, реле и шаговым двигателем.

    Публичные атрибуты ``distance_sensor``, ``eng_all``, ``eng_l``, ``eng_r``,
    ``servo``, ``relay`` и ``stepper`` являются частью рабочего API и
    используются как удобные фасады поверх базового serial-протокола.
    """

    def __init__(
        self,
        port: str | None = None,
        *,
        baudrate: int = 115200,
        timeout: float = 1.0,
        retry_count: int = 1,
        logger: logging.Logger | None = None,
        serial_factory: Callable[..., Any] | None = None,
        port_enumerator: Callable[[], list[Any]] | None = None,
    ) -> None:
        """Создаёт сервис обмена с Arduino и немедленно открывает соединение.

        Args:
            port: Явный serial-порт Arduino. Если ``None``, используется
                автопоиск по доступным портам с последующей проверкой через
                ``ping``.
            baudrate: Скорость serial-соединения.
            timeout: Таймаут чтения и записи serial-соединения в секундах.
            retry_count: Количество повторов для идемпотентных команд чтения.
            logger: Пользовательский logger. Если не передан, используется
                модульный logger.
            serial_factory: Точка расширения для подмены конструктора serial-
                подключения, полезна для тестов.
            port_enumerator: Точка расширения для подмены механизма поиска
                serial-портов, полезна для тестов.

        Raises:
            ArduinoDependencyError: Если в среде отсутствует pyserial.
            ArduinoUnavailableError: Если Arduino не найдена.
            ArduinoProtocolError: Если найденное устройство не прошло проверку
                протокола.
        """
        self._logger = logger or LOGGER
        self._serial_factory = serial_factory or _default_serial_factory
        self._port_enumerator = port_enumerator or _default_port_enumerator
        self._timeout = timeout
        self._retry_count = retry_count
        self._request_ids = count(1)
        self._lock = threading.RLock()
        self._closed = False

        self.port = self._connect(port, baudrate)

        self.distance_sensor = DistanceSensorAccessor(self)
        self.eng_all = MotorChannel(self, "all")
        self.eng_l = MotorChannel(self, "left")
        self.eng_r = MotorChannel(self, "right")
        self.servo = ServoController(self)
        self.relay = RelayController(self)
        self.stepper = StepperController(self)

    def _connect(self, requested_port: str | None, baudrate: int) -> str:
        if requested_port is not None:
            try:
                connection = self._open_serial(requested_port, baudrate)
                self._validate_connection(connection, requested_port)
                self._serial = connection
                return requested_port
            except Exception as exc:
                self._logger.fatal("Не удалось инициализировать Arduino на %s: %s", requested_port, exc)
                raise

        last_error: Exception | None = None
        for candidate in self._discover_candidate_ports():
            try:
                connection = self._open_serial(candidate, baudrate)
                self._validate_connection(connection, candidate)
                self._serial = connection
                self._logger.info("Arduino подключена на %s", candidate)
                return candidate
            except Exception as exc:
                last_error = exc
                self._logger.debug("Кандидат serial-порта %s отклонён: %s", candidate, exc)

        if last_error is None:
            last_error = ArduinoUnavailableError("Не найдено ни одного serial-порта.")

        self._logger.fatal("Не удалось найти совместимую Arduino: %s", last_error)
        raise last_error

    def _discover_candidate_ports(self) -> list[str]:
        ports = self._port_enumerator()
        devices: list[str] = []
        for port_info in ports:
            device = getattr(port_info, "device", None)
            if device:
                devices.append(str(device))
        return devices

    def _open_serial(self, port: str, baudrate: int) -> Any:
        connection = self._serial_factory(
            port=port,
            baudrate=baudrate,
            timeout=self._timeout,
            write_timeout=self._timeout,
        )
        if hasattr(connection, "reset_input_buffer"):
            connection.reset_input_buffer()
        if hasattr(connection, "reset_output_buffer"):
            connection.reset_output_buffer()
        return connection

    def _validate_connection(self, connection: Any, port: str) -> None:
        original_serial = getattr(self, "_serial", None)
        self._serial = connection
        try:
            payload = self._send_request("ping", idempotent=True, retries=self._retry_count)
        except Exception:
            if hasattr(connection, "close"):
                connection.close()
            if original_serial is not None:
                self._serial = original_serial
            raise

        if not payload.get("pong"):
            if hasattr(connection, "close"):
                connection.close()
            if original_serial is not None:
                self._serial = original_serial
            raise ArduinoUnavailableError(f"Устройство на {port} не ответило корректным pong")

    @property
    def is_closed(self) -> bool:
        return self._closed

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        serial_connection = getattr(self, "_serial", None)
        if serial_connection is not None and hasattr(serial_connection, "close"):
            serial_connection.close()

    def __enter__(self) -> "ArduinoService":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    def ping(self) -> dict[str, Any]:
        """Проверяет, что Arduino отвечает по протоколу управления.

        Returns:
            Словарь ``data`` из ответа Arduino. Обычно содержит ``pong=True`` и
            строку версии прошивки.

        Raises:
            SerialTimeoutError: Если ответ не пришёл вовремя.
            ArduinoProtocolError: Если ответ повреждён или содержит ошибку.
        """
        return self._send_request("ping", idempotent=True, retries=self._retry_count)

    def status(self) -> dict[str, Any]:
        """Запрашивает сводное состояние Arduino и подключённой периферии.

        Returns:
            Словарь со снимком текущего состояния: статус кнопки, текущие PWM
            обоих моторов, активность watchdog, реле, шагового двигателя и
            последние измерения расстояния.

        Raises:
            SerialTimeoutError: Если Arduino не ответила вовремя.
            ArduinoProtocolError: Если протокол ответа нарушен.
        """
        return self._send_request("get_status", idempotent=True, retries=self._retry_count)

    def button_status(self) -> bool:
        """Возвращает текущее состояние кнопки запуска или пользовательской кнопки.

        Returns:
            ``True``, если кнопка нажата, иначе ``False``.

        Raises:
            SerialTimeoutError: Если Arduino не ответила вовремя.
            ArduinoProtocolError: Если ответ повреждён или вернул ошибку.
        """
        response = self._send_request("get_button", idempotent=True, retries=self._retry_count)
        return bool(response["pressed"])

    def stop_all(self) -> dict[str, Any]:
        """Немедленно останавливает все движения на Arduino.

        Команда сбрасывает PWM обоих моторов и останавливает шаговый двигатель,
        если он активен.

        Returns:
            Словарь ``data`` из подтверждения Arduino.
        """
        return self._send_request("stop_all")

    def align_parallel_to_wall(
        self,
        *,
        front_sensor_id: int = 1,
        rear_sensor_id: int = 2,
        wall_side: WallSide = "right",
        tolerance_mm: float = 10.0,
        turn_power: float = 18.0,
        pulse_seconds: float = 0.12,
        settle_seconds: float = 0.05,
        max_iterations: int = 10,
    ) -> dict[str, Any]:
        """Поворачивает робота до состояния, близкого к параллели со стеной.

        Метод рассчитан на схему, в которой два ультразвуковых датчика стоят на
        одной стороне робота: один ближе к передней части, второй ближе к
        задней. Сервис сравнивает их показания и делает короткие повороты на
        месте, пока расстояния не станут одинаковыми или почти одинаковыми.

        Логика не требует отдельной команды в прошивке Arduino: Raspberry Pi
        использует уже существующие команды чтения датчиков и управления
        моторами, поэтому метод можно добавить без изменения serial-протокола.

        Args:
            front_sensor_id: Идентификатор датчика, стоящего ближе к носу
                робота.
            rear_sensor_id: Идентификатор датчика, стоящего ближе к хвосту
                робота.
            wall_side: С какой стороны робота находится стена: ``"left"`` или
                ``"right"``.
            tolerance_mm: Допустимая разница между показаниями датчиков в
                миллиметрах.
            turn_power: Мощность корректирующего поворота в процентах. Берётся
                по модулю и ограничивается диапазоном ``0..100``.
            pulse_seconds: Длительность одного корректирующего поворота.
            settle_seconds: Небольшая пауза после остановки, чтобы датчики и
                корпус успели стабилизироваться перед новым измерением.
            max_iterations: Максимальное число циклов измерения. Если за это
                время допуск не достигнут, метод вернёт ``aligned=False``.

        Returns:
            Словарь с итогом выравнивания:
            - ``aligned``: удалось ли уложиться в допуск;
            - ``iterations``: сколько циклов измерения было выполнено;
            - ``correction_steps``: сколько корректирующих импульсов поворота
              понадобилось;
            - ``front_distance_mm`` и ``rear_distance_mm``: последние измерения;
            - ``delta_mm``: разница ``front - rear``;
            - ``tolerance_mm``: использованный допуск;
            - ``wall_side``: сторона стены;
            - ``last_turn_direction``: последнее направление коррекции или
              ``None``, если робот уже был выровнен.

        Raises:
            ValueError: Если параметры метода заданы некорректно.
            SerialTimeoutError: Если Arduino перестала отвечать во время
                выравнивания.
            ArduinoProtocolError: Если одна из команд чтения или управления
                завершилась ошибкой.
        """
        if front_sensor_id not in (1, 2):
            raise ValueError("front_sensor_id должен быть равен 1 или 2")
        if rear_sensor_id not in (1, 2):
            raise ValueError("rear_sensor_id должен быть равен 1 или 2")
        if front_sensor_id == rear_sensor_id:
            raise ValueError("front_sensor_id и rear_sensor_id должны указывать на разные датчики")
        if wall_side not in ("left", "right"):
            raise ValueError('wall_side должен быть равен "left" или "right"')
        if tolerance_mm < 0:
            raise ValueError("tolerance_mm не может быть отрицательным")
        if pulse_seconds <= 0:
            raise ValueError("pulse_seconds должно быть больше нуля")
        if settle_seconds < 0:
            raise ValueError("settle_seconds не может быть отрицательным")
        if max_iterations <= 0:
            raise ValueError("max_iterations должно быть больше нуля")

        bounded_turn_power = abs(_clamp_pwm(turn_power))
        if bounded_turn_power == 0:
            raise ValueError("turn_power должно быть больше нуля")

        correction_steps = 0
        last_front_distance_mm = 0.0
        last_rear_distance_mm = 0.0
        last_delta_mm = 0.0
        last_turn_direction: RotationDirection | None = None

        for iteration in range(1, max_iterations + 1):
            last_front_distance_mm = self.distance_sensor.get(front_sensor_id, unit="mm")
            last_rear_distance_mm = self.distance_sensor.get(rear_sensor_id, unit="mm")
            last_delta_mm = float(last_front_distance_mm - last_rear_distance_mm)

            self._logger.info(
                "Проверка параллельности со стеной: итерация=%s/%s, front=%.1f мм, rear=%.1f мм, delta=%.1f мм",
                iteration,
                max_iterations,
                last_front_distance_mm,
                last_rear_distance_mm,
                last_delta_mm,
            )

            if abs(last_delta_mm) <= tolerance_mm:
                return {
                    "aligned": True,
                    "iterations": iteration,
                    "correction_steps": correction_steps,
                    "front_distance_mm": last_front_distance_mm,
                    "rear_distance_mm": last_rear_distance_mm,
                    "delta_mm": last_delta_mm,
                    "tolerance_mm": float(tolerance_mm),
                    "wall_side": wall_side,
                    "last_turn_direction": last_turn_direction,
                }

            if iteration >= max_iterations:
                break

            last_turn_direction = self._select_wall_alignment_turn(last_delta_mm, wall_side)
            correction_steps += 1
            self._logger.info(
                "Корректируем положение робота относительно %s стены: шаг=%s, направление=%s, мощность=%s%%, импульс=%.3f с",
                wall_side,
                correction_steps,
                last_turn_direction,
                bounded_turn_power,
                pulse_seconds,
            )
            self._apply_turn_pulse(
                direction=last_turn_direction,
                turn_power=bounded_turn_power,
                pulse_seconds=pulse_seconds,
                settle_seconds=settle_seconds,
            )

        self._logger.warning(
            "Не удалось вывести робота в допуск параллельности за %s итераций: front=%.1f мм, rear=%.1f мм, delta=%.1f мм",
            max_iterations,
            last_front_distance_mm,
            last_rear_distance_mm,
            last_delta_mm,
        )
        return {
            "aligned": False,
            "iterations": max_iterations,
            "correction_steps": correction_steps,
            "front_distance_mm": last_front_distance_mm,
            "rear_distance_mm": last_rear_distance_mm,
            "delta_mm": last_delta_mm,
            "tolerance_mm": float(tolerance_mm),
            "wall_side": wall_side,
            "last_turn_direction": last_turn_direction,
        }

    def set_servo(self, angle_deg: float) -> dict[str, Any]:
        """Устанавливает угол сервопривода в градусах.

        Args:
            angle_deg: Целевой угол. На стороне Arduino значение будет
                ограничено диапазоном ``0..180``.

        Returns:
            Словарь ``data`` с фактическим установленным углом.

        Raises:
            UnsupportedHardwareError: Если сервопривод отключён в конфигурации
                прошивки.
            ArduinoProtocolError: Если команда отклонена прошивкой.
        """
        return self._send_request("set_servo", {"angle_deg": int(round(angle_deg))})

    def set_relay(self, enabled: bool) -> dict[str, Any]:
        """Переключает реле в состояние включено/выключено.

        Args:
            enabled: ``True`` для включения реле, ``False`` для выключения.

        Returns:
            Словарь ``data`` с итоговым состоянием реле.

        Raises:
            UnsupportedHardwareError: Если реле отключено в прошивке.
            ArduinoProtocolError: Если команда отклонена прошивкой.
        """
        return self._send_request("set_relay", {"enabled": bool(enabled)})

    def move_stepper(
        self,
        *,
        steps: int | None = None,
        rpm: float = 60.0,
        direction: Literal["forward", "reverse"] = "forward",
        duration_ms: int | None = None,
    ) -> dict[str, Any]:
        """Запускает шаговый двигатель с заданными параметрами.

        Args:
            steps: Количество шагов. Если ``None``, двигатель работает без
                ограничения по шагам до отдельной остановки или истечения
                ``duration_ms``.
            rpm: Целевая скорость в оборотах в минуту.
            direction: Направление ``forward`` или ``reverse``.
            duration_ms: Ограничение времени работы в миллисекундах.

        Returns:
            Словарь ``data`` с подтверждением запуска и параметрами движения.

        Raises:
            UnsupportedHardwareError: Если шаговый двигатель отключён.
            ArduinoProtocolError: Если команда отклонена прошивкой.
        """
        options = _StepperMoveOptions(
            steps=steps,
            rpm=rpm,
            direction=direction,
            duration_ms=duration_ms,
        )
        args: dict[str, Any] = {
            "rpm": float(options.rpm),
            "direction": options.direction,
        }
        if options.steps is not None:
            args["steps"] = int(options.steps)
        if options.duration_ms is not None:
            args["duration_ms"] = int(options.duration_ms)
        return self._send_request("stepper_move", args)

    def stop_stepper(self) -> dict[str, Any]:
        """Останавливает шаговый двигатель.

        Returns:
            Словарь ``data`` из ответа Arduino.

        Raises:
            UnsupportedHardwareError: Если шаговый двигатель не поддерживается.
            ArduinoProtocolError: Если команда завершилась ошибкой.
        """
        return self._send_request("stepper_stop")

    @staticmethod
    def _select_wall_alignment_turn(delta_mm: float, wall_side: WallSide) -> RotationDirection:
        if wall_side == "right":
            return "right" if delta_mm > 0 else "left"
        return "left" if delta_mm > 0 else "right"

    def _apply_turn_pulse(
        self,
        *,
        direction: RotationDirection,
        turn_power: int,
        pulse_seconds: float,
        settle_seconds: float,
    ) -> None:
        left_pwm = turn_power if direction == "right" else -turn_power
        right_pwm = -turn_power if direction == "right" else turn_power

        self.eng_l.pwm(left_pwm).now()
        try:
            self.eng_r.pwm(right_pwm).now()
            time.sleep(pulse_seconds)
        finally:
            try:
                self.stop_all()
            except Exception:
                self._logger.debug(
                    "Не удалось аварийно остановить моторы после корректирующего импульса выравнивания",
                    exc_info=True,
                )

        if settle_seconds > 0:
            time.sleep(settle_seconds)

    def _send_request(
        self,
        op: str,
        args: dict[str, Any] | None = None,
        *,
        idempotent: bool = False,
        retries: int = 0,
    ) -> dict[str, Any]:
        last_error: Exception | None = None
        for attempt in range(retries + 1):
            try:
                return self._send_request_once(op, args)
            except (SerialTimeoutError, ArduinoProtocolError) as exc:
                last_error = exc
                if not idempotent or attempt >= retries:
                    raise
                self._logger.warning(
                    "Повторяем идемпотентный запрос к Arduino %s после ошибки %s",
                    op,
                    exc,
                )
        if last_error is not None:
            raise last_error
        raise ArduinoProtocolError(f"Запрос {op} завершился неуспешно без конкретной ошибки")

    def _send_request_once(self, op: str, args: dict[str, Any] | None = None) -> dict[str, Any]:
        if self._closed:
            raise ArduinoUnavailableError("Сервис Arduino уже закрыт")

        with self._lock:
            request_id = next(self._request_ids)
            payload = {"id": request_id, "op": op, "args": args or {}}
            encoded = json.dumps(payload, separators=(",", ":")) + "\n"
            self._serial.write(encoded.encode("utf-8"))
            if hasattr(self._serial, "flush"):
                self._serial.flush()
            message = self._read_response(request_id)

        if not isinstance(message, dict):
            raise ArduinoProtocolError(f"Arduino вернула ответ не в виде объекта: {message!r}")

        if message.get("id") != request_id:
            raise ArduinoProtocolError(
                f"Несовпадение id ответа: ожидался {request_id}, получен {message.get('id')}"
            )

        if message.get("ok") is True:
            return dict(message.get("data") or {})

        error = message.get("error") or {}
        code = str(error.get("code") or "unknown_error")
        message_text = str(error.get("message") or "Arduino вернула неуточнённую ошибку")
        if code in {"unsupported", "not_configured"}:
            raise UnsupportedHardwareError(message_text)
        raise ArduinoProtocolError(f"{code}: {message_text}")

    def _read_response(self, request_id: int) -> dict[str, Any]:
        raw_line = self._serial.readline()
        if isinstance(raw_line, bytes):
            decoded = raw_line.decode("utf-8", errors="replace").strip()
        else:
            decoded = str(raw_line).strip()

        if not decoded:
            raise SerialTimeoutError(f"Истекло время ожидания ответа на запрос {request_id}")

        try:
            parsed = json.loads(decoded)
        except json.JSONDecodeError as exc:
            raise ArduinoProtocolError(f"Получен повреждённый JSON: {decoded}") from exc
        return parsed


class DistanceSensorAccessor:
    """Фасад для чтения данных с ультразвуковых датчиков расстояния."""

    def __init__(self, service: ArduinoService) -> None:
        self._service = service

    def get(self, sensor_id: int, unit: UnitName = "mm") -> float:
        """Читает расстояние с выбранного ультразвукового датчика.

        Args:
            sensor_id: Номер датчика, поддерживаются только ``1`` и ``2``.
            unit: Единица измерения результата: миллиметры, сантиметры или
                метры.

        Returns:
            Расстояние в выбранной единице измерения.

        Raises:
            ValueError: Если передан неверный номер датчика или единица.
            SerialTimeoutError: Если Arduino не ответила вовремя.
            ArduinoProtocolError: Если датчик временно недоступен или ответ
                повреждён.
        """
        if sensor_id not in (1, 2):
            raise ValueError("sensor_id должен быть равен 1 или 2")
        response = self._service._send_request(
            "get_distance",
            {"sensor": sensor_id},
            idempotent=True,
            retries=self._service._retry_count,
        )
        distance_mm = float(response["distance_mm"])
        return _convert_distance(distance_mm, unit)


class MotorChannel:
    """Фасад для формирования команд управления конкретной группой моторов."""

    def __init__(self, service: ArduinoService, target: MotorTarget) -> None:
        self._service = service
        self._target = target

    def pwm(self, percent: float) -> "MotorCommandBuilder":
        """Создаёт объект-команду для установки мощности моторов.

        Args:
            percent: Мощность в процентах от ``-100`` до ``100``. Отрицательные
                значения означают обратное направление.

        Returns:
            Объект ``MotorCommandBuilder``, который можно сразу отправить через
            ``.now()`` или дополнить длительностью через ``.time(seconds)``.
        """
        return MotorCommandBuilder(self._service, self._target, _clamp_pwm(percent))


class MotorCommandBuilder:
    """Построитель одной атомарной команды управления моторами.

    Экземпляр создаётся методом ``MotorChannel.pwm()`` и предназначен для
    ровно одной отправки команды на Arduino.
    """

    def __init__(self, service: ArduinoService, target: MotorTarget, percent: int) -> None:
        self._service = service
        self._target = target
        self._percent = percent
        self._sent = False

    def _send(self, *, duration_ms: int | None = None) -> dict[str, Any]:
        if self._sent:
            raise RuntimeError("Команда управления мотором уже была отправлена")
        self._sent = True
        args: dict[str, Any] = {"target": self._target, "pwm": self._percent}
        if duration_ms is not None:
            args["duration_ms"] = duration_ms
        return self._service._send_request("set_motor", args)

    def time(self, seconds: float) -> dict[str, Any]:
        """Отправляет команду с ограничением по времени.

        Args:
            seconds: Сколько секунд моторы должны работать с указанной мощностью.

        Returns:
            Словарь ``data`` из подтверждения Arduino.

        Raises:
            ValueError: Если передана неположительная длительность.
            ArduinoProtocolError: Если команда отклонена.
        """
        return self._send(duration_ms=_seconds_to_ms(seconds))

    def now(self) -> dict[str, Any]:
        """Немедленно отправляет команду без ограничения по времени.

        Returns:
            Словарь ``data`` из подтверждения Arduino.

        Notes:
            Моторы будут работать до следующей команды, вызова ``stop_all`` или
            срабатывания watchdog на стороне Arduino.
        """
        return self._send()

    def __del__(self) -> None:  # pragma: no cover - время вызова деструктора зависит от рантайма
        if self._sent or self._service.is_closed:
            return
        try:
            self._send()
        except Exception:
            self._service._logger.debug("Не удалось отправить неявную команду мотора при очистке объекта", exc_info=True)


class ServoController:
    """Удобный фасад для команд управления сервоприводом."""

    def __init__(self, service: ArduinoService) -> None:
        self._service = service

    def set(self, angle_deg: float) -> dict[str, Any]:
        """Устанавливает угол сервопривода.

        Args:
            angle_deg: Целевой угол в градусах.

        Returns:
            Словарь ``data`` с подтверждённым углом.
        """
        return self._service.set_servo(angle_deg)


class RelayController:
    """Удобный фасад для включения и выключения реле."""

    def __init__(self, service: ArduinoService) -> None:
        self._service = service

    def on(self) -> dict[str, Any]:
        """Включает реле и возвращает подтверждение Arduino."""
        return self._service.set_relay(True)

    def off(self) -> dict[str, Any]:
        """Выключает реле и возвращает подтверждение Arduino."""
        return self._service.set_relay(False)

    def set(self, enabled: bool) -> dict[str, Any]:
        """Устанавливает реле в произвольное состояние.

        Args:
            enabled: ``True`` для включения, ``False`` для выключения.

        Returns:
            Словарь ``data`` с итоговым состоянием реле.
        """
        return self._service.set_relay(enabled)


class StepperController:
    """Удобный фасад для команд управления шаговым двигателем."""

    def __init__(self, service: ArduinoService) -> None:
        self._service = service

    def move(
        self,
        *,
        steps: int | None = None,
        rpm: float = 60.0,
        direction: Literal["forward", "reverse"] = "forward",
        duration: float | None = None,
    ) -> dict[str, Any]:
        """Запускает шаговый двигатель через high-level Python API.

        Args:
            steps: Ограничение по числу шагов или ``None`` для непрерывной
                работы.
            rpm: Скорость в оборотах в минуту.
            direction: Направление ``forward`` или ``reverse``.
            duration: Ограничение по времени в секундах.

        Returns:
            Словарь ``data`` из ответа Arduino.
        """
        duration_ms = _seconds_to_ms(duration) if duration is not None else None
        return self._service.move_stepper(
            steps=steps,
            rpm=rpm,
            direction=direction,
            duration_ms=duration_ms,
        )

    def stop(self) -> dict[str, Any]:
        """Останавливает шаговый двигатель и возвращает подтверждение Arduino."""
        return self._service.stop_stepper()
