"""Клиентский API для управления Arduino по serial-протоколу Rescue Maze.

Модуль инкапсулирует:
- поиск и валидацию Arduino по USB serial;
- обмен newline-delimited JSON сообщениями;
- удобные Python-объекты для моторов, датчиков и опциональных актуаторов.
"""

from __future__ import annotations

import json
import math
import logging
import re
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from html import escape
from itertools import count
from pathlib import Path
from typing import Any, Callable, Literal

try:
    import serial  # type: ignore[import-not-found]
    from serial.tools import list_ports  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - проверяется через внедрение зависимостей в тестах
    serial = None
    list_ports = None


LOGGER = logging.getLogger(__name__)
DEFAULT_CONNECT_WARMUP_SECONDS = 2.0
LOW_MOTOR_PWM_WARNING_THRESHOLD = 60
MOTION_ROUTE_SLICE_SECONDS = 0.05
UnitName = Literal["mm", "cm", "m"]
MotorTarget = Literal["all", "left", "right"]
WallSide = Literal["left", "right"]
RotationDirection = Literal["left", "right"]
DistanceSensorKind = Literal["disabled", "hc_sr04", "urm37"]
Urm37MeasureMode = Literal["pwm_passive", "auto"]

MIN_DISTANCE_SENSOR_ID = 1
MAX_DISTANCE_SENSOR_ID = 4
URM37_AUTO_INTERVAL_MIN_MS = 25
URM37_AUTO_INTERVAL_MAX_MS = 255
URM37_COMPARE_DISTANCE_MIN_CM = 0
URM37_COMPARE_DISTANCE_MAX_CM = 1000
URM37_SENSITIVITY_MIN = 10
URM37_SENSITIVITY_MAX = 200


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


def _validate_sensor_id(sensor_id: int, *, field_name: str = "sensor_id") -> int:
    normalized_sensor_id = int(sensor_id)
    if not MIN_DISTANCE_SENSOR_ID <= normalized_sensor_id <= MAX_DISTANCE_SENSOR_ID:
        raise ValueError(f"{field_name} должен быть в диапазоне от 1 до 4")
    return normalized_sensor_id


def _validate_urm37_measure_mode(measure_mode: str) -> Urm37MeasureMode:
    normalized_mode = str(measure_mode)
    if normalized_mode not in ("pwm_passive", "auto"):
        raise ValueError('measure_mode должен быть равен "pwm_passive" или "auto"')
    return normalized_mode


def _validate_urm37_auto_measure_interval_ms(value: int) -> int:
    normalized_value = int(value)
    if not URM37_AUTO_INTERVAL_MIN_MS <= normalized_value <= URM37_AUTO_INTERVAL_MAX_MS:
        raise ValueError(
            f"auto_measure_interval_ms должен быть в диапазоне от {URM37_AUTO_INTERVAL_MIN_MS} до {URM37_AUTO_INTERVAL_MAX_MS}"
        )
    return normalized_value


def _validate_urm37_compare_distance_cm(value: int) -> int:
    normalized_value = int(value)
    if not URM37_COMPARE_DISTANCE_MIN_CM <= normalized_value <= URM37_COMPARE_DISTANCE_MAX_CM:
        raise ValueError(
            f"compare_distance_cm должен быть в диапазоне от {URM37_COMPARE_DISTANCE_MIN_CM} до {URM37_COMPARE_DISTANCE_MAX_CM}"
        )
    return normalized_value


def _validate_urm37_sensitivity(value: int) -> int:
    normalized_value = int(value)
    if not URM37_SENSITIVITY_MIN <= normalized_value <= URM37_SENSITIVITY_MAX:
        raise ValueError(
            f"sensitivity должен быть в диапазоне от {URM37_SENSITIVITY_MIN} до {URM37_SENSITIVITY_MAX}"
        )
    return normalized_value


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


@dataclass(slots=True)
class MotionMapCalibration:
    """Коэффициенты для приблизительной карты движения без энкодеров."""

    max_linear_speed_mm_per_sec: float = 320.0
    max_turn_deg_per_sec: float = 180.0


@dataclass(slots=True, frozen=True)
class DistanceSensorInfo:
    """РЎРІРѕРґРЅРѕРµ РѕРїРёСЃР°РЅРёРµ СЃР»РѕС‚Р° СѓР»СЊС‚СЂР°Р·РІСѓРєРѕРІРѕРіРѕ РґР°С‚С‡РёРєР°."""

    sensor_id: int
    enabled: bool
    kind: DistanceSensorKind
    trigger_pin: int | None
    echo_pin: int | None
    serial_rx_pin: int | None
    serial_tx_pin: int | None
    serial_settings_available: bool
    distance_mm: float | None


@dataclass(slots=True, frozen=True)
class Urm37Settings:
    """Р‘РµР·РѕРїР°СЃРЅРѕРµ РїСЂРµРґСЃС‚Р°РІР»РµРЅРёРµ РЅР°СЃС‚СЂРѕРµРє URM37 РІ Python API."""

    sensor_id: int
    measure_mode: Urm37MeasureMode
    auto_measure_interval_ms: int
    compare_distance_cm: int
    sensitivity: int


@dataclass(slots=True)
class _RoutePoint:
    monotonic_seconds: float
    x_mm: float
    y_mm: float
    heading_deg: float
    label: str | None = None


@dataclass(slots=True)
class _ActivityEvent:
    timestamp_iso: str
    monotonic_seconds: float
    category: str
    action: str
    success: bool
    text: str
    params: dict[str, Any]
    result: dict[str, Any] | None = None
    protocol_op: str | None = None
    error_type: str | None = None
    error_message: str | None = None
    position: dict[str, float] | None = None
    route_label: str | None = None

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "timestamp_iso": self.timestamp_iso,
            "monotonic_seconds": self.monotonic_seconds,
            "category": self.category,
            "action": self.action,
            "success": self.success,
            "text": self.text,
            "params": self.params,
            "result": self.result,
            "protocol_op": self.protocol_op,
            "error_type": self.error_type,
            "error_message": self.error_message,
            "position": self.position,
        }
        if self.route_label is not None:
            payload["route_label"] = self.route_label
        return payload


@dataclass(slots=True)
class _MotionState:
    last_monotonic: float
    x_mm: float = 0.0
    y_mm: float = 0.0
    heading_deg: float = 0.0
    left_pwm: int = 0
    right_pwm: int = 0
    left_auto_stop_at: float | None = None
    right_auto_stop_at: float | None = None
    left_ramp_start_pwm: int = 0
    right_ramp_start_pwm: int = 0
    left_ramp_stop_pwm: int = 0
    right_ramp_stop_pwm: int = 0
    left_ramp_started_at: float | None = None
    right_ramp_started_at: float | None = None
    left_ramp_ends_at: float | None = None
    right_ramp_ends_at: float | None = None


@dataclass(slots=True)
class _ActivitySession:
    output_dir: Path
    include_map: bool
    calibration: MotionMapCalibration
    started_at_iso: str
    monotonic_zero: float
    motion_state: _MotionState
    events: list[_ActivityEvent] = field(default_factory=list)
    actions_lines: list[str] = field(default_factory=list)
    route_points: list[_RoutePoint] = field(default_factory=list)


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
        connect_warmup_seconds: float = DEFAULT_CONNECT_WARMUP_SECONDS,
        logger: logging.Logger | None = None,
        serial_factory: Callable[..., Any] | None = None,
        port_enumerator: Callable[[], list[Any]] | None = None,
        monotonic_clock: Callable[[], float] | None = None,
    ) -> None:
        """Создаёт сервис обмена с Arduino и немедленно открывает соединение.

        Args:
            port: Явный serial-порт Arduino. Если ``None``, используется
                автопоиск по доступным портам с последующей проверкой через
                ``ping``.
            baudrate: Скорость serial-соединения.
            timeout: Таймаут чтения и записи serial-соединения в секундах.
            retry_count: Количество повторов для идемпотентных команд чтения.
            connect_warmup_seconds: Пауза после открытия реального COM-порта,
                чтобы Arduino успела перезагрузиться и начать отвечать на
                стартовый ``ping``.
            logger: Пользовательский logger. Если не передан, используется
                модульный logger.
            serial_factory: Точка расширения для подмены конструктора serial-
                подключения, полезна для тестов.
            port_enumerator: Точка расширения для подмены механизма поиска
                serial-портов, полезна для тестов.
            monotonic_clock: Источник монотонного времени. Используется для
                приблизительной карты движения и полезен в тестах.

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
        self._connect_warmup_seconds = max(0.0, float(connect_warmup_seconds))
        self._request_ids = count(1)
        self._lock = threading.RLock()
        self._closed = False
        self._monotonic_clock = monotonic_clock or time.monotonic
        self._activity_session: _ActivitySession | None = None

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
        if self._should_wait_for_device_warmup(connection):
            self._logger.info(
                "Ждём %.1f с после открытия %s, чтобы Arduino завершила перезагрузку",
                self._connect_warmup_seconds,
                port,
            )
            time.sleep(self._connect_warmup_seconds)
            if hasattr(connection, "reset_input_buffer"):
                connection.reset_input_buffer()
        return connection

    def _should_wait_for_device_warmup(self, connection: Any) -> bool:
        if self._connect_warmup_seconds <= 0:
            return False
        return any(hasattr(connection, attr) for attr in ("port", "portstr", "name"))

    def _validate_connection(self, connection: Any, port: str) -> None:
        original_serial = getattr(self, "_serial", None)
        self._serial = connection
        try:
            self._logger.info("Проверяем связь с Arduino на %s через ping", port)
            payload = self._send_request("ping", idempotent=True, retries=self._retry_count)
        except Exception as exc:
            self._logger.error("Стартовый ping к Arduino на %s завершился ошибкой: %s", port, exc)
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

    @property
    def activity_session_active(self) -> bool:
        """Показывает, включена ли запись действий и карты движения."""
        return self._activity_session is not None

    def close(self) -> None:
        if self._closed:
            return

        if self._activity_session is not None:
            try:
                self._record_activity_event(
                    category="connection",
                    action="close",
                    success=True,
                    text=f"Соединение с Arduino на порту {self.port} закрыто — Выполнено",
                    params={"port": self.port},
                    route_label="Финиш",
                )
                self._finalize_activity_session(
                    text="Сессия записи действий завершена из-за закрытия сервиса — Выполнено"
                )
            except Exception:
                self._logger.exception("Не удалось корректно завершить сессию записи действий перед закрытием ArduinoService")

        self._closed = True
        serial_connection = getattr(self, "_serial", None)
        if serial_connection is not None and hasattr(serial_connection, "close"):
            serial_connection.close()

    def __enter__(self) -> "ArduinoService":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    def start_activity_session(
        self,
        output_dir: str | Path | None = None,
        *,
        session_name: str | None = None,
        include_map: bool = True,
        calibration: MotionMapCalibration | None = None,
    ) -> Path:
        """Включает запись человекочитаемого журнала и оценочной карты движения.

        Args:
            output_dir: Готовая директория для артефактов сессии. Если не
                указана, сервис создаёт папку внутри ``logs/arduino_sessions``.
            session_name: Необязательное имя сессии для директории по умолчанию.
            include_map: Нужно ли строить SVG-карту движения.
            calibration: Коэффициенты приблизительной модели движения.

        Returns:
            Путь к директории, в которую будут сохранены артефакты сессии.

        Raises:
            RuntimeError: Если запись уже включена.
            ValueError: Если калибровка задана некорректно.
        """
        if self._activity_session is not None:
            raise RuntimeError("Сессия записи действий уже запущена")

        selected_calibration = calibration or MotionMapCalibration()
        if selected_calibration.max_linear_speed_mm_per_sec <= 0:
            raise ValueError("max_linear_speed_mm_per_sec должно быть больше нуля")
        if selected_calibration.max_turn_deg_per_sec <= 0:
            raise ValueError("max_turn_deg_per_sec должно быть больше нуля")

        session_dir = self._prepare_activity_output_dir(output_dir, session_name=session_name)
        monotonic_zero = self._monotonic_now()
        session = _ActivitySession(
            output_dir=session_dir,
            include_map=include_map,
            calibration=selected_calibration,
            started_at_iso=self._iso_timestamp(),
            monotonic_zero=monotonic_zero,
            motion_state=_MotionState(last_monotonic=monotonic_zero),
        )
        if include_map:
            session.route_points.append(
                _RoutePoint(
                    monotonic_seconds=0.0,
                    x_mm=0.0,
                    y_mm=0.0,
                    heading_deg=0.0,
                    label="Старт",
                )
            )
        self._activity_session = session
        self._record_activity_event(
            category="session",
            action="start_activity_session",
            success=True,
            text=f"Сессия записи действий начата. Arduino подключена на порту {self.port} — Выполнено",
            params={
                "output_dir": str(session_dir),
                "session_name": session_name,
                "include_map": include_map,
                "calibration": self._json_safe(
                    {
                        "max_linear_speed_mm_per_sec": selected_calibration.max_linear_speed_mm_per_sec,
                        "max_turn_deg_per_sec": selected_calibration.max_turn_deg_per_sec,
                    }
                ),
            },
            route_label="Старт сессии",
        )
        return session_dir

    def stop_activity_session(self) -> dict[str, Any]:
        """Останавливает запись действий и сохраняет все артефакты сессии."""
        if self._activity_session is None:
            raise RuntimeError("Сессия записи действий не запущена")
        return self._finalize_activity_session(text="Сессия записи действий завершена — Выполнено")

    def ping(self) -> dict[str, Any]:
        """Проверяет, что Arduino отвечает по протоколу управления.

        Returns:
            Словарь ``data`` из ответа Arduino. Обычно содержит ``pong=True`` и
            строку версии прошивки.

        Raises:
            SerialTimeoutError: Если ответ не пришёл вовремя.
            ArduinoProtocolError: Если ответ повреждён или содержит ошибку.
        """
        return self._run_logged_action(
            category="connection",
            action="ping",
            params={"port": self.port},
            protocol_op="ping",
            operation=lambda: self._send_request("ping", idempotent=True, retries=self._retry_count),
            success_text=lambda _: f"Проверка связи с Arduino на порту {self.port} выполнена — Удачно",
            error_text=lambda exc: f"Проверка связи с Arduino на порту {self.port} завершилась ошибкой: {exc}",
        )

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
        return self._run_logged_action(
            category="telemetry",
            action="status",
            params={},
            protocol_op="get_status",
            operation=lambda: self._send_request("get_status", idempotent=True, retries=self._retry_count),
            success_text=lambda _: "Получен сводный статус Arduino и подключённой периферии — Удачно",
            error_text=lambda exc: f"Не удалось получить сводный статус Arduino: {exc}",
        )

    def button_status(self) -> bool:
        """Возвращает текущее состояние кнопки запуска или пользовательской кнопки.

        Returns:
            ``True``, если кнопка нажата, иначе ``False``.

        Raises:
            SerialTimeoutError: Если Arduino не ответила вовремя.
            ArduinoProtocolError: Если ответ повреждён или вернул ошибку.
        """
        return self._run_logged_action(
            category="sensor",
            action="button_status",
            params={},
            protocol_op="get_button",
            operation=lambda: bool(
                self._send_request("get_button", idempotent=True, retries=self._retry_count)["pressed"]
            ),
            success_text=lambda pressed: (
                f"Считано состояние кнопки: {'нажата' if pressed else 'отпущена'} — Удачно"
            ),
            error_text=lambda exc: f"Не удалось прочитать состояние кнопки: {exc}",
            route_label=lambda pressed: "Кнопка нажата" if pressed else "Кнопка отпущена",
        )

    def stop_all(self) -> dict[str, Any]:
        """Немедленно останавливает все движения на Arduino.

        Команда сбрасывает PWM обоих моторов и останавливает шаговый двигатель,
        если он активен.

        Returns:
            Словарь ``data`` из подтверждения Arduino.
        """
        return self._run_logged_action(
            category="movement",
            action="stop_all",
            params={},
            protocol_op="stop_all",
            operation=self._stop_all_with_activity,
            success_text=lambda _: "Робот остановлен — Выполнено",
            error_text=lambda exc: f"Не удалось остановить робот: {exc}",
            route_label="Стоп",
        )

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
        params = {
            "front_sensor_id": front_sensor_id,
            "rear_sensor_id": rear_sensor_id,
            "wall_side": wall_side,
            "tolerance_mm": tolerance_mm,
            "turn_power": turn_power,
            "pulse_seconds": pulse_seconds,
            "settle_seconds": settle_seconds,
            "max_iterations": max_iterations,
        }
        return self._run_logged_action(
            category="alignment",
            action="align_parallel_to_wall",
            params=params,
            operation=lambda: self._align_parallel_to_wall_impl(
                front_sensor_id=front_sensor_id,
                rear_sensor_id=rear_sensor_id,
                wall_side=wall_side,
                tolerance_mm=tolerance_mm,
                turn_power=turn_power,
                pulse_seconds=pulse_seconds,
                settle_seconds=settle_seconds,
                max_iterations=max_iterations,
            ),
            success_text=lambda result: (
                f"Робот выровнился по {'правой' if wall_side == 'right' else 'левой'} стене "
                f"с погрешностью не более {float(tolerance_mm):.1f} мм — Удачно"
                if result["aligned"]
                else (
                    f"Робот не смог выровняться по {'правой' if wall_side == 'right' else 'левой'} стене "
                    f"с допуском {float(tolerance_mm):.1f} мм — Ошибка: достигнут предел коррекций"
                )
            ),
            error_text=lambda exc: (
                f"Не удалось выполнить выравнивание по {'правой' if wall_side == 'right' else 'левой'} стене: {exc}"
            ),
            success_predicate=lambda result: bool(result["aligned"]),
            route_label=lambda result: (
                f"Параллель {wall_side}"
                if result["aligned"]
                else f"Не выровнен {wall_side}"
            ),
        )

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
        target_angle = int(round(angle_deg))
        return self._run_logged_action(
            category="actuator",
            action="set_servo",
            params={"angle_deg": target_angle},
            protocol_op="set_servo",
            operation=lambda: self._send_request("set_servo", {"angle_deg": target_angle}),
            success_text=lambda result: (
                f"Сервопривод установлен на угол {result.get('angle_deg', target_angle)}° — Выполнено"
            ),
            error_text=lambda exc: f"Не удалось установить сервопривод на угол {target_angle}°: {exc}",
            route_label=f"Серво {target_angle}°",
        )

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
        relay_enabled = bool(enabled)
        return self._run_logged_action(
            category="actuator",
            action="set_relay",
            params={"enabled": relay_enabled},
            protocol_op="set_relay",
            operation=lambda: self._send_request("set_relay", {"enabled": relay_enabled}),
            success_text=lambda _: f"Реле {'включено' if relay_enabled else 'выключено'} — Выполнено",
            error_text=lambda exc: f"Не удалось {'включить' if relay_enabled else 'выключить'} реле: {exc}",
            route_label="Реле вкл" if relay_enabled else "Реле выкл",
        )

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
        duration_text = (
            f", ограничение {options.duration_ms / 1000.0:.2f} с"
            if options.duration_ms is not None
            else ""
        )
        steps_text = f", {options.steps} шагов" if options.steps is not None else ", без ограничения по шагам"
        return self._run_logged_action(
            category="actuator",
            action="move_stepper",
            params=args,
            protocol_op="stepper_move",
            operation=lambda: self._send_request("stepper_move", args),
            success_text=lambda _: (
                f"Шаговый двигатель запущен в направлении {options.direction} "
                f"со скоростью {float(options.rpm):.1f} об/мин{steps_text}{duration_text} — Выполнено"
            ),
            error_text=lambda exc: f"Не удалось запустить шаговый двигатель: {exc}",
            route_label="Шаговый двигатель",
        )

    def stop_stepper(self) -> dict[str, Any]:
        """Останавливает шаговый двигатель.

        Returns:
            Словарь ``data`` из ответа Arduino.

        Raises:
            UnsupportedHardwareError: Если шаговый двигатель не поддерживается.
            ArduinoProtocolError: Если команда завершилась ошибкой.
        """
        return self._run_logged_action(
            category="actuator",
            action="stop_stepper",
            params={},
            protocol_op="stepper_stop",
            operation=lambda: self._send_request("stepper_stop"),
            success_text=lambda _: "Шаговый двигатель остановлен — Выполнено",
            error_text=lambda exc: f"Не удалось остановить шаговый двигатель: {exc}",
            route_label="Шаговый стоп",
        )

    def _get_distance_sensor_info(self, sensor_id: int) -> DistanceSensorInfo:
        normalized_sensor_id = _validate_sensor_id(sensor_id)
        return self._run_logged_action(
            category="sensor",
            action="distance_sensor.info",
            params={"sensor_id": normalized_sensor_id},
            protocol_op="get_status",
            operation=lambda: self._read_distance_sensor_info(normalized_sensor_id),
            success_text=lambda info: (
                f"Получена конфигурация датчика {info.sensor_id}: kind={info.kind}, enabled={info.enabled} — Удачно"
            ),
            error_text=lambda exc: f"Не удалось получить конфигурацию датчика {normalized_sensor_id}: {exc}",
            result_transform=lambda info: asdict(info),
        )

    def _read_distance_sensor_info(self, sensor_id: int) -> DistanceSensorInfo:
        status_payload = self._send_request("get_status", idempotent=True, retries=self._retry_count)
        return self._parse_distance_sensor_info(status_payload, sensor_id)

    def _parse_distance_sensor_info(self, status_payload: dict[str, Any], sensor_id: int) -> DistanceSensorInfo:
        normalized_sensor_id = _validate_sensor_id(sensor_id)
        sensors = status_payload.get("distance_sensors")
        if isinstance(sensors, dict):
            raw_sensor = sensors.get(str(normalized_sensor_id), sensors.get(normalized_sensor_id))
            if isinstance(raw_sensor, dict):
                pins = raw_sensor.get("pins")
                if not isinstance(pins, dict):
                    pins = {}
                enabled = bool(raw_sensor.get("enabled"))
                raw_kind = str(raw_sensor.get("kind") or ("disabled" if not enabled else "hc_sr04"))
                kind: DistanceSensorKind = raw_kind if raw_kind in ("disabled", "hc_sr04", "urm37") else "disabled"
                raw_distance_mm = raw_sensor.get("distance_mm")
                return DistanceSensorInfo(
                    sensor_id=normalized_sensor_id,
                    enabled=enabled,
                    kind=kind,
                    trigger_pin=self._coerce_optional_int(pins.get("trigger")),
                    echo_pin=self._coerce_optional_int(pins.get("echo")),
                    serial_rx_pin=self._coerce_optional_int(pins.get("serial_rx")),
                    serial_tx_pin=self._coerce_optional_int(pins.get("serial_tx")),
                    serial_settings_available=bool(raw_sensor.get("serial_settings_available")),
                    distance_mm=None if raw_distance_mm is None else float(raw_distance_mm),
                )

        legacy_distance_map = status_payload.get("distance_mm")
        if normalized_sensor_id in (1, 2):
            raw_distance_mm = None
            if isinstance(legacy_distance_map, dict):
                raw_distance_mm = legacy_distance_map.get(
                    str(normalized_sensor_id),
                    legacy_distance_map.get(normalized_sensor_id),
                )
            return DistanceSensorInfo(
                sensor_id=normalized_sensor_id,
                enabled=True,
                kind="hc_sr04",
                trigger_pin=None,
                echo_pin=None,
                serial_rx_pin=None,
                serial_tx_pin=None,
                serial_settings_available=False,
                distance_mm=None if raw_distance_mm is None else float(raw_distance_mm),
            )

        return DistanceSensorInfo(
            sensor_id=normalized_sensor_id,
            enabled=False,
            kind="disabled",
            trigger_pin=None,
            echo_pin=None,
            serial_rx_pin=None,
            serial_tx_pin=None,
            serial_settings_available=False,
            distance_mm=None,
        )

    def _get_urm37_temperature(self, sensor_id: int) -> float:
        normalized_sensor_id = _validate_sensor_id(sensor_id)
        return self._run_logged_action(
            category="sensor",
            action="distance_sensor.get_temperature",
            params={"sensor_id": normalized_sensor_id},
            protocol_op="get_urm37_temperature",
            operation=lambda: self._read_urm37_temperature(normalized_sensor_id),
            success_text=lambda temperature_c: (
                f"Получена температура URM37 с датчика {normalized_sensor_id}: {temperature_c:.2f} C — Удачно"
            ),
            error_text=lambda exc: f"Не удалось получить температуру URM37 с датчика {normalized_sensor_id}: {exc}",
            result_transform=lambda temperature_c: {
                "sensor_id": normalized_sensor_id,
                "temperature_c": round(float(temperature_c), 4),
            },
        )

    def _read_urm37_temperature(self, sensor_id: int) -> float:
        response = self._send_request(
            "get_urm37_temperature",
            {"sensor": _validate_sensor_id(sensor_id)},
            idempotent=True,
            retries=self._retry_count,
        )
        return float(response["temperature_c"])

    def _get_urm37_settings(self, sensor_id: int) -> Urm37Settings:
        normalized_sensor_id = _validate_sensor_id(sensor_id)
        return self._run_logged_action(
            category="sensor",
            action="distance_sensor.get_urm37_settings",
            params={"sensor_id": normalized_sensor_id},
            protocol_op="get_urm37_settings",
            operation=lambda: self._read_urm37_settings(normalized_sensor_id),
            success_text=lambda settings: (
                f"Получены настройки URM37 для датчика {settings.sensor_id}: mode={settings.measure_mode} — Удачно"
            ),
            error_text=lambda exc: f"Не удалось получить настройки URM37 для датчика {normalized_sensor_id}: {exc}",
            result_transform=lambda settings: asdict(settings),
        )

    def _read_urm37_settings(self, sensor_id: int) -> Urm37Settings:
        response = self._send_request(
            "get_urm37_settings",
            {"sensor": _validate_sensor_id(sensor_id)},
            idempotent=True,
            retries=self._retry_count,
        )
        return self._parse_urm37_settings_response(response, sensor_id=sensor_id)

    def _configure_urm37(
        self,
        sensor_id: int,
        *,
        measure_mode: Urm37MeasureMode | None = None,
        auto_measure_interval_ms: int | None = None,
        compare_distance_cm: int | None = None,
        sensitivity: int | None = None,
    ) -> Urm37Settings:
        normalized_sensor_id = _validate_sensor_id(sensor_id)
        args: dict[str, Any] = {"sensor": normalized_sensor_id}
        if measure_mode is not None:
            args["measure_mode"] = _validate_urm37_measure_mode(measure_mode)
        if auto_measure_interval_ms is not None:
            args["auto_measure_interval_ms"] = _validate_urm37_auto_measure_interval_ms(auto_measure_interval_ms)
        if compare_distance_cm is not None:
            args["compare_distance_cm"] = _validate_urm37_compare_distance_cm(compare_distance_cm)
        if sensitivity is not None:
            args["sensitivity"] = _validate_urm37_sensitivity(sensitivity)
        if len(args) == 1:
            raise ValueError("Нужно указать хотя бы одну настройку URM37 для обновления")

        return self._run_logged_action(
            category="sensor",
            action="distance_sensor.configure_urm37",
            params={key: value for key, value in args.items() if key != "sensor"} | {"sensor_id": normalized_sensor_id},
            protocol_op="set_urm37_settings",
            operation=lambda: self._write_urm37_settings(args, sensor_id=normalized_sensor_id),
            success_text=lambda settings: (
                f"Настройки URM37 для датчика {settings.sensor_id} обновлены: mode={settings.measure_mode} — Выполнено"
            ),
            error_text=lambda exc: f"Не удалось обновить настройки URM37 для датчика {normalized_sensor_id}: {exc}",
            result_transform=lambda settings: asdict(settings),
        )

    def _write_urm37_settings(self, args: dict[str, Any], *, sensor_id: int) -> Urm37Settings:
        response = self._send_request("set_urm37_settings", args)
        return self._parse_urm37_settings_response(response, sensor_id=sensor_id)

    def _parse_urm37_settings_response(self, response: dict[str, Any], *, sensor_id: int) -> Urm37Settings:
        return Urm37Settings(
            sensor_id=_validate_sensor_id(sensor_id),
            measure_mode=_validate_urm37_measure_mode(str(response["measure_mode"])),
            auto_measure_interval_ms=_validate_urm37_auto_measure_interval_ms(int(response["auto_measure_interval_ms"])),
            compare_distance_cm=_validate_urm37_compare_distance_cm(int(response["compare_distance_cm"])),
            sensitivity=_validate_urm37_sensitivity(int(response["sensitivity"])),
        )

    @staticmethod
    def _coerce_optional_int(value: Any) -> int | None:
        if value is None:
            return None
        return int(value)

    def _align_parallel_to_wall_impl(
        self,
        *,
        front_sensor_id: int,
        rear_sensor_id: int,
        wall_side: WallSide,
        tolerance_mm: float,
        turn_power: float,
        pulse_seconds: float,
        settle_seconds: float,
        max_iterations: int,
    ) -> dict[str, Any]:
        normalized_front_sensor_id = _validate_sensor_id(front_sensor_id, field_name="front_sensor_id")
        normalized_rear_sensor_id = _validate_sensor_id(rear_sensor_id, field_name="rear_sensor_id")
        if normalized_front_sensor_id == normalized_rear_sensor_id:
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
            last_front_distance_mm = self.distance_sensor.get(normalized_front_sensor_id, unit="mm")
            last_rear_distance_mm = self.distance_sensor.get(normalized_rear_sensor_id, unit="mm")
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

    def _get_distance_value(self, sensor_id: int, unit: UnitName = "mm") -> float:
        raw_distance_holder = {"distance_mm": 0.0}
        return self._run_logged_action(
            category="sensor",
            action="distance_sensor.get",
            params={"sensor_id": sensor_id, "unit": unit},
            protocol_op="get_distance",
            operation=lambda: self._read_distance_value(
                sensor_id,
                unit,
                raw_distance_holder=lambda value: raw_distance_holder.__setitem__("distance_mm", value),
            ),
            success_text=lambda value: (
                f"Получено расстояние с датчика {sensor_id}: {value:.1f} {self._unit_label(unit)} — Удачно"
            ),
            error_text=lambda exc: f"Не удалось получить расстояние с датчика {sensor_id}: {exc}",
            result_transform=lambda value: {
                "sensor_id": sensor_id,
                "unit": unit,
                "value": round(float(value), 4),
                "distance_mm": round(float(raw_distance_holder["distance_mm"]), 4),
            },
            route_label=lambda value: f"Д{sensor_id}: {value:.1f} {self._unit_label(unit)}",
        )

    def _read_distance_value(
        self,
        sensor_id: int,
        unit: UnitName,
        *,
        raw_distance_holder: Callable[[float], None] | None = None,
    ) -> float:
        normalized_sensor_id = _validate_sensor_id(sensor_id)
        response = self._send_request(
            "get_distance",
            {"sensor": normalized_sensor_id},
            idempotent=True,
            retries=self._retry_count,
        )
        distance_mm = float(response["distance_mm"])
        if raw_distance_holder is not None:
            raw_distance_holder(distance_mm)
        return _convert_distance(distance_mm, unit)

    def _send_motor_command(
        self,
        *,
        target: MotorTarget,
        pwm: int,
        duration_ms: int | None = None,
        start_pwm: int | None = None,
        ramp_duration_ms: int | None = None,
    ) -> dict[str, Any]:
        if start_pwm is None and 0 < abs(pwm) < LOW_MOTOR_PWM_WARNING_THRESHOLD:
            self._logger.warning(
                "Для команды мотора %s задан PWM %s. При значениях ниже %s двигатель может работать нестабильно "
                "или вообще не запуститься из-за нагрузки, питания или драйвера.",
                target,
                pwm,
                LOW_MOTOR_PWM_WARNING_THRESHOLD,
            )
        if ramp_duration_ms is not None and ramp_duration_ms <= 0:
            start_pwm = None
            ramp_duration_ms = None
        if start_pwm is not None and ramp_duration_ms is not None:
            self._warn_about_motor_pwm_profile(
                target=target,
                start_pwm=start_pwm,
                stop_pwm=pwm,
                ramp_duration_ms=ramp_duration_ms,
            )
        params: dict[str, Any] = {"target": target, "pwm": pwm}
        if duration_ms is not None:
            params["duration_ms"] = duration_ms
        if start_pwm is not None and ramp_duration_ms is not None:
            params["start_pwm"] = start_pwm
            params["ramp_duration_ms"] = ramp_duration_ms
        return self._run_logged_action(
            category="movement",
            action="set_motor",
            params=params,
            protocol_op="set_motor",
            operation=lambda: self._set_motor_request_with_activity(
                target=target,
                pwm=pwm,
                duration_ms=duration_ms,
                start_pwm=start_pwm,
                ramp_duration_ms=ramp_duration_ms,
            ),
            success_text=lambda _: self._motor_success_text(
                target=target,
                pwm=pwm,
                duration_ms=duration_ms,
                start_pwm=start_pwm,
                ramp_duration_ms=ramp_duration_ms,
            ),
            error_text=lambda exc: self._motor_error_text(
                target=target,
                pwm=pwm,
                duration_ms=duration_ms,
                start_pwm=start_pwm,
                ramp_duration_ms=ramp_duration_ms,
                exc=exc,
            ),
            route_label=self._motor_route_label(
                target=target,
                pwm=pwm,
                duration_ms=duration_ms,
                start_pwm=start_pwm,
                ramp_duration_ms=ramp_duration_ms,
            ),
        )

    def _set_motor_request_with_activity(
        self,
        *,
        target: MotorTarget,
        pwm: int,
        duration_ms: int | None,
        start_pwm: int | None = None,
        ramp_duration_ms: int | None = None,
    ) -> dict[str, Any]:
        args: dict[str, Any] = {"target": target, "pwm": pwm}
        if duration_ms is not None:
            args["duration_ms"] = duration_ms
        if start_pwm is not None and ramp_duration_ms is not None:
            args["start_pwm"] = start_pwm
            args["ramp_duration_ms"] = ramp_duration_ms
        response = self._send_request("set_motor", args)
        self._apply_motor_command_to_activity(
            target=target,
            pwm=pwm,
            duration_ms=duration_ms,
            start_pwm=start_pwm,
            ramp_duration_ms=ramp_duration_ms,
        )
        return response

    def _stop_all_with_activity(self) -> dict[str, Any]:
        response = self._send_request("stop_all")
        self._apply_stop_all_to_activity()
        return response

    def _run_logged_action(
        self,
        *,
        category: str,
        action: str,
        params: dict[str, Any],
        operation: Callable[[], Any],
        success_text: str | Callable[[Any], str],
        error_text: str | Callable[[Exception], str],
        protocol_op: str | None = None,
        success_predicate: Callable[[Any], bool] | None = None,
        result_transform: Callable[[Any], dict[str, Any] | None] | None = None,
        route_label: str | Callable[[Any], str | None] | None = None,
    ) -> Any:
        safe_params = self._json_safe(params)
        try:
            result = operation()
        except Exception as exc:
            error_message = error_text(exc) if callable(error_text) else error_text
            self._record_activity_event(
                category=category,
                action=action,
                success=False,
                text=error_message,
                params=safe_params,
                protocol_op=protocol_op,
                error=exc,
            )
            raise

        success = success_predicate(result) if success_predicate is not None else True
        message = success_text(result) if callable(success_text) else success_text
        label: str | None
        if callable(route_label):
            label = route_label(result)
        else:
            label = route_label
        normalized_result = (
            result_transform(result)
            if result_transform is not None
            else self._normalize_activity_result(result)
        )
        self._record_activity_event(
            category=category,
            action=action,
            success=success,
            text=message,
            params=safe_params,
            result=self._json_safe(normalized_result),
            protocol_op=protocol_op,
            route_label=label,
        )
        return result

    def _record_activity_event(
        self,
        *,
        category: str,
        action: str,
        success: bool,
        text: str,
        params: dict[str, Any],
        result: dict[str, Any] | None = None,
        protocol_op: str | None = None,
        error: Exception | None = None,
        route_label: str | None = None,
    ) -> None:
        session = self._activity_session
        if session is None:
            return

        self._advance_motion_to(self._monotonic_now())
        event = _ActivityEvent(
            timestamp_iso=self._iso_timestamp(),
            monotonic_seconds=round(session.motion_state.last_monotonic - session.monotonic_zero, 6),
            category=category,
            action=action,
            success=success,
            text=text,
            params=self._json_safe(params),
            result=self._json_safe(result) if result is not None else None,
            protocol_op=protocol_op,
            error_type=type(error).__name__ if error is not None else None,
            error_message=str(error) if error is not None else None,
            position=self._position_snapshot(session),
            route_label=route_label,
        )
        session.events.append(event)
        status = "OK" if success else "ERROR"
        session.actions_lines.append(f"{event.timestamp_iso} | {status} | {text}")
        if route_label:
            self._append_route_point(label=route_label)

    def _position_snapshot(self, session: _ActivitySession) -> dict[str, float]:
        state = session.motion_state
        return {
            "x_mm": round(state.x_mm, 3),
            "y_mm": round(state.y_mm, 3),
            "heading_deg": round(state.heading_deg, 3),
        }

    def _normalize_activity_result(self, result: Any) -> dict[str, Any] | None:
        if result is None:
            return None
        if isinstance(result, dict):
            return self._json_safe(result)
        if isinstance(result, (str, bool, int, float)):
            return {"value": result}
        return {"value": repr(result)}

    def _json_safe(self, value: Any) -> Any:
        if isinstance(value, dict):
            return {str(key): self._json_safe(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._json_safe(item) for item in value]
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, float):
            return round(value, 6)
        if isinstance(value, (str, bool, int)) or value is None:
            return value
        return repr(value)

    def _prepare_activity_output_dir(self, output_dir: str | Path | None, *, session_name: str | None) -> Path:
        if output_dir is not None:
            path = Path(output_dir)
            path.mkdir(parents=True, exist_ok=True)
            return path

        root = Path("logs") / "arduino_sessions"
        root.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = self._slugify_session_name(session_name) or "session"
        candidate = root / f"{timestamp}_{suffix}"
        counter = 1
        while candidate.exists():
            counter += 1
            candidate = root / f"{timestamp}_{suffix}_{counter}"
        candidate.mkdir(parents=True, exist_ok=False)
        return candidate

    @staticmethod
    def _slugify_session_name(session_name: str | None) -> str:
        if not session_name:
            return ""
        normalized = re.sub(r"[<>:\"/\\\\|?*\s]+", "_", session_name.strip())
        return normalized.strip("._")

    def _finalize_activity_session(self, text: str | None = None) -> dict[str, Any]:
        session = self._activity_session
        if session is None:
            raise RuntimeError("Сессия записи действий не запущена")

        self._finish_motion_for_session_end()
        if text is not None:
            self._record_activity_event(
                category="session",
                action="stop_activity_session",
                success=True,
                text=text,
                params={},
                route_label="Конец сессии",
            )
            self._finish_motion_for_session_end()

        actions_path = session.output_dir / "actions.txt"
        events_path = session.output_dir / "events.json"
        route_path = session.output_dir / "route.svg"

        actions_payload = "\n".join(session.actions_lines)
        if actions_payload:
            actions_payload += "\n"
        actions_path.write_text(actions_payload, encoding="utf-8")
        events_payload = [event.to_dict() for event in session.events]
        events_path.write_text(json.dumps(events_payload, ensure_ascii=False, indent=2), encoding="utf-8")
        if session.include_map:
            route_path.write_text(self._render_route_svg(session), encoding="utf-8")

        summary = {
            "output_dir": str(session.output_dir),
            "actions_path": str(actions_path),
            "events_path": str(events_path),
            "route_path": str(route_path) if session.include_map else None,
            "event_count": len(session.events),
            "action_count": len(session.actions_lines),
            "final_pose": self._position_snapshot(session),
        }
        self._activity_session = None
        return summary

    def _finish_motion_for_session_end(self) -> None:
        session = self._activity_session
        if session is None:
            return

        state = session.motion_state
        self._advance_motion_to(self._monotonic_now())
        state.left_pwm = 0
        state.right_pwm = 0
        state.left_auto_stop_at = None
        state.right_auto_stop_at = None
        self._clear_motor_ramp_state(state, "left")
        self._clear_motor_ramp_state(state, "right")
        self._append_route_point(label="Финиш")

    def _wait_for_timed_completion(self, duration_ms: int) -> None:
        if duration_ms <= 0:
            return
        time.sleep(duration_ms / 1000.0)

    def _apply_motor_command_to_activity(
        self,
        *,
        target: MotorTarget,
        pwm: int,
        duration_ms: int | None,
        start_pwm: int | None = None,
        ramp_duration_ms: int | None = None,
    ) -> None:
        session = self._activity_session
        if session is None:
            return

        now = self._monotonic_now()
        self._advance_motion_to(now)
        deadline = now + (duration_ms / 1000.0) if duration_ms is not None else None
        state = session.motion_state
        if target in {"all", "left"}:
            self._set_motor_channel_activity(
                state,
                "left",
                stop_pwm=pwm,
                deadline=deadline,
                started_at=now,
                start_pwm=start_pwm,
                ramp_duration_ms=ramp_duration_ms,
            )
        if target in {"all", "right"}:
            self._set_motor_channel_activity(
                state,
                "right",
                stop_pwm=pwm,
                deadline=deadline,
                started_at=now,
                start_pwm=start_pwm,
                ramp_duration_ms=ramp_duration_ms,
            )
        self._append_route_point()

    def _apply_stop_all_to_activity(self) -> None:
        session = self._activity_session
        if session is None:
            return
        self._advance_motion_to(self._monotonic_now())
        state = session.motion_state
        state.left_pwm = 0
        state.right_pwm = 0
        state.left_auto_stop_at = None
        state.right_auto_stop_at = None
        self._clear_motor_ramp_state(state, "left")
        self._clear_motor_ramp_state(state, "right")
        self._append_route_point(label="Стоп")

    def _advance_motion_to(self, target_monotonic: float) -> None:
        session = self._activity_session
        if session is None:
            return

        state = session.motion_state
        epsilon = 1e-9
        if target_monotonic <= state.last_monotonic + epsilon:
            return

        while target_monotonic > state.last_monotonic + epsilon:
            segment_end = target_monotonic
            for deadline in (
                state.left_auto_stop_at,
                state.right_auto_stop_at,
                state.left_ramp_ends_at,
                state.right_ramp_ends_at,
            ):
                if deadline is not None and state.last_monotonic + epsilon < deadline < segment_end:
                    segment_end = deadline

            delta_seconds = segment_end - state.last_monotonic
            if delta_seconds > epsilon:
                self._advance_motion_segment(session, segment_end, epsilon=epsilon)
            else:
                state.last_monotonic = segment_end

            self._complete_elapsed_motor_ramps(state, epsilon=epsilon)
            if state.left_auto_stop_at is not None and state.last_monotonic >= state.left_auto_stop_at - epsilon:
                state.left_pwm = 0
                state.left_auto_stop_at = None
                self._clear_motor_ramp_state(state, "left")
            if state.right_auto_stop_at is not None and state.last_monotonic >= state.right_auto_stop_at - epsilon:
                state.right_pwm = 0
                state.right_auto_stop_at = None
                self._clear_motor_ramp_state(state, "right")

    def _advance_motion_segment(
        self,
        session: _ActivitySession,
        target_monotonic: float,
        *,
        epsilon: float,
    ) -> None:
        state = session.motion_state
        while target_monotonic > state.last_monotonic + epsilon:
            slice_end = target_monotonic
            if self._should_sample_motion_segment(state, session.calibration, state.last_monotonic, target_monotonic):
                slice_end = min(state.last_monotonic + MOTION_ROUTE_SLICE_SECONDS, target_monotonic)

            self._integrate_motion_segment(state, session.calibration, state.last_monotonic, slice_end)
            state.last_monotonic = slice_end
            self._append_route_point()

    def _should_sample_motion_segment(
        self,
        state: _MotionState,
        calibration: MotionMapCalibration,
        started_at: float,
        ended_at: float,
    ) -> bool:
        if self._motor_ramp_overlaps_interval(state, "left", started_at, ended_at):
            return True
        if self._motor_ramp_overlaps_interval(state, "right", started_at, ended_at):
            return True

        left_ratio = self._average_motor_pwm_for_interval(state, "left", started_at, ended_at) / 100.0
        right_ratio = self._average_motor_pwm_for_interval(state, "right", started_at, ended_at) / 100.0
        angular_speed_deg = ((right_ratio - left_ratio) / 2.0) * calibration.max_turn_deg_per_sec
        return abs(angular_speed_deg) > 1e-9

    def _integrate_motion_segment(
        self,
        state: _MotionState,
        calibration: MotionMapCalibration,
        started_at: float,
        ended_at: float,
    ) -> None:
        delta_seconds = ended_at - started_at
        if delta_seconds <= 0:
            return

        left_ratio = self._average_motor_pwm_for_interval(state, "left", started_at, ended_at) / 100.0
        right_ratio = self._average_motor_pwm_for_interval(state, "right", started_at, ended_at) / 100.0
        linear_speed = ((left_ratio + right_ratio) / 2.0) * calibration.max_linear_speed_mm_per_sec
        angular_speed_deg = ((right_ratio - left_ratio) / 2.0) * calibration.max_turn_deg_per_sec
        heading_delta_deg = angular_speed_deg * delta_seconds
        if abs(angular_speed_deg) <= 1e-9:
            heading_rad = math.radians(state.heading_deg)
            distance_mm = linear_speed * delta_seconds
            state.x_mm += distance_mm * math.cos(heading_rad)
            state.y_mm += distance_mm * math.sin(heading_rad)
        else:
            heading_start_rad = math.radians(state.heading_deg)
            heading_end_rad = math.radians(state.heading_deg + heading_delta_deg)
            angular_speed_rad = math.radians(angular_speed_deg)
            turn_radius_mm = linear_speed / angular_speed_rad
            state.x_mm += turn_radius_mm * (math.sin(heading_end_rad) - math.sin(heading_start_rad))
            state.y_mm += turn_radius_mm * (math.cos(heading_start_rad) - math.cos(heading_end_rad))
        state.heading_deg = self._normalize_heading(state.heading_deg + heading_delta_deg)

    def _warn_about_motor_pwm_profile(
        self,
        *,
        target: MotorTarget,
        start_pwm: int,
        stop_pwm: int,
        ramp_duration_ms: int,
    ) -> None:
        if not self._motor_profile_enters_low_pwm_zone(start_pwm, stop_pwm):
            return
        self._logger.warning(
            "Для ramp-команды мотора %s задан профиль PWM %s -> %s за %.3f с. "
            "В диапазоне ниже %s двигатель может работать нестабильно "
            "или вообще не запуститься из-за нагрузки, питания или драйвера.",
            target,
            start_pwm,
            stop_pwm,
            ramp_duration_ms / 1000.0,
            LOW_MOTOR_PWM_WARNING_THRESHOLD,
        )

    @staticmethod
    def _motor_profile_enters_low_pwm_zone(start_pwm: int, stop_pwm: int) -> bool:
        if start_pwm == stop_pwm:
            return 0 < abs(start_pwm) < LOW_MOTOR_PWM_WARNING_THRESHOLD
        if start_pwm == 0 or stop_pwm == 0:
            return start_pwm != stop_pwm
        if start_pwm * stop_pwm < 0:
            return True
        return min(abs(start_pwm), abs(stop_pwm)) < LOW_MOTOR_PWM_WARNING_THRESHOLD

    def _set_motor_channel_activity(
        self,
        state: _MotionState,
        channel: Literal["left", "right"],
        *,
        stop_pwm: int,
        deadline: float | None,
        started_at: float,
        start_pwm: int | None,
        ramp_duration_ms: int | None,
    ) -> None:
        setattr(state, f"{channel}_auto_stop_at", deadline)
        if start_pwm is not None and ramp_duration_ms is not None:
            setattr(state, f"{channel}_pwm", start_pwm)
            setattr(state, f"{channel}_ramp_start_pwm", start_pwm)
            setattr(state, f"{channel}_ramp_stop_pwm", stop_pwm)
            setattr(state, f"{channel}_ramp_started_at", started_at)
            setattr(state, f"{channel}_ramp_ends_at", started_at + (ramp_duration_ms / 1000.0))
            return
        setattr(state, f"{channel}_pwm", stop_pwm)
        self._clear_motor_ramp_state(state, channel)

    @staticmethod
    def _clear_motor_ramp_state(state: _MotionState, channel: Literal["left", "right"]) -> None:
        setattr(state, f"{channel}_ramp_start_pwm", 0)
        setattr(state, f"{channel}_ramp_stop_pwm", 0)
        setattr(state, f"{channel}_ramp_started_at", None)
        setattr(state, f"{channel}_ramp_ends_at", None)

    def _complete_elapsed_motor_ramps(self, state: _MotionState, *, epsilon: float) -> None:
        for channel in ("left", "right"):
            ramp_end = getattr(state, f"{channel}_ramp_ends_at")
            if ramp_end is None or state.last_monotonic < ramp_end - epsilon:
                continue
            setattr(state, f"{channel}_pwm", getattr(state, f"{channel}_ramp_stop_pwm"))
            self._clear_motor_ramp_state(state, channel)

    def _average_motor_pwm_for_interval(
        self,
        state: _MotionState,
        channel: Literal["left", "right"],
        started_at: float,
        ended_at: float,
    ) -> float:
        current_pwm = float(getattr(state, f"{channel}_pwm"))
        ramp_started_at = getattr(state, f"{channel}_ramp_started_at")
        ramp_ends_at = getattr(state, f"{channel}_ramp_ends_at")
        if ramp_started_at is None or ramp_ends_at is None:
            return current_pwm

        ramp_duration = ramp_ends_at - ramp_started_at
        if ramp_duration <= 0:
            return float(getattr(state, f"{channel}_ramp_stop_pwm"))

        interval_start = max(started_at, ramp_started_at)
        interval_end = min(ended_at, ramp_ends_at)
        if interval_end <= interval_start:
            return current_pwm

        start_pwm = float(getattr(state, f"{channel}_ramp_start_pwm"))
        stop_pwm = float(getattr(state, f"{channel}_ramp_stop_pwm"))
        start_progress = (interval_start - ramp_started_at) / ramp_duration
        end_progress = (interval_end - ramp_started_at) / ramp_duration
        pwm_at_start = start_pwm + ((stop_pwm - start_pwm) * start_progress)
        pwm_at_end = start_pwm + ((stop_pwm - start_pwm) * end_progress)
        return (pwm_at_start + pwm_at_end) / 2.0

    @staticmethod
    def _motor_ramp_overlaps_interval(
        state: _MotionState,
        channel: Literal["left", "right"],
        started_at: float,
        ended_at: float,
    ) -> bool:
        ramp_started_at = getattr(state, f"{channel}_ramp_started_at")
        ramp_ends_at = getattr(state, f"{channel}_ramp_ends_at")
        if ramp_started_at is None or ramp_ends_at is None:
            return False
        return started_at < ramp_ends_at and ended_at > ramp_started_at

    def _append_route_point(self, label: str | None = None) -> None:
        session = self._activity_session
        if session is None or not session.include_map:
            return

        state = session.motion_state
        point = _RoutePoint(
            monotonic_seconds=round(state.last_monotonic - session.monotonic_zero, 6),
            x_mm=round(state.x_mm, 6),
            y_mm=round(state.y_mm, 6),
            heading_deg=round(state.heading_deg, 6),
            label=label,
        )
        if session.route_points:
            last = session.route_points[-1]
            if (
                abs(last.x_mm - point.x_mm) < 1e-6
                and abs(last.y_mm - point.y_mm) < 1e-6
                and abs(last.heading_deg - point.heading_deg) < 1e-6
            ):
                if label:
                    last.label = f"{last.label}; {label}" if last.label else label
                return
        session.route_points.append(point)

    def _render_route_svg(self, session: _ActivitySession) -> str:
        points = session.route_points or [
            _RoutePoint(monotonic_seconds=0.0, x_mm=0.0, y_mm=0.0, heading_deg=0.0, label="Старт")
        ]
        min_x = min(point.x_mm for point in points)
        max_x = max(point.x_mm for point in points)
        min_y = min(point.y_mm for point in points)
        max_y = max(point.y_mm for point in points)

        width = 960
        height = 720
        padding = 80
        footer = 90
        world_width = max(max_x - min_x, 1.0)
        world_height = max(max_y - min_y, 1.0)
        scale = min(
            (width - (padding * 2)) / world_width,
            (height - footer - (padding * 2)) / world_height,
        )

        def to_svg_coords(route_point: _RoutePoint) -> tuple[float, float]:
            x = padding + ((route_point.x_mm - min_x) * scale)
            y = (height - footer - padding) - ((route_point.y_mm - min_y) * scale)
            return (round(x, 2), round(y, 2))

        polyline_points = " ".join(f"{x},{y}" for x, y in (to_svg_coords(point) for point in points))
        start_x, start_y = to_svg_coords(points[0])
        end_x, end_y = to_svg_coords(points[-1])
        heading_rad = math.radians(-points[-1].heading_deg)
        arrow_size = 18.0
        back_x = end_x - (math.cos(heading_rad) * arrow_size)
        back_y = end_y - (math.sin(heading_rad) * arrow_size)
        left_x = back_x + (math.cos(heading_rad + math.pi / 2.0) * (arrow_size / 2.2))
        left_y = back_y + (math.sin(heading_rad + math.pi / 2.0) * (arrow_size / 2.2))
        right_x = back_x + (math.cos(heading_rad - math.pi / 2.0) * (arrow_size / 2.2))
        right_y = back_y + (math.sin(heading_rad - math.pi / 2.0) * (arrow_size / 2.2))

        labels: list[str] = []
        for point in points:
            if not point.label:
                continue
            x, y = to_svg_coords(point)
            label_text = escape(point.label)
            labels.append(
                f'<text x="{x + 10:.2f}" y="{y - 10:.2f}" font-size="13" fill="#17324d">{label_text}</text>'
            )

        calibration_text = (
            f"Калибровка: до {session.calibration.max_linear_speed_mm_per_sec:.1f} мм/с, "
            f"до {session.calibration.max_turn_deg_per_sec:.1f} град/с"
        )
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
            f'viewBox="0 0 {width} {height}">'
            '<rect width="100%" height="100%" fill="#f4f8fb" />'
            f'<rect x="20" y="20" width="{width - 40}" height="{height - 40}" '
            'rx="18" fill="#ffffff" stroke="#c4d2df" stroke-width="2" />'
            '<text x="48" y="68" font-size="28" font-family="Arial, sans-serif" fill="#10263d">'
            'Оценочная карта движения'
            "</text>"
            '<text x="48" y="96" font-size="14" font-family="Arial, sans-serif" fill="#4b6177">'
            'Оценочная карта движения, не точная одометрия'
            "</text>"
            f'<text x="48" y="{height - 46}" font-size="14" font-family="Arial, sans-serif" fill="#4b6177">'
            f"{escape(calibration_text)}"
            "</text>"
            f'<polyline fill="none" stroke="#1d6fb8" stroke-width="4" stroke-linecap="round" '
            f'stroke-linejoin="round" points="{polyline_points}" />'
            f'<circle cx="{start_x:.2f}" cy="{start_y:.2f}" r="7" fill="#2f9e44" />'
            f'<circle cx="{end_x:.2f}" cy="{end_y:.2f}" r="8" fill="#d6336c" />'
            f'<line x1="{back_x:.2f}" y1="{back_y:.2f}" x2="{end_x:.2f}" y2="{end_y:.2f}" '
            'stroke="#d6336c" stroke-width="3" />'
            f'<polygon points="{end_x:.2f},{end_y:.2f} {left_x:.2f},{left_y:.2f} {right_x:.2f},{right_y:.2f}" '
            'fill="#d6336c" />'
            + "".join(labels)
            + "</svg>"
        )

    def _motor_success_text(
        self,
        *,
        target: MotorTarget,
        pwm: int,
        duration_ms: int | None,
        start_pwm: int | None = None,
        ramp_duration_ms: int | None = None,
    ) -> str:
        target_name = self._motor_target_name(target)
        direction = self._motor_direction_name(pwm)
        ramp_text = (
            self._motor_ramp_text(start_pwm=start_pwm, pwm=pwm, ramp_duration_ms=ramp_duration_ms)
            if start_pwm is not None and ramp_duration_ms is not None
            else ""
        )
        if start_pwm is not None and ramp_duration_ms is not None:
            if target == "all":
                if duration_ms is not None:
                    return (
                        f"Робот проехал {direction}{ramp_text} "
                        f"в течение {duration_ms / 1000.0:.2f} с — Выполнено"
                    )
                return f"Робот начал движение {direction}{ramp_text} — Выполнено"
            if duration_ms is not None:
                return (
                    f"{target_name} запущен в направлении {direction}{ramp_text} "
                    f"на {duration_ms / 1000.0:.2f} с — Выполнено"
                )
            return f"{target_name} запущен в направлении {direction}{ramp_text} — Выполнено"
        if target == "all":
            if duration_ms is not None:
                return (
                    f"Робот проехал {direction} на команде {abs(pwm)}% "
                    f"в течение {duration_ms / 1000.0:.2f} с — Выполнено"
                )
            return f"Робот начал движение {direction} на команде {abs(pwm)}% — Выполнено"
        if duration_ms is not None:
            return (
                f"{target_name} запущен в направлении {direction} с мощностью {abs(pwm)}% "
                f"на {duration_ms / 1000.0:.2f} с — Выполнено"
            )
        return f"{target_name} запущен в направлении {direction} с мощностью {abs(pwm)}% — Выполнено"

    def _motor_error_text(
        self,
        *,
        target: MotorTarget,
        pwm: int,
        duration_ms: int | None,
        start_pwm: int | None = None,
        ramp_duration_ms: int | None = None,
        exc: Exception,
    ) -> str:
        target_name = "роботом" if target == "all" else self._motor_target_name(target).lower()
        duration_text = f" на {duration_ms / 1000.0:.2f} с" if duration_ms is not None else ""
        return (
            f"Не удалось выполнить команду управления {target_name} "
            f"в направлении {self._motor_direction_name(pwm)} с мощностью {abs(pwm)}%{duration_text}: {exc}"
        )

    def _motor_route_label(
        self,
        *,
        target: MotorTarget,
        pwm: int,
        duration_ms: int | None,
        start_pwm: int | None = None,
        ramp_duration_ms: int | None = None,
    ) -> str:
        base = "Робот" if target == "all" else self._motor_target_name(target)
        if duration_ms is not None:
            return f"{base}: {self._motor_direction_name(pwm)} {abs(pwm)}% {duration_ms / 1000.0:.2f}с"
        return f"{base}: {self._motor_direction_name(pwm)} {abs(pwm)}%"

    @staticmethod
    def _motor_ramp_text(*, start_pwm: int, pwm: int, ramp_duration_ms: int) -> str:
        return f" с ramp PWM {start_pwm}% -> {pwm}% за {ramp_duration_ms / 1000.0:.2f} с"

    @staticmethod
    def _motor_ramp_label_suffix(*, start_pwm: int, pwm: int, ramp_duration_ms: int) -> str:
        return f" ramp {start_pwm}%->{pwm}% {ramp_duration_ms / 1000.0:.2f}с"

    @staticmethod
    def _motor_target_name(target: MotorTarget) -> str:
        if target == "left":
            return "Левый мотор"
        if target == "right":
            return "Правый мотор"
        return "Робот"

    @staticmethod
    def _motor_direction_name(pwm: int) -> str:
        if pwm > 0:
            return "вперёд"
        if pwm < 0:
            return "назад"
        return "стоп"

    @staticmethod
    def _unit_label(unit: UnitName) -> str:
        return {"mm": "мм", "cm": "см", "m": "м"}[unit]

    @staticmethod
    def _normalize_heading(angle_deg: float) -> float:
        return ((angle_deg + 180.0) % 360.0) - 180.0

    def _monotonic_now(self) -> float:
        return float(self._monotonic_clock())

    @staticmethod
    def _iso_timestamp() -> str:
        return datetime.now().astimezone().isoformat(timespec="milliseconds")

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
            self._logger.debug("Отправляем запрос к Arduino id=%s op=%s args=%s", request_id, op, payload["args"])
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
            self._logger.warning(
                "Arduino не ответила на запрос id=%s в течение %.2f с",
                request_id,
                self._timeout,
            )
            raise SerialTimeoutError(f"Истекло время ожидания ответа на запрос {request_id}")

        self._logger.debug("Получена строка ответа Arduino для id=%s: %s", request_id, decoded)
        try:
            parsed = json.loads(decoded)
        except json.JSONDecodeError as exc:
            self._logger.error("Arduino вернула некорректный JSON для id=%s: %s", request_id, decoded)
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
        return self._service._get_distance_value(sensor_id, unit)

    def info(self, sensor_id: int) -> DistanceSensorInfo:
        """Возвращает сводную конфигурацию слота датчика расстояния."""

        return self._service._get_distance_sensor_info(sensor_id)

    def get_temperature(self, sensor_id: int) -> float:
        """Читает температуру с URM37 в градусах Цельсия."""

        return self._service._get_urm37_temperature(sensor_id)

    def get_urm37_settings(self, sensor_id: int) -> Urm37Settings:
        """Возвращает безопасный набор настроек URM37."""

        return self._service._get_urm37_settings(sensor_id)

    def configure_urm37(
        self,
        sensor_id: int,
        *,
        measure_mode: Urm37MeasureMode | None = None,
        auto_measure_interval_ms: int | None = None,
        compare_distance_cm: int | None = None,
        sensitivity: int | None = None,
    ) -> Urm37Settings:
        """Частично обновляет безопасный набор настроек URM37."""

        return self._service._configure_urm37(
            sensor_id,
            measure_mode=measure_mode,
            auto_measure_interval_ms=auto_measure_interval_ms,
            compare_distance_cm=compare_distance_cm,
            sensitivity=sensitivity,
        )


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

    def ramp(self, *, start_pwm: float, stop_pwm: float, ramp_seconds: float) -> "MotorRampCommandBuilder":
        if ramp_seconds < 0:
            raise ValueError("ramp_seconds не может быть отрицательным")
        ramp_duration_ms = int(round(ramp_seconds * 1000.0))
        if ramp_duration_ms < 0:
            raise ValueError("ramp_seconds не может быть отрицательным")
        return MotorRampCommandBuilder(
            self._service,
            self._target,
            _clamp_pwm(start_pwm),
            _clamp_pwm(stop_pwm),
            ramp_duration_ms,
        )


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
        return self._service._send_motor_command(
            target=self._target,
            pwm=self._percent,
            duration_ms=duration_ms,
        )

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
        duration_ms = _seconds_to_ms(seconds)
        response = self._send(duration_ms=duration_ms)
        self._service._wait_for_timed_completion(duration_ms)
        return response

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


class MotorRampCommandBuilder:
    """Построитель одной атомарной ramp-команды для моторов."""

    def __init__(
        self,
        service: ArduinoService,
        target: MotorTarget,
        start_pwm: int,
        stop_pwm: int,
        ramp_duration_ms: int,
    ) -> None:
        self._service = service
        self._target = target
        self._start_pwm = start_pwm
        self._stop_pwm = stop_pwm
        self._ramp_duration_ms = max(0, ramp_duration_ms)
        self._sent = False

    def _send(self, *, duration_ms: int | None = None) -> dict[str, Any]:
        if self._sent:
            raise RuntimeError("Команда ramp-управления мотором уже была отправлена")
        self._sent = True
        if self._ramp_duration_ms <= 0:
            return self._service._send_motor_command(
                target=self._target,
                pwm=self._stop_pwm,
                duration_ms=duration_ms,
            )
        return self._service._send_motor_command(
            target=self._target,
            pwm=self._stop_pwm,
            duration_ms=duration_ms,
            start_pwm=self._start_pwm,
            ramp_duration_ms=self._ramp_duration_ms,
        )

    def time(self, seconds: float) -> dict[str, Any]:
        duration_ms = _seconds_to_ms(seconds)
        if duration_ms < self._ramp_duration_ms:
            raise ValueError("total_seconds не может быть меньше ramp_seconds")
        response = self._send(duration_ms=duration_ms)
        self._service._wait_for_timed_completion(duration_ms)
        return response

    def now(self) -> dict[str, Any]:
        return self._send()

    def __del__(self) -> None:  # pragma: no cover
        if self._sent or self._service.is_closed:
            return
        try:
            self._send()
        except Exception:
            self._service._logger.debug("Не удалось отправить неявную ramp-команду мотора при очистке объекта", exc_info=True)


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
        response = self._service.move_stepper(
            steps=steps,
            rpm=rpm,
            direction=direction,
            duration_ms=duration_ms,
        )
        if duration_ms is not None:
            self._service._wait_for_timed_completion(duration_ms)
        return response

    def stop(self) -> dict[str, Any]:
        """Останавливает шаговый двигатель и возвращает подтверждение Arduino."""
        return self._service.stop_stepper()
