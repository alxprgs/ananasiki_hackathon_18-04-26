from __future__ import annotations

import json
import logging
import threading
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
    """Высокоуровневый API управления поверх построчного JSON serial-протокола."""

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
        return self._send_request("ping", idempotent=True, retries=self._retry_count)

    def status(self) -> dict[str, Any]:
        return self._send_request("get_status", idempotent=True, retries=self._retry_count)

    def button_status(self) -> bool:
        response = self._send_request("get_button", idempotent=True, retries=self._retry_count)
        return bool(response["pressed"])

    def stop_all(self) -> dict[str, Any]:
        return self._send_request("stop_all")

    def set_servo(self, angle_deg: float) -> dict[str, Any]:
        return self._send_request("set_servo", {"angle_deg": int(round(angle_deg))})

    def set_relay(self, enabled: bool) -> dict[str, Any]:
        return self._send_request("set_relay", {"enabled": bool(enabled)})

    def move_stepper(
        self,
        *,
        steps: int | None = None,
        rpm: float = 60.0,
        direction: Literal["forward", "reverse"] = "forward",
        duration_ms: int | None = None,
    ) -> dict[str, Any]:
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
        return self._send_request("stepper_stop")

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
    def __init__(self, service: ArduinoService) -> None:
        self._service = service

    def get(self, sensor_id: int, unit: UnitName = "mm") -> float:
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
    def __init__(self, service: ArduinoService, target: MotorTarget) -> None:
        self._service = service
        self._target = target

    def pwm(self, percent: float) -> "MotorCommandBuilder":
        return MotorCommandBuilder(self._service, self._target, _clamp_pwm(percent))


class MotorCommandBuilder:
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
        return self._send(duration_ms=_seconds_to_ms(seconds))

    def now(self) -> dict[str, Any]:
        return self._send()

    def __del__(self) -> None:  # pragma: no cover - время вызова деструктора зависит от рантайма
        if self._sent or self._service.is_closed:
            return
        try:
            self._send()
        except Exception:
            self._service._logger.debug("Не удалось отправить неявную команду мотора при очистке объекта", exc_info=True)


class ServoController:
    def __init__(self, service: ArduinoService) -> None:
        self._service = service

    def set(self, angle_deg: float) -> dict[str, Any]:
        return self._service.set_servo(angle_deg)


class RelayController:
    def __init__(self, service: ArduinoService) -> None:
        self._service = service

    def on(self) -> dict[str, Any]:
        return self._service.set_relay(True)

    def off(self) -> dict[str, Any]:
        return self._service.set_relay(False)

    def set(self, enabled: bool) -> dict[str, Any]:
        return self._service.set_relay(enabled)


class StepperController:
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
        duration_ms = _seconds_to_ms(duration) if duration is not None else None
        return self._service.move_stepper(
            steps=steps,
            rpm=rpm,
            direction=direction,
            duration_ms=duration_ms,
        )

    def stop(self) -> dict[str, Any]:
        return self._service.stop_stepper()
