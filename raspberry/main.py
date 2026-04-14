from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Sequence
from typing import Callable, TextIO

try:
    from .arduino_service import ArduinoProtocolError
    from .arduino_service import ArduinoService
    from .arduino_service import SerialTimeoutError
except ImportError:  # pragma: no cover - позволяет запускать файл напрямую
    from arduino_service import ArduinoProtocolError  # type: ignore[no-redef]
    from arduino_service import ArduinoService  # type: ignore[no-redef]
    from arduino_service import SerialTimeoutError  # type: ignore[no-redef]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Мониторинг и быстрая диагностика датчиков расстояния Arduino."
    )
    parser.add_argument("--port", default="COM3", help="Serial-порт Arduino, по умолчанию COM3.")
    parser.add_argument(
        "--sensors",
        nargs="+",
        type=int,
        choices=(1, 2),
        default=[1, 2],
        help="Какие датчики проверять: 1, 2 или оба сразу.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Пауза между циклами мониторинга в секундах.",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=None,
        help="Сколько циклов мониторинга выполнить. По умолчанию мониторинг бесконечный.",
    )
    parser.add_argument(
        "--diagnose-sensors",
        action="store_true",
        help="Выполнить ограниченную диагностику выбранных датчиков и завершиться.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="Сколько выборок собрать в режиме диагностики.",
    )
    return parser


def _print_sensor_value(
    robot: ArduinoService,
    sensor_id: int,
    *,
    output: TextIO,
) -> bool:
    try:
        value_mm = robot.distance_sensor.get(sensor_id)
    except (ArduinoProtocolError, SerialTimeoutError) as exc:
        print(f"[WARN] Датчик {sensor_id}: {exc}", file=output)
        return False

    print(f"Датчик {sensor_id}: {value_mm:.1f} мм", file=output)
    return True


def monitor_sensors(
    robot: ArduinoService,
    *,
    sensors: Sequence[int],
    interval: float,
    iterations: int | None,
    output: TextIO,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> int:
    completed_iterations = 0
    while iterations is None or completed_iterations < iterations:
        for sensor_id in sensors:
            _print_sensor_value(robot, sensor_id, output=output)
        completed_iterations += 1
        if iterations is not None and completed_iterations >= iterations:
            break
        sleep_fn(interval)
    return 0


def diagnose_sensors(
    robot: ArduinoService,
    *,
    sensors: Sequence[int],
    samples: int,
    interval: float,
    output: TextIO,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> int:
    summary = {sensor_id: {"ok": 0, "errors": 0} for sensor_id in sensors}

    for sample_index in range(1, samples + 1):
        print(f"Проверка {sample_index}/{samples}", file=output)
        for sensor_id in sensors:
            success = _print_sensor_value(robot, sensor_id, output=output)
            summary[sensor_id]["ok" if success else "errors"] += 1
        if sample_index < samples:
            sleep_fn(interval)

    print("Итог диагностики:", file=output)
    exit_code = 0
    for sensor_id in sensors:
        ok_count = summary[sensor_id]["ok"]
        error_count = summary[sensor_id]["errors"]
        print(
            f"- Датчик {sensor_id}: успешных чтений {ok_count}, ошибок {error_count}",
            file=output,
        )
        if ok_count == 0:
            exit_code = 1

    return exit_code


def main(
    argv: Sequence[str] | None = None,
    *,
    output: TextIO | None = None,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    output = output or sys.stdout

    if args.interval <= 0:
        parser.error("--interval должен быть больше нуля.")
    if args.samples <= 0:
        parser.error("--samples должен быть больше нуля.")
    if args.iterations is not None and args.iterations <= 0:
        parser.error("--iterations должен быть больше нуля.")

    try:
        with ArduinoService(port=args.port) as robot:
            robot.start_activity_session()
            if args.diagnose_sensors:
                return diagnose_sensors(
                    robot,
                    sensors=args.sensors,
                    samples=args.samples,
                    interval=args.interval,
                    output=output,
                    sleep_fn=sleep_fn,
                )
            return monitor_sensors(
                robot,
                sensors=args.sensors,
                interval=args.interval,
                iterations=args.iterations,
                output=output,
                sleep_fn=sleep_fn,
            )
    except KeyboardInterrupt:
        print("Остановлено пользователем.", file=output)
        return 130
    except Exception as exc:
        print(f"[ERROR] Не удалось запустить работу с Arduino: {exc}", file=output)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
