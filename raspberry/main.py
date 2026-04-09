from __future__ import annotations

import argparse
import logging
import sys

try:
    from .arduino_service import ArduinoService
    from .raspberry_service import RaspberryService
except ImportError:  # pragma: no cover - позволяет запускать файл напрямую
    from arduino_service import ArduinoService  # type: ignore[no-redef]
    from raspberry_service import RaspberryService  # type: ignore[no-redef]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Демонстрационный запуск низкоуровневого стека Rescue Maze")
    parser.add_argument("--port", help="Serial-порт Arduino. Если не указан, будет использован автопоиск.")
    parser.add_argument("--baudrate", type=int, default=115200, help="Скорость serial-соединения с Arduino.")
    parser.add_argument(
        "--motion-percent",
        type=int,
        default=0,
        help="Мощность гусениц для опциональной демонстрации движения. Ноль отключает этот шаг.",
    )
    parser.add_argument(
        "--motion-seconds",
        type=float,
        default=0.0,
        help="Длительность опциональной демонстрации движения. И процент, и время должны быть ненулевыми.",
    )
    parser.add_argument(
        "--enable-ap",
        action="store_true",
        help="Включить AP-профиль Raspberry Pi после проверки Arduino.",
    )
    parser.add_argument("--ap-ssid", default="RescueMazeRobot", help="SSID для AP-режима.")
    parser.add_argument("--ap-password", default="RescueMaze123", help="Пароль для AP-режима.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    logger = logging.getLogger("rescue_maze.bootstrap")

    raspberry_service = RaspberryService(logger=logger.getChild("rpi"))

    try:
        with ArduinoService(
            port=args.port,
            baudrate=args.baudrate,
            logger=logger.getChild("arduino"),
        ) as arduino_service:
            logger.info("Arduino подключена на %s", arduino_service.port)
            logger.info("Ответ на ping: %s", arduino_service.ping())
            logger.info("Снимок состояния: %s", arduino_service.status())

            for sensor_id in (1, 2):
                try:
                    distance_cm = arduino_service.distance_sensor.get(sensor_id, unit="cm")
                except Exception as exc:  # pragma: no cover - зависит от железа
                    logger.warning("Датчик расстояния %s пока недоступен: %s", sensor_id, exc)
                else:
                    logger.info("Датчик расстояния %s: %.1f см", sensor_id, distance_cm)

            logger.info("Кнопка нажата: %s", arduino_service.button_status())

            if args.motion_percent and args.motion_seconds > 0:
                logger.info(
                    "Запускаем демонстрацию движения на %s%% в течение %.2f секунд",
                    args.motion_percent,
                    args.motion_seconds,
                )
                arduino_service.eng_all.pwm(args.motion_percent).time(args.motion_seconds)
            else:
                logger.info("Демонстрация движения пропущена. Укажите --motion-percent и --motion-seconds, чтобы запустить её.")

            if args.enable_ap:
                logger.info("Настраиваем и включаем AP-профиль %s", args.ap_ssid)
                raspberry_service.configure_ap(args.ap_ssid, args.ap_password)
                raspberry_service.enable_ap()
                logger.info("AP-профиль включён")
    except Exception as exc:
        logger.exception("Демонстрационный запуск завершился ошибкой: %s", exc)
        return 1

    logger.info("Демонстрационный запуск завершён успешно")
    return 0


if __name__ == "__main__":  # pragma: no cover - точка входа CLI
    raise SystemExit(main(sys.argv[1:]))
