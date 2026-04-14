from __future__ import annotations

import io
import unittest
from unittest.mock import patch

from raspberry import main as main_module


class FakeDistanceSensor:
    def __init__(self, scripted_results: dict[int, list[object]]) -> None:
        self._scripted_results = {sensor_id: list(values) for sensor_id, values in scripted_results.items()}

    def get(self, sensor_id: int) -> float:
        scripted_values = self._scripted_results[sensor_id]
        if not scripted_values:
            raise AssertionError(f"No scripted values left for sensor {sensor_id}")
        next_value = scripted_values.pop(0)
        if isinstance(next_value, Exception):
            raise next_value
        return float(next_value)


class FakeArduinoService:
    scripted_results: dict[int, list[object]] = {}
    created_ports: list[str] = []
    started_sessions = 0

    def __init__(self, port: str) -> None:
        self.port = port
        self.distance_sensor = FakeDistanceSensor(self.scripted_results)
        self.closed = False
        type(self).created_ports.append(port)

    def __enter__(self) -> "FakeArduinoService":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.closed = True

    def start_activity_session(self) -> None:
        type(self).started_sessions += 1


class MainTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeArduinoService.scripted_results = {}
        FakeArduinoService.created_ports = []
        FakeArduinoService.started_sessions = 0

    def test_monitor_mode_keeps_running_after_transient_sensor_error(self) -> None:
        FakeArduinoService.scripted_results = {
            1: [main_module.ArduinoProtocolError("sensor_timeout"), 145.0],
            2: [250.0, 240.0],
        }
        output = io.StringIO()

        with patch("raspberry.main.ArduinoService", FakeArduinoService):
            exit_code = main_module.main(
                ["--port", "COM9", "--iterations", "2", "--interval", "0.01"],
                output=output,
                sleep_fn=lambda _: None,
            )

        rendered = output.getvalue()
        self.assertEqual(exit_code, 0)
        self.assertIn("[WARN] Датчик 1: sensor_timeout", rendered)
        self.assertIn("Датчик 1: 145.0 мм", rendered)
        self.assertIn("Датчик 2: 250.0 мм", rendered)
        self.assertEqual(FakeArduinoService.created_ports, ["COM9"])
        self.assertEqual(FakeArduinoService.started_sessions, 1)

    def test_diagnostic_mode_returns_error_when_sensor_never_answers(self) -> None:
        FakeArduinoService.scripted_results = {
            1: [main_module.ArduinoProtocolError("sensor_timeout")] * 3,
            2: [180.0, 181.0, 179.0],
        }
        output = io.StringIO()

        with patch("raspberry.main.ArduinoService", FakeArduinoService):
            exit_code = main_module.main(
                ["--diagnose-sensors", "--samples", "3", "--interval", "0.01"],
                output=output,
                sleep_fn=lambda _: None,
            )

        rendered = output.getvalue()
        self.assertEqual(exit_code, 1)
        self.assertIn("Проверка 1/3", rendered)
        self.assertIn("Итог диагностики:", rendered)
        self.assertIn("Датчик 1: успешных чтений 0, ошибок 3", rendered)
        self.assertIn("Датчик 2: успешных чтений 3, ошибок 0", rendered)


    def test_monitor_mode_accepts_fourth_sensor(self) -> None:
        FakeArduinoService.scripted_results = {
            4: [321.0],
        }
        output = io.StringIO()

        with patch("raspberry.main.ArduinoService", FakeArduinoService):
            exit_code = main_module.main(
                ["--port", "COM7", "--iterations", "1", "--sensors", "4"],
                output=output,
                sleep_fn=lambda _: None,
            )

        rendered = output.getvalue()
        self.assertEqual(exit_code, 0)
        self.assertIn("Датчик 4: 321.0 мм", rendered)
        self.assertEqual(FakeArduinoService.created_ports, ["COM7"])


if __name__ == "__main__":
    unittest.main()
