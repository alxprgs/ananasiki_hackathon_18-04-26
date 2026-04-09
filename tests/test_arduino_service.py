from __future__ import annotations

import json
import logging
import unittest
from dataclasses import dataclass
from unittest.mock import patch

from raspberry.arduino_service import ArduinoProtocolError, ArduinoService, ArduinoUnavailableError


@dataclass
class FakePort:
    device: str


class FakeSerial:
    def __init__(self, scripted_lines: list[str]) -> None:
        self.scripted_lines = [line.encode("utf-8") for line in scripted_lines]
        self.writes: list[str] = []
        self.closed = False

    def reset_input_buffer(self) -> None:
        return None

    def reset_output_buffer(self) -> None:
        return None

    def write(self, payload: bytes) -> int:
        self.writes.append(payload.decode("utf-8"))
        return len(payload)

    def flush(self) -> None:
        return None

    def readline(self) -> bytes:
        if self.scripted_lines:
            return self.scripted_lines.pop(0)
        return b""

    def close(self) -> None:
        self.closed = True


class ArduinoServiceTests(unittest.TestCase):
    def test_autodetects_first_valid_port(self) -> None:
        connections = {
            "COM_BAD": FakeSerial(['{"id":1,"ok":false,"error":{"code":"bad","message":"nope"}}\n']),
            "COM_GOOD": FakeSerial(['{"id":2,"ok":true,"data":{"pong":true}}\n']),
        }

        def serial_factory(**kwargs):
            return connections[kwargs["port"]]

        service = ArduinoService(
            logger=logging.getLogger("test.autodetect"),
            retry_count=0,
            serial_factory=serial_factory,
            port_enumerator=lambda: [FakePort("COM_BAD"), FakePort("COM_GOOD")],
        )
        self.assertEqual(service.port, "COM_GOOD")
        self.assertEqual(len(connections["COM_BAD"].writes), 1)
        self.assertEqual(len(connections["COM_GOOD"].writes), 1)
        service.close()

    def test_logs_fatal_and_raises_when_no_valid_port_exists(self) -> None:
        bad_connection = FakeSerial(['{"id":1,"ok":false,"error":{"code":"bad","message":"nope"}}\n'])

        def serial_factory(**kwargs):
            return bad_connection

        with self.assertLogs("test.no_ports", level="FATAL") as logs:
            with self.assertRaises(ArduinoProtocolError):
                ArduinoService(
                    logger=logging.getLogger("test.no_ports"),
                    serial_factory=serial_factory,
                    port_enumerator=lambda: [FakePort("COM_BAD")],
                )

        self.assertIn("Не удалось найти совместимую Arduino", "\n".join(logs.output))

    def test_response_id_mismatch_raises_protocol_error(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":999,"ok":true,"data":{"pressed":true}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.mismatch"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        with self.assertRaises(ArduinoProtocolError):
            service.button_status()
        service.close()

    def test_distance_conversion_uses_millimeters_from_arduino(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"distance_mm":345}}\n',
                '{"id":3,"ok":true,"data":{"distance_mm":345}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.distance"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        self.assertAlmostEqual(service.distance_sensor.get(1, unit="cm"), 34.5)
        self.assertAlmostEqual(service.distance_sensor.get(1, unit="m"), 0.345)
        service.close()

    def test_timed_pwm_builder_sends_single_atomic_command(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"target":"all","pwm":55,"duration_ms":1500}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.motor"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        service.eng_all.pwm(55).time(1.5)

        self.assertEqual(len(connection.writes), 2)
        payload = json.loads(connection.writes[1])
        self.assertEqual(payload["op"], "set_motor")
        self.assertEqual(payload["args"]["target"], "all")
        self.assertEqual(payload["args"]["pwm"], 55)
        self.assertEqual(payload["args"]["duration_ms"], 1500)
        service.close()

    def test_requested_port_failure_raises_original_error(self) -> None:
        connection = FakeSerial(['{"id":1,"ok":false,"error":{"code":"bad","message":"nope"}}\n'])

        with self.assertRaises(ArduinoProtocolError):
            ArduinoService(
                port="COM1",
                logger=logging.getLogger("test.port_override"),
                retry_count=0,
                serial_factory=lambda **kwargs: connection,
                port_enumerator=lambda: [],
            )

    def test_no_serial_ports_raises_unavailable_error(self) -> None:
        with self.assertRaises(ArduinoUnavailableError):
            ArduinoService(
                logger=logging.getLogger("test.empty"),
                serial_factory=lambda **kwargs: FakeSerial([]),
                port_enumerator=lambda: [],
            )

    @patch("raspberry.arduino_service.time.sleep", return_value=None)
    def test_align_parallel_to_wall_turns_right_for_right_wall(self, _sleep) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"distance_mm":140}}\n',
                '{"id":3,"ok":true,"data":{"distance_mm":100}}\n',
                '{"id":4,"ok":true,"data":{"target":"left","pwm":20}}\n',
                '{"id":5,"ok":true,"data":{"target":"right","pwm":-20}}\n',
                '{"id":6,"ok":true,"data":{"stopped":true}}\n',
                '{"id":7,"ok":true,"data":{"distance_mm":109}}\n',
                '{"id":8,"ok":true,"data":{"distance_mm":104}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.align.right"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        result = service.align_parallel_to_wall(
            front_sensor_id=1,
            rear_sensor_id=2,
            wall_side="right",
            tolerance_mm=6.0,
            turn_power=20,
            pulse_seconds=0.1,
            settle_seconds=0.02,
            max_iterations=3,
        )

        self.assertTrue(result["aligned"])
        self.assertEqual(result["correction_steps"], 1)
        self.assertEqual(result["last_turn_direction"], "right")

        left_motor_payload = json.loads(connection.writes[3])
        right_motor_payload = json.loads(connection.writes[4])
        self.assertEqual(left_motor_payload["args"]["target"], "left")
        self.assertEqual(left_motor_payload["args"]["pwm"], 20)
        self.assertEqual(right_motor_payload["args"]["target"], "right")
        self.assertEqual(right_motor_payload["args"]["pwm"], -20)
        service.close()

    @patch("raspberry.arduino_service.time.sleep", return_value=None)
    def test_align_parallel_to_wall_turns_left_for_left_wall(self, _sleep) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"distance_mm":150}}\n',
                '{"id":3,"ok":true,"data":{"distance_mm":100}}\n',
                '{"id":4,"ok":true,"data":{"target":"left","pwm":-15}}\n',
                '{"id":5,"ok":true,"data":{"target":"right","pwm":15}}\n',
                '{"id":6,"ok":true,"data":{"stopped":true}}\n',
                '{"id":7,"ok":true,"data":{"distance_mm":106}}\n',
                '{"id":8,"ok":true,"data":{"distance_mm":102}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.align.left"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        result = service.align_parallel_to_wall(
            front_sensor_id=1,
            rear_sensor_id=2,
            wall_side="left",
            tolerance_mm=5.0,
            turn_power=15,
            pulse_seconds=0.1,
            settle_seconds=0.02,
            max_iterations=3,
        )

        self.assertTrue(result["aligned"])
        self.assertEqual(result["last_turn_direction"], "left")
        left_motor_payload = json.loads(connection.writes[3])
        right_motor_payload = json.loads(connection.writes[4])
        self.assertEqual(left_motor_payload["args"]["pwm"], -15)
        self.assertEqual(right_motor_payload["args"]["pwm"], 15)
        service.close()

    def test_align_parallel_to_wall_returns_false_after_max_iterations(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"distance_mm":150}}\n',
                '{"id":3,"ok":true,"data":{"distance_mm":100}}\n',
                '{"id":4,"ok":true,"data":{"target":"left","pwm":18}}\n',
                '{"id":5,"ok":true,"data":{"target":"right","pwm":-18}}\n',
                '{"id":6,"ok":true,"data":{"stopped":true}}\n',
                '{"id":7,"ok":true,"data":{"distance_mm":148}}\n',
                '{"id":8,"ok":true,"data":{"distance_mm":100}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.align.fail"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        with patch("raspberry.arduino_service.time.sleep", return_value=None):
            result = service.align_parallel_to_wall(max_iterations=2)

        self.assertFalse(result["aligned"])
        self.assertEqual(result["correction_steps"], 1)
        self.assertAlmostEqual(result["delta_mm"], 48.0)
        service.close()


if __name__ == "__main__":
    unittest.main()
