from __future__ import annotations

import json
import logging
import unittest
from dataclasses import dataclass

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


if __name__ == "__main__":
    unittest.main()
