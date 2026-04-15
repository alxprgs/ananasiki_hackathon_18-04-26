from __future__ import annotations

import json
import logging
import os
import re
import shutil
import unittest
from dataclasses import dataclass
from pathlib import Path
from unittest.mock import patch

from raspberry.arduino_service import (
    ArduinoProtocolError,
    ArduinoService,
    ArduinoUnavailableError,
    BuzzerStep,
    MotionMapCalibration,
    SerialTimeoutError,
    UnsupportedHardwareError,
)


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


class FakeClock:
    def __init__(self, start: float = 0.0) -> None:
        self.current = float(start)

    def now(self) -> float:
        return self.current

    def advance(self, seconds: float) -> None:
        self.current += float(seconds)


class ArduinoServiceTests(unittest.TestCase):
    def _make_test_dir(self, name: str) -> Path:
        root = Path("tests") / ".tmp_activity_tests" / name
        if root.exists():
            shutil.rmtree(root)
        root.mkdir(parents=True, exist_ok=True)
        self.addCleanup(lambda: shutil.rmtree(root, ignore_errors=True))
        return root

    @staticmethod
    def _extract_polyline_points(svg: str) -> list[str]:
        match = re.search(r'<polyline[^>]*points="([^"]+)"', svg)
        if match is None:
            raise AssertionError("route.svg does not contain a polyline")
        return [point for point in match.group(1).split() if point]

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

    def test_status_returns_extended_distance_sensor_metadata(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"led_enabled":true,"buzzer_playing":true,"buzzer_frequency_hz":2200,"distance_sensors":{"1":{"enabled":true,"kind":"hc_sr04","pins":{"trigger":2,"echo":3,"serial_rx":null,"serial_tx":null},"serial_settings_available":false,"distance_mm":345},"2":{"enabled":true,"kind":"urm37","pins":{"trigger":8,"echo":9,"serial_rx":10,"serial_tx":11},"serial_settings_available":true,"distance_mm":680},"3":{"enabled":false,"kind":"disabled","pins":{"trigger":null,"echo":null,"serial_rx":null,"serial_tx":null},"serial_settings_available":false,"distance_mm":null},"4":{"enabled":false,"kind":"disabled","pins":{"trigger":null,"echo":null,"serial_rx":null,"serial_tx":null},"serial_settings_available":false,"distance_mm":null}},"features":{"led":true,"buzzer":true}}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.status.sensors"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        payload = service.status()
        self.assertIn("distance_sensors", payload)
        self.assertEqual(payload["distance_sensors"]["2"]["kind"], "urm37")
        self.assertTrue(payload["distance_sensors"]["2"]["serial_settings_available"])
        self.assertTrue(payload["led_enabled"])
        self.assertTrue(payload["buzzer_playing"])
        self.assertEqual(payload["buzzer_frequency_hz"], 2200)
        self.assertTrue(payload["features"]["led"])
        self.assertTrue(payload["features"]["buzzer"])
        service.close()

    def test_distance_sensor_info_reads_extended_status(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"distance_sensors":{"1":{"enabled":true,"kind":"hc_sr04","pins":{"trigger":2,"echo":3,"serial_rx":null,"serial_tx":null},"serial_settings_available":false,"distance_mm":340},"2":{"enabled":true,"kind":"urm37","pins":{"trigger":8,"echo":9,"serial_rx":10,"serial_tx":11},"serial_settings_available":true,"distance_mm":680},"3":{"enabled":false,"kind":"disabled","pins":{"trigger":null,"echo":null,"serial_rx":null,"serial_tx":null},"serial_settings_available":false,"distance_mm":null},"4":{"enabled":false,"kind":"disabled","pins":{"trigger":null,"echo":null,"serial_rx":null,"serial_tx":null},"serial_settings_available":false,"distance_mm":null}}}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.sensor.info"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        info = service.distance_sensor.info(2)
        self.assertEqual(info.sensor_id, 2)
        self.assertEqual(info.kind, "urm37")
        self.assertEqual(info.trigger_pin, 8)
        self.assertEqual(info.serial_rx_pin, 10)
        self.assertTrue(info.serial_settings_available)
        self.assertAlmostEqual(info.distance_mm or 0.0, 680.0)
        service.close()

    def test_urm37_temperature_and_settings_api(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"temperature_c":23.75}}\n',
                '{"id":3,"ok":true,"data":{"measure_mode":"auto","auto_measure_interval_ms":70,"compare_distance_cm":150,"sensitivity":120}}\n',
                '{"id":4,"ok":true,"data":{"measure_mode":"pwm_passive","auto_measure_interval_ms":55,"compare_distance_cm":90,"sensitivity":110}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.urm37.api"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        self.assertAlmostEqual(service.distance_sensor.get_temperature(2), 23.75)
        settings = service.distance_sensor.get_urm37_settings(2)
        self.assertEqual(settings.measure_mode, "auto")
        self.assertEqual(settings.auto_measure_interval_ms, 70)

        updated_settings = service.distance_sensor.configure_urm37(
            2,
            measure_mode="pwm_passive",
            auto_measure_interval_ms=55,
            compare_distance_cm=90,
            sensitivity=110,
        )
        self.assertEqual(updated_settings.measure_mode, "pwm_passive")

        payload = json.loads(connection.writes[3])
        self.assertEqual(payload["op"], "set_urm37_settings")
        self.assertEqual(payload["args"]["sensor"], 2)
        self.assertEqual(payload["args"]["auto_measure_interval_ms"], 55)
        self.assertEqual(payload["args"]["compare_distance_cm"], 90)
        self.assertEqual(payload["args"]["sensitivity"], 110)
        service.close()

    @patch("raspberry.arduino_service.time.sleep", return_value=None)
    def test_timed_pwm_builder_sends_single_atomic_command(self, sleep_mock) -> None:
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

        with self.assertLogs("test.motor", level="WARNING") as logs:
            service.eng_all.pwm(55).time(1.5)

        self.assertEqual(len(connection.writes), 2)
        payload = json.loads(connection.writes[1])
        self.assertEqual(payload["op"], "set_motor")
        self.assertEqual(payload["args"]["target"], "all")
        self.assertEqual(payload["args"]["pwm"], 55)
        self.assertEqual(payload["args"]["duration_ms"], 1500)
        sleep_mock.assert_called_once_with(1.5)
        self.assertIn("При значениях ниже 60 двигатель может работать нестабильно", "\n".join(logs.output))
        service.close()

    @patch("raspberry.arduino_service.time.sleep", return_value=None)
    def test_timed_ramp_builder_sends_single_atomic_command(self, sleep_mock) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"target":"right","pwm":100,"duration_ms":3000,"start_pwm":10,"ramp_duration_ms":500}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.motor.ramp"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        with self.assertLogs("test.motor.ramp", level="WARNING") as logs:
            service.eng_r.ramp(start_pwm=10, stop_pwm=100, ramp_seconds=0.5).time(3)

        payload = json.loads(connection.writes[1])
        self.assertEqual(payload["op"], "set_motor")
        self.assertEqual(payload["args"]["target"], "right")
        self.assertEqual(payload["args"]["pwm"], 100)
        self.assertEqual(payload["args"]["start_pwm"], 10)
        self.assertEqual(payload["args"]["ramp_duration_ms"], 500)
        self.assertEqual(payload["args"]["duration_ms"], 3000)
        sleep_mock.assert_called_once_with(3.0)
        self.assertIn("ramp", "\n".join(logs.output))
        service.close()

    def test_ramp_now_sends_command_without_duration(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"target":"left","pwm":90,"start_pwm":70,"ramp_duration_ms":250}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.motor.ramp.now"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        service.eng_l.ramp(start_pwm=70, stop_pwm=90, ramp_seconds=0.25).now()

        payload = json.loads(connection.writes[1])
        self.assertEqual(payload["args"]["target"], "left")
        self.assertEqual(payload["args"]["start_pwm"], 70)
        self.assertEqual(payload["args"]["pwm"], 90)
        self.assertEqual(payload["args"]["ramp_duration_ms"], 250)
        self.assertNotIn("duration_ms", payload["args"])
        service.close()

    def test_ramp_time_rejects_total_shorter_than_ramp(self) -> None:
        connection = FakeSerial(['{"id":1,"ok":true,"data":{"pong":true}}\n'])
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.motor.ramp.validation"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        with self.assertRaises(ValueError):
            service.eng_all.ramp(start_pwm=10, stop_pwm=100, ramp_seconds=0.6).time(0.5)

        service.close()

    def test_zero_second_ramp_falls_back_to_plain_pwm_command(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"target":"all","pwm":80,"duration_ms":1000}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.motor.ramp.zero"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        with patch("raspberry.arduino_service.time.sleep", return_value=None):
            service.eng_all.ramp(start_pwm=10, stop_pwm=80, ramp_seconds=0).time(1)

        payload = json.loads(connection.writes[1])
        self.assertEqual(payload["args"]["pwm"], 80)
        self.assertNotIn("start_pwm", payload["args"])
        self.assertNotIn("ramp_duration_ms", payload["args"])
        service.close()

    def test_led_service_and_controller_send_expected_payloads(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"enabled":true}}\n',
                '{"id":3,"ok":true,"data":{"enabled":false}}\n',
                '{"id":4,"ok":true,"data":{"enabled":true}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.led"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        self.assertTrue(service.set_led(True)["enabled"])
        self.assertFalse(service.led.off()["enabled"])
        self.assertTrue(service.led.set(True)["enabled"])

        payloads = [json.loads(raw) for raw in connection.writes[1:4]]
        self.assertEqual([payload["op"] for payload in payloads], ["set_led", "set_led", "set_led"])
        self.assertEqual([payload["args"]["enabled"] for payload in payloads], [True, False, True])
        service.close()

    def test_buzzer_service_and_controller_send_expected_ops(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"playing":true,"frequency_hz":2400,"duration_ms":150}}\n',
                '{"id":3,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
                '{"id":4,"ok":true,"data":{"playing":true,"frequency_hz":1800,"duration_ms":null}}\n',
                '{"id":5,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.buzzer.service"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        self.assertEqual(service.play_buzzer(2400, duration_ms=150)["frequency_hz"], 2400)
        self.assertFalse(service.stop_buzzer()["playing"])
        self.assertEqual(service.buzzer.on(1800)["frequency_hz"], 1800)
        self.assertFalse(service.buzzer.off()["playing"])

        first_payload = json.loads(connection.writes[1])
        self.assertEqual(first_payload["op"], "buzzer_play")
        self.assertEqual(first_payload["args"]["frequency_hz"], 2400)
        self.assertEqual(first_payload["args"]["duration_ms"], 150)
        stop_payload = json.loads(connection.writes[2])
        self.assertEqual(stop_payload["op"], "buzzer_stop")
        on_payload = json.loads(connection.writes[3])
        self.assertEqual(on_payload["args"]["frequency_hz"], 1800)
        self.assertNotIn("duration_ms", on_payload["args"])
        service.close()

    @patch("raspberry.arduino_service.time.sleep", return_value=None)
    def test_buzzer_tone_waits_for_requested_duration(self, sleep_mock) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"playing":true,"frequency_hz":1500,"duration_ms":250}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.buzzer.tone"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        response = service.buzzer.tone(1500, duration=0.25)

        self.assertEqual(response["duration_ms"], 250)
        sleep_mock.assert_called_once_with(0.25)
        service.close()

    @patch("raspberry.arduino_service.time.sleep", return_value=None)
    def test_buzzer_beep_issues_expected_sequence(self, sleep_mock) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"playing":true,"frequency_hz":2000,"duration_ms":100}}\n',
                '{"id":3,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
                '{"id":4,"ok":true,"data":{"playing":true,"frequency_hz":2000,"duration_ms":100}}\n',
                '{"id":5,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
                '{"id":6,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.buzzer.beep"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        result = service.buzzer.beep(duration=0.1, pause=0.05, repeat=2)

        self.assertEqual(result["repeat"], 2)
        ops = [json.loads(raw)["op"] for raw in connection.writes[1:]]
        self.assertEqual(ops, ["buzzer_play", "buzzer_stop", "buzzer_play", "buzzer_stop", "buzzer_stop"])
        self.assertEqual(sleep_mock.call_args_list[0].args[0], 0.1)
        self.assertEqual(sleep_mock.call_args_list[1].args[0], 0.05)
        self.assertEqual(sleep_mock.call_args_list[2].args[0], 0.1)
        self.assertEqual(sleep_mock.call_args_list[3].args[0], 0.05)
        service.close()

    @patch("raspberry.arduino_service.time.sleep", return_value=None)
    def test_buzzer_play_sequence_accepts_tone_and_pause_steps(self, sleep_mock) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"playing":true,"frequency_hz":1200,"duration_ms":80}}\n',
                '{"id":3,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
                '{"id":4,"ok":true,"data":{"playing":true,"frequency_hz":1600,"duration_ms":90}}\n',
                '{"id":5,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.buzzer.sequence"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        result = service.buzzer.play_sequence(
            [
                BuzzerStep(frequency_hz=1200, duration_ms=80),
                BuzzerStep(frequency_hz=None, duration_ms=40),
                BuzzerStep(frequency_hz=1600, duration_ms=90),
            ]
        )

        self.assertEqual(result["total_duration_ms"], 210)
        ops = [json.loads(raw)["op"] for raw in connection.writes[1:]]
        self.assertEqual(ops, ["buzzer_play", "buzzer_stop", "buzzer_play", "buzzer_stop"])
        self.assertEqual([call.args[0] for call in sleep_mock.call_args_list], [0.08, 0.04, 0.09])
        service.close()

    @patch("raspberry.arduino_service.time.sleep", return_value=None)
    def test_buzzer_presets_use_fixed_sequences(self, _sleep) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"playing":true,"frequency_hz":1600,"duration_ms":90}}\n',
                '{"id":3,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
                '{"id":4,"ok":true,"data":{"playing":true,"frequency_hz":2200,"duration_ms":140}}\n',
                '{"id":5,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
                '{"id":6,"ok":true,"data":{"playing":true,"frequency_hz":1800,"duration_ms":120}}\n',
                '{"id":7,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
                '{"id":8,"ok":true,"data":{"playing":true,"frequency_hz":1800,"duration_ms":120}}\n',
                '{"id":9,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
                '{"id":10,"ok":true,"data":{"playing":true,"frequency_hz":700,"duration_ms":220}}\n',
                '{"id":11,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
                '{"id":12,"ok":true,"data":{"playing":true,"frequency_hz":520,"duration_ms":320}}\n',
                '{"id":13,"ok":true,"data":{"playing":false,"frequency_hz":null}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.buzzer.presets"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        service.buzzer.success()
        service.buzzer.warning()
        service.buzzer.error()

        payloads = [json.loads(raw) for raw in connection.writes[1:]]
        play_frequencies = [payload["args"]["frequency_hz"] for payload in payloads if payload["op"] == "buzzer_play"]
        self.assertEqual(play_frequencies, [1600, 2200, 1800, 1800, 700, 520])
        service.close()

    def test_buzzer_validation_errors_happen_before_serial_write(self) -> None:
        connection = FakeSerial(['{"id":1,"ok":true,"data":{"pong":true}}\n'])
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.buzzer.validation"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        with self.assertRaises(ValueError):
            service.play_buzzer(10)
        with self.assertRaises(ValueError):
            service.buzzer.beep(duration=0)
        with self.assertRaises(ValueError):
            service.buzzer.play_sequence([], repeat=1)
        with self.assertRaises(ValueError):
            service.buzzer.play_sequence([BuzzerStep(frequency_hz=None, duration_ms=50)], repeat=0)

        self.assertEqual(len(connection.writes), 1)
        service.close()

    def test_led_and_buzzer_unsupported_errors_are_mapped(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":false,"error":{"code":"unsupported","message":"LED is not configured"}}\n',
                '{"id":3,"ok":false,"error":{"code":"unsupported","message":"buzzer is not configured"}}\n',
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.led.buzzer.unsupported"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        with self.assertRaises(UnsupportedHardwareError):
            service.set_led(True)
        with self.assertRaises(UnsupportedHardwareError):
            service.play_buzzer(2000)

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

    @patch("raspberry.arduino_service.time.sleep", return_value=None)
    def test_requested_port_waits_for_device_warmup_before_ping(self, sleep_mock) -> None:
        connection = FakeSerial(['{"id":1,"ok":true,"data":{"pong":true}}\n'])
        connection.port = "COM1"  # type: ignore[attr-defined]

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.warmup"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        sleep_mock.assert_called_once_with(2.0)
        service.close()

    def test_no_serial_ports_raises_unavailable_error(self) -> None:
        with self.assertRaises(ArduinoUnavailableError):
            ArduinoService(
                logger=logging.getLogger("test.empty"),
                serial_factory=lambda **kwargs: FakeSerial([]),
                port_enumerator=lambda: [],
            )

    def test_timeout_logs_request_id_and_timeout(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                "",
            ]
        )

        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.timeout"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        with self.assertLogs("test.timeout", level="WARNING") as logs:
            with self.assertRaises(SerialTimeoutError):
                service.button_status()

        self.assertIn("Arduino не ответила на запрос id=2 в течение 1.00 с", "\n".join(logs.output))
        service.close()

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

    def test_activity_session_writes_actions_json_and_svg(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"pong":true}}\n',
                '{"id":3,"ok":true,"data":{"pressed":false}}\n',
                '{"id":4,"ok":true,"data":{"distance_mm":345}}\n',
                '{"id":5,"ok":true,"data":{"target":"all","pwm":40,"duration_ms":1500}}\n',
            ]
        )
        clock = FakeClock(start=10.0)
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.activity.files"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
            monotonic_clock=clock.now,
        )

        session_dir = self._make_test_dir("activity_files") / "session"
        service.start_activity_session(
            output_dir=session_dir,
            calibration=MotionMapCalibration(
                max_linear_speed_mm_per_sec=200.0,
                max_turn_deg_per_sec=180.0,
            ),
        )

        service.ping()
        service.button_status()
        service.distance_sensor.get(1, unit="cm")
        with patch("raspberry.arduino_service.time.sleep", side_effect=lambda seconds: clock.advance(seconds)):
            service.eng_all.pwm(40).time(1.5)
        summary = service.stop_activity_session()

        self.assertEqual(Path(summary["output_dir"]), session_dir)
        self.assertTrue((session_dir / "actions.txt").exists())
        self.assertTrue((session_dir / "events.json").exists())
        self.assertTrue((session_dir / "route.svg").exists())
        self.assertAlmostEqual(summary["final_pose"]["x_mm"], 120.0, places=3)

        actions_text = (session_dir / "actions.txt").read_text(encoding="utf-8")
        self.assertIn("Проверка связи с Arduino на порту COM1 выполнена — Удачно", actions_text)
        self.assertIn("Считано состояние кнопки: отпущена — Удачно", actions_text)
        self.assertIn("Получено расстояние с датчика 1: 34.5 см — Удачно", actions_text)
        self.assertIn("Робот проехал вперёд на команде 40% в течение 1.50 с — Выполнено", actions_text)

        events = json.loads((session_dir / "events.json").read_text(encoding="utf-8"))
        self.assertTrue(any(event["action"] == "set_motor" for event in events))
        self.assertTrue(any(event["protocol_op"] == "get_distance" for event in events))

        route_svg = (session_dir / "route.svg").read_text(encoding="utf-8")
        self.assertIn("Оценочная карта движения, не точная одометрия", route_svg)
        self.assertIn("polyline", route_svg)

        service.close()

    def test_activity_session_does_not_extrapolate_future_timed_motion_on_close(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"target":"all","pwm":40,"duration_ms":1500}}\n',
            ]
        )
        clock = FakeClock(start=50.0)
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.activity.no_extrapolation"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
            monotonic_clock=clock.now,
        )

        session_dir = self._make_test_dir("activity_no_extrapolation") / "session"
        service.start_activity_session(
            output_dir=session_dir,
            calibration=MotionMapCalibration(
                max_linear_speed_mm_per_sec=200.0,
                max_turn_deg_per_sec=180.0,
            ),
        )
        service._send_motor_command(target="all", pwm=40, duration_ms=1500)
        summary = service.stop_activity_session()

        self.assertAlmostEqual(summary["final_pose"]["x_mm"], 0.0, places=3)

        events = json.loads((session_dir / "events.json").read_text(encoding="utf-8"))
        stop_event = next(event for event in events if event["action"] == "stop_activity_session")
        self.assertAlmostEqual(stop_event["monotonic_seconds"], 0.0, places=3)

        service.close()

    def test_activity_session_logs_errors_without_suppressing_exception(self) -> None:
        connection = FakeSerial(['{"id":1,"ok":true,"data":{"pong":true}}\n'])
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.activity.errors"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        session_dir = self._make_test_dir("activity_errors") / "session"
        service.start_activity_session(output_dir=session_dir, include_map=False)

        with self.assertRaises(ValueError):
            service.distance_sensor.get(5)

        summary = service.stop_activity_session()
        self.assertIsNone(summary["route_path"])
        actions_text = (session_dir / "actions.txt").read_text(encoding="utf-8")
        self.assertIn("Не удалось получить расстояние с датчика 5", actions_text)
        self.assertIn("| ERROR |", actions_text)

        events = json.loads((session_dir / "events.json").read_text(encoding="utf-8"))
        error_events = [event for event in events if event["action"] == "distance_sensor.get" and not event["success"]]
        self.assertEqual(len(error_events), 1)
        self.assertEqual(error_events[0]["error_type"], "ValueError")

        service.close()

    @patch("raspberry.arduino_service.time.sleep", return_value=None)
    def test_align_parallel_to_wall_records_summary_event(self, _sleep) -> None:
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
            logger=logging.getLogger("test.activity.align"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        session_dir = self._make_test_dir("activity_align") / "session"
        service.start_activity_session(output_dir=session_dir, include_map=False)
        service.align_parallel_to_wall(
            front_sensor_id=1,
            rear_sensor_id=2,
            wall_side="right",
            tolerance_mm=8.0,
            turn_power=20,
            pulse_seconds=0.1,
            settle_seconds=0.02,
            max_iterations=3,
        )
        service.stop_activity_session()

        actions_text = (session_dir / "actions.txt").read_text(encoding="utf-8")
        self.assertIn("Робот выровнился по правой стене с погрешностью не более 8.0 мм — Удачно", actions_text)

        events = json.loads((session_dir / "events.json").read_text(encoding="utf-8"))
        align_events = [event for event in events if event["action"] == "align_parallel_to_wall"]
        self.assertEqual(len(align_events), 1)
        self.assertTrue(align_events[0]["success"])
        self.assertEqual(align_events[0]["result"]["correction_steps"], 1)

        service.close()

    def test_activity_session_integrates_now_motion_until_stop_all(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"target":"all","pwm":50}}\n',
                '{"id":3,"ok":true,"data":{"stopped":true}}\n',
            ]
        )
        clock = FakeClock(start=100.0)
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.activity.now"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
            monotonic_clock=clock.now,
        )

        session_dir = self._make_test_dir("activity_now") / "session"
        service.start_activity_session(
            output_dir=session_dir,
            calibration=MotionMapCalibration(
                max_linear_speed_mm_per_sec=100.0,
                max_turn_deg_per_sec=180.0,
            ),
        )
        service.eng_all.pwm(50).now()
        clock.advance(2.0)
        service.stop_all()
        summary = service.stop_activity_session()

        self.assertAlmostEqual(summary["final_pose"]["x_mm"], 100.0, places=3)
        self.assertAlmostEqual(summary["final_pose"]["y_mm"], 0.0, places=3)

        service.close()

    def test_activity_session_integrates_ramp_motion_using_actual_elapsed_time(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"target":"all","pwm":100,"duration_ms":3000,"start_pwm":0,"ramp_duration_ms":1000}}\n',
            ]
        )
        clock = FakeClock(start=200.0)
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.activity.ramp"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
            monotonic_clock=clock.now,
        )

        session_dir = self._make_test_dir("activity_ramp") / "session"
        service.start_activity_session(
            output_dir=session_dir,
            calibration=MotionMapCalibration(
                max_linear_speed_mm_per_sec=100.0,
                max_turn_deg_per_sec=180.0,
            ),
        )

        with patch("raspberry.arduino_service.time.sleep", side_effect=lambda seconds: clock.advance(seconds)):
            service.eng_all.ramp(start_pwm=0, stop_pwm=100, ramp_seconds=1.0).time(3.0)
        summary = service.stop_activity_session()

        self.assertAlmostEqual(summary["final_pose"]["x_mm"], 250.0, places=3)
        events = json.loads((session_dir / "events.json").read_text(encoding="utf-8"))
        set_motor_event = next(event for event in events if event["action"] == "set_motor")
        self.assertEqual(set_motor_event["params"]["start_pwm"], 0)
        self.assertEqual(set_motor_event["params"]["ramp_duration_ms"], 1000)
        service.close()

    def test_activity_session_integrates_curved_motion_for_single_right_motor(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"target":"right","pwm":100,"duration_ms":10000}}\n',
            ]
        )
        clock = FakeClock(start=300.0)
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.activity.curved.right"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
            monotonic_clock=clock.now,
        )

        session_dir = self._make_test_dir("activity_curved_right") / "session"
        service.start_activity_session(output_dir=session_dir)

        with patch("raspberry.arduino_service.time.sleep", side_effect=lambda seconds: clock.advance(seconds)):
            service.eng_r.pwm(100).time(10.0)
        summary = service.stop_activity_session()

        self.assertAlmostEqual(summary["final_pose"]["x_mm"], 0.0, places=3)
        self.assertAlmostEqual(summary["final_pose"]["y_mm"], 203.718, places=3)
        self.assertAlmostEqual(summary["final_pose"]["heading_deg"], -180.0, places=3)

        route_svg = (session_dir / "route.svg").read_text(encoding="utf-8")
        self.assertGreater(len(self._extract_polyline_points(route_svg)), 2)
        service.close()

    def test_activity_session_close_records_partial_curved_timed_motion(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"target":"right","pwm":50,"duration_ms":10000}}\n',
            ]
        )
        clock = FakeClock(start=400.0)
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.activity.curved.partial"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
            monotonic_clock=clock.now,
        )

        session_dir = self._make_test_dir("activity_curved_partial") / "session"
        service.start_activity_session(output_dir=session_dir)
        service._send_motor_command(target="right", pwm=50, duration_ms=10000)
        clock.advance(2.485)
        service.close()

        events = json.loads((session_dir / "events.json").read_text(encoding="utf-8"))
        stop_event = next(event for event in events if event["action"] == "stop_activity_session")
        self.assertAlmostEqual(stop_event["position"]["x_mm"], 94.558, places=3)
        self.assertAlmostEqual(stop_event["position"]["y_mm"], 139.728, places=3)
        self.assertAlmostEqual(stop_event["position"]["heading_deg"], 111.825, places=3)

        route_svg = (session_dir / "route.svg").read_text(encoding="utf-8")
        self.assertGreater(len(self._extract_polyline_points(route_svg)), 2)

    def test_activity_session_integrates_curved_motion_for_single_left_motor(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"target":"left","pwm":100,"duration_ms":10000}}\n',
            ]
        )
        clock = FakeClock(start=500.0)
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.activity.curved.left"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
            monotonic_clock=clock.now,
        )

        session_dir = self._make_test_dir("activity_curved_left") / "session"
        service.start_activity_session(output_dir=session_dir)

        with patch("raspberry.arduino_service.time.sleep", side_effect=lambda seconds: clock.advance(seconds)):
            service.eng_l.pwm(100).time(10.0)
        summary = service.stop_activity_session()

        self.assertAlmostEqual(summary["final_pose"]["x_mm"], 0.0, places=3)
        self.assertAlmostEqual(summary["final_pose"]["y_mm"], -203.718, places=3)
        self.assertAlmostEqual(summary["final_pose"]["heading_deg"], -180.0, places=3)

        route_svg = (session_dir / "route.svg").read_text(encoding="utf-8")
        self.assertGreater(len(self._extract_polyline_points(route_svg)), 2)
        service.close()

    def test_no_activity_artifacts_created_when_recording_is_disabled(self) -> None:
        connection = FakeSerial(
            [
                '{"id":1,"ok":true,"data":{"pong":true}}\n',
                '{"id":2,"ok":true,"data":{"pong":true}}\n',
            ]
        )
        service = ArduinoService(
            port="COM1",
            logger=logging.getLogger("test.activity.disabled"),
            retry_count=0,
            serial_factory=lambda **kwargs: connection,
            port_enumerator=lambda: [],
        )

        work_dir = self._make_test_dir("activity_disabled")
        old_cwd = os.getcwd()
        os.chdir(work_dir)
        try:
            service.ping()
            self.assertFalse(service.activity_session_active)
            self.assertFalse((work_dir / "logs" / "arduino_sessions").exists())
        finally:
            os.chdir(old_cwd)

        service.close()


if __name__ == "__main__":
    unittest.main()
