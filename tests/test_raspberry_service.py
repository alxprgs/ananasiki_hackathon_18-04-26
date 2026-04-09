from __future__ import annotations

import os
import signal
import subprocess
import unittest
from pathlib import Path
from unittest.mock import patch

from raspberry.raspberry_service import (
    ProcessInfo,
    RaspberryCommandError,
    RaspberryService,
    WifiAccessPointInfo,
)


class RecordingRunner:
    def __init__(self, responses: dict[tuple[str, ...], subprocess.CompletedProcess[str]]) -> None:
        self.responses = responses
        self.commands: list[list[str]] = []

    def __call__(self, command, capture_output, text, check):
        self.commands.append(command)
        key = tuple(command)
        if key in self.responses:
            return self.responses[key]
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")


class RaspberryServiceTests(unittest.TestCase):
    def test_configure_ap_creates_and_modifies_profile(self) -> None:
        runner = RecordingRunner(
            {
                ("nmcli", "connection", "show", "rescue-maze-ap"): subprocess.CompletedProcess(
                    ["nmcli"], 10, stdout="", stderr="unknown connection"
                )
            }
        )
        service = RaspberryService(
            runner=runner,
            raspberry_pi_detector=lambda: False,
            euid_getter=lambda: 1000,
        )

        service.configure_ap("MazeBot", "RescueMaze123", channel=6, ipv4_cidr="192.168.10.1/24")

        self.assertEqual(runner.commands[0], ["nmcli", "connection", "show", "rescue-maze-ap"])
        self.assertEqual(runner.commands[1][:5], ["nmcli", "connection", "add", "type", "wifi"])
        self.assertIn("802-11-wireless.mode", runner.commands[2])
        self.assertIn("192.168.10.1/24", runner.commands[2])

    def test_configure_ap_propagates_nmcli_failure(self) -> None:
        def failing_runner(command, capture_output, text, check):
            if command[:4] == ["nmcli", "connection", "show", "rescue-maze-ap"]:
                return subprocess.CompletedProcess(command, 0, stdout="", stderr="")
            return subprocess.CompletedProcess(command, 4, stdout="", stderr="permission denied")

        service = RaspberryService(
            runner=failing_runner,
            raspberry_pi_detector=lambda: False,
            euid_getter=lambda: 1000,
        )

        with self.assertRaises(RaspberryCommandError):
            service.configure_ap("MazeBot", "RescueMaze123")

    def test_configure_ap_accepts_auto_channel(self) -> None:
        runner = RecordingRunner(
            {
                ("nmcli", "device", "status"): subprocess.CompletedProcess(
                    ["nmcli"], 0, stdout="DEVICE TYPE STATE CONNECTION\nwlan0 wifi disconnected --\n", stderr=""
                ),
                ("iw", "phy"): subprocess.CompletedProcess(
                    ["iw"], 0, stdout="* 2412 MHz [1]\n* 2437 MHz [6]\n* 2462 MHz [11]\n", stderr=""
                ),
                (
                    "nmcli",
                    "-t",
                    "-f",
                    "CHAN,SIGNAL",
                    "device",
                    "wifi",
                    "list",
                    "--rescan",
                    "yes",
                    "ifname",
                    "wlan0",
                ): subprocess.CompletedProcess(
                    ["nmcli"], 0, stdout="1:80\n6:30\n11:10\n", stderr=""
                ),
                ("nmcli", "connection", "show", "rescue-maze-ap"): subprocess.CompletedProcess(
                    ["nmcli"], 10, stdout="", stderr="unknown connection"
                ),
            }
        )
        service = RaspberryService(
            runner=runner,
            raspberry_pi_detector=lambda: False,
            euid_getter=lambda: 1000,
        )

        service.configure_ap("MazeBot", "RescueMaze123", channel="auto")

        modify_command = runner.commands[-1]
        channel_index = modify_command.index("802-11-wireless.channel") + 1
        self.assertEqual(modify_command[channel_index], "11")

    def test_get_temperature_telemetry_prefers_sysfs(self) -> None:
        runner = RecordingRunner({})
        service = RaspberryService(
            runner=runner,
            raspberry_pi_detector=lambda: False,
            euid_getter=lambda: 1000,
        )

        with patch.object(service, "_path_exists", return_value=True):
            with patch.object(service, "_read_text_file", return_value="47500\n"):
                telemetry = service.get_temperature_telemetry()

        self.assertEqual(telemetry["celsius"], 47.5)
        self.assertEqual(telemetry["state"], "normal")
        self.assertEqual(runner.commands, [])

    def test_get_temperature_telemetry_falls_back_to_vcgencmd(self) -> None:
        runner = RecordingRunner(
            {
                ("vcgencmd", "measure_temp"): subprocess.CompletedProcess(
                    ["vcgencmd"], 0, stdout="temp=71.2'C\n", stderr=""
                )
            }
        )
        service = RaspberryService(
            runner=runner,
            raspberry_pi_detector=lambda: False,
            euid_getter=lambda: 1000,
        )

        with patch.object(service, "_path_exists", return_value=False):
            telemetry = service.get_temperature_telemetry()

        self.assertEqual(telemetry["celsius"], 71.2)
        self.assertEqual(telemetry["state"], "warm")
        self.assertEqual(telemetry["source"], "vcgencmd measure_temp")

    def test_get_power_telemetry_parses_vcgencmd_flags(self) -> None:
        runner = RecordingRunner(
            {
                ("vcgencmd", "get_throttled"): subprocess.CompletedProcess(
                    ["vcgencmd"], 0, stdout="throttled=0x50005\n", stderr=""
                ),
                ("vcgencmd", "measure_volts", "core"): subprocess.CompletedProcess(
                    ["vcgencmd"], 0, stdout="volt=0.8500V\n", stderr=""
                ),
            }
        )
        service = RaspberryService(
            runner=runner,
            raspberry_pi_detector=lambda: False,
            euid_getter=lambda: 1000,
        )

        telemetry = service.get_power_telemetry()

        self.assertEqual(telemetry["throttled_raw"], "0x50005")
        self.assertEqual(telemetry["throttled_mask"], 0x50005)
        self.assertEqual(telemetry["core_voltage_volts"], 0.85)
        self.assertTrue(telemetry["undervoltage_now"])
        self.assertTrue(telemetry["throttled_now"])
        self.assertTrue(telemetry["undervoltage_occurred"])
        self.assertTrue(telemetry["throttling_occurred"])
        self.assertFalse(telemetry["power_good_now"])
        self.assertTrue(telemetry["performance_limited_now"])

    def test_get_board_telemetry_combines_temperature_and_power(self) -> None:
        service = RaspberryService(
            runner=RecordingRunner({}),
            raspberry_pi_detector=lambda: False,
            euid_getter=lambda: 1000,
        )

        with patch.object(service, "get_temperature_telemetry", return_value={"celsius": 48.0}) as temperature_mock:
            with patch.object(service, "get_power_telemetry", return_value={"power_good_now": True}) as power_mock:
                telemetry = service.get_board_telemetry()

        self.assertEqual(telemetry, {"temperature": {"celsius": 48.0}, "power": {"power_good_now": True}})
        temperature_mock.assert_called_once_with()
        power_mock.assert_called_once_with()

    def test_disconnect_all_ssh_terminates_session_processes_but_not_master(self) -> None:
        processes = [
            ProcessInfo(pid=10, ppid=1, command="sshd", args="/usr/sbin/sshd -D [listener] 0 of 10-100 startups"),
            ProcessInfo(pid=21, ppid=10, command="sshd", args="sshd: pi [priv]"),
            ProcessInfo(pid=22, ppid=21, command="sshd", args="sshd: pi@pts/0"),
            ProcessInfo(pid=23, ppid=22, command="bash", args="-bash"),
            ProcessInfo(pid=24, ppid=23, command="python3", args="python3 worker.py"),
        ]

        selected = RaspberryService._select_ssh_session_pids(processes)
        self.assertEqual(selected, {21, 22, 23, 24})

    def test_disconnect_all_ssh_calls_kill_for_session_tree(self) -> None:
        ps_output = "\n".join(
            [
                " 10 1 sshd /usr/sbin/sshd -D [listener] 0 of 10-100 startups",
                " 21 10 sshd sshd: pi [priv]",
                " 22 21 sshd sshd: pi@pts/0",
                " 23 22 bash -bash",
            ]
        )
        runner = RecordingRunner(
            {
                ("ps", "-eo", "pid=,ppid=,comm=,args="): subprocess.CompletedProcess(
                    ["ps"], 0, stdout=ps_output, stderr=""
                )
            }
        )
        service = RaspberryService(
            runner=runner,
            raspberry_pi_detector=lambda: False,
            euid_getter=lambda: 1000,
        )

        with patch.object(os, "kill") as kill_mock:
            terminated = service.disconnect_all_ssh()

        self.assertEqual(terminated, [23, 22, 21])
        kill_mock.assert_any_call(23, signal.SIGTERM)
        kill_mock.assert_any_call(22, signal.SIGTERM)
        kill_mock.assert_any_call(21, signal.SIGTERM)
        self.assertEqual(kill_mock.call_count, 3)

    def test_disable_ap_restores_previous_client_connection(self) -> None:
        state_file = Path("tests") / ".tmp_ap_state.json"
        try:
            state_file.write_text('{"previous_client":"home-wifi"}', encoding="utf-8")
            runner = RecordingRunner(
                {
                    ("nmcli", "device", "status"): subprocess.CompletedProcess(
                        ["nmcli"], 0, stdout="DEVICE TYPE STATE CONNECTION\nwlan0 wifi connected home-wifi\n", stderr=""
                    )
                }
            )
            service = RaspberryService(
                runner=runner,
                state_file=state_file,
                raspberry_pi_detector=lambda: False,
                euid_getter=lambda: 1000,
            )

            service.disable_ap()

            self.assertIn(["nmcli", "connection", "down", "rescue-maze-ap"], runner.commands)
            self.assertIn(["nmcli", "connection", "up", "home-wifi", "ifname", "wlan0"], runner.commands)
        finally:
            if state_file.exists():
                state_file.unlink()

    def test_parse_iw_phy_channels_extracts_supported_24ghz_values(self) -> None:
        output = """
        Wiphy phy0
                Frequencies:
                        * 2412 MHz [1] (20.0 dBm)
                        * 2437 MHz [6] (20.0 dBm)
                        * 2462 MHz [11] (20.0 dBm)
                        * 2472 MHz [13] (disabled)
                        * 5180 MHz [36] (20.0 dBm)
        """

        channels = RaspberryService._parse_iw_phy_channels(output)

        self.assertEqual(channels, [1, 6, 11])

    def test_parse_wifi_scan_output_reads_channel_and_signal(self) -> None:
        output = "1:77\n6:55\n11:22\n"

        access_points = RaspberryService._parse_wifi_scan_output(output)

        self.assertEqual(
            access_points,
            [
                WifiAccessPointInfo(channel=1, signal=77),
                WifiAccessPointInfo(channel=6, signal=55),
                WifiAccessPointInfo(channel=11, signal=22),
            ],
        )

    def test_select_ap_channel_prefers_least_loaded_non_overlapping_channel(self) -> None:
        runner = RecordingRunner(
            {
                ("iw", "phy"): subprocess.CompletedProcess(
                    ["iw"], 0, stdout="* 2412 MHz [1]\n* 2437 MHz [6]\n* 2462 MHz [11]\n", stderr=""
                ),
                (
                    "nmcli",
                    "-t",
                    "-f",
                    "CHAN,SIGNAL",
                    "device",
                    "wifi",
                    "list",
                    "--rescan",
                    "yes",
                    "ifname",
                    "wlan0",
                ): subprocess.CompletedProcess(
                    ["nmcli"], 0, stdout="1:90\n1:60\n6:50\n11:15\n", stderr=""
                ),
            }
        )
        service = RaspberryService(
            runner=runner,
            raspberry_pi_detector=lambda: False,
            euid_getter=lambda: 1000,
        )

        channel = service.select_ap_channel("wlan0")

        self.assertEqual(channel, 11)

    def test_privilege_check_blocks_raspberry_pi_without_root(self) -> None:
        runner = RecordingRunner({})
        service = RaspberryService(
            runner=runner,
            raspberry_pi_detector=lambda: True,
            euid_getter=lambda: 1000,
        )

        with self.assertRaises(RaspberryCommandError) as error:
            service.configure_ap("MazeBot", "RescueMaze123")

        self.assertIn("нужны повышенные права", str(error.exception))
        self.assertEqual(runner.commands, [])

    def test_privilege_check_is_skipped_outside_raspberry_pi(self) -> None:
        runner = RecordingRunner(
            {
                ("nmcli", "connection", "show", "rescue-maze-ap"): subprocess.CompletedProcess(
                    ["nmcli"], 10, stdout="", stderr="unknown connection"
                )
            }
        )
        service = RaspberryService(
            runner=runner,
            raspberry_pi_detector=lambda: False,
            euid_getter=lambda: 1000,
        )

        service.configure_ap("MazeBot", "RescueMaze123")

        self.assertGreater(len(runner.commands), 0)


if __name__ == "__main__":
    unittest.main()
