from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable


LOGGER = logging.getLogger(__name__)


class RaspberryServiceError(RuntimeError):
    """Базовое исключение для ошибок сервисов на стороне Raspberry Pi."""


class RaspberryCommandError(RaspberryServiceError):
    """Выбрасывается, когда системная команда недоступна или завершилась ошибкой."""


@dataclass(slots=True)
class ProcessInfo:
    pid: int
    ppid: int
    command: str
    args: str


class RaspberryService:
    def __init__(
        self,
        *,
        logger: logging.Logger | None = None,
        runner: Callable[..., subprocess.CompletedProcess[str]] | None = None,
        state_file: str | Path | None = None,
        ap_profile_name: str = "rescue-maze-ap",
    ) -> None:
        self._logger = logger or LOGGER
        self._runner = runner or subprocess.run
        self._state_file = Path(state_file or "/tmp/rescue_maze_ap_state.json")
        self._ap_profile_name = ap_profile_name

    def configure_ap(
        self,
        ssid: str,
        password: str,
        *,
        channel: int = 1,
        ipv4_cidr: str = "192.168.4.1/24",
    ) -> None:
        if not ssid:
            raise ValueError("ssid не должен быть пустым")
        if len(password) < 8:
            raise ValueError("password должен содержать минимум 8 символов")
        if not 1 <= channel <= 13:
            raise ValueError("channel должен быть в диапазоне от 1 до 13")

        if not self._connection_exists(self._ap_profile_name):
            self._nmcli(
                "connection",
                "add",
                "type",
                "wifi",
                "con-name",
                self._ap_profile_name,
                "ssid",
                ssid,
                "autoconnect",
                "no",
            )

        self._nmcli(
            "connection",
            "modify",
            self._ap_profile_name,
            "802-11-wireless.mode",
            "ap",
            "802-11-wireless.ssid",
            ssid,
            "802-11-wireless.band",
            "bg",
            "802-11-wireless.channel",
            str(channel),
            "wifi-sec.key-mgmt",
            "wpa-psk",
            "wifi-sec.psk",
            password,
            "ipv4.method",
            "manual",
            "ipv4.addresses",
            ipv4_cidr,
            "ipv6.method",
            "disabled",
            "connection.autoconnect",
            "no",
        )

    def enable_ap(self) -> None:
        wifi_device = self._detect_wifi_device()
        previous_client = self._active_wifi_connection_name(wifi_device)
        if previous_client and previous_client != self._ap_profile_name:
            self._save_state({"previous_client": previous_client})
            self._nmcli("connection", "down", previous_client, check=False)

        self._nmcli("device", "disconnect", wifi_device, check=False)
        self._nmcli("connection", "up", self._ap_profile_name, "ifname", wifi_device)

    def disable_ap(self) -> None:
        state = self._load_state()
        previous_client = state.get("previous_client")
        wifi_device = self._detect_wifi_device()

        self._nmcli("connection", "down", self._ap_profile_name, check=False)

        if previous_client:
            self._nmcli("connection", "up", previous_client, "ifname", wifi_device)
            self._save_state({})

    def disconnect_all_ssh(self) -> list[int]:
        processes = self._read_process_table()
        session_pids = self._select_ssh_session_pids(processes)
        if not session_pids:
            return []

        terminated = []
        for pid in sorted(session_pids, reverse=True):
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                continue
            terminated.append(pid)
        return terminated

    def _connection_exists(self, name: str) -> bool:
        result = self._nmcli("connection", "show", name, check=False)
        return result.returncode == 0

    def _detect_wifi_device(self) -> str:
        result = self._nmcli("device", "status")
        for line in result.stdout.splitlines():
            if not line.strip() or line.startswith("DEVICE"):
                continue
            parts = line.split()
            if len(parts) >= 2 and parts[1] == "wifi":
                return parts[0]
        raise RaspberryCommandError("NetworkManager не обнаружил Wi‑Fi-устройство")

    def _active_wifi_connection_name(self, wifi_device: str) -> str | None:
        result = self._nmcli("-t", "-f", "NAME,DEVICE", "connection", "show", "--active")
        for line in result.stdout.splitlines():
            if not line.strip():
                continue
            name, _, device = line.partition(":")
            if device == wifi_device:
                return name
        return None

    def _read_process_table(self) -> list[ProcessInfo]:
        result = self._run_command(["ps", "-eo", "pid=,ppid=,comm=,args="])
        processes: list[ProcessInfo] = []
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            fields = stripped.split(None, 3)
            if len(fields) < 4:
                continue
            pid_text, ppid_text, command, args = fields
            processes.append(
                ProcessInfo(
                    pid=int(pid_text),
                    ppid=int(ppid_text),
                    command=command,
                    args=args,
                )
            )
        return processes

    @staticmethod
    def _select_ssh_session_pids(processes: Iterable[ProcessInfo]) -> set[int]:
        processes = list(processes)
        children: dict[int, list[int]] = {}
        process_map = {process.pid: process for process in processes}

        for process in processes:
            children.setdefault(process.ppid, []).append(process.pid)

        master_pids = {
            process.pid
            for process in processes
            if process.command == "sshd"
            and ("-D" in process.args or "[listener]" in process.args or "(sshd)" in process.args)
        }

        session_roots = {
            process.pid
            for process in processes
            if process.command == "sshd" and process.pid not in master_pids
        }

        result: set[int] = set()
        stack = list(session_roots)
        while stack:
            pid = stack.pop()
            process = process_map.get(pid)
            if process is None or pid in master_pids:
                continue
            if pid not in result:
                result.add(pid)
                stack.extend(children.get(pid, []))

        return result

    def _nmcli(self, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        return self._run_command(["nmcli", *args], check=check)

    def _run_command(
        self,
        command: list[str],
        *,
        check: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        try:
            result = self._runner(command, capture_output=True, text=True, check=False)
        except FileNotFoundError as exc:
            raise RaspberryCommandError(f"Команда недоступна: {command[0]}") from exc

        if check and result.returncode != 0:
            stderr = (result.stderr or "").strip()
            raise RaspberryCommandError(
                f"Команда {' '.join(command)} завершилась с кодом {result.returncode}: {stderr}"
            )
        return result

    def _save_state(self, state: dict[str, str]) -> None:
        self._state_file.parent.mkdir(parents=True, exist_ok=True)
        self._state_file.write_text(json.dumps(state), encoding="utf-8")

    def _load_state(self) -> dict[str, str]:
        if not self._state_file.exists():
            return {}
        try:
            return json.loads(self._state_file.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            self._logger.warning("Файл состояния AP %s повреждён, он будет проигнорирован", self._state_file)
            return {}
