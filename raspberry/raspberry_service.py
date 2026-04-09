"""Сервисные команды Raspberry Pi для Wi‑Fi AP, SSH и телеметрии платы."""

from __future__ import annotations

import json
import logging
import os
import platform
import re
import signal
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Literal


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


@dataclass(slots=True)
class WifiAccessPointInfo:
    channel: int
    signal: int


class RaspberryService:
    """Высокоуровневый сервис управления системными функциями Raspberry Pi.

    Класс предназначен для операций, которые не выполняются Arduino:
    - настройка и переключение Raspberry Pi в AP-режим через NetworkManager;
    - восстановление клиентского Wi-Fi-профиля после выхода из AP;
    - завершение активных SSH-сессий без остановки master-процесса ``sshd``;
    - чтение штатной телеметрии платы: температуры и признаков проблем с
      питанием.
    """

    def __init__(
        self,
        *,
        logger: logging.Logger | None = None,
        runner: Callable[..., subprocess.CompletedProcess[str]] | None = None,
        state_file: str | Path | None = None,
        ap_profile_name: str = "rescue-maze-ap",
        raspberry_pi_detector: Callable[[], bool] | None = None,
        euid_getter: Callable[[], int] | None = None,
    ) -> None:
        """Создаёт сервис управления системными командами Raspberry Pi.

        Args:
            logger: Пользовательский logger для служебных сообщений.
            runner: Функция запуска внешних команд. По умолчанию используется
                ``subprocess.run``.
            state_file: Путь к файлу, в котором сохраняется предыдущий
                клиентский Wi‑Fi-профиль для последующего восстановления.
            ap_profile_name: Имя профиля NetworkManager, используемого для
                режима точки доступа.
            raspberry_pi_detector: Необязательная функция определения того,
                что код выполняется именно на Raspberry Pi. Нужна в основном
                для тестов и подмены окружения.
            euid_getter: Необязательная функция получения эффективного UID.
                Нужна в основном для тестов.
        """
        self._logger = logger or LOGGER
        self._runner = runner or subprocess.run
        self._state_file = Path(state_file or "/tmp/rescue_maze_ap_state.json")
        self._ap_profile_name = ap_profile_name
        self._raspberry_pi_detector = raspberry_pi_detector or self._default_raspberry_pi_detector
        self._euid_getter = euid_getter or getattr(os, "geteuid", None)

    def configure_ap(
        self,
        ssid: str,
        password: str,
        *,
        channel: int | Literal["auto"] = 1,
        ipv4_cidr: str = "192.168.4.1/24",
    ) -> None:
        """Создаёт или обновляет профиль точки доступа в NetworkManager.

        Args:
            ssid: Имя Wi‑Fi сети, которую будет раздавать Raspberry Pi.
            password: Пароль WPA-PSK для точки доступа.
            channel: Радиоканал Wi‑Fi. Можно передать число или строку
                ``"auto"``, чтобы выбрать канал автоматически на основе
                загруженности эфира и поддерживаемых адаптером частот.
            ipv4_cidr: Адрес и маска сети для интерфейса AP в формате CIDR.

        Raises:
            ValueError: Если входные параметры заведомо некорректны.
            RaspberryCommandError: Если ``nmcli`` недоступна или вернула
                ошибку.
        """
        self._ensure_elevated_privileges()
        if not ssid:
            raise ValueError("ssid не должен быть пустым")
        if len(password) < 8:
            raise ValueError("password должен содержать минимум 8 символов")
        if channel != "auto" and not 1 <= channel <= 13:
            raise ValueError("channel должен быть в диапазоне от 1 до 13")

        selected_channel = channel
        if channel == "auto":
            selected_channel = self.select_ap_channel()

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
            str(selected_channel),
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

    def select_ap_channel(self, wifi_device: str | None = None) -> int:
        """Автоматически выбирает наименее загруженный канал для AP.

        Алгоритм рассчитан на режим точки доступа в диапазоне 2.4 ГГц и
        опирается на две группы данных:
        - какие каналы реально поддерживает Wi‑Fi адаптер Raspberry Pi;
        - какие соседние точки доступа сейчас видны в эфире.

        Приоритетно рассматриваются непересекающиеся каналы ``1``, ``6`` и
        ``11``. Если адаптер или регуляторный домен не позволяют использовать
        эти каналы, сервис аккуратно переходит к поддерживаемым альтернативам.

        Args:
            wifi_device: Имя Wi‑Fi интерфейса. Если не указано, определяется
                автоматически.

        Returns:
            Номер канала, который стоит использовать для точки доступа.

        Raises:
            RaspberryCommandError: Если не удалось определить Wi‑Fi интерфейс,
                получить список поддерживаемых каналов или просканировать эфир.
        """
        self._ensure_elevated_privileges()
        wifi_device = wifi_device or self._detect_wifi_device()
        supported_channels = self._get_supported_24ghz_channels(wifi_device)
        if not supported_channels:
            raise RaspberryCommandError(
                f"Не удалось определить поддерживаемые 2.4 ГГц каналы для интерфейса {wifi_device}."
            )

        access_points = self._scan_wifi_access_points(wifi_device)
        preferred_channels = [channel for channel in (1, 6, 11) if channel in supported_channels]
        candidate_channels = preferred_channels or supported_channels

        best_channel = min(
            candidate_channels,
            key=lambda candidate: (
                self._channel_interference_score(candidate, access_points),
                0 if candidate in preferred_channels else 1,
                candidate,
            ),
        )
        self._logger.info(
            "Автоматически выбран Wi‑Fi канал %s для интерфейса %s",
            best_channel,
            wifi_device,
        )
        return best_channel

    def enable_ap(self) -> None:
        """Активирует AP-профиль на Wi-Fi интерфейсе Raspberry Pi.

        Метод пытается:
        1. определить Wi-Fi-интерфейс;
        2. запомнить активный клиентский профиль;
        3. отключить конфликтующие клиентские подключения;
        4. поднять профиль точки доступа.

        Raises:
            RaspberryCommandError: Если Wi‑Fi устройство не найдено или команды
                ``nmcli`` завершились ошибкой.
        """
        self._ensure_elevated_privileges()
        wifi_device = self._detect_wifi_device()
        previous_client = self._active_wifi_connection_name(wifi_device)
        if previous_client and previous_client != self._ap_profile_name:
            self._save_state({"previous_client": previous_client})
            self._nmcli("connection", "down", previous_client, check=False)

        self._nmcli("device", "disconnect", wifi_device, check=False)
        self._nmcli("connection", "up", self._ap_profile_name, "ifname", wifi_device)

    def disable_ap(self) -> None:
        """Отключает AP-профиль и пытается восстановить прошлый Wi-Fi-клиент.

        Если ранее был сохранён активный клиентский профиль, сервис поднимет
        его на том же Wi-Fi интерфейсе после отключения точки доступа.

        Raises:
            RaspberryCommandError: Если ``nmcli`` недоступна или вернула ошибку.
        """
        self._ensure_elevated_privileges()
        state = self._load_state()
        previous_client = state.get("previous_client")
        wifi_device = self._detect_wifi_device()

        self._nmcli("connection", "down", self._ap_profile_name, check=False)

        if previous_client:
            self._nmcli("connection", "up", previous_client, "ifname", wifi_device)
            self._save_state({})

    def disconnect_all_ssh(self) -> list[int]:
        """Завершает все активные SSH-сессии, не трогая master ``sshd``.

        Метод анализирует дерево процессов, находит дочерние процессы
        интерактивных или рабочих SSH-сессий и отправляет им ``SIGTERM`` в
        обратном порядке, начиная с самых глубоких потомков.

        Returns:
            Список PID, которым был отправлен сигнал завершения.
        """
        self._ensure_elevated_privileges()
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

    def get_temperature_telemetry(self) -> dict[str, Any]:
        """Возвращает текущую температуру Raspberry Pi и простую оценку нагрева.

        Метод сначала пытается прочитать стандартный Linux sysfs-файл
        ``/sys/class/thermal/thermal_zone0/temp``. Если он недоступен, сервис
        делает резервную попытку через ``vcgencmd measure_temp``.

        Returns:
            Словарь со следующими полями:
            - ``celsius``: температура в градусах Цельсия;
            - ``fahrenheit``: та же температура в градусах Фаренгейта;
            - ``state``: приблизительная словесная оценка нагрева
              ``normal``/``warm``/``hot``/``critical``;
            - ``source``: откуда были получены данные.

        Raises:
            RaspberryCommandError: Если ни один источник температурной
                телеметрии не доступен или вернул повреждённые данные.
        """
        thermal_zone_path = Path("/sys/class/thermal/thermal_zone0/temp")
        temperature_celsius: float | None = None
        source: str | None = None

        if self._path_exists(thermal_zone_path):
            try:
                temperature_celsius = self._parse_thermal_zone_temperature(self._read_text_file(thermal_zone_path))
                source = str(thermal_zone_path)
            except (OSError, ValueError) as exc:
                self._logger.debug(
                    "Не удалось прочитать температуру Raspberry Pi из %s: %s",
                    thermal_zone_path,
                    exc,
                )

        if temperature_celsius is None:
            result = self._run_command(["vcgencmd", "measure_temp"], check=False)
            if result.returncode != 0 or not result.stdout.strip():
                stderr = (result.stderr or "").strip()
                raise RaspberryCommandError(
                    "Не удалось получить температуру Raspberry Pi через sysfs или vcgencmd: "
                    f"{stderr or 'команда не вернула данных'}"
                )
            try:
                temperature_celsius = self._parse_vcgencmd_temperature(result.stdout)
            except ValueError as exc:
                raise RaspberryCommandError(
                    f"Не удалось разобрать температуру из vcgencmd: {result.stdout.strip()}"
                ) from exc
            source = "vcgencmd measure_temp"

        return {
            "celsius": round(temperature_celsius, 2),
            "fahrenheit": round((temperature_celsius * 9.0 / 5.0) + 32.0, 2),
            "state": self._temperature_state(temperature_celsius),
            "source": source,
        }

    def get_power_telemetry(self) -> dict[str, Any]:
        """Возвращает штатную телеметрию питания Raspberry Pi.

        Важно понимать, что это не измерение аккумулятора, тока или внешнего
        блока питания. Сервис читает только встроенные признаки состояния самой
        платы Raspberry Pi: флаги undervoltage, throttling, ограничения частоты
        и, если доступно, внутреннее напряжение ядра через ``vcgencmd``.

        Returns:
            Словарь с полями:
            - ``throttled_raw``: исходная строка из ``vcgencmd get_throttled``;
            - ``throttled_mask``: целочисленная битовая маска;
            - ``core_voltage_volts``: напряжение ядра в вольтах или ``None``;
            - ``voltage_source``: источник измерения напряжения или ``None``;
            - ``undervoltage_now`` и ``undervoltage_occurred``;
            - ``frequency_capped_now`` и ``frequency_capped_occurred``;
            - ``throttled_now`` и ``throttling_occurred``;
            - ``soft_temperature_limit_now`` и
              ``soft_temperature_limit_occurred``;
            - ``power_good_now``: нет ли сейчас признака просадки питания;
            - ``performance_limited_now``: ограничена ли сейчас
              производительность.

        Raises:
            RaspberryCommandError: Если ``vcgencmd get_throttled`` недоступна
                или вернула неожиданный ответ.
        """
        throttled_result = self._run_command(["vcgencmd", "get_throttled"], check=False)
        if throttled_result.returncode != 0 or not throttled_result.stdout.strip():
            stderr = (throttled_result.stderr or "").strip()
            raise RaspberryCommandError(
                "Не удалось получить телеметрию питания Raspberry Pi через vcgencmd get_throttled: "
                f"{stderr or 'команда не вернула данных'}"
            )

        try:
            throttled_raw, throttled_mask = self._parse_vcgencmd_throttled(throttled_result.stdout)
        except ValueError as exc:
            raise RaspberryCommandError(
                f"Не удалось разобрать ответ vcgencmd get_throttled: {throttled_result.stdout.strip()}"
            ) from exc

        core_voltage_volts: float | None = None
        voltage_source: str | None = None
        voltage_result = self._run_command(["vcgencmd", "measure_volts", "core"], check=False)
        if voltage_result.returncode == 0 and voltage_result.stdout.strip():
            try:
                core_voltage_volts = self._parse_vcgencmd_voltage(voltage_result.stdout)
                voltage_source = "vcgencmd measure_volts core"
            except ValueError:
                self._logger.debug(
                    "Не удалось разобрать напряжение ядра из vcgencmd: %s",
                    voltage_result.stdout.strip(),
                )

        undervoltage_now = bool(throttled_mask & 0x1)
        frequency_capped_now = bool(throttled_mask & 0x2)
        throttled_now = bool(throttled_mask & 0x4)
        soft_temperature_limit_now = bool(throttled_mask & 0x8)

        return {
            "throttled_raw": throttled_raw,
            "throttled_mask": throttled_mask,
            "core_voltage_volts": core_voltage_volts,
            "voltage_source": voltage_source,
            "undervoltage_now": undervoltage_now,
            "undervoltage_occurred": bool(throttled_mask & 0x10000),
            "frequency_capped_now": frequency_capped_now,
            "frequency_capped_occurred": bool(throttled_mask & 0x20000),
            "throttled_now": throttled_now,
            "throttling_occurred": bool(throttled_mask & 0x40000),
            "soft_temperature_limit_now": soft_temperature_limit_now,
            "soft_temperature_limit_occurred": bool(throttled_mask & 0x80000),
            "power_good_now": not undervoltage_now,
            "performance_limited_now": frequency_capped_now or throttled_now or soft_temperature_limit_now,
        }

    def get_board_telemetry(self) -> dict[str, Any]:
        """Возвращает сводную телеметрию платы Raspberry Pi.

        Это удобный метод верхнего уровня, который одним вызовом собирает
        температуру и признаки проблем с питанием.

        Returns:
            Словарь с двумя ключами:
            - ``temperature``: результат ``get_temperature_telemetry()``;
            - ``power``: результат ``get_power_telemetry()``.
        """
        return {
            "temperature": self.get_temperature_telemetry(),
            "power": self.get_power_telemetry(),
        }

    def _connection_exists(self, name: str) -> bool:
        result = self._nmcli("connection", "show", name, check=False)
        return result.returncode == 0

    def _ensure_elevated_privileges(self) -> None:
        """Требует root-права только при работе на реальной Raspberry Pi.

        На обычных машинах разработчика, CI и других не-RPi окружениях
        проверка не мешает использованию класса. На Raspberry Pi без
        повышенных прав будет выброшено исключение до запуска системных
        команд.
        """
        if not self._raspberry_pi_detector():
            return

        if self._euid_getter is None:
            raise RaspberryCommandError(
                "На Raspberry Pi не удалось проверить повышенные права: функция geteuid недоступна."
            )

        if self._euid_getter() != 0:
            raise RaspberryCommandError(
                "Для методов RaspberryService на Raspberry Pi нужны повышенные права. "
                "Запустите код от root или через sudo."
            )

    @staticmethod
    def _default_raspberry_pi_detector() -> bool:
        """Определяет, выполняется ли код на реальной Raspberry Pi."""
        if platform.system() != "Linux":
            return False

        model_paths = (
            Path("/proc/device-tree/model"),
            Path("/sys/firmware/devicetree/base/model"),
        )
        for model_path in model_paths:
            try:
                model = model_path.read_text(encoding="utf-8", errors="ignore").strip("\x00\r\n ")
            except OSError:
                continue
            if "raspberry pi" in model.lower():
                return True
        return False

    def _detect_wifi_device(self) -> str:
        result = self._nmcli("device", "status")
        for line in result.stdout.splitlines():
            if not line.strip() or line.startswith("DEVICE"):
                continue
            parts = line.split()
            if len(parts) >= 2 and parts[1] == "wifi":
                return parts[0]
        raise RaspberryCommandError("NetworkManager не обнаружил Wi-Fi-устройство")

    def _active_wifi_connection_name(self, wifi_device: str) -> str | None:
        result = self._nmcli("-t", "-f", "NAME,DEVICE", "connection", "show", "--active")
        for line in result.stdout.splitlines():
            if not line.strip():
                continue
            name, _, device = line.partition(":")
            if device == wifi_device:
                return name
        return None

    def _get_supported_24ghz_channels(self, wifi_device: str) -> list[int]:
        channels = self._parse_iw_phy_channels(self._run_command(["iw", "phy"], check=False).stdout)
        if channels:
            return channels

        iwlist_result = self._run_command(["iwlist", wifi_device, "frequency"], check=False)
        channels = self._parse_iwlist_channels(iwlist_result.stdout)
        if channels:
            return channels

        return []

    @staticmethod
    def _parse_iw_phy_channels(output: str) -> list[int]:
        channels: set[int] = set()
        for raw_line in output.splitlines():
            line = raw_line.strip()
            if "MHz" not in line or "disabled" in line:
                continue
            match = re.search(r"\[(\d+)\]", line)
            if match is None:
                continue
            channel = int(match.group(1))
            if 1 <= channel <= 13:
                channels.add(channel)
        return sorted(channels)

    @staticmethod
    def _parse_iwlist_channels(output: str) -> list[int]:
        channels: set[int] = set()
        for raw_line in output.splitlines():
            match = re.search(r"Channel\s+(\d+)", raw_line, re.IGNORECASE)
            if match is None:
                continue
            channel = int(match.group(1))
            if 1 <= channel <= 13:
                channels.add(channel)
        return sorted(channels)

    def _scan_wifi_access_points(self, wifi_device: str) -> list[WifiAccessPointInfo]:
        result = self._nmcli(
            "-t",
            "-f",
            "CHAN,SIGNAL",
            "device",
            "wifi",
            "list",
            "--rescan",
            "yes",
            "ifname",
            wifi_device,
        )
        return self._parse_wifi_scan_output(result.stdout)

    @staticmethod
    def _parse_wifi_scan_output(output: str) -> list[WifiAccessPointInfo]:
        access_points: list[WifiAccessPointInfo] = []
        for raw_line in output.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split(":")
            if len(parts) < 2:
                continue
            channel_text = parts[0].strip()
            signal_text = parts[-1].strip()
            if not channel_text.isdigit() or not signal_text.isdigit():
                continue
            channel = int(channel_text)
            signal = int(signal_text)
            if 1 <= channel <= 13:
                access_points.append(WifiAccessPointInfo(channel=channel, signal=signal))
        return access_points

    @staticmethod
    def _channel_interference_score(channel: int, access_points: Iterable[WifiAccessPointInfo]) -> float:
        weights = {
            0: 1.0,
            1: 2.5,
            2: 2.0,
            3: 1.5,
            4: 1.2,
        }
        score = 0.0
        for access_point in access_points:
            distance = abs(access_point.channel - channel)
            weight = weights.get(distance)
            if weight is None:
                continue
            score += access_point.signal * weight
        return score

    @staticmethod
    def _parse_thermal_zone_temperature(raw_text: str) -> float:
        match = re.search(r"(-?\d+)", raw_text)
        if match is None:
            raise ValueError("Не найдено целочисленное значение температуры")
        return int(match.group(1)) / 1000.0

    @staticmethod
    def _parse_vcgencmd_temperature(output: str) -> float:
        match = re.search(r"temp=([0-9]+(?:\.[0-9]+)?)'C", output)
        if match is None:
            raise ValueError("Не найдено значение температуры vcgencmd")
        return float(match.group(1))

    @staticmethod
    def _parse_vcgencmd_voltage(output: str) -> float:
        match = re.search(r"volt=([0-9]+(?:\.[0-9]+)?)V", output)
        if match is None:
            raise ValueError("Не найдено значение напряжения vcgencmd")
        return float(match.group(1))

    @staticmethod
    def _parse_vcgencmd_throttled(output: str) -> tuple[str, int]:
        stripped = output.strip()
        match = re.search(r"throttled=(0x[0-9a-fA-F]+)", stripped)
        if match is None:
            raise ValueError("Не найдена throttled-маска")
        raw_mask = match.group(1)
        return raw_mask, int(raw_mask, 16)

    @staticmethod
    def _temperature_state(temperature_celsius: float) -> str:
        if temperature_celsius < 60.0:
            return "normal"
        if temperature_celsius < 75.0:
            return "warm"
        if temperature_celsius < 80.0:
            return "hot"
        return "critical"

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

    @staticmethod
    def _read_text_file(path: Path) -> str:
        return path.read_text(encoding="utf-8", errors="ignore")

    @staticmethod
    def _path_exists(path: Path) -> bool:
        return path.exists()
