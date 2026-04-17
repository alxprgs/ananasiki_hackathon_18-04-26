"""
left_hand_rule.py
=================
Алгоритм «правило левой руки» для RoboCupJunior Rescue Maze.

Конфигурация датчиков (2 датчика, оба HC-SR04, оба смотрят ВЛЕВО):
    Слот 1 (HC-SR04) — левый передний
    Слот 2 (HC-SR04) — левый задний

Принцип правила левой руки:
    На каждом шаге проверяем приоритеты по порядку:
      1. Слева открыто?  → повернуть налево, ехать вперёд
      2. Спереди открыто? → ехать вперёд (продолжать вдоль левой стены)
      3. Иначе           → повернуть направо
         (если после поворота направо снова всё закрыто — следующий шаг
          опять повернёт направо, итого 180° = выход из тупика)

Обнаружение передней стены:
    Датчики смотрят ВЛЕВО и напрямую не видят переднюю стену.
    Используется «пробный заезд»:
      — робот чуть проезжает вперёд на PROBE_SEC секунд;
      — если левый передний датчик резко уменьшился (начал «видеть» угол
        между левой и фронтальной стенами) → спереди стена;
      — робот откатывается назад на то же расстояние.

Запуск и остановка:
    — Нажмите кнопку на Arduino, чтобы стартовать.
    — Нажмите кнопку повторно, чтобы остановить.
    — Робот также остановится при достижении MAX_STEPS.

Калибровка:
    Начните с дефолтных значений и откорректируйте:
      — CELL_SEC     : как долго ехать одну клетку (30 см);
      — TURN_90_SEC  : как долго крутиться для поворота на 90°;
      — WALL_THRESHOLD_MM : дистанция, при которой считаем что стена есть.
    Запустите с include_map=True, посмотрите route.svg — это поможет понять,
    насколько реальные движения совпадают с ожидаемыми.
"""

import time
import logging

from raspberry.arduino_service import (
    ArduinoService,
    ArduinoProtocolError,
    ArduinoUnavailableError,
    MotionMapCalibration,
    SerialTimeoutError,
)

# ──────────────────────────────────────────────────────────────────────────────
# НАСТРОЙКИ — меняйте под своего робота
# ──────────────────────────────────────────────────────────────────────────────

# Номера слотов датчиков
SENSOR_FRONT_LEFT: int = 1   # левый передний (HC-SR04, слот 1)
SENSOR_REAR_LEFT:  int = 2   # левый задний   (HC-SR04, слот 2)

# Пороги расстояний (мм)
WALL_THRESHOLD_MM: float = 200.0   # ближе → стена есть
OPEN_MIN_MM:       float = 280.0   # дальше → проход точно открыт

# Порог обнаружения передней стены.
# После пробного заезда: если левый передний датчик уменьшился
# более чем на FRONT_DROP_MM — спереди стена.
FRONT_DROP_MM: float = 70.0

# Мощность двигателей (%)
DRIVE_POWER: int = 65   # прямолинейное движение
TURN_POWER:  int = 52   # вращение на месте
PROBE_POWER: int = 40   # медленный пробный заезд

# Времена движения (секунды) — калибруйте!
CELL_SEC:    float = 0.85   # одна клетка вперёд (~30 см)
TURN_90_SEC: float = 0.55   # поворот на 90°
PROBE_SEC:   float = 0.20   # пробный заезд для обнаружения стены спереди
SETTLE_SEC:  float = 0.12   # пауза после остановки (датчики стабилизируются)

# Параметры выравнивания по левой стене
ALIGN_TOLERANCE_MM: float = 12.0
ALIGN_TURN_POWER:   int   = 18
ALIGN_PULSE_SEC:    float = 0.10
ALIGN_MAX_ITER:     int   = 8

# Калибровка карты маршрута (для route.svg, подберите под реальный робот)
MAP_LINEAR_SPEED:  float = 250.0   # мм/с при 100% мощности
MAP_TURN_SPEED:    float = 140.0   # °/с при полной разнице каналов

# Максимальное число шагов (аварийный выход из цикла)
MAX_STEPS: int = 500

# Serial-порт (None → автоопределение)
SERIAL_PORT = None   # например: "/dev/ttyACM0"

# ──────────────────────────────────────────────────────────────────────────────
# ЛОГИРОВАНИЕ
# ──────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("left_hand_rule")


# ──────────────────────────────────────────────────────────────────────────────
# НИЗКОУРОВНЕВЫЕ ДВИЖЕНИЯ
# ──────────────────────────────────────────────────────────────────────────────

def move_forward(arduino: ArduinoService, seconds: float = CELL_SEC) -> None:
    """Движение вперёд на заданное время с паузой после остановки."""
    arduino.eng_all.pwm(DRIVE_POWER).time(seconds)
    time.sleep(SETTLE_SEC)


def move_backward(arduino: ArduinoService, seconds: float = PROBE_SEC) -> None:
    """Движение назад на заданное время (компенсация пробного заезда)."""
    arduino.eng_all.pwm(-PROBE_POWER).time(seconds)
    time.sleep(SETTLE_SEC)


def turn_left_90(arduino: ArduinoService) -> None:
    """
    Поворот на месте влево на 90°.
    Левая гусеница назад, правая вперёд → разворот влево.
    """
    log.info("    ↰  ПОВОРОТ НАЛЕВО 90°")
    arduino.eng_l.pwm(-TURN_POWER).now()
    arduino.eng_r.pwm(TURN_POWER).now()
    time.sleep(TURN_90_SEC)
    arduino.stop_all()
    time.sleep(SETTLE_SEC)


def turn_right_90(arduino: ArduinoService) -> None:
    """
    Поворот на месте вправо на 90°.
    Левая гусеница вперёд, правая назад → разворот вправо.
    """
    log.info("    ↱  ПОВОРОТ НАПРАВО 90°")
    arduino.eng_l.pwm(TURN_POWER).now()
    arduino.eng_r.pwm(-TURN_POWER).now()
    time.sleep(TURN_90_SEC)
    arduino.stop_all()
    time.sleep(SETTLE_SEC)


# ──────────────────────────────────────────────────────────────────────────────
# ЧТЕНИЕ ДАТЧИКОВ
# ──────────────────────────────────────────────────────────────────────────────

def read_mm(arduino: ArduinoService, sensor_id: int) -> float | None:
    """
    Читает расстояние с датчика в мм.
    Возвращает None при любой ошибке; код везде трактует None как «стена».
    """
    try:
        return arduino.distance_sensor.get(sensor_id, unit="mm")
    except Exception as exc:
        log.warning(f"    Датчик {sensor_id}: ошибка чтения — {exc}")
        return None


def _is_wall(dist_mm: float | None) -> bool:
    """True если расстояние меньше порога стены или датчик вернул ошибку."""
    if dist_mm is None:
        return True
    return dist_mm < WALL_THRESHOLD_MM


def _is_open(dist_mm: float | None) -> bool:
    """True если расстояние говорит о чётком открытом проходе."""
    if dist_mm is None:
        return False
    return dist_mm > OPEN_MIN_MM


# ──────────────────────────────────────────────────────────────────────────────
# ОПРЕДЕЛЕНИЕ СТЕН
# ──────────────────────────────────────────────────────────────────────────────

def is_left_wall(arduino: ArduinoService) -> bool:
    """
    True если слева есть стена.

    Логика: стена слева есть тогда, когда ХОТЯ БЫ ОДИН датчик
    показывает близкое расстояние. Это не позволяет роботу
    повернуть налево раньше, чем проход полностью откроется
    (оба датчика должны перестать видеть стену).
    """
    d_front = read_mm(arduino, SENSOR_FRONT_LEFT)
    d_rear  = read_mm(arduino, SENSOR_REAR_LEFT)
    wall = _is_wall(d_front) or _is_wall(d_rear)
    log.debug(
        f"    Слева: d_front={d_front} мм, d_rear={d_rear} мм "
        f"→ {'СТЕНА' if wall else 'ОТКРЫТО'}"
    )
    return wall


def is_front_wall(arduino: ArduinoService) -> bool:
    """
    Определяет, есть ли стена спереди через «пробный заезд».

    Алгоритм:
      1. Читаем d_before с левого переднего датчика.
      2. Едем вперёд на PROBE_SEC с малой мощностью.
      3. Читаем d_after.
      4. Если датчик резко уменьшился (падение > FRONT_DROP_MM) —
         спереди стена: передний-левый датчик начал видеть угол
         между левой и фронтальной стенами.
      5. Откатываемся назад на то же расстояние.

    Примечание: чем меньше PROBE_SEC, тем меньше смещение робота,
    но тем слабее сигнал. Подбирайте совместно с FRONT_DROP_MM.
    """
    d_before = read_mm(arduino, SENSOR_FRONT_LEFT)

    # Пробный заезд
    arduino.eng_all.pwm(PROBE_POWER).time(PROBE_SEC)
    time.sleep(SETTLE_SEC)

    d_after = read_mm(arduino, SENSOR_FRONT_LEFT)

    # Откат на исходную позицию
    move_backward(arduino, PROBE_SEC)

    # Нет данных → считаем стеной (безопаснее)
    if d_before is None or d_after is None:
        log.warning("    Спереди: датчик не ответил → считаем стеной")
        return True

    drop = d_before - d_after
    wall = drop > FRONT_DROP_MM
    log.debug(
        f"    Спереди: до={d_before:.0f} мм, после={d_after:.0f} мм, "
        f"падение={drop:.0f} мм → {'СТЕНА' if wall else 'ОТКРЫТО'}"
    )
    return wall


# ──────────────────────────────────────────────────────────────────────────────
# ВЫРАВНИВАНИЕ ПО ЛЕВОЙ СТЕНЕ
# ──────────────────────────────────────────────────────────────────────────────

def try_align_to_left_wall(arduino: ArduinoService) -> None:
    """
    Если слева есть стена — выравниваемся параллельно ей.
    Вызываем перед прямолинейным движением, чтобы уменьшить накопленный дрейф.
    Пропускаем выравнивание, если стены нет.
    """
    d_front = read_mm(arduino, SENSOR_FRONT_LEFT)
    d_rear  = read_mm(arduino, SENSOR_REAR_LEFT)

    # Выравниваться можно только если оба датчика видят стену
    if not (_is_wall(d_front) and _is_wall(d_rear)):
        return

    result = arduino.align_parallel_to_wall(
        front_sensor_id=SENSOR_FRONT_LEFT,
        rear_sensor_id=SENSOR_REAR_LEFT,
        wall_side="left",
        tolerance_mm=ALIGN_TOLERANCE_MM,
        turn_power=ALIGN_TURN_POWER,
        pulse_seconds=ALIGN_PULSE_SEC,
        settle_seconds=SETTLE_SEC,
        max_iterations=ALIGN_MAX_ITER,
    )
    if result["aligned"]:
        log.debug("    Выравнивание: ОК")
    else:
        log.debug(f"    Выравнивание: не уложился в допуск (delta={result['delta_mm']:.1f} мм)")


# ──────────────────────────────────────────────────────────────────────────────
# ОДИН ШАГ АЛГОРИТМА
# ──────────────────────────────────────────────────────────────────────────────

def maze_step(arduino: ArduinoService) -> str:
    """
    Выполняет один шаг правила левой руки.

    Дерево решений:
    ┌──────────────────────────────────────┐
    │  Стена слева?                        │
    │   НЕТ → повернуть налево, вперёд    │
    │   ДА  → Стена спереди?              │
    │          НЕТ → вперёд               │
    │          ДА  → повернуть направо    │
    └──────────────────────────────────────┘

    Про тупик: при тупике мы поворачиваем направо. На следующем шаге
    слева снова будет стена (была задняя стена тупика), спереди тоже
    стена → снова направо. Итого два поворота направо = разворот 180°.

    Возвращает строку с принятым решением.
    """
    left_blocked = is_left_wall(arduino)

    if not left_blocked:
        # ── Приоритет 1: слева открыто ──────────────────────────────────────
        log.info("  РЕШЕНИЕ: слева ОТКРЫТО → поворот налево + вперёд")
        turn_left_90(arduino)
        move_forward(arduino)
        return "turn_left"

    # Слева стена — проверяем спереди (через пробный заезд)
    front_blocked = is_front_wall(arduino)

    if not front_blocked:
        # ── Приоритет 2: прямо открыто ──────────────────────────────────────
        log.info("  РЕШЕНИЕ: прямо ОТКРЫТО → выравнивание + вперёд")
        try_align_to_left_wall(arduino)
        move_forward(arduino)
        return "forward"

    else:
        # ── Приоритет 3: слева и спереди заблокировано ──────────────────────
        # Поворачиваем направо. Если и справа тупик — на следующем шаге
        # снова повернём направо (итого 180°).
        log.info("  РЕШЕНИЕ: слева и спереди СТЕНА → поворот направо")
        turn_right_90(arduino)
        return "turn_right"


# ──────────────────────────────────────────────────────────────────────────────
# ГЛАВНЫЙ ЦИКЛ
# ──────────────────────────────────────────────────────────────────────────────

def run_maze(arduino: ArduinoService) -> None:
    """
    Основной цикл алгоритма.

    Старт: нажмите кнопку на Arduino.
    Стоп:  нажмите кнопку ещё раз (или Ctrl+C, или MAX_STEPS исчерпан).
    """
    log.info("Ожидание нажатия кнопки для старта...")
    while not arduino.button_status():
        time.sleep(0.05)

    log.info("▶  Старт! Нажмите кнопку ещё раз для остановки.")
    time.sleep(0.5)   # небольшая пауза перед первым движением

    for step in range(1, MAX_STEPS + 1):
        log.info(f"━━ Шаг {step:03d} ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

        # Повторное нажатие кнопки → остановка
        if arduino.button_status():
            log.info("■  Кнопка нажата — остановка по команде оператора.")
            break

        try:
            decision = maze_step(arduino)
            log.info(f"   Принято: {decision}")

        except SerialTimeoutError:
            log.error("Arduino не отвечает → аварийная остановка!")
            arduino.stop_all()
            break

        except ArduinoProtocolError as exc:
            # Ошибка протокола — логируем и пробуем продолжить
            log.error(f"Ошибка протокола: {exc} — пропускаем шаг")
            arduino.stop_all()
            time.sleep(0.3)
            continue

    else:
        log.info(f"■  Достигнут лимит {MAX_STEPS} шагов.")

    arduino.stop_all()
    log.info("■  Робот остановлен.")


# ──────────────────────────────────────────────────────────────────────────────
# ТОЧКА ВХОДА
# ──────────────────────────────────────────────────────────────────────────────

def main() -> None:
    try:
        with ArduinoService(port=SERIAL_PORT) as arduino:
            ping = arduino.ping()
            log.info(f"Подключено к Arduino: {ping}")

            # Запускаем запись сессии (route.svg + actions.txt + events.json)
            session_dir = arduino.start_activity_session(
                session_name="left_hand_rule",
                include_map=True,
                calibration=MotionMapCalibration(
                    max_linear_speed_mm_per_sec=MAP_LINEAR_SPEED,
                    max_turn_deg_per_sec=MAP_TURN_SPEED,
                ),
            )
            log.info(f"Логи сессии: {session_dir}")

            try:
                run_maze(arduino)
            finally:
                summary = arduino.stop_activity_session()
                log.info(f"Сессия сохранена:")
                log.info(f"  actions : {summary.get('actions_path')}")
                log.info(f"  events  : {summary.get('events_path')}")
                log.info(f"  route   : {summary.get('route_path')}")
                log.info(f"  финальная поза: {summary.get('final_pose')}")

    except ArduinoUnavailableError:
        log.critical("Arduino не найдена. Проверьте кабель и порт.")
    except KeyboardInterrupt:
        log.info("Прервано оператором (Ctrl+C).")


if __name__ == "__main__":
    main()
