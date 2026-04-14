#include <Arduino.h>
#include <Servo.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Низкоуровневая прошивка для Rescue Maze робота.
 *
 * Архитектура здесь двухуровневая:
 * - Raspberry Pi отвечает за "умную" часть, сценарии, высокоуровневую логику и
 *   отправку команд;
 * - Arduino занимается только железом и выполняет команды максимально быстро и
 *   предсказуемо.
 *
 * Как идёт обмен:
 * - Raspberry Pi отправляет одну JSON-команду в одну строку;
 * - Arduino читает строку до '\n', выполняет команду и возвращает один JSON-ответ;
 * - такой протокол удобно отлаживать и из Python, и вручную через Serial Monitor.
 *
 * Почему здесь почти нет блокирующих задержек:
 * - робот не должен "замирать" на чтении датчика или шаговика и переставать
 *   обрабатывать команды;
 * - поэтому датчики, timed-моторы и stepper обновляются малыми шагами в loop().
 *
 * Что проверить в первую очередь, если прошивка "не работает":
 * 1. Совпадает ли SERIAL_BAUD с настройкой на Raspberry Pi.
 * 2. Действительно ли каждая команда заканчивается переводом строки '\n'.
 * 3. Верна ли распиновка в namespace Config.
 * 4. Не перепутаны ли левый и правый моторы.
 * 5. Не нужно ли инвертировать направление одного из моторов.
 * 6. Есть ли общая земля между всеми модулями.
 * 7. Хватает ли питания моторам, серве, ультразвуку и stepper driver.
 */

namespace Config {
/*
 * Весь конфиг собран в одном месте специально, чтобы не искать "магические числа"
 * по всей прошивке. Если меняется шасси, Motor Shield или пины, почти всегда
 * достаточно править только этот блок.
 *
 * Ключевые флаги для отладки:
 * - SWAP_TRACK_MOTORS:
 *   если команды left/right физически попали на противоположные гусеницы.
 * - LEFT_TRACK_INVERTED / RIGHT_TRACK_INVERTED:
 *   если мотор при положительной команде едет не вперёд, а назад.
 * - SERVO_ENABLED / RELAY_ENABLED / STEPPER_ENABLED:
 *   позволяют быстро отключить опциональное оборудование, не ломая остальную
 *   прошивку.
 */
constexpr unsigned long SERIAL_BAUD = 115200UL;
constexpr unsigned long COMMAND_WATCHDOG_MS = 1500UL;
constexpr unsigned long SENSOR_SAMPLE_INTERVAL_MS = 75UL;
constexpr unsigned long SENSOR_STALE_MS = 500UL;
constexpr unsigned long SENSOR_TRIGGER_PULSE_US = 10UL;
constexpr unsigned long SENSOR_ECHO_TIMEOUT_US = 25000UL;
constexpr unsigned long STEPPER_PULSE_HIGH_US = 10UL;
constexpr long STEPPER_STEPS_PER_REV = 200L;
constexpr bool SWAP_TRACK_MOTORS = false;
constexpr bool LEFT_TRACK_INVERTED = false;
constexpr bool RIGHT_TRACK_INVERTED = true;
constexpr bool SERVO_ENABLED = false;
constexpr bool RELAY_ENABLED = false;
constexpr bool STEPPER_ENABLED = false;
constexpr bool STEPPER_ENABLE_ACTIVE_LOW = true;
constexpr uint8_t SENSOR1_TRIG_PIN = 2;
constexpr uint8_t SENSOR1_ECHO_PIN = 3;
constexpr uint8_t LEFT_DIR_PIN = 4;
constexpr uint8_t LEFT_PWM_PIN = 5;
constexpr uint8_t RIGHT_PWM_PIN = 6;
constexpr uint8_t RIGHT_DIR_PIN = 7;
constexpr uint8_t SENSOR2_TRIG_PIN = 8;
constexpr uint8_t SENSOR2_ECHO_PIN = 9;
constexpr uint8_t SERVO_PIN = 10;
constexpr uint8_t RELAY_PIN = 11;
constexpr uint8_t STEPPER_STEP_PIN = 12;
constexpr uint8_t STEPPER_DIR_PIN = 13;
constexpr uint8_t STEPPER_ENABLE_PIN = A0;
constexpr uint8_t BUTTON_PIN = A1;
}  // namespace Config

/*
 * Этапы опроса ультразвукового датчика.
 *
 * Мы не используем pulseIn(), потому что это блокирующая функция. Вместо этого
 * датчик проходит через небольшой конечный автомат:
 * - подняли trig;
 * - дождались нужной длины импульса;
 * - ждём начала echo;
 * - ждём завершения echo.
 *
 * Благодаря этому прошивка остаётся отзывчивой и продолжает принимать команды.
 */
enum SensorStage : uint8_t {
  SENSOR_IDLE = 0,
  SENSOR_TRIGGER_HIGH = 1,
  SENSOR_WAIT_ECHO_HIGH = 2,
  SENSOR_WAIT_ECHO_LOW = 3
};

struct MotorPins {
  uint8_t dirPin;
  uint8_t pwmPin;
  bool invertDirection;
};

/*
 * Состояние одного ультразвукового датчика.
 *
 * Здесь хранятся:
 * - аппаратные пины;
 * - текущая стадия измерения;
 * - флаг валидности последнего измерения;
 * - последнее расстояние;
 * - временные метки для неблокирующего автомата.
 *
 * Это позволяет не просто "замерить расстояние", а понимать, насколько свежие
 * данные сейчас лежат в памяти и можно ли их отдавать наружу.
 */
struct DistanceSensorState {
  uint8_t trigPin;
  uint8_t echoPin;
  SensorStage stage;
  bool valid;
  long lastDistanceMm;
  unsigned long stageStartedUs;
  unsigned long echoStartedUs;
  unsigned long nextTriggerAtMs;
  unsigned long lastSuccessAtMs;
};

// Текущее состояние одного DC-мотора и опциональный дедлайн timed-команды.
struct MotorRuntimeState {
  int currentPercent;
  bool timed;
  unsigned long stopAtMs;
  bool rampActive;
  int rampStartPercent;
  int rampStopPercent;
  unsigned long rampStartedAtMs;
  unsigned long rampDurationMs;
};

/*
 * Состояние stepper driver.
 *
 * Здесь мы отслеживаем не только факт "крутится / не крутится", но и:
 * - ограничение по шагам;
 * - ограничение по времени;
 * - состояние STEP-пина;
 * - интервал между шагами.
 */
struct StepperRuntimeState {
  bool running;
  bool stepPinHigh;
  bool hasDeadline;
  bool hasStepLimit;
  long stepsRemaining;
  unsigned long stopAtMs;
  unsigned long lastPulseUs;
  unsigned long stepIntervalUs;
};

constexpr size_t REQUEST_BUFFER_SIZE = 220;
char gRequestBuffer[REQUEST_BUFFER_SIZE];
size_t gRequestLength = 0;
bool gRequestOverflow = false;

// Два датчика живут в массиве, так как их логика полностью одинакова.
DistanceSensorState gSensors[2] = {
    {Config::SENSOR1_TRIG_PIN, Config::SENSOR1_ECHO_PIN, SENSOR_IDLE, false, 0, 0, 0, 0, 0},
    {Config::SENSOR2_TRIG_PIN, Config::SENSOR2_ECHO_PIN, SENSOR_IDLE, false, 0, 0, 0, 0, 0},
};
int gActiveSensorIndex = -1;
int gNextSensorIndex = 0;

MotorRuntimeState gLeftMotor = {0, false, 0, false, 0, 0, 0, 0};
MotorRuntimeState gRightMotor = {0, false, 0, false, 0, 0, 0, 0};

StepperRuntimeState gStepper = {false, false, false, false, 0, 0, 0, 0};

Servo gServo;
bool gServoAttached = false;
int gServoAngle = 90;
bool gRelayEnabled = false;
unsigned long gLastValidCommandAtMs = 0;
bool gWatchdogTriggered = false;

const MotorPins LEFT_MOTOR_PINS = {Config::LEFT_DIR_PIN, Config::LEFT_PWM_PIN, Config::LEFT_TRACK_INVERTED};
const MotorPins RIGHT_MOTOR_PINS = {Config::RIGHT_DIR_PIN, Config::RIGHT_PWM_PIN, Config::RIGHT_TRACK_INVERTED};

// Позволяет одной настройкой поменять местами левый и правый моторный канал.
const MotorPins& leftMotorPins() {
  return Config::SWAP_TRACK_MOTORS ? RIGHT_MOTOR_PINS : LEFT_MOTOR_PINS;
}

const MotorPins& rightMotorPins() {
  return Config::SWAP_TRACK_MOTORS ? LEFT_MOTOR_PINS : RIGHT_MOTOR_PINS;
}

// Безопасная проверка дедлайна, устойчивая к переполнению millis()/micros().
bool hasElapsed(unsigned long nowValue, unsigned long deadlineValue) {
  return static_cast<long>(nowValue - deadlineValue) >= 0;
}

// Все команды мощности насильно ограничиваем диапазоном -100..100 процентов.
int clampPercent(long value) {
  if (value > 100L) {
    return 100;
  }
  if (value < -100L) {
    return -100;
  }
  return static_cast<int>(value);
}

// Переводим "проценты" в реальный PWM 0..255, который понимает analogWrite().
unsigned long pwmToDutyCycle(int percent) {
  unsigned long absolutePercent = static_cast<unsigned long>(abs(percent));
  return (absolutePercent * 255UL) / 100UL;
}

// Маленькая утилита для сериализации bool в JSON без внешних библиотек.
void printBool(bool value) {
  Serial.print(value ? F("true") : F("false"));
}

/*
 * Формирование ответа об ошибке.
 *
 * Поля code и message нужны не только для человека, но и для Raspberry Pi:
 * python-сервис может различать unsupported, bad_request и другие ситуации.
 */
void sendErrorResponse(long requestId, const char* code, const char* message) {
  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.print(F(",\"ok\":false,\"error\":{\"code\":\""));
  Serial.print(code);
  Serial.print(F("\",\"message\":\""));
  Serial.print(message);
  Serial.println(F("\"}}"));
}

// Упрощённый успешный ответ для команд, которым не нужно возвращать данные.
void sendEmptyOkResponse(long requestId) {
  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.println(F(",\"ok\":true,\"data\":{}}"));
}

/*
 * Ниже идёт очень лёгкий "ручной" разбор JSON.
 *
 * Важно понимать: это не универсальный парсер JSON, а минималистичный набор
 * функций под строго контролируемый формат сообщений от Raspberry Pi.
 * Для AVR это выгодно по памяти и по предсказуемости поведения.
 */
bool findFieldValue(const char* payload, const char* key, const char*& valueStart) {
  char pattern[32];
  int length = snprintf(pattern, sizeof(pattern), "\"%s\":", key);
  if (length <= 0 || length >= static_cast<int>(sizeof(pattern))) {
    return false;
  }
  const char* found = strstr(payload, pattern);
  if (found == nullptr) {
    return false;
  }
  valueStart = found + length;
  while (*valueStart == ' ') {
    ++valueStart;
  }
  return true;
}

// Вытаскивает "сырой" токен числа, чтобы потом преобразовать его в long или double.
bool extractNumberToken(const char* payload, const char* key, char* out, size_t outSize) {
  const char* start = nullptr;
  if (!findFieldValue(payload, key, start)) {
    return false;
  }
  size_t index = 0;
  while (*start != '\0' && *start != ',' && *start != '}' && *start != ']') {
    if (index + 1 >= outSize) {
      return false;
    }
    out[index++] = *start;
    ++start;
  }
  out[index] = '\0';
  return index > 0;
}

// Извлечение целого числа из JSON-поля.
bool extractLongField(const char* payload, const char* key, long& value) {
  char token[24];
  if (!extractNumberToken(payload, key, token, sizeof(token))) {
    return false;
  }
  char* endPtr = nullptr;
  long parsed = strtol(token, &endPtr, 10);
  if (endPtr == token) {
    return false;
  }
  value = parsed;
  return true;
}

// Извлечение числа с плавающей точкой. Нужен в первую очередь для rpm шаговика.
bool extractDoubleField(const char* payload, const char* key, double& value) {
  char token[24];
  if (!extractNumberToken(payload, key, token, sizeof(token))) {
    return false;
  }
  value = atof(token);
  return true;
}

// Извлечение bool в формате true/false.
bool extractBoolField(const char* payload, const char* key, bool& value) {
  const char* start = nullptr;
  if (!findFieldValue(payload, key, start)) {
    return false;
  }
  if (strncmp(start, "true", 4) == 0) {
    value = true;
    return true;
  }
  if (strncmp(start, "false", 5) == 0) {
    value = false;
    return true;
  }
  return false;
}

// Извлечение строки без полноценной обработки escape-последовательностей.
bool extractStringField(const char* payload, const char* key, char* out, size_t outSize) {
  const char* start = nullptr;
  if (!findFieldValue(payload, key, start) || *start != '"') {
    return false;
  }
  ++start;
  size_t index = 0;
  while (*start != '\0' && *start != '"') {
    if (index + 1 >= outSize) {
      return false;
    }
    out[index++] = *start;
    ++start;
  }
  if (*start != '"') {
    return false;
  }
  out[index] = '\0';
  return true;
}

// Кнопка подключена как INPUT_PULLUP, поэтому LOW означает "нажата".
bool isButtonPressed() {
  return digitalRead(Config::BUTTON_PIN) == LOW;
}

/*
 * Непосредственное применение команды к мотору.
 *
 * Если мотор крутится "не туда", править нужно не эту функцию, а флаг
 * invertDirection в конфигурации конкретного канала.
 */
void applyMotorOutput(const MotorPins& pins, int percent) {
  int boundedPercent = clampPercent(percent);
  if (boundedPercent == 0) {
    analogWrite(pins.pwmPin, 0);
    return;
  }

  bool forward = boundedPercent > 0;
  if (pins.invertDirection) {
    forward = !forward;
  }

  digitalWrite(pins.dirPin, forward ? HIGH : LOW);
  analogWrite(pins.pwmPin, pwmToDutyCycle(boundedPercent));
}

void stopStepper();

// Полная остановка всех исполнительных механизмов, которые могут двигать робота.
void stopAllMotion() {
  gLeftMotor.currentPercent = 0;
  gLeftMotor.timed = false;
  gLeftMotor.stopAtMs = 0;
  gLeftMotor.rampActive = false;
  gLeftMotor.rampStartPercent = 0;
  gLeftMotor.rampStopPercent = 0;
  gLeftMotor.rampStartedAtMs = 0;
  gLeftMotor.rampDurationMs = 0;
  gRightMotor.currentPercent = 0;
  gRightMotor.timed = false;
  gRightMotor.stopAtMs = 0;
  gRightMotor.rampActive = false;
  gRightMotor.rampStartPercent = 0;
  gRightMotor.rampStopPercent = 0;
  gRightMotor.rampStartedAtMs = 0;
  gRightMotor.rampDurationMs = 0;
  applyMotorOutput(leftMotorPins(), 0);
  applyMotorOutput(rightMotorPins(), 0);
  stopStepper();
}

// Запоминаем новое состояние моторного канала и сразу применяем его к железу.
void setMotorState(MotorRuntimeState& state, const MotorPins& pins, int percent, long durationMs, bool rampEnabled = false, int startPercent = 0, long rampDurationMs = 0) {
  int boundedPercent = clampPercent(percent);
  int boundedStartPercent = clampPercent(startPercent);
  if (durationMs > 0) {
    state.timed = true;
    state.stopAtMs = millis() + static_cast<unsigned long>(durationMs);
  } else {
    state.timed = false;
    state.stopAtMs = 0;
  }
  if (rampEnabled && rampDurationMs > 0) {
    state.rampActive = true;
    state.rampStartPercent = boundedStartPercent;
    state.rampStopPercent = boundedPercent;
    state.rampStartedAtMs = millis();
    state.rampDurationMs = static_cast<unsigned long>(rampDurationMs);
    state.currentPercent = boundedStartPercent;
  } else {
    state.rampActive = false;
    state.rampStartPercent = 0;
    state.rampStopPercent = 0;
    state.rampStartedAtMs = 0;
    state.rampDurationMs = 0;
    state.currentPercent = boundedPercent;
  }
  applyMotorOutput(pins, state.currentPercent);
}

// Разводим high-level команду по одному или двум каналам.
void setMotorCommand(const char* target, int percent, long durationMs, bool rampEnabled = false, int startPercent = 0, long rampDurationMs = 0) {
  if (strcmp(target, "all") == 0) {
    setMotorState(gLeftMotor, leftMotorPins(), percent, durationMs, rampEnabled, startPercent, rampDurationMs);
    setMotorState(gRightMotor, rightMotorPins(), percent, durationMs, rampEnabled, startPercent, rampDurationMs);
    return;
  }
  if (strcmp(target, "left") == 0) {
    setMotorState(gLeftMotor, leftMotorPins(), percent, durationMs, rampEnabled, startPercent, rampDurationMs);
    return;
  }
  if (strcmp(target, "right") == 0) {
    setMotorState(gRightMotor, rightMotorPins(), percent, durationMs, rampEnabled, startPercent, rampDurationMs);
    return;
  }
}

/*
 * Здесь живут две важные вещи:
 * - автоостановка timed-команд;
 * - watchdog, который спасает от "вечного" движения при обрыве связи.
 */
void updateMotorTimers() {
  unsigned long nowMs = millis();

  if (gLeftMotor.rampActive) {
    if (gLeftMotor.rampDurationMs == 0 || hasElapsed(nowMs, gLeftMotor.rampStartedAtMs + gLeftMotor.rampDurationMs)) {
      gLeftMotor.rampActive = false;
      gLeftMotor.currentPercent = gLeftMotor.rampStopPercent;
      applyMotorOutput(leftMotorPins(), gLeftMotor.currentPercent);
    } else {
      long elapsedMs = static_cast<long>(nowMs - gLeftMotor.rampStartedAtMs);
      long deltaPercent = static_cast<long>(gLeftMotor.rampStopPercent - gLeftMotor.rampStartPercent);
      int interpolatedPercent =
          clampPercent(gLeftMotor.rampStartPercent + ((deltaPercent * elapsedMs) / static_cast<long>(gLeftMotor.rampDurationMs)));
      if (interpolatedPercent != gLeftMotor.currentPercent) {
        gLeftMotor.currentPercent = interpolatedPercent;
        applyMotorOutput(leftMotorPins(), gLeftMotor.currentPercent);
      }
    }
  }

  if (gRightMotor.rampActive) {
    if (gRightMotor.rampDurationMs == 0 || hasElapsed(nowMs, gRightMotor.rampStartedAtMs + gRightMotor.rampDurationMs)) {
      gRightMotor.rampActive = false;
      gRightMotor.currentPercent = gRightMotor.rampStopPercent;
      applyMotorOutput(rightMotorPins(), gRightMotor.currentPercent);
    } else {
      long elapsedMs = static_cast<long>(nowMs - gRightMotor.rampStartedAtMs);
      long deltaPercent = static_cast<long>(gRightMotor.rampStopPercent - gRightMotor.rampStartPercent);
      int interpolatedPercent =
          clampPercent(gRightMotor.rampStartPercent + ((deltaPercent * elapsedMs) / static_cast<long>(gRightMotor.rampDurationMs)));
      if (interpolatedPercent != gRightMotor.currentPercent) {
        gRightMotor.currentPercent = interpolatedPercent;
        applyMotorOutput(rightMotorPins(), gRightMotor.currentPercent);
      }
    }
  }

  if (gLeftMotor.timed && hasElapsed(nowMs, gLeftMotor.stopAtMs)) {
    gLeftMotor.timed = false;
    gLeftMotor.currentPercent = 0;
    gLeftMotor.rampActive = false;
    applyMotorOutput(leftMotorPins(), 0);
  }

  if (gRightMotor.timed && hasElapsed(nowMs, gRightMotor.stopAtMs)) {
    gRightMotor.timed = false;
    gRightMotor.currentPercent = 0;
    gRightMotor.rampActive = false;
    applyMotorOutput(rightMotorPins(), 0);
  }

  if (!gWatchdogTriggered && hasElapsed(nowMs, gLastValidCommandAtMs + Config::COMMAND_WATCHDOG_MS)) {
    bool stoppedUnsafeMotion = false;

    if (!gLeftMotor.timed && gLeftMotor.currentPercent != 0) {
      gLeftMotor.currentPercent = 0;
      gLeftMotor.rampActive = false;
      applyMotorOutput(leftMotorPins(), 0);
      stoppedUnsafeMotion = true;
    }

    if (!gRightMotor.timed && gRightMotor.currentPercent != 0) {
      gRightMotor.currentPercent = 0;
      gRightMotor.rampActive = false;
      applyMotorOutput(rightMotorPins(), 0);
      stoppedUnsafeMotion = true;
    }

    if (gStepper.running && !gStepper.hasDeadline && !gStepper.hasStepLimit) {
      stopStepper();
      stoppedUnsafeMotion = true;
    }

    if (stoppedUnsafeMotion) {
      gWatchdogTriggered = true;
    }
  }
}

// Унифицированное управление линией ENABLE у stepper driver.
void enableStepperDriver(bool enabled) {
  if (!Config::STEPPER_ENABLED) {
    return;
  }

  bool outputState = enabled;
  if (Config::STEPPER_ENABLE_ACTIVE_LOW) {
    outputState = !enabled;
  }
  digitalWrite(Config::STEPPER_ENABLE_PIN, outputState ? HIGH : LOW);
}

// Безопасная остановка шагового двигателя и перевод драйвера в покой.
void stopStepper() {
  if (!Config::STEPPER_ENABLED) {
    return;
  }

  gStepper.running = false;
  gStepper.stepPinHigh = false;
  gStepper.hasDeadline = false;
  gStepper.hasStepLimit = false;
  gStepper.stepsRemaining = 0;
  digitalWrite(Config::STEPPER_STEP_PIN, LOW);
  enableStepperDriver(false);
}

/*
 * Подготовка stepper driver к работе.
 *
 * Если шаговик дёргается, но не крутится стабильно, обычно проблема одна из этих:
 * - слишком большой rpm;
 * - перепутаны STEP/DIR;
 * - неверная логика ENABLE;
 * - недостаток питания драйвера или мотора.
 */
void startStepper(bool forward, unsigned long rpm, long steps, long durationMs) {
  if (!Config::STEPPER_ENABLED) {
    return;
  }

  enableStepperDriver(true);
  digitalWrite(Config::STEPPER_DIR_PIN, forward ? HIGH : LOW);

  unsigned long intervalUs = 60000000UL / (static_cast<unsigned long>(Config::STEPPER_STEPS_PER_REV) * rpm);
  if (intervalUs < Config::STEPPER_PULSE_HIGH_US + 20UL) {
    intervalUs = Config::STEPPER_PULSE_HIGH_US + 20UL;
  }

  gStepper.running = true;
  gStepper.stepPinHigh = false;
  gStepper.stepIntervalUs = intervalUs;
  gStepper.lastPulseUs = micros();
  gStepper.hasDeadline = durationMs > 0;
  gStepper.stopAtMs = millis() + static_cast<unsigned long>(durationMs > 0 ? durationMs : 0);
  gStepper.hasStepLimit = steps > 0;
  gStepper.stepsRemaining = steps > 0 ? steps : 0;
}

/*
 * Неблокирующее "тикание" шаговика.
 *
 * STEP поднимается и опускается как отдельные фазы, чтобы импульс имел нужную
 * длительность и не ломал тайминг драйвера.
 */
void updateStepper() {
  if (!Config::STEPPER_ENABLED || !gStepper.running) {
    return;
  }

  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  if (gStepper.hasDeadline && hasElapsed(nowMs, gStepper.stopAtMs)) {
    stopStepper();
    return;
  }

  if (gStepper.stepPinHigh) {
    if (hasElapsed(nowUs, gStepper.lastPulseUs + Config::STEPPER_PULSE_HIGH_US)) {
      digitalWrite(Config::STEPPER_STEP_PIN, LOW);
      gStepper.stepPinHigh = false;
      gStepper.lastPulseUs = nowUs;
      if (gStepper.hasStepLimit && gStepper.stepsRemaining <= 0) {
        stopStepper();
      }
    }
    return;
  }

  if (gStepper.hasStepLimit && gStepper.stepsRemaining <= 0) {
    stopStepper();
    return;
  }

  if (hasElapsed(nowUs, gStepper.lastPulseUs + gStepper.stepIntervalUs)) {
    digitalWrite(Config::STEPPER_STEP_PIN, HIGH);
    gStepper.stepPinHigh = true;
    gStepper.lastPulseUs = nowUs;
    if (gStepper.hasStepLimit && gStepper.stepsRemaining > 0) {
      --gStepper.stepsRemaining;
    }
  }
}

// Стартуем очередное измерение ультразвуковым датчиком.
void beginSensorCycle(DistanceSensorState& sensor) {
  digitalWrite(sensor.trigPin, HIGH);
  sensor.stage = SENSOR_TRIGGER_HIGH;
  sensor.stageStartedUs = micros();
}

// Завершаем цикл измерения и сохраняем результат только если он валиден.
bool finishSensorCycle(DistanceSensorState& sensor, bool validMeasurement, long distanceMm) {
  if (validMeasurement) {
    sensor.valid = true;
    sensor.lastDistanceMm = distanceMm;
    sensor.lastSuccessAtMs = millis();
  }
  sensor.stage = SENSOR_IDLE;
  sensor.nextTriggerAtMs = millis() + Config::SENSOR_SAMPLE_INTERVAL_MS;
  return true;
}

/*
 * Пошаговый опрос активного ультразвукового датчика.
 *
 * Формула расстояния:
 * - скорость звука примерно 343 м/с;
 * - echo проходит путь туда и обратно, поэтому делим на два.
 */
bool updateActiveSensor(DistanceSensorState& sensor) {
  unsigned long nowUs = micros();

  switch (sensor.stage) {
    case SENSOR_TRIGGER_HIGH:
      if (hasElapsed(nowUs, sensor.stageStartedUs + Config::SENSOR_TRIGGER_PULSE_US)) {
        digitalWrite(sensor.trigPin, LOW);
        sensor.stage = SENSOR_WAIT_ECHO_HIGH;
        sensor.stageStartedUs = nowUs;
      }
      break;

    case SENSOR_WAIT_ECHO_HIGH:
      if (digitalRead(sensor.echoPin) == HIGH) {
        sensor.echoStartedUs = nowUs;
        sensor.stage = SENSOR_WAIT_ECHO_LOW;
      } else if (hasElapsed(nowUs, sensor.stageStartedUs + Config::SENSOR_ECHO_TIMEOUT_US)) {
        return finishSensorCycle(sensor, false, 0);
      }
      break;

    case SENSOR_WAIT_ECHO_LOW:
      if (digitalRead(sensor.echoPin) == LOW) {
        unsigned long pulseWidthUs = nowUs - sensor.echoStartedUs;
        long distanceMm = static_cast<long>((pulseWidthUs * 343UL) / 2000UL);
        return finishSensorCycle(sensor, true, distanceMm);
      }
      if (hasElapsed(nowUs, sensor.echoStartedUs + Config::SENSOR_ECHO_TIMEOUT_US)) {
        return finishSensorCycle(sensor, false, 0);
      }
      break;

    case SENSOR_IDLE:
    default:
      return true;
  }

  return false;
}

/*
 * Два датчика запускаются по очереди, а не одновременно.
 *
 * Это уменьшает вероятность того, что один HC-SR04 поймает echo от другого.
 */
void updateSensors() {
  if (gActiveSensorIndex >= 0) {
    if (updateActiveSensor(gSensors[gActiveSensorIndex])) {
      gActiveSensorIndex = -1;
    }
    return;
  }

  unsigned long nowMs = millis();
  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    int candidate = (gNextSensorIndex + attempt) % 2;
    if (hasElapsed(nowMs, gSensors[candidate].nextTriggerAtMs)) {
      gActiveSensorIndex = candidate;
      gNextSensorIndex = (candidate + 1) % 2;
      beginSensorCycle(gSensors[candidate]);
      return;
    }
  }
}

// Возвращаем расстояние только если последнее измерение ещё не считается устаревшим.
bool sensorDistanceAvailable(int sensorIndex, long& distanceMm) {
  if (sensorIndex < 0 || sensorIndex >= 2) {
    return false;
  }
  DistanceSensorState& sensor = gSensors[sensorIndex];
  if (!sensor.valid) {
    return false;
  }
  if (!hasElapsed(millis(), sensor.lastSuccessAtMs + Config::SENSOR_STALE_MS)) {
    distanceMm = sensor.lastDistanceMm;
    return true;
  }
  return false;
}

// Сводный статус нужен для быстрой диагностики без отдельного вызова каждого датчика.
void sendStatusResponse(long requestId) {
  long distance1 = 0;
  long distance2 = 0;
  bool sensor1Valid = sensorDistanceAvailable(0, distance1);
  bool sensor2Valid = sensorDistanceAvailable(1, distance2);

  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.print(F(",\"ok\":true,\"data\":{"));
  Serial.print(F("\"button_pressed\":"));
  printBool(isButtonPressed());
  Serial.print(F(",\"left_pwm\":"));
  Serial.print(gLeftMotor.currentPercent);
  Serial.print(F(",\"right_pwm\":"));
  Serial.print(gRightMotor.currentPercent);
  Serial.print(F(",\"watchdog_ms\":"));
  Serial.print(Config::COMMAND_WATCHDOG_MS);
  Serial.print(F(",\"relay_enabled\":"));
  printBool(gRelayEnabled);
  Serial.print(F(",\"stepper_running\":"));
  printBool(gStepper.running);
  Serial.print(F(",\"features\":{\"servo\":"));
  printBool(Config::SERVO_ENABLED);
  Serial.print(F(",\"relay\":"));
  printBool(Config::RELAY_ENABLED);
  Serial.print(F(",\"stepper\":"));
  printBool(Config::STEPPER_ENABLED);
  Serial.print(F("},\"distance_mm\":{\"1\":"));
  if (sensor1Valid) {
    Serial.print(distance1);
  } else {
    Serial.print(F("null"));
  }
  Serial.print(F(",\"2\":"));
  if (sensor2Valid) {
    Serial.print(distance2);
  } else {
    Serial.print(F("null"));
  }
  Serial.println(F("}}}"));
}

// Сброс watchdog-таймера после любой корректно распознанной команды.
void markCommandReceived() {
  gLastValidCommandAtMs = millis();
  gWatchdogTriggered = false;
}

// ping помогает Raspberry Pi убедиться, что на порту действительно наша прошивка.
void handlePing(long requestId) {
  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.println(F(",\"ok\":true,\"data\":{\"pong\":true,\"firmware\":\"rescue_maze_low_level_v1\"}}"));
}

// Отдаём только одно конкретное расстояние, чтобы транспортный протокол был простым.
void handleDistanceRequest(long requestId, const char* payload) {
  long sensorId = 0;
  if (!extractLongField(payload, "sensor", sensorId) || sensorId < 1 || sensorId > 2) {
    sendErrorResponse(requestId, "bad_request", "sensor должен быть равен 1 или 2");
    return;
  }

  long distanceMm = 0;
  if (!sensorDistanceAvailable(static_cast<int>(sensorId) - 1, distanceMm)) {
    sendErrorResponse(requestId, "sensor_timeout", "измерение расстояния недоступно");
    return;
  }

  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.print(F(",\"ok\":true,\"data\":{\"sensor\":"));
  Serial.print(sensorId);
  Serial.print(F(",\"distance_mm\":"));
  Serial.print(distanceMm);
  Serial.println(F("}}"));
}

// Состояние кнопки читается мгновенно и не требует отдельного автомата.
void handleButtonRequest(long requestId) {
  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.print(F(",\"ok\":true,\"data\":{\"pressed\":"));
  printBool(isButtonPressed());
  Serial.println(F("}}"));
}

/*
 * Базовая команда для движения гусениц.
 *
 * duration_ms опционален:
 * - если он > 0, Arduino сама остановит мотор позже;
 * - если его нет, команда действует до следующего изменения состояния.
 */
void handleSetMotor(long requestId, const char* payload) {
  char target[8];
  long pwm = 0;
  long durationMs = 0;
  long startPwm = 0;
  long rampDurationMs = 0;
  bool hasStartPwm = false;
  bool hasRampDuration = false;

  if (!extractStringField(payload, "target", target, sizeof(target))) {
    sendErrorResponse(requestId, "bad_request", "поле target обязательно");
    return;
  }

  if (strcmp(target, "all") != 0 && strcmp(target, "left") != 0 && strcmp(target, "right") != 0) {
    sendErrorResponse(requestId, "bad_request", "target должен быть all, left или right");
    return;
  }

  if (!extractLongField(payload, "pwm", pwm)) {
    sendErrorResponse(requestId, "bad_request", "поле pwm обязательно");
    return;
  }

  if (!extractLongField(payload, "duration_ms", durationMs)) {
    durationMs = 0;
  }
  if (durationMs < 0) {
    durationMs = 0;
  }

  hasStartPwm = extractLongField(payload, "start_pwm", startPwm);
  hasRampDuration = extractLongField(payload, "ramp_duration_ms", rampDurationMs);
  if (!hasRampDuration) {
    rampDurationMs = 0;
  }
  if (rampDurationMs < 0) {
    rampDurationMs = 0;
  }
  if (hasRampDuration && rampDurationMs > 0 && !hasStartPwm) {
    sendErrorResponse(requestId, "bad_request", "для ramp нужно поле start_pwm");
    return;
  }
  if (hasRampDuration && durationMs > 0 && rampDurationMs > durationMs) {
    sendErrorResponse(requestId, "bad_request", "ramp_duration_ms не должен превышать duration_ms");
    return;
  }

  bool rampEnabled = hasStartPwm && hasRampDuration && rampDurationMs > 0;
  setMotorCommand(target, clampPercent(pwm), durationMs, rampEnabled, clampPercent(startPwm), rampDurationMs);

  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.print(F(",\"ok\":true,\"data\":{\"target\":\""));
  Serial.print(target);
  Serial.print(F("\",\"pwm\":"));
  Serial.print(clampPercent(pwm));
  Serial.print(F(",\"duration_ms\":"));
  Serial.print(durationMs);
  if (rampEnabled) {
    Serial.print(F(",\"start_pwm\":"));
    Serial.print(clampPercent(startPwm));
    Serial.print(F(",\"ramp_duration_ms\":"));
    Serial.print(rampDurationMs);
  }
  Serial.println(F("}}"));
}

// Управление сервой вынесено отдельно, чтобы можно было честно вернуть unsupported.
void handleSetServo(long requestId, const char* payload) {
  if (!Config::SERVO_ENABLED) {
    sendErrorResponse(requestId, "unsupported", "сервопривод не настроен");
    return;
  }

  long angle = 0;
  if (!extractLongField(payload, "angle_deg", angle)) {
    sendErrorResponse(requestId, "bad_request", "поле angle_deg обязательно");
    return;
  }

  if (angle < 0) {
    angle = 0;
  } else if (angle > 180) {
    angle = 180;
  }

  if (!gServoAttached) {
    gServo.attach(Config::SERVO_PIN);
    gServoAttached = true;
  }

  gServoAngle = static_cast<int>(angle);
  gServo.write(gServoAngle);

  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.print(F(",\"ok\":true,\"data\":{\"angle_deg\":"));
  Serial.print(gServoAngle);
  Serial.println(F("}}"));
}

// Управление реле бинарное: либо включено, либо выключено.
void handleSetRelay(long requestId, const char* payload) {
  if (!Config::RELAY_ENABLED) {
    sendErrorResponse(requestId, "unsupported", "реле не настроено");
    return;
  }

  bool enabled = false;
  if (!extractBoolField(payload, "enabled", enabled)) {
    sendErrorResponse(requestId, "bad_request", "поле enabled обязательно");
    return;
  }

  gRelayEnabled = enabled;
  digitalWrite(Config::RELAY_PIN, enabled ? HIGH : LOW);

  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.print(F(",\"ok\":true,\"data\":{\"enabled\":"));
  printBool(gRelayEnabled);
  Serial.println(F("}}"));
}

/*
 * Запуск шаговика с параметрами из JSON.
 *
 * Если steps отрицательный, логика специально разворачивает направление:
 * это позволяет вызывать команду более гибко с Raspberry Pi.
 */
void handleStepperMove(long requestId, const char* payload) {
  if (!Config::STEPPER_ENABLED) {
    sendErrorResponse(requestId, "unsupported", "шаговый двигатель не настроен");
    return;
  }

  double rpmValue = 0.0;
  if (!extractDoubleField(payload, "rpm", rpmValue) || rpmValue <= 0.0) {
    sendErrorResponse(requestId, "bad_request", "rpm должно быть положительным");
    return;
  }

  char direction[10];
  if (!extractStringField(payload, "direction", direction, sizeof(direction))) {
    strcpy(direction, "forward");
  }

  bool forward = true;
  if (strcmp(direction, "forward") == 0) {
    forward = true;
  } else if (strcmp(direction, "reverse") == 0) {
    forward = false;
  } else {
    sendErrorResponse(requestId, "bad_request", "direction должно быть forward или reverse");
    return;
  }

  long steps = 0;
  bool hasSteps = extractLongField(payload, "steps", steps);
  if (hasSteps && steps < 0) {
    steps = abs(steps);
    forward = !forward;
  }

  long durationMs = 0;
  if (!extractLongField(payload, "duration_ms", durationMs)) {
    durationMs = 0;
  }
  if (durationMs < 0) {
    durationMs = 0;
  }

  startStepper(forward, static_cast<unsigned long>(rpmValue), hasSteps ? steps : 0, durationMs);

  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.print(F(",\"ok\":true,\"data\":{\"running\":"));
  printBool(gStepper.running);
  Serial.print(F(",\"rpm\":"));
  Serial.print(rpmValue, 2);
  Serial.print(F(",\"direction\":\""));
  Serial.print(forward ? F("forward") : F("reverse"));
  Serial.print(F("\",\"steps\":"));
  if (hasSteps) {
    Serial.print(steps);
  } else {
    Serial.print(F("null"));
  }
  Serial.print(F(",\"duration_ms\":"));
  Serial.print(durationMs);
  Serial.println(F("}}"));
}

/*
 * Центральный диспетчер протокола.
 *
 * При добавлении новой команды обычно нужно:
 * - сделать новый handle...();
 * - добавить ветку strcmp ниже;
 * - при необходимости обновить get_status и python-сервис.
 */
void handleRequest(const char* payload) {
  long requestId = 0;
  char operation[20];

  if (!extractLongField(payload, "id", requestId)) {
    sendErrorResponse(0, "bad_request", "поле id обязательно");
    return;
  }

  if (!extractStringField(payload, "op", operation, sizeof(operation))) {
    sendErrorResponse(requestId, "bad_request", "поле op обязательно");
    return;
  }

  markCommandReceived();

  if (strcmp(operation, "ping") == 0) {
    handlePing(requestId);
    return;
  }

  if (strcmp(operation, "get_distance") == 0) {
    handleDistanceRequest(requestId, payload);
    return;
  }

  if (strcmp(operation, "get_button") == 0) {
    handleButtonRequest(requestId);
    return;
  }

  if (strcmp(operation, "set_motor") == 0) {
    handleSetMotor(requestId, payload);
    return;
  }

  if (strcmp(operation, "stop_all") == 0) {
    stopAllMotion();
    sendEmptyOkResponse(requestId);
    return;
  }

  if (strcmp(operation, "get_status") == 0) {
    sendStatusResponse(requestId);
    return;
  }

  if (strcmp(operation, "set_servo") == 0) {
    handleSetServo(requestId, payload);
    return;
  }

  if (strcmp(operation, "set_relay") == 0) {
    handleSetRelay(requestId, payload);
    return;
  }

  if (strcmp(operation, "stepper_move") == 0) {
    handleStepperMove(requestId, payload);
    return;
  }

  if (strcmp(operation, "stepper_stop") == 0) {
    if (!Config::STEPPER_ENABLED) {
      sendErrorResponse(requestId, "unsupported", "шаговый двигатель не настроен");
      return;
    }
    stopStepper();
    sendEmptyOkResponse(requestId);
    return;
  }

  sendErrorResponse(requestId, "unknown_op", "операция не поддерживается");
}

/*
 * Построчное чтение serial-команд.
 *
 * Правило простое: одна команда занимает одну строку и обязательно завершается
 * символом '\n'. Пока перевод строки не пришёл, Arduino считает, что команда ещё
 * не закончена.
 *
 * Если Raspberry Pi пишет в порт без '\n', внешне это выглядит так, будто
 * прошивка "не отвечает", хотя на самом деле она просто ждёт конец строки.
 */
void readSerialRequests() {
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      if (gRequestOverflow) {
        sendErrorResponse(0, "bad_request", "запрос слишком длинный");
      } else if (gRequestLength > 0) {
        gRequestBuffer[gRequestLength] = '\0';
        handleRequest(gRequestBuffer);
      }
      gRequestLength = 0;
      gRequestOverflow = false;
      continue;
    }

    if (gRequestOverflow) {
      continue;
    }

    if (gRequestLength + 1 >= REQUEST_BUFFER_SIZE) {
      gRequestOverflow = true;
      continue;
    }

    gRequestBuffer[gRequestLength++] = incoming;
  }
}

/*
 * Инициализация всех пинов в безопасное стартовое состояние.
 *
 * Если после прошивки что-то работает не так, проверять обычно нужно отсюда:
 * - верна ли распиновка в Config;
 * - на тех ли пинах сидят trig/echo;
 * - не перепутаны ли dir/pwm у моторов;
 * - не наоборот ли логика enable у stepper driver.
 */
void setupPins() {
  pinMode(leftMotorPins().dirPin, OUTPUT);
  pinMode(leftMotorPins().pwmPin, OUTPUT);
  pinMode(rightMotorPins().dirPin, OUTPUT);
  pinMode(rightMotorPins().pwmPin, OUTPUT);
  digitalWrite(leftMotorPins().dirPin, LOW);
  digitalWrite(rightMotorPins().dirPin, LOW);
  analogWrite(leftMotorPins().pwmPin, 0);
  analogWrite(rightMotorPins().pwmPin, 0);

  for (uint8_t index = 0; index < 2; ++index) {
    pinMode(gSensors[index].trigPin, OUTPUT);
    pinMode(gSensors[index].echoPin, INPUT);
    digitalWrite(gSensors[index].trigPin, LOW);
    gSensors[index].stage = SENSOR_IDLE;
    gSensors[index].nextTriggerAtMs = millis() + (index * Config::SENSOR_SAMPLE_INTERVAL_MS);
  }

  pinMode(Config::BUTTON_PIN, INPUT_PULLUP);

  if (Config::SERVO_ENABLED) {
    gServo.attach(Config::SERVO_PIN);
    gServo.write(gServoAngle);
    gServoAttached = true;
  }

  if (Config::RELAY_ENABLED) {
    pinMode(Config::RELAY_PIN, OUTPUT);
    digitalWrite(Config::RELAY_PIN, LOW);
    gRelayEnabled = false;
  }

  if (Config::STEPPER_ENABLED) {
    pinMode(Config::STEPPER_STEP_PIN, OUTPUT);
    pinMode(Config::STEPPER_DIR_PIN, OUTPUT);
    pinMode(Config::STEPPER_ENABLE_PIN, OUTPUT);
    digitalWrite(Config::STEPPER_STEP_PIN, LOW);
    digitalWrite(Config::STEPPER_DIR_PIN, LOW);
    enableStepperDriver(false);
  }
}

// setup() выполняется один раз после старта Arduino.
void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  setupPins();
  gLastValidCommandAtMs = millis();
}

/*
 * Главный цикл прошивки.
 *
 * Порядок вызовов важен:
 * - сначала читаем новые команды;
 * - потом обновляем датчики;
 * - затем timed-моторы и watchdog;
 * - в конце шаговый двигатель.
 *
 * Здесь намеренно нет delay(), иначе робот начнёт "слепнуть" к новым командам.
 */
void loop() {
  readSerialRequests();
  updateSensors();
  updateMotorTimers();
  updateStepper();
}
