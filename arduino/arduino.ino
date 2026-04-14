#include <Arduino.h>
#include <SoftwareSerial.h>
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
constexpr unsigned long URM37_TRIGGER_PULSE_US = 25UL;
constexpr unsigned long URM37_ECHO_TIMEOUT_US = 50000UL;
constexpr unsigned long URM37_SERIAL_BAUD = 9600UL;
constexpr unsigned long URM37_SERIAL_TIMEOUT_MS = 120UL;
constexpr unsigned long STEPPER_PULSE_HIGH_US = 10UL;
constexpr long STEPPER_STEPS_PER_REV = 200L;
constexpr bool SWAP_TRACK_MOTORS = false;
constexpr bool LEFT_TRACK_INVERTED = false;
constexpr bool RIGHT_TRACK_INVERTED = true;
constexpr bool SERVO_ENABLED = false;
constexpr bool RELAY_ENABLED = false;
constexpr bool STEPPER_ENABLED = false;
constexpr bool STEPPER_ENABLE_ACTIVE_LOW = true;
constexpr uint8_t PIN_NOT_ASSIGNED = 0xFF;
constexpr uint8_t DISTANCE_SENSOR_COUNT = 4;
constexpr uint8_t URM37_DEFAULT_AUTO_INTERVAL_MS = 25;
constexpr uint8_t URM37_DEFAULT_SENSITIVITY = 120;
constexpr uint8_t SENSOR1_TRIG_PIN = 2;
constexpr uint8_t SENSOR1_ECHO_PIN = 3;
constexpr uint8_t LEFT_DIR_PIN = 4;
constexpr uint8_t LEFT_PWM_PIN = 5;
constexpr uint8_t RIGHT_PWM_PIN = 6;
constexpr uint8_t RIGHT_DIR_PIN = 7;
constexpr uint8_t SENSOR2_TRIG_PIN = 8;
constexpr uint8_t SENSOR2_ECHO_PIN = 9;
constexpr uint8_t SENSOR1_SERIAL_RX_PIN = PIN_NOT_ASSIGNED;
constexpr uint8_t SENSOR1_SERIAL_TX_PIN = PIN_NOT_ASSIGNED;
constexpr uint8_t SERVO_PIN = 10;
constexpr uint8_t RELAY_PIN = 11;
constexpr uint8_t STEPPER_STEP_PIN = 12;
constexpr uint8_t STEPPER_DIR_PIN = 13;
constexpr uint8_t SENSOR2_SERIAL_RX_PIN = A2;
constexpr uint8_t SENSOR2_SERIAL_TX_PIN = A3;
constexpr uint8_t SENSOR3_TRIG_PIN = PIN_NOT_ASSIGNED;
constexpr uint8_t SENSOR3_ECHO_PIN = PIN_NOT_ASSIGNED;
constexpr uint8_t SENSOR3_SERIAL_RX_PIN = PIN_NOT_ASSIGNED;
constexpr uint8_t SENSOR3_SERIAL_TX_PIN = PIN_NOT_ASSIGNED;
constexpr uint8_t SENSOR4_TRIG_PIN = PIN_NOT_ASSIGNED;
constexpr uint8_t SENSOR4_ECHO_PIN = PIN_NOT_ASSIGNED;
constexpr uint8_t SENSOR4_SERIAL_RX_PIN = PIN_NOT_ASSIGNED;
constexpr uint8_t SENSOR4_SERIAL_TX_PIN = PIN_NOT_ASSIGNED;
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
  SENSOR_TRIGGER_ACTIVE = 1,
  SENSOR_WAIT_ECHO_ACTIVE = 2,
  SENSOR_WAIT_ECHO_INACTIVE = 3
};

enum DistanceSensorKind : uint8_t {
  DISTANCE_SENSOR_DISABLED = 0,
  DISTANCE_SENSOR_HC_SR04 = 1,
  DISTANCE_SENSOR_URM37 = 2
};

enum Urm37MeasureMode : uint8_t {
  URM37_MEASURE_PWM_PASSIVE = 0,
  URM37_MEASURE_AUTO = 1
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
  bool enabled;
  DistanceSensorKind kind;
  uint8_t trigPin;
  uint8_t echoPin;
  uint8_t serialRxPin;
  uint8_t serialTxPin;
  Urm37MeasureMode urm37MeasureMode;
  uint8_t urm37AutoIntervalMs;
  uint16_t urm37CompareDistanceCm;
  uint8_t urm37Sensitivity;
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

// All distance slots live in one array so the polling and status code stay shared.
DistanceSensorState gSensors[Config::DISTANCE_SENSOR_COUNT] = {
    {
        true,
        DISTANCE_SENSOR_HC_SR04,
        Config::SENSOR1_TRIG_PIN,
        Config::SENSOR1_ECHO_PIN,
        Config::SENSOR1_SERIAL_RX_PIN,
        Config::SENSOR1_SERIAL_TX_PIN,
        URM37_MEASURE_PWM_PASSIVE,
        Config::URM37_DEFAULT_AUTO_INTERVAL_MS,
        0,
        Config::URM37_DEFAULT_SENSITIVITY,
        SENSOR_IDLE,
        false,
        0,
        0,
        0,
        0,
        0,
    },
    {
        true,
        DISTANCE_SENSOR_URM37,
        Config::SENSOR2_TRIG_PIN,
        Config::SENSOR2_ECHO_PIN,
        Config::SENSOR2_SERIAL_RX_PIN,
        Config::SENSOR2_SERIAL_TX_PIN,
        URM37_MEASURE_PWM_PASSIVE,
        Config::URM37_DEFAULT_AUTO_INTERVAL_MS,
        0,
        Config::URM37_DEFAULT_SENSITIVITY,
        SENSOR_IDLE,
        false,
        0,
        0,
        0,
        0,
        0,
    },
    {
        false,
        DISTANCE_SENSOR_DISABLED,
        Config::SENSOR3_TRIG_PIN,
        Config::SENSOR3_ECHO_PIN,
        Config::SENSOR3_SERIAL_RX_PIN,
        Config::SENSOR3_SERIAL_TX_PIN,
        URM37_MEASURE_PWM_PASSIVE,
        Config::URM37_DEFAULT_AUTO_INTERVAL_MS,
        0,
        Config::URM37_DEFAULT_SENSITIVITY,
        SENSOR_IDLE,
        false,
        0,
        0,
        0,
        0,
        0,
    },
    {
        false,
        DISTANCE_SENSOR_DISABLED,
        Config::SENSOR4_TRIG_PIN,
        Config::SENSOR4_ECHO_PIN,
        Config::SENSOR4_SERIAL_RX_PIN,
        Config::SENSOR4_SERIAL_TX_PIN,
        URM37_MEASURE_PWM_PASSIVE,
        Config::URM37_DEFAULT_AUTO_INTERVAL_MS,
        0,
        Config::URM37_DEFAULT_SENSITIVITY,
        SENSOR_IDLE,
        false,
        0,
        0,
        0,
        0,
        0,
    },
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

bool isAssignedPin(uint8_t pin) {
  return pin != Config::PIN_NOT_ASSIGNED;
}

bool isReservedSerialPin(uint8_t pin) {
  return pin == 0 || pin == 1;
}

bool isBaseHardwarePin(uint8_t pin) {
  if (!isAssignedPin(pin)) {
    return false;
  }
  if (pin == Config::LEFT_DIR_PIN || pin == Config::LEFT_PWM_PIN || pin == Config::RIGHT_PWM_PIN ||
      pin == Config::RIGHT_DIR_PIN || pin == Config::BUTTON_PIN) {
    return true;
  }
  if (Config::SERVO_ENABLED && pin == Config::SERVO_PIN) {
    return true;
  }
  if (Config::RELAY_ENABLED && pin == Config::RELAY_PIN) {
    return true;
  }
  if (Config::STEPPER_ENABLED &&
      (pin == Config::STEPPER_STEP_PIN || pin == Config::STEPPER_DIR_PIN || pin == Config::STEPPER_ENABLE_PIN)) {
    return true;
  }
  return false;
}

void disableSensorSlot(DistanceSensorState& sensor) {
  sensor.enabled = false;
  sensor.kind = DISTANCE_SENSOR_DISABLED;
  sensor.trigPin = Config::PIN_NOT_ASSIGNED;
  sensor.echoPin = Config::PIN_NOT_ASSIGNED;
  sensor.serialRxPin = Config::PIN_NOT_ASSIGNED;
  sensor.serialTxPin = Config::PIN_NOT_ASSIGNED;
  sensor.stage = SENSOR_IDLE;
  sensor.valid = false;
  sensor.lastDistanceMm = 0;
  sensor.stageStartedUs = 0;
  sensor.echoStartedUs = 0;
  sensor.nextTriggerAtMs = 0;
  sensor.lastSuccessAtMs = 0;
}

void clearSensorSerialPins(DistanceSensorState& sensor) {
  sensor.serialRxPin = Config::PIN_NOT_ASSIGNED;
  sensor.serialTxPin = Config::PIN_NOT_ASSIGNED;
}

bool sensorUsesConfiguredPin(const DistanceSensorState& sensor, uint8_t pin) {
  if (!sensor.enabled || !isAssignedPin(pin)) {
    return false;
  }
  return sensor.trigPin == pin || sensor.echoPin == pin || sensor.serialRxPin == pin || sensor.serialTxPin == pin;
}

bool sensorHasDistancePins(const DistanceSensorState& sensor) {
  return sensor.enabled && sensor.kind != DISTANCE_SENSOR_DISABLED && isAssignedPin(sensor.trigPin) &&
         isAssignedPin(sensor.echoPin);
}

bool sensorHasSerialPins(const DistanceSensorState& sensor) {
  return sensor.enabled && sensor.kind == DISTANCE_SENSOR_URM37 && isAssignedPin(sensor.serialRxPin) &&
         isAssignedPin(sensor.serialTxPin);
}

bool sensorIsUrm37(const DistanceSensorState& sensor) {
  return sensor.enabled && sensor.kind == DISTANCE_SENSOR_URM37;
}

bool sensorTriggerActiveLevel(const DistanceSensorState& sensor) {
  return sensor.kind == DISTANCE_SENSOR_HC_SR04;
}

bool sensorEchoActiveLevel(const DistanceSensorState& sensor) {
  return sensor.kind == DISTANCE_SENSOR_HC_SR04;
}

unsigned long sensorTriggerPulseUs(const DistanceSensorState& sensor) {
  return sensor.kind == DISTANCE_SENSOR_URM37 ? Config::URM37_TRIGGER_PULSE_US : Config::SENSOR_TRIGGER_PULSE_US;
}

unsigned long sensorEchoTimeoutUs(const DistanceSensorState& sensor) {
  return sensor.kind == DISTANCE_SENSOR_URM37 ? Config::URM37_ECHO_TIMEOUT_US : Config::SENSOR_ECHO_TIMEOUT_US;
}

unsigned long sensorEchoWaitTimeoutUs(const DistanceSensorState& sensor) {
  if (sensor.kind == DISTANCE_SENSOR_URM37 && sensor.urm37MeasureMode == URM37_MEASURE_AUTO) {
    return static_cast<unsigned long>(sensor.urm37AutoIntervalMs) * 1000UL + sensorEchoTimeoutUs(sensor);
  }
  return sensorEchoTimeoutUs(sensor);
}

long pulseWidthToDistanceMm(const DistanceSensorState& sensor, unsigned long pulseWidthUs) {
  if (sensor.kind == DISTANCE_SENSOR_URM37) {
    unsigned long centimeters = (pulseWidthUs + 25UL) / 50UL;
    return static_cast<long>(centimeters * 10UL);
  }
  return static_cast<long>((pulseWidthUs * 343UL) / 2000UL);
}

const char* sensorKindName(const DistanceSensorState& sensor) {
  switch (sensor.kind) {
    case DISTANCE_SENSOR_HC_SR04:
      return "hc_sr04";
    case DISTANCE_SENSOR_URM37:
      return "urm37";
    case DISTANCE_SENSOR_DISABLED:
    default:
      return "disabled";
  }
}

const char* urm37MeasureModeName(Urm37MeasureMode mode) {
  return mode == URM37_MEASURE_AUTO ? "auto" : "pwm_passive";
}

void printNullablePin(uint8_t pin) {
  if (isAssignedPin(pin)) {
    Serial.print(pin);
  } else {
    Serial.print(F("null"));
  }
}

void printNullableDistance(bool valid, long distanceMm) {
  if (valid) {
    Serial.print(distanceMm);
  } else {
    Serial.print(F("null"));
  }
}

void resetSensorRuntimeState(DistanceSensorState& sensor, unsigned long nextTriggerAtMs) {
  sensor.stage = SENSOR_IDLE;
  sensor.valid = false;
  sensor.lastDistanceMm = 0;
  sensor.stageStartedUs = 0;
  sensor.echoStartedUs = 0;
  sensor.nextTriggerAtMs = nextTriggerAtMs;
  sensor.lastSuccessAtMs = 0;
}

void configureSensorPins(DistanceSensorState& sensor) {
  if (!sensorHasDistancePins(sensor)) {
    return;
  }

  pinMode(sensor.echoPin, INPUT);

  if (sensor.kind == DISTANCE_SENSOR_URM37 && sensor.urm37MeasureMode == URM37_MEASURE_AUTO) {
    pinMode(sensor.trigPin, INPUT);
    return;
  }

  pinMode(sensor.trigPin, OUTPUT);
  digitalWrite(sensor.trigPin, sensorTriggerActiveLevel(sensor) ? LOW : HIGH);
}

void sanitizeSensorConfiguration() {
  for (uint8_t index = 0; index < Config::DISTANCE_SENSOR_COUNT; ++index) {
    DistanceSensorState& sensor = gSensors[index];

    if (!sensor.enabled || sensor.kind == DISTANCE_SENSOR_DISABLED) {
      disableSensorSlot(sensor);
      continue;
    }

    if (!isAssignedPin(sensor.trigPin) || !isAssignedPin(sensor.echoPin) || sensor.trigPin == sensor.echoPin ||
        isReservedSerialPin(sensor.trigPin) || isReservedSerialPin(sensor.echoPin) || isBaseHardwarePin(sensor.trigPin) ||
        isBaseHardwarePin(sensor.echoPin)) {
      disableSensorSlot(sensor);
      continue;
    }

    bool duplicateDistancePin = false;
    for (uint8_t previousIndex = 0; previousIndex < index; ++previousIndex) {
      if (sensorUsesConfiguredPin(gSensors[previousIndex], sensor.trigPin) ||
          sensorUsesConfiguredPin(gSensors[previousIndex], sensor.echoPin)) {
        duplicateDistancePin = true;
        break;
      }
    }
    if (duplicateDistancePin) {
      disableSensorSlot(sensor);
      continue;
    }

    if (!sensorIsUrm37(sensor) || isAssignedPin(sensor.serialRxPin) != isAssignedPin(sensor.serialTxPin)) {
      clearSensorSerialPins(sensor);
    }

    if (sensorHasSerialPins(sensor)) {
      bool invalidSerialPins = sensor.serialRxPin == sensor.serialTxPin || sensor.serialRxPin == sensor.trigPin ||
                               sensor.serialRxPin == sensor.echoPin || sensor.serialTxPin == sensor.trigPin ||
                               sensor.serialTxPin == sensor.echoPin || isReservedSerialPin(sensor.serialRxPin) ||
                               isReservedSerialPin(sensor.serialTxPin) || isBaseHardwarePin(sensor.serialRxPin) ||
                               isBaseHardwarePin(sensor.serialTxPin);

      if (!invalidSerialPins) {
        for (uint8_t previousIndex = 0; previousIndex < index; ++previousIndex) {
          if (sensorUsesConfiguredPin(gSensors[previousIndex], sensor.serialRxPin) ||
              sensorUsesConfiguredPin(gSensors[previousIndex], sensor.serialTxPin)) {
            invalidSerialPins = true;
            break;
          }
        }
      }

      if (invalidSerialPins) {
        clearSensorSerialPins(sensor);
      }
    }
  }
}

uint8_t urm37Checksum(uint8_t command, uint8_t data0, uint8_t data1) {
  return static_cast<uint8_t>(command + data0 + data1);
}

bool readUrm37Response(SoftwareSerial& urm37Serial, uint8_t response[4]) {
  unsigned long deadlineAtMs = millis() + Config::URM37_SERIAL_TIMEOUT_MS;
  uint8_t responseIndex = 0;
  while (responseIndex < 4) {
    if (urm37Serial.available() > 0) {
      response[responseIndex++] = static_cast<uint8_t>(urm37Serial.read());
      continue;
    }
    if (hasElapsed(millis(), deadlineAtMs)) {
      return false;
    }
  }
  return response[3] == urm37Checksum(response[0], response[1], response[2]);
}

bool runUrm37Command(const DistanceSensorState& sensor, uint8_t command, uint8_t data0, uint8_t data1, uint8_t response[4]) {
  if (!sensorHasSerialPins(sensor)) {
    return false;
  }

  SoftwareSerial urm37Serial(sensor.serialRxPin, sensor.serialTxPin);
  urm37Serial.begin(Config::URM37_SERIAL_BAUD);
  urm37Serial.listen();
  while (urm37Serial.available() > 0) {
    urm37Serial.read();
  }

  uint8_t request[4] = {command, data0, data1, urm37Checksum(command, data0, data1)};
  size_t written = urm37Serial.write(request, sizeof(request));
  urm37Serial.flush();
  if (written != sizeof(request)) {
    urm37Serial.end();
    return false;
  }

  bool ok = readUrm37Response(urm37Serial, response);
  urm37Serial.end();
  return ok;
}

bool readUrm37EepromByte(const DistanceSensorState& sensor, uint8_t address, uint8_t& value) {
  uint8_t response[4];
  if (!runUrm37Command(sensor, 0x33, address, 0x00, response)) {
    return false;
  }
  if (response[0] != 0x33 || response[1] != address) {
    return false;
  }
  value = response[2];
  return true;
}

bool writeUrm37EepromByte(const DistanceSensorState& sensor, uint8_t address, uint8_t value) {
  uint8_t response[4];
  if (!runUrm37Command(sensor, 0x44, address, value, response)) {
    return false;
  }
  return response[0] == 0x44 && response[1] == address && response[2] == value;
}

bool refreshUrm37SettingsFromSensor(DistanceSensorState& sensor) {
  if (!sensorHasSerialPins(sensor)) {
    return false;
  }

  uint8_t compareLow = 0;
  uint8_t compareHigh = 0;
  uint8_t measureModeByte = 0;
  uint8_t autoIntervalMs = 0;
  if (!readUrm37EepromByte(sensor, 0x00, compareLow) || !readUrm37EepromByte(sensor, 0x01, compareHigh) ||
      !readUrm37EepromByte(sensor, 0x02, measureModeByte) || !readUrm37EepromByte(sensor, 0x04, autoIntervalMs)) {
    return false;
  }

  sensor.urm37CompareDistanceCm = static_cast<uint16_t>((static_cast<uint16_t>(compareHigh) << 8) | compareLow);
  if (sensor.urm37CompareDistanceCm > 1000U) {
    sensor.urm37CompareDistanceCm = 1000U;
  }
  sensor.urm37MeasureMode = measureModeByte == 0xAA ? URM37_MEASURE_AUTO : URM37_MEASURE_PWM_PASSIVE;
  if (autoIntervalMs < Config::URM37_DEFAULT_AUTO_INTERVAL_MS) {
    sensor.urm37AutoIntervalMs = Config::URM37_DEFAULT_AUTO_INTERVAL_MS;
  } else {
    sensor.urm37AutoIntervalMs = autoIntervalMs;
  }
  configureSensorPins(sensor);
  return true;
}

bool readUrm37TemperatureC(const DistanceSensorState& sensor, float& temperatureC) {
  uint8_t response[4];
  if (!runUrm37Command(sensor, 0x11, 0x00, 0x00, response)) {
    return false;
  }
  if (response[0] != 0x11) {
    return false;
  }
  if (response[1] == 0xFF && response[2] == 0xFF) {
    return false;
  }

  uint16_t magnitude = static_cast<uint16_t>(((response[1] & 0x0F) << 8) | response[2]);
  temperatureC = static_cast<float>(magnitude) / 10.0f;
  if ((response[1] & 0xF0) == 0xF0) {
    temperatureC = -temperatureC;
  }
  return true;
}

void sendUrm37SettingsResponse(long requestId, uint8_t sensorId, const DistanceSensorState& sensor) {
  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.print(F(",\"ok\":true,\"data\":{\"sensor\":"));
  Serial.print(sensorId);
  Serial.print(F(",\"measure_mode\":\""));
  Serial.print(urm37MeasureModeName(sensor.urm37MeasureMode));
  Serial.print(F("\",\"auto_measure_interval_ms\":"));
  Serial.print(sensor.urm37AutoIntervalMs);
  Serial.print(F(",\"compare_distance_cm\":"));
  Serial.print(sensor.urm37CompareDistanceCm);
  Serial.print(F(",\"sensitivity\":"));
  Serial.print(sensor.urm37Sensitivity);
  Serial.println(F("}}"));
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
  sensor.stageStartedUs = micros();
  if (sensor.kind == DISTANCE_SENSOR_URM37 && sensor.urm37MeasureMode == URM37_MEASURE_AUTO) {
    sensor.stage = SENSOR_WAIT_ECHO_ACTIVE;
    return;
  }
  digitalWrite(sensor.trigPin, sensorTriggerActiveLevel(sensor) ? HIGH : LOW);
  sensor.stage = SENSOR_TRIGGER_ACTIVE;
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
  bool echoActiveLevel = sensorEchoActiveLevel(sensor);

  switch (sensor.stage) {
    case SENSOR_TRIGGER_ACTIVE:
      if (hasElapsed(nowUs, sensor.stageStartedUs + sensorTriggerPulseUs(sensor))) {
        digitalWrite(sensor.trigPin, sensorTriggerActiveLevel(sensor) ? LOW : HIGH);
        sensor.stage = SENSOR_WAIT_ECHO_ACTIVE;
        sensor.stageStartedUs = nowUs;
      }
      break;

    case SENSOR_WAIT_ECHO_ACTIVE:
      if (digitalRead(sensor.echoPin) == (echoActiveLevel ? HIGH : LOW)) {
        sensor.echoStartedUs = nowUs;
        sensor.stage = SENSOR_WAIT_ECHO_INACTIVE;
      } else if (hasElapsed(nowUs, sensor.stageStartedUs + sensorEchoWaitTimeoutUs(sensor))) {
        return finishSensorCycle(sensor, false, 0);
      }
      break;

    case SENSOR_WAIT_ECHO_INACTIVE:
      if (digitalRead(sensor.echoPin) != (echoActiveLevel ? HIGH : LOW)) {
        unsigned long pulseWidthUs = nowUs - sensor.echoStartedUs;
        long distanceMm = pulseWidthToDistanceMm(sensor, pulseWidthUs);
        return finishSensorCycle(sensor, true, distanceMm);
      }
      if (hasElapsed(nowUs, sensor.echoStartedUs + sensorEchoTimeoutUs(sensor))) {
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
  for (uint8_t attempt = 0; attempt < Config::DISTANCE_SENSOR_COUNT; ++attempt) {
    int candidate = (gNextSensorIndex + attempt) % Config::DISTANCE_SENSOR_COUNT;
    if (sensorHasDistancePins(gSensors[candidate]) && hasElapsed(nowMs, gSensors[candidate].nextTriggerAtMs)) {
      gActiveSensorIndex = candidate;
      gNextSensorIndex = (candidate + 1) % Config::DISTANCE_SENSOR_COUNT;
      beginSensorCycle(gSensors[candidate]);
      return;
    }
  }
}

// Возвращаем расстояние только если последнее измерение ещё не считается устаревшим.
bool sensorDistanceAvailable(int sensorIndex, long& distanceMm) {
  if (sensorIndex < 0 || sensorIndex >= Config::DISTANCE_SENSOR_COUNT) {
    return false;
  }
  DistanceSensorState& sensor = gSensors[sensorIndex];
  if (!sensorHasDistancePins(sensor) || !sensor.valid) {
    return false;
  }
  if (!hasElapsed(millis(), sensor.lastSuccessAtMs + Config::SENSOR_STALE_MS)) {
    distanceMm = sensor.lastDistanceMm;
    return true;
  }
  return false;
}

bool extractSensorIndex(long requestId, const char* payload, int& sensorIndex) {
  long sensorId = 0;
  if (!extractLongField(payload, "sensor", sensorId) || sensorId < 1 ||
      sensorId > static_cast<long>(Config::DISTANCE_SENSOR_COUNT)) {
    sendErrorResponse(requestId, "bad_request", "sensor must be in range 1..4");
    return false;
  }
  sensorIndex = static_cast<int>(sensorId) - 1;
  return true;
}

bool ensureUrm37SerialCommandSupported(long requestId, const DistanceSensorState& sensor) {
  if (!sensor.enabled || sensor.kind == DISTANCE_SENSOR_DISABLED || !sensorHasDistancePins(sensor)) {
    sendErrorResponse(requestId, "not_configured", "distance sensor slot is disabled");
    return false;
  }
  if (sensor.kind != DISTANCE_SENSOR_URM37) {
    sendErrorResponse(requestId, "unsupported", "selected sensor is not URM37");
    return false;
  }
  if (!sensorHasSerialPins(sensor)) {
    sendErrorResponse(requestId, "not_configured", "URM37 serial pins are not configured");
    return false;
  }
  return true;
}

void printDistanceSensorStatusEntry(uint8_t sensorIndex) {
  const DistanceSensorState& sensor = gSensors[sensorIndex];
  long distanceMm = 0;
  bool distanceValid = sensorDistanceAvailable(sensorIndex, distanceMm);

  Serial.print(F("\""));
  Serial.print(sensorIndex + 1);
  Serial.print(F("\":{\"enabled\":"));
  printBool(sensor.enabled && sensor.kind != DISTANCE_SENSOR_DISABLED);
  Serial.print(F(",\"kind\":\""));
  Serial.print(sensorKindName(sensor));
  Serial.print(F("\",\"pins\":{\"trigger\":"));
  printNullablePin(sensor.trigPin);
  Serial.print(F(",\"echo\":"));
  printNullablePin(sensor.echoPin);
  Serial.print(F(",\"serial_rx\":"));
  printNullablePin(sensor.serialRxPin);
  Serial.print(F(",\"serial_tx\":"));
  printNullablePin(sensor.serialTxPin);
  Serial.print(F("},\"serial_settings_available\":"));
  printBool(sensorHasSerialPins(sensor));
  Serial.print(F(",\"distance_mm\":"));
  printNullableDistance(distanceValid, distanceMm);
  Serial.print(F("}"));
}

// Сводный статус нужен для быстрой диагностики без отдельного вызова каждого датчика.
void sendStatusResponse(long requestId) {
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
  Serial.print(F("},\"distance_mm\":{"));
  for (uint8_t sensorIndex = 0; sensorIndex < Config::DISTANCE_SENSOR_COUNT; ++sensorIndex) {
    long distanceMm = 0;
    bool distanceValid = sensorDistanceAvailable(sensorIndex, distanceMm);
    if (sensorIndex > 0) {
      Serial.print(F(","));
    }
    Serial.print(F("\""));
    Serial.print(sensorIndex + 1);
    Serial.print(F("\":"));
    printNullableDistance(distanceValid, distanceMm);
  }
  Serial.print(F("},\"distance_sensors\":{"));
  for (uint8_t sensorIndex = 0; sensorIndex < Config::DISTANCE_SENSOR_COUNT; ++sensorIndex) {
    if (sensorIndex > 0) {
      Serial.print(F(","));
    }
    printDistanceSensorStatusEntry(sensorIndex);
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
  Serial.println(F(",\"ok\":true,\"data\":{\"pong\":true,\"firmware\":\"rescue_maze_low_level_v2\"}}"));
}

// Отдаём только одно конкретное расстояние, чтобы транспортный протокол был простым.
void handleDistanceRequest(long requestId, const char* payload) {
  int sensorIndex = 0;
  if (!extractSensorIndex(requestId, payload, sensorIndex)) {
    return;
  }
  uint8_t sensorId = static_cast<uint8_t>(sensorIndex + 1);
  DistanceSensorState& sensor = gSensors[sensorIndex];
  if (!sensor.enabled || sensor.kind == DISTANCE_SENSOR_DISABLED || !sensorHasDistancePins(sensor)) {
    sendErrorResponse(requestId, "not_configured", "distance sensor slot is disabled");
    return;
  }

  long distanceMm = 0;
  if (!sensorDistanceAvailable(sensorIndex, distanceMm)) {
    sendErrorResponse(requestId, "sensor_timeout", "distance measurement is unavailable");
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

void handleUrm37TemperatureRequest(long requestId, const char* payload) {
  int sensorIndex = 0;
  if (!extractSensorIndex(requestId, payload, sensorIndex)) {
    return;
  }

  DistanceSensorState& sensor = gSensors[sensorIndex];
  if (!ensureUrm37SerialCommandSupported(requestId, sensor)) {
    return;
  }

  float temperatureC = 0.0f;
  if (!readUrm37TemperatureC(sensor, temperatureC)) {
    sendErrorResponse(requestId, "sensor_timeout", "URM37 temperature is unavailable");
    return;
  }

  Serial.print(F("{\"id\":"));
  Serial.print(requestId);
  Serial.print(F(",\"ok\":true,\"data\":{\"sensor\":"));
  Serial.print(sensorIndex + 1);
  Serial.print(F(",\"temperature_c\":"));
  Serial.print(temperatureC, 1);
  Serial.println(F("}}"));
}

void handleUrm37SettingsRequest(long requestId, const char* payload) {
  int sensorIndex = 0;
  if (!extractSensorIndex(requestId, payload, sensorIndex)) {
    return;
  }

  DistanceSensorState& sensor = gSensors[sensorIndex];
  if (!ensureUrm37SerialCommandSupported(requestId, sensor)) {
    return;
  }

  if (!refreshUrm37SettingsFromSensor(sensor)) {
    sendErrorResponse(requestId, "sensor_timeout", "URM37 settings are unavailable");
    return;
  }

  sendUrm37SettingsResponse(requestId, static_cast<uint8_t>(sensorIndex + 1), sensor);
}

void handleUrm37SettingsUpdateRequest(long requestId, const char* payload) {
  int sensorIndex = 0;
  if (!extractSensorIndex(requestId, payload, sensorIndex)) {
    return;
  }

  DistanceSensorState& sensor = gSensors[sensorIndex];
  if (!ensureUrm37SerialCommandSupported(requestId, sensor)) {
    return;
  }

  bool hasAnyUpdate = false;

  char measureModeText[20];
  bool hasMeasureMode = extractStringField(payload, "measure_mode", measureModeText, sizeof(measureModeText));
  Urm37MeasureMode nextMeasureMode = sensor.urm37MeasureMode;
  if (hasMeasureMode) {
    hasAnyUpdate = true;
    if (strcmp(measureModeText, "pwm_passive") == 0) {
      nextMeasureMode = URM37_MEASURE_PWM_PASSIVE;
    } else if (strcmp(measureModeText, "auto") == 0) {
      nextMeasureMode = URM37_MEASURE_AUTO;
    } else {
      sendErrorResponse(requestId, "bad_request", "measure_mode must be pwm_passive or auto");
      return;
    }
  }

  long autoMeasureIntervalMs = 0;
  bool hasAutoMeasureInterval = extractLongField(payload, "auto_measure_interval_ms", autoMeasureIntervalMs);
  if (hasAutoMeasureInterval) {
    hasAnyUpdate = true;
    if (autoMeasureIntervalMs < 25 || autoMeasureIntervalMs > 255) {
      sendErrorResponse(requestId, "bad_request", "auto_measure_interval_ms must be in range 25..255");
      return;
    }
  }

  long compareDistanceCm = 0;
  bool hasCompareDistance = extractLongField(payload, "compare_distance_cm", compareDistanceCm);
  if (hasCompareDistance) {
    hasAnyUpdate = true;
    if (compareDistanceCm < 0 || compareDistanceCm > 1000) {
      sendErrorResponse(requestId, "bad_request", "compare_distance_cm must be in range 0..1000");
      return;
    }
  }

  long sensitivity = 0;
  bool hasSensitivity = extractLongField(payload, "sensitivity", sensitivity);
  if (hasSensitivity) {
    hasAnyUpdate = true;
    if (sensitivity < 10 || sensitivity > 200) {
      sendErrorResponse(requestId, "bad_request", "sensitivity must be in range 10..200");
      return;
    }
  }

  if (!hasAnyUpdate) {
    sendErrorResponse(requestId, "bad_request", "at least one URM37 setting must be provided");
    return;
  }

  if (hasCompareDistance) {
    uint16_t boundedCompareDistance = static_cast<uint16_t>(compareDistanceCm);
    if (!writeUrm37EepromByte(sensor, 0x00, static_cast<uint8_t>(boundedCompareDistance & 0xFFU)) ||
        !writeUrm37EepromByte(sensor, 0x01, static_cast<uint8_t>((boundedCompareDistance >> 8) & 0xFFU))) {
      sendErrorResponse(requestId, "sensor_timeout", "failed to write URM37 compare distance");
      return;
    }
    sensor.urm37CompareDistanceCm = boundedCompareDistance;
  }

  if (hasMeasureMode) {
    uint8_t modeByte = nextMeasureMode == URM37_MEASURE_AUTO ? 0xAA : 0xBB;
    if (!writeUrm37EepromByte(sensor, 0x02, modeByte)) {
      sendErrorResponse(requestId, "sensor_timeout", "failed to write URM37 measure mode");
      return;
    }
    sensor.urm37MeasureMode = nextMeasureMode;
  }

  if (hasAutoMeasureInterval) {
    if (!writeUrm37EepromByte(sensor, 0x04, static_cast<uint8_t>(autoMeasureIntervalMs))) {
      sendErrorResponse(requestId, "sensor_timeout", "failed to write URM37 auto interval");
      return;
    }
    sensor.urm37AutoIntervalMs = static_cast<uint8_t>(autoMeasureIntervalMs);
  }

  if (hasSensitivity) {
    sensor.urm37Sensitivity = static_cast<uint8_t>(sensitivity);
  }

  if (gActiveSensorIndex == sensorIndex) {
    gActiveSensorIndex = -1;
  }
  resetSensorRuntimeState(sensor, millis() + Config::SENSOR_SAMPLE_INTERVAL_MS);
  configureSensorPins(sensor);

  sendUrm37SettingsResponse(requestId, static_cast<uint8_t>(sensorIndex + 1), sensor);
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

  if (strcmp(operation, "get_urm37_temperature") == 0) {
    handleUrm37TemperatureRequest(requestId, payload);
    return;
  }

  if (strcmp(operation, "get_urm37_settings") == 0) {
    handleUrm37SettingsRequest(requestId, payload);
    return;
  }

  if (strcmp(operation, "set_urm37_settings") == 0) {
    handleUrm37SettingsUpdateRequest(requestId, payload);
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

  sanitizeSensorConfiguration();

  unsigned long nowMs = millis();
  for (uint8_t index = 0; index < Config::DISTANCE_SENSOR_COUNT; ++index) {
    DistanceSensorState& sensor = gSensors[index];
    resetSensorRuntimeState(sensor, nowMs + (index * Config::SENSOR_SAMPLE_INTERVAL_MS));
    if (!sensorHasDistancePins(sensor)) {
      continue;
    }
    configureSensorPins(sensor);
    if (sensorHasSerialPins(sensor)) {
      refreshUrm37SettingsFromSensor(sensor);
      resetSensorRuntimeState(sensor, nowMs + (index * Config::SENSOR_SAMPLE_INTERVAL_MS));
    }
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
