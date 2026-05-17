/* ==========================================================================
   GSM СИГНАЛИЗАЦИЯ НА ATMEGA328P + SIM800L
   Мониторинг температуры (улица, дом, котёл) + управление по SMS
   Кварц: 8 МГц | Платформа: PlatformIO
   ========================================================================== */

// ==========================================
// 1. БИБЛИОТЕКИ И АППАРАТНЫЕ ОБЪЕКТЫ
// ==========================================
#include <EEPROM.h>
#include <HardwareSerial.h>    // Программный UART для связи с SIM800L
#include <OneWire.h>           // Шина OneWire для DS18B20
#include <DallasTemperature.h> // Работа с цифровыми датчиками DS18B20
#include <GyverWDT.h>          // Сторожевой пёс, охраняет от зависаний МК
#include "config.h"            // Номера телефонов 

#define FW_VERSION "0.4.2"


// Объекты связи
OneWire oneWire(4);                  // Шина данных датчиков температуры
DallasTemperature sensors(&oneWire); // Контроллер датчиков

// ==========================================
// 2. КОНСТАНТЫ И ПИНЫ
// ==========================================

// ==========================================
// EEPROM whitelist
// ==========================================

#define MAX_WHITELIST_NUMBERS 10
#define PHONE_LENGTH 16

#define EEPROM_MAGIC_ADDR 0
#define EEPROM_COUNT_ADDR 1
#define EEPROM_DATA_ADDR 2

#define EEPROM_MAGIC 0x55

// Пин hearbeat LED
#define HEARTBEAT_LED LED_BUILTIN

// Пороговые значения температур для аварийных условий
const float TEMP_BOILER_MIN = 10.0; // Минимальная температура котла (°C)
const float TEMP_BOILER_MAX = 50.0; // Максимальная температура котла (°C)
const float TEMP_HOME_MIN = 5.0;    // Минимальная температура в доме (°C)
const float TEMP_HOME_MAX = 30.0;   // Максимальная температура в доме (°C)
const int BAT_LEVEL_MIN = 20;       // Минимальный уровень заряда батареи, в %

// Таймеры и интервалы (в миллисекундах)
const unsigned long INTERVAL_DAILY = 41546016UL; // Интервал планового отчёта (~12 часов с коррекцией)
const unsigned long INTERVAL_ALARM = 60000UL;    // Интервал проверки аварийных условий (1 минута)
const unsigned long ALARM_COOLDOWN = 300000UL;   // Период "охлаждения" повторных алармов (5 минут)

// Задержки (в миллисекундах)
const unsigned long DELAY_AT_COMMAND = 200;       // Задержка после AT-команды
const unsigned long DELAY_AFTER_SMS_SEND = 10000; // Задержка после отправки SMS перед звонком
const unsigned long DELAY_CALL_DURATION = 20000;  // Длительность звонка

// ==========================================
// 3. ТАЙМЕРЫ (ОТСЧЁТ ВРЕМЕНИ)
// ==========================================
unsigned long timerAlarm = 0;  // Таймер проверки аварийных условий (1 минута)
unsigned long gsmLockTime = 0; // Защита от вечного gsmLock

// ==========================================
// 4. ПЕРЕМЕННЫЕ ТЕМПЕРАТУРЫ
// ==========================================
float tAmbient = 0.0;
float tHome = 0.0;
float tBoiler = 0.0;

// ==========================================
// 5. СТРОКИ И ТЕКСТЫ СООБЩЕНИЙ
// ==========================================
String batLevel;     // Буфер для ответа модуля об уровне заряда (+CBC)
int batPercent = -1; // Процент батареи, извлечённый из ответа +CBC
String signalLevel;  // Буфер для ответа модуля об уровне сигнала
int signalRssi = -1; // Уровень сигнала извлечённый из ответа
String smsBuffer;    // Буфер для входящего SMS
String currentSender;// Номер отправителя СМС
String msg;          // Исходящее сообщение
String gsmDateTime;  // Время из SIM800L
String currentDate;  // Время из SIM800L 
String currentTime;  // Время из SIM800L
String resetReason;  // Причина перезакгрузки

// ==========================================
// 6. Heartbeat LED
// ==========================================
bool fastBlinkMode = false;
bool fastBlinkInitialized = false;

unsigned long heartbeatTimer = 0;
unsigned long fastBlinkTimer = 0;
unsigned long bootBlinkTimer = 0;

bool ledState = false;

// ==========================================
// 6. RTC на Arduino
// ==========================================
unsigned long timeSyncMillis = 0;

uint8_t rtcHour = 0;
uint8_t rtcMinute = 0;
uint8_t rtcSecond = 0;

uint8_t rtcDay = 0;
uint8_t rtcMonth = 0;
uint8_t rtcYear = 0;

unsigned long lastClockSync = 0;
const unsigned long CLOCK_SYNC_INTERVAL = 604800000UL; // 7 дней

// ==========================================
// 6. ФЛАГИ СОСТОЯНИЯ СИСТЕМЫ
// ==========================================
boolean systemWorking = false;                    // Разрешение работы циклов daily() и alarm()
bool gsmBusy = false;                             // Флаг занятости SIM800
bool boilerAlarmState = false;                    // Флаг текущей аварии котла
bool homeAlarmState = false;                      // Флаг текущей аварии дома
bool batAlarmState = false;                       // Флаг разряда батареи
bool ambientSensorDisconnected = false;           // Флаг обрыва уличного датчика
bool homeSensorDisconnected = false;              // Флаг обрыва домашнего датчика
bool boilerSensorDisconnected = false;            // Флаг обрыва датчика котла
unsigned long lastAlarmSentTime = ALARM_COOLDOWN; // Время отправки последнего тревожного сообщения
                                                  // Инициализация = ALARM_COOLDOWN позволяет первому аларму сработать сразу

// ==========================================
// 7. ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
void daily();                                       // Отправка планового отчёта каждые ~12 часов
void alarm();                                       // Мониторинг аварийных температур, отправка SMS + авто-звонок
void receivingSMS();                                // Приём, парсинг и обработка входящих SMS-команд
void sendSMS(const String &number, const String &msg); // Отправка SMS
bool sendATCommand(const String &command, const String &expected, unsigned long timeout, String *response = nullptr); // Отправка AT-команды с ожиданием ответа
void clearBuffer();                                 // Полная очистка буфера программного UART
void deleteAllSMS();                                // Удаление всех смс
void constructInfoMessage();                        // Конструктор информационного сообщения
void constructAlarmMessage();                       // Конструктор предупредительного сообщения
void getBatLevel();                                 // Получение уровня заряда батареи
void getSignalLevel();                              // Получение уровня GSM сигнала
int parseSignalLevel(const String &response);       // Извлечение RSSI из ответа +CSQ
void handleInfoCommand();                           // Обработчик команды Info
void handleStartCommand();                          // Обработчик команды Start
void handleStopCommand();                           // Обработчик команды Stop
void makeCall();                                    // Совершить звонок
void getAllTemperature();                           // Опрос датчиков температуры
bool gsmLock();                                     // SIM800 занять
void gsmUnlock();                                   // SIM800 освободить
bool isWhitelistedSender(const String &smsPayload); // Проверка номера отправителя смс на наличие в "белом списке"
int parseBatteryPercent(const String &response);    // Извлечение процента батареи из ответа +CBC
void getDateTime();                                 // Получение даты и времени
bool parseDateTime(const String &response);         // Парсинг +CCLK
void syncInternalClock();                           // Синхронизация времени и даты с SIM800L
void updateInternalClock();                         // Обновление времени и даты внутри Arduino
bool isTimeValid();                                 // Проверка валидности времени и даты полученных от SIM800L
void detectResetReason();                           // Определение причины перезагрузки
void sendBootMessage();                             // SMS после запуска устройства
void heartbeatLED();
void fastBlinkHeartbeat();
void initWhitelist();
bool addWhitelistNumber(const String &number);
bool deleteWhitelistNumber(const String &number);
bool whitelistContains(const String &number);
bool readWhitelistNumber(uint8_t index, char *buffer);
void writeWhitelistNumber(uint8_t index, const String &number);
uint8_t getWhitelistCount();
void setWhitelistCount(uint8_t count);
void handleAddNumberCommand(const String &sms);
void handleDeleteNumberCommand(const String &sms);
void handleShowNumbersCommand();
bool isAdminSender(const String &smsPayload);
String extractPhoneNumber(const String &sms);
String readGsmResponse(unsigned long timeout);      // Чтение ответа модема в строку
String extractSenderNumber(const String &smsPayload);// Получение номера телефона отправителя

void setup()

{
  //EEPROM.write(EEPROM_MAGIC_ADDR, 0);  // Очистка EEPROM
  Watchdog.disable();
  detectResetReason();

  if (resetReason == "WATCHDOG" || resetReason == "BROWNOUT" || resetReason == "EXTERNAL RESET")
  {
    fastBlinkMode = true;
  }

  smsBuffer.reserve(160);
  msg.reserve(160);
  batLevel.reserve(32);
  signalLevel.reserve(32);

  sensors.begin();                                // включаем датчики температуры
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW);
  Serial.begin(9600);                             // настройка скорости обмена данными с SIM800L

  sendATCommand("AT", "OK", 1000);                // установка соединения с SIM800L
  sendATCommand("AT+CMGF=1", "OK", 1000);         // включаем TextMode для SMS
  sendATCommand("AT+CNMI=1,2,0,0,0", "OK", 1000); // устанавливаем режим обработки поступившие SMS
  sendATCommand("AT+CSCLK=0", "OK", 1000);        // отключаем возможность работы энергосбережения
  sendATCommand("AT+CLTS=1", "OK", 3000);         // включаем синхронизацию времени от сети
  // sendATCommand("AT&W", "OK", 3000);           // сохраняем настройки в EEPROM SIM800L. После сделать HARD RESET.

  syncInternalClock();

  initWhitelist();

  sendBootMessage();

  Watchdog.enable(RESET_MODE, WDT_TIMEOUT_8S);    // Включаем watchdog c таймаутом сброса 8 секунд
}

void loop()
{
  Watchdog.reset();

  if (fastBlinkMode)
  {
    fastBlinkHeartbeat();
  }
  else
  {
    heartbeatLED();
  }

  updateInternalClock();
  receivingSMS(); 
  daily();

  if (systemWorking == true)
  {
    alarm();
  }

  if (millis() - lastClockSync >= CLOCK_SYNC_INTERVAL)
  {
    if (gsmLock())
    {
      syncInternalClock();
      lastClockSync = millis();
      gsmUnlock();
    }
  }
}

void initWhitelist()
{
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC)
  {
    return;
  }

  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);

  EEPROM.write(EEPROM_COUNT_ADDR, 1);

  writeWhitelistNumber(0, PHONE_NUMBER);
}

uint8_t getWhitelistCount()
{
  return EEPROM.read(EEPROM_COUNT_ADDR);
}

void setWhitelistCount(uint8_t count)
{
  EEPROM.write(EEPROM_COUNT_ADDR, count);
}

void writeWhitelistNumber(uint8_t index, const String &number)
{
  int addr = EEPROM_DATA_ADDR + index * PHONE_LENGTH;

  for (uint8_t i = 0; i < PHONE_LENGTH; i++)
  {
    if (i < number.length())
      EEPROM.write(addr + i, number[i]);
    else
      EEPROM.write(addr + i, 0);
  }
}

bool readWhitelistNumber(uint8_t index, char *buffer)
{
  if (index >= getWhitelistCount())
    return false;

  int addr = EEPROM_DATA_ADDR + index * PHONE_LENGTH;

  for (uint8_t i = 0; i < PHONE_LENGTH; i++)
  {
    buffer[i] = EEPROM.read(addr + i);
  }

  buffer[PHONE_LENGTH - 1] = '\0';

  return true;
}

bool whitelistContains(const String &number)
{
  char buf[PHONE_LENGTH];

  for (uint8_t i = 0; i < getWhitelistCount(); i++)
  {
    readWhitelistNumber(i, buf);

    if (number == String(buf))
      return true;
  }

  return false;
}

bool addWhitelistNumber(const String &number)
{
  if (number.length() == 0)
    return false;

  if (whitelistContains(number))
    return false;

  uint8_t count = getWhitelistCount();

  if (count >= MAX_WHITELIST_NUMBERS)
    return false;

  writeWhitelistNumber(count, number);

  setWhitelistCount(count + 1);

  return true;
}

bool deleteWhitelistNumber(const String &number)
{
  if (number == String(PHONE_NUMBER))
  {
    return false;
  }

  uint8_t count = getWhitelistCount();

  char buf[PHONE_LENGTH];

  for (uint8_t i = 0; i < count; i++)
  {
    readWhitelistNumber(i, buf);

    if (number == String(buf))
    {
      for (uint8_t j = i; j < count - 1; j++)
      {
        char nextBuf[PHONE_LENGTH];

        readWhitelistNumber(j + 1, nextBuf);

        writeWhitelistNumber(j, String(nextBuf));
      }

      writeWhitelistNumber(count - 1, "");

      setWhitelistCount(count - 1);

      return true;
    }
  }

  return false;
}

String extractSenderNumber(const String &smsPayload)
{
  int senderStart = smsPayload.indexOf("\"+");

  if (senderStart < 0)
    return "";

  int senderEnd = smsPayload.indexOf("\"", senderStart + 1);

  if (senderEnd < 0)
    return "";

  return smsPayload.substring(senderStart + 1, senderEnd);
}

void heartbeatLED()
{
  if (millis() - heartbeatTimer >= 3000)
  {
    heartbeatTimer = millis();

    digitalWrite(HEARTBEAT_LED, HIGH);
    ledState = true;
  }

  if (ledState && millis() - heartbeatTimer >= 60)
  {
    digitalWrite(HEARTBEAT_LED, LOW);
    ledState = false;
  }
}

void fastBlinkHeartbeat()
{
  // Инициализация только после старта millis()
  if (!fastBlinkInitialized)
  {
    bootBlinkTimer = millis();
    fastBlinkTimer = millis();

    fastBlinkInitialized = true;
  }

  if (millis() - fastBlinkTimer >= 120)
  {
    fastBlinkTimer = millis();

    ledState = !ledState;
    digitalWrite(HEARTBEAT_LED, ledState);
  }

  // 5 секунд быстрого мигания
  if (millis() - bootBlinkTimer >= 5000)
  {
    fastBlinkMode = false;
    fastBlinkInitialized = false;

    digitalWrite(HEARTBEAT_LED, LOW);
  }
}

void flushGsmInput()
{
  while (Serial.available())
  {
    Serial.read();
  }
}

void detectResetReason()
{
  uint8_t resetFlags = MCUSR;

  MCUSR = 0;

  if (resetFlags & (1 << WDRF))
  {
    resetReason = "WATCHDOG";
  }
  else if (resetFlags & (1 << BORF))
  {
    resetReason = "BROWNOUT";
  }
  else if (resetFlags & (1 << EXTRF))
  {
    resetReason = "EXTERNAL RESET";
  }
  else if (resetFlags & (1 << PORF))
  {
    resetReason = "POWER ON";
  }
  else
  {
    resetReason = "UNKNOWN";
  }

  // WATCHDOG      -> зависание программы
  // BROWNOUT      -> просадка питания
  // EXTERNAL RESET-> reset pin
  // POWER ON      -> включение питания
}

void sendBootMessage()
{
  msg.remove(0);
  msg += "System rebooted";
  msg += "\nReason: ";
  msg += resetReason;

  sendSMS(PHONE_NUMBER, msg);
}

void syncInternalClock()
{
  getDateTime();

  if (!isTimeValid())
    return;

  rtcYear = currentDate.substring(0, 2).toInt();
  rtcMonth = currentDate.substring(3, 5).toInt();
  rtcDay = currentDate.substring(6, 8).toInt();

  rtcHour = currentTime.substring(0, 2).toInt();
  rtcMinute = currentTime.substring(3, 5).toInt();
  rtcSecond = currentTime.substring(6, 8).toInt();

  timeSyncMillis = millis();
}

bool isTimeValid()
{
  if (currentDate.startsWith("80/"))
    return false;

  if (currentTime == "N/A")
    return false;

  return true;
}

void updateInternalClock()
{
  unsigned long elapsedSeconds =
      (millis() - timeSyncMillis) / 1000UL;

  static unsigned long lastElapsed = 0;
  static uint16_t correctionCounter = 0;

  if (elapsedSeconds == lastElapsed)
    return;

  // сколько секунд реально прошло
  unsigned long delta = elapsedSeconds - lastElapsed;
  lastElapsed = elapsedSeconds;

  while (delta--)
  {
    rtcSecond++;

    // программная коррекция
    correctionCounter++;

    if (correctionCounter >= 288)
    {
      correctionCounter = 0;
      rtcSecond++; // добавляем компенсирующую секунду
    }

    if (rtcSecond >= 60)
    {
      rtcSecond -= 60;
      rtcMinute++;

      if (rtcMinute >= 60)
      {
        rtcMinute = 0;
        rtcHour++;

        if (rtcHour >= 24)
        {
          rtcHour = 0;
          rtcDay++;
        }
      }
    }
  }
}

String getFormattedTime()
{
  char buf[16];

  sprintf(buf, "%02u:%02u:%02u",
          rtcHour,
          rtcMinute,
          rtcSecond);

  return String(buf);
}

String getFormattedDate()
{
  char buf[16];

  sprintf(buf, "%02u/%02u/%02u",
          rtcYear,
          rtcMonth,
          rtcDay);

  return String(buf);
}

String readGsmResponse(unsigned long timeout)
{
  String response;
  unsigned long start = millis();
  unsigned long lastData = start;

  while (millis() - start < timeout)
  {
    Watchdog.reset();
    while (Serial.available())
    {
      response += (char)Serial.read();
      lastData = millis();
    }

    if (response.length() > 0 && (millis() - lastData) > 120)
    {
      break;
    }
  }

  response.replace("\r", "");
  response.trim();
  return response;
}

bool sendATCommand(const String &command, const String &expected, unsigned long timeout, String *response)
{
  flushGsmInput();
  Serial.println(command);

  String resp = readGsmResponse(timeout);

  if (response != nullptr)
  {
    *response = resp;
  }

  if (resp.length() == 0)
  {
    return false;
  }

  if (resp.indexOf("ERROR") >= 0)
  {
    return false;
  }

  if (expected.length() == 0)
  {
    return true;
  }

  return resp.indexOf(expected) >= 0;
}

int parseBatteryPercent(const String &response)
{
  int firstComma = response.indexOf(',');
  if (firstComma < 0)
    return -1;

  int secondComma = response.indexOf(',', firstComma + 1);
  if (secondComma < 0)
    return -1;

  return response.substring(firstComma + 1, secondComma).toInt();
}

int parseSignalLevel(const String &response)
{
  int colon = response.indexOf(':');
  if (colon < 0)
    return -1;

  int comma = response.indexOf(',', colon + 1);
  if (comma < 0)
    return -1;

  return response.substring(colon + 1, comma).toInt();
}

bool parseDateTime(const String &response)
{
  int firstQuote = response.indexOf('\"');
  int secondQuote = response.indexOf('\"', firstQuote + 1);

  if (firstQuote < 0 || secondQuote < 0)
    return false;

  String dt = response.substring(firstQuote + 1, secondQuote);

  // Формат:
  // "26/05/08,19:42:11+12"

  int commaIndex = dt.indexOf(',');

  if (commaIndex < 0)
    return false;

  currentDate = dt.substring(0, commaIndex);

  String timePart = dt.substring(commaIndex + 1);

  int zoneIndex = timePart.indexOf('+');

  if (zoneIndex < 0)
    zoneIndex = timePart.indexOf('-');

  if (zoneIndex > 0)
    currentTime = timePart.substring(0, zoneIndex);
  else
    currentTime = timePart;

  return true;
}

void daily()
{
  static bool sentMorning = false;
  static bool sentEvening = false;
  // ======================
  // 07:00 -> сброс флага ниже!!!
  // ======================
  if (rtcHour == 7 &&
      rtcMinute == 00 &&
      !sentMorning)
  {
    if (!gsmLock())
      return;
    getAllTemperature();
    getBatLevel();
    getSignalLevel();
    constructInfoMessage();
    sendSMS(PHONE_NUMBER, msg);
    deleteAllSMS();
    sentMorning = true;
    gsmUnlock();
  }
  // ======================
  // 20:00 -> сброс флага ниже!!!
  // ======================
  if (rtcHour == 20 &&
      rtcMinute == 0 &&
      !sentEvening)
  {
    if (!gsmLock())
      return;
    getAllTemperature();
    getBatLevel();
    getSignalLevel();
    constructInfoMessage();
    sendSMS(PHONE_NUMBER, msg);
    deleteAllSMS();
    sentEvening = true;
    gsmUnlock();
  }

  // ======================
  // Сброс флагов
  // ======================
  if (!(rtcHour == 7 && rtcMinute == 00))
    sentMorning = false;

  if (!(rtcHour == 20 && rtcMinute == 0))
    sentEvening = false;
}

void alarm()
{
  if (millis() - timerAlarm >= INTERVAL_ALARM)
  {
    getAllTemperature();
    getBatLevel();
    timerAlarm += INTERVAL_ALARM;

    // Определяем текущее состояние датчиков
    bool boilerFault = (tBoiler <= TEMP_BOILER_MIN || tBoiler >= TEMP_BOILER_MAX);
    bool homeFault = (tHome <= TEMP_HOME_MIN || tHome >= TEMP_HOME_MAX);
    bool batFault = (batPercent >= 0 && batPercent <= BAT_LEVEL_MIN);

    // Логика перехода в состояние аварии
    if (boilerFault)
    {
      boilerAlarmState = true;
    }
    else if (!boilerFault)
    {
      boilerAlarmState = false;
    }

    if (homeFault)
    {
      homeAlarmState = true;
    }
    else if (!homeFault)
    {
      homeAlarmState = false;
    }

    if (batFault)
    {
      batAlarmState = true;
    }
    else if (!batFault)
    {
      batAlarmState = false;
    }
    // Отправляем оповещение ТОЛЬКО при смене состояния + соблюдении кулдауна
    if ((homeAlarmState || boilerAlarmState || batAlarmState || ambientSensorDisconnected || homeSensorDisconnected || boilerSensorDisconnected) && (millis() - lastAlarmSentTime >= ALARM_COOLDOWN))
    {
      if (!gsmLock())
        return;
      lastAlarmSentTime = millis();
      getDateTime();
      constructAlarmMessage();
      sendSMS(PHONE_NUMBER, msg);
      deleteAllSMS();
      makeCall();
      clearBuffer();
      gsmUnlock();
    }
  }
}

void makeCall()
{
  delay(1000);

  String callCommand = "ATD";
  callCommand += PHONE_NUMBER;
  callCommand += ";";

  if (!sendATCommand(callCommand, "OK", 5000))
  {
    return;
  }

  unsigned long start = millis();

  while (millis() - start < DELAY_CALL_DURATION)
  {
    Watchdog.reset();
    delay(100);
  }
  sendATCommand("ATH", "OK", 3000); // кладём трубку
}

void receivingSMS()
{
  if (gsmBusy)
    return;

  if (Serial.available())
  {
    smsBuffer = Serial.readString(); // чтение ответа от модуля в переменную smsBuffer
    smsBuffer.replace("\r", "");     // замена символа возврат каретки
    smsBuffer.trim();                // удаляем пробелы вначале и вконце строки
    currentSender = extractSenderNumber(smsBuffer); 

    if (!isWhitelistedSender(smsBuffer))
    {
      deleteAllSMS();
      smsBuffer.remove(0);
      return;
    }

    // Маршрутизация команд
    if (smsBuffer.indexOf("Add number") >= 0)
    {
      if (isAdminSender(smsBuffer))
      {
        handleAddNumberCommand(smsBuffer);
      }
      else
      {
        sendSMS(currentSender, "Access denied");
        deleteAllSMS();
      }
    }
    else if (smsBuffer.indexOf("Del number") >= 0)
    {
      if (isAdminSender(smsBuffer))
      {
        handleDeleteNumberCommand(smsBuffer);
      }
      else
      {
        sendSMS(currentSender, "Access denied");
        deleteAllSMS();
      }
    }
    else if (smsBuffer.indexOf("Show numbers") >= 0)
    {
      if (isAdminSender(smsBuffer))
      {
        handleShowNumbersCommand();
      }
      else
      {
        sendSMS(currentSender, "Access denied");
        deleteAllSMS();
      }
    }
    else if (smsBuffer.endsWith("Info"))
    {
      handleInfoCommand();
    }
    else if (smsBuffer.endsWith("Start"))
    {
      handleStartCommand();
    }
    else if (smsBuffer.endsWith("Stop"))
    {
      handleStopCommand();
    }
    else
    {
      // Неизвестная команда: просто очищаем память SIM800
      deleteAllSMS();
    }
    smsBuffer.remove(0); // Сброс буфера после обработки
  }
}

bool isWhitelistedSender(const String &smsPayload)
{
  return whitelistContains(extractSenderNumber(smsPayload));
}

bool isAdminSender(const String &smsPayload)
{
  return extractSenderNumber(smsPayload) == String(PHONE_NUMBER);
}

String extractPhoneNumber(const String &sms)
{
  int cmdIndex = sms.indexOf("number");

  if (cmdIndex < 0)
    return "";

  int plusIndex = sms.indexOf('+', cmdIndex);

  if (plusIndex < 0)
    return "";

  int endIndex = sms.indexOf('\n', plusIndex);

  String number;

  if (endIndex > plusIndex)
    number = sms.substring(plusIndex, endIndex);
  else
    number = sms.substring(plusIndex);

  number.trim();

  // защита от мусора
  for (uint8_t i = 0; i < number.length(); i++)
  {
    char c = number[i];

    if (!(c == '+' || (c >= '0' && c <= '9')))
    {
      number = number.substring(0, i);
      break;
    }
  }

  return number;
}

void handleAddNumberCommand(const String &sms)
{
  if (!gsmLock())
    return;

  String number = extractPhoneNumber(sms);

  if (number.length() == 0)
  {
    sendSMS(currentSender, "Whitelist:\nAdd failed");
    gsmUnlock();
    return;
  }

  if (addWhitelistNumber(number))
  {
    sendSMS(currentSender, "Whitelist:\nAdded " + number);
  }
  else
  {
    sendSMS(currentSender, "Whitelist:\nAdd failed");
  }

  deleteAllSMS();
  gsmUnlock();
}

void handleDeleteNumberCommand(const String &sms)
{
  if (!gsmLock())
    return;

  String number = extractPhoneNumber(sms);

  if (number.length() == 0)
  {
    sendSMS(currentSender, "Whitelist:\nDelete failed");
    gsmUnlock();
    return;
  }

  if (deleteWhitelistNumber(number))
  {
    sendSMS(currentSender, "Whitelist:\n Deleted " + number);
  }
  else
  {
    sendSMS(currentSender, "Whitelist:\nDelete failed");
  }

  deleteAllSMS();
  gsmUnlock();
}

void handleShowNumbersCommand()
{
  if (!gsmLock())
    return;

  msg = "Whitelist:\n";

  char buf[PHONE_LENGTH];

  uint8_t count = getWhitelistCount();

  for (uint8_t i = 0; i < count; i++)
  {
    readWhitelistNumber(i, buf);

    msg += String(i + 1);
    msg += ". ";
    msg += String(buf);
    msg += "\n";
  }

  sendSMS(currentSender, msg);

  deleteAllSMS();

  gsmUnlock();
}

void handleInfoCommand()
{
  if (!gsmLock())
    return;
  getAllTemperature();
  getBatLevel();
  getSignalLevel();
  getDateTime();
  constructInfoMessage();
  sendSMS(currentSender, msg);
  deleteAllSMS();
  gsmUnlock();
}

void handleStartCommand()
{
  if (!gsmLock())
    return;
  systemWorking = true;
  sendSMS(currentSender, "Start OK");
  deleteAllSMS();
  gsmUnlock();
}

void handleStopCommand()
{
  if (!gsmLock())
    return;
  systemWorking = false;
  sendSMS(currentSender, "Stop OK");
  deleteAllSMS();
  gsmUnlock();
}

void deleteAllSMS()
{
  sendATCommand("AT+CMGDA=\"DEL ALL\"", "OK", 5000);
  clearBuffer();
}

void clearBuffer()
{
  delay(50);
  flushGsmInput();
}

bool gsmLock()
{
  if (gsmBusy)
  {
    // защита от зависания
    if (millis() - gsmLockTime > 30000)
    {
      gsmBusy = false;
    }
    else
    {
      return false;
    }
  }
  gsmBusy = true;
  gsmLockTime = millis();
  return true;
}

void gsmUnlock()
{
  gsmBusy = false;
}

void getBatLevel()
{
  if (sendATCommand("AT+CBC", "+CBC:", 2000, &batLevel))
  {
    batPercent = parseBatteryPercent(batLevel);
  }
  else
  {
    batLevel.remove(0);
    batPercent = -1;
  }
  clearBuffer();
}

void getSignalLevel()
{
  if (sendATCommand("AT+CSQ", "+CSQ:", 2000, &signalLevel))
  {
    signalRssi = parseSignalLevel(signalLevel);
  }
  else
  {
    signalLevel.remove(0);
    signalRssi = -1;
  }

  clearBuffer();
}

void getDateTime()
{
  if (sendATCommand("AT+CCLK?", "+CCLK:", 3000, &gsmDateTime))
  {
    if (!parseDateTime(gsmDateTime))
    {
      currentDate = "N/A";
      currentTime = "N/A";
    }
  }
  else
  {
    currentDate = "N/A";
    currentTime = "N/A";
  }

  clearBuffer();
}

void getAllTemperature()
{
  sensors.requestTemperatures();
  tAmbient = sensors.getTempCByIndex(0);
  tHome = sensors.getTempCByIndex(1);
  tBoiler = sensors.getTempCByIndex(2);

  ambientSensorDisconnected = (tAmbient == DEVICE_DISCONNECTED_C);
  homeSensorDisconnected = (tHome == DEVICE_DISCONNECTED_C);
  boilerSensorDisconnected = (tBoiler == DEVICE_DISCONNECTED_C);
}

void constructInfoMessage()
{
  msg.remove(0);

  msg += "Date ";
  msg += getFormattedDate();

  msg += "\nTime ";
  msg += getFormattedTime();
  msg += "\n\n";

  msg += "Ambient temp ";
  if (ambientSensorDisconnected)
    msg += "Error";
  else
    msg += tAmbient;

  msg += "\nHome temp ";
  if (homeSensorDisconnected)
    msg += "Error";
  else
    msg += tHome;

  msg += "\nBoiler temp ";
  if (boilerSensorDisconnected)
    msg += "Error";
  else
    msg += tBoiler;

  msg += "\nBat ";
  if (batPercent >= 0)
  {
    msg += batPercent;
    msg += "%";
  }
  else
  {
    msg += "Error";
  }

  msg += "\nSignal ";
  if (signalRssi >= 0)
  {
    msg += signalRssi;
    msg += "/31";

    // Оценка качества сигнала:
    if (signalRssi <= 9) // 0-9   = очень плохой сигнал
      msg += " Very bad";
    else if (signalRssi <= 14) // 10-14 = слабый сигнал
      msg += " Weak";
    else if (signalRssi <= 19) // 15-19 = нормальный сигнал
      msg += " Normal";
    else if (signalRssi <= 24) // 20-24 = хороший сигнал
      msg += " Good";
    else
      msg += " Excellent"; // 25-31 = отличный сигнал
  }
  else
  {
    msg += "Error";
  }

  msg += "\nStatus";
  if (systemWorking)
    msg += " working.";
  if (!systemWorking)
    msg += " stoped";

  msg += "\nFW ";
  msg += FW_VERSION;
}

void constructAlarmMessage()
{
  msg.remove(0);

  msg += "Date ";
  msg += getFormattedDate();

  msg += "\nTime ";
  msg += getFormattedTime();
  msg += "\n\n";

  msg += "Warning!";

  if (boilerSensorDisconnected)
  {
    msg += "\nBoiler sensor error";
  }
  else if (boilerAlarmState)
  {
    msg += "\nBoiler temp ";
    msg += tBoiler;
  }

  if (homeSensorDisconnected)
  {
    msg += "\nHome sensor break";
  }
  else if (homeAlarmState)
  {
    msg += "\nHome temp ";
    msg += tHome;
  }

  if (batAlarmState)
  {
    msg += "\nBat ";
    if (batPercent >= 0)
    {
      msg += batPercent;
      msg += "%";
    }
    else
    {
      msg += "Error";
    }
  }
}

void sendSMS(const String &number, const String &msg)
{
  String smsCommand = "AT+CMGS=\"";

  smsCommand += number;
  smsCommand += "\"";

  if (!sendATCommand(smsCommand, ">", 5000))
  {
    return;
  }

  Serial.print(msg);

  Serial.write(26);

  String response = readGsmResponse(DELAY_AFTER_SMS_SEND);

  if (response.indexOf("OK") < 0 ||
      response.indexOf("+CMGS:") < 0)
  {
    clearBuffer();
    return;
  }

  clearBuffer();
}