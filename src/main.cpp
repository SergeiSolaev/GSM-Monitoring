/* ==========================================================================
   GSM СИГНАЛИЗАЦИЯ НА ATMEGA328P + SIM800L
   Мониторинг температуры (улица, дом, котёл) + управление по SMS
   Кварц: 8 МГц | Платформа: PlatformIO
   ========================================================================== */

// ==========================================
// 1. БИБЛИОТЕКИ И АППАРАТНЫЕ ОБЪЕКТЫ
// ==========================================
#include <HardwareSerial.h>    // Программный UART для связи с SIM800L
#include <Wire.h>              // I2C шина (требуется ядром AVR, хотя DS18B20 использует OneWire)
#include <DallasTemperature.h> // Работа с цифровыми датчиками DS18B20

#define FW_VERSION "0.2.0"

// Объекты связи
OneWire oneWire(4);                  // Шина данных датчиков температуры
DallasTemperature sensors(&oneWire); // Контроллер датчиков

// ==========================================
// 2. КОНСТАНТЫ И ПИНЫ
// ==========================================

const char PHONE_NUMBER[] = "+79277749863";

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
unsigned long timerDaily = 0;  // Таймер планового отчёта (~12 часов)
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
String batLevel = "";  // Буфер для ответа модуля об уровне заряда (+CBC)
String smsBuffer = ""; // Буфер для входящего SMS
String msg = "";       // Исходящее сообщение

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
void daily();                    // Отправка планового отчёта каждые ~12 часов
void alarm();                    // Мониторинг аварийных температур, отправка SMS + авто-звонок
void receivingSMS();             // Приём, парсинг и обработка входящих SMS-команд
void sendSMS(const String &msg); // Отправка SMS
void clearBuffer();              // Полная очистка буфера программного UART
void deleteAllSMS();             // Удаление всех смс
void constructInfoMessage();     // Конструктор информационного сообщения
void constructAlarmMessage();    // Конструктор предупредительного сообщения
void getBatLevel();              // Получение уровня заряда батареи
void handleInfoCommand();        // Обработчик команды Info
void handleStartCommand();       // Обработчик команды Start
void handleStopCommand();        // Обработчик команды Stop
void makeCall();                 // Совершить звонок
void getAllTemperature();        // Опрос датчиков температуры
bool gsmLock();                  // SIM800 занять
void gsmUnlock();                // SIM800 освободить

void setup()

{
  smsBuffer.reserve(160);
  msg.reserve(160);
  batLevel.reserve(32);
  sensors.begin();      // включаем датчики температуры
  Serial.begin(9600);   // настройка скорости обмена данными с SIM800L
  Serial.println("AT"); // установка соединения с SIM800L
  delay(DELAY_AT_COMMAND);
  Serial.println("AT+CMGF=1"); // включаем TextMode для SMS
  delay(DELAY_AT_COMMAND);
  Serial.println("AT+CNMI=1,2,0,0,0"); // устанавливаем режим обработки поступившие SMS. Данный режим сразу выводит поступившее SMS
  delay(DELAY_AT_COMMAND);
  Serial.println("AT+CSCLK=0"); // отключаем возможность работы энергосбережения
  delay(DELAY_AT_COMMAND);
}

void loop()
{
  receivingSMS(); // обработка входящих СМС
  if (systemWorking == true)
  {
    daily(); // 12 часовое оповещение
    alarm(); // сигнализация, опрос раз в 1 минуту
  }
}

void daily()
{

  if (millis() - timerDaily >= INTERVAL_DAILY)
  {
    timerDaily += INTERVAL_DAILY;
    if (!gsmLock())
      return;
    getAllTemperature();
    getBatLevel();
    constructInfoMessage();
    sendSMS(msg);
    deleteAllSMS();
    gsmUnlock();
  }
}

void alarm()
{
  if (millis() - timerAlarm >= INTERVAL_ALARM)
  {
    getAllTemperature();
    timerAlarm += INTERVAL_ALARM;

    // Определяем текущее состояние датчиков
    bool boilerFault = (tBoiler <= TEMP_BOILER_MIN || tBoiler >= TEMP_BOILER_MAX);
    bool homeFault = (tHome <= TEMP_HOME_MIN || tHome >= TEMP_HOME_MAX);
    bool batFault = (batLevel.toInt() <= BAT_LEVEL_MIN);

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
      constructAlarmMessage();
      sendSMS(msg);
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
  Serial.print("ATD");
  Serial.print(PHONE_NUMBER);
  Serial.println(";");
  delay(DELAY_CALL_DURATION);
  Serial.println("ATH"); // кладём трубку
  delay(DELAY_AT_COMMAND);
}

void receivingSMS()
{
  if (gsmBusy)
    return;

  if (Serial.available())
  {
    smsBuffer = Serial.readString(); // чтение ответа от модуля в переменную smsBuffer
    smsBuffer.replace("\n", "");     // замена символа переноса строки, что бы весь ответ был одной строкой и можно было выполнить её парсинг
    smsBuffer.trim();                // удаляем пробелы вначале и вконце строки

    // Маршрутизация команд
    if (smsBuffer.endsWith("Info"))
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
    smsBuffer = ""; // Сброс буфера после обработки
  }
}

void handleInfoCommand()
{
  if (!gsmLock())
    return;
  getAllTemperature();
  getBatLevel();
  constructInfoMessage();
  sendSMS(msg);
  deleteAllSMS();
  gsmUnlock();
}

void handleStartCommand()
{
  if (!gsmLock())
    return;
  systemWorking = true;
  sendSMS("Start OK");
  deleteAllSMS();
  gsmUnlock();
}

void handleStopCommand()
{
  if (!gsmLock())
    return;
  systemWorking = false;
  sendSMS("Stop OK");
  deleteAllSMS();
  gsmUnlock();
}

void deleteAllSMS()
{
  Serial.println("AT+CMGDA=\"DEL ALL\"");
  delay(DELAY_AT_COMMAND);
  clearBuffer();
}

void clearBuffer()
{
  delay(200);
  while (Serial.available())
  {
    Serial.read();
  }
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
  Serial.println("AT+CBC"); // запрос состояния батареи
  delay(DELAY_AT_COMMAND);  // пауза для обработки модулем АТ-комнады
  if (Serial.available())
  {                                 // проверка информации в буфере
    batLevel = Serial.readString(); // чтение ответа от модуля в переменную batLevel
  }
  batLevel.replace("\n", ""); // замена символа переноса строки, что бы весь ответ был одной строкой и можно было выполнить её парсинг
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
  msg = "";
  msg += "Ambient temp ";
  if (ambientSensorDisconnected)
    msg += "SENSOR BREAK";
  else
    msg += tAmbient;

  msg += " | Home temp ";
  if (homeSensorDisconnected)
    msg += "SENSOR BREAK";
  else
    msg += tHome;

  msg += " | Boiler temp ";
  if (boilerSensorDisconnected)
    msg += "SENSOR BREAK";
  else
    msg += tBoiler;

  msg += " | Bat ";
  msg += batLevel.substring(16, 18);
  msg += "%";

  msg += " | Status";
  if (systemWorking)
    msg += " working.";
  if (!systemWorking)
    msg += " stoped.";
}

void constructAlarmMessage()
{
  msg = "";
  msg += "Warning!";

  if (boilerSensorDisconnected)
  {
    msg += " | Boiler sensor break";
  }
  else if (boilerAlarmState)
  {
    msg += " | Boiler temp ";
    msg += tBoiler;
  }

  if (homeSensorDisconnected)
  {
    msg += " | Home sensor break";
  }
  else if (homeAlarmState)
  {
    msg += " | Home temp ";
    msg += tHome;
  }

  if (batAlarmState)
  {
    msg += " | Bat ";
    msg += batLevel;
  }
}

void sendSMS(const String &msg)
{
  Serial.print("AT+CMGS=\"");
  Serial.print(PHONE_NUMBER);
  Serial.println("\"");
  delay(DELAY_AT_COMMAND);
  Serial.println(msg);
  delay(DELAY_AT_COMMAND);
  Serial.write(26);
  delay(DELAY_AFTER_SMS_SEND);
  clearBuffer();
}