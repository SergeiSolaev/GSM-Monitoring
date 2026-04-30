/* ==========================================================================
   GSM СИГНАЛИЗАЦИЯ НА ATMEGA328P + SIM800L
   Мониторинг температуры (улица, дом, котёл) + управление по SMS
   Кварц: 8 МГц | Платформа: PlatformIO
   ========================================================================== */

// ==========================================
// 1. БИБЛИОТЕКИ И АППАРАТНЫЕ ОБЪЕКТЫ
// ==========================================
#include <SoftwareSerial.h>    // Программный UART для связи с SIM800L
#include <Wire.h>              // I2C шина (требуется ядром AVR, хотя DS18B20 использует OneWire)
#include <DallasTemperature.h> // Работа с цифровыми датчиками DS18B20
#include <GyverPower.h>        // Управление сном МК и калибровка WDT
// #include <GyverWDT.h>

#define FW_VERSION "0.1.1"

// Объекты связи
SoftwareSerial SerialSIM800L(6, 2);  // RX, TX пины для программного UART с SIM800L
OneWire oneWire(4);                  // Шина данных датчиков температуры
DallasTemperature sensors(&oneWire); // Контроллер датчиков

// Функция программного сброса МК (переход на адрес 0)
void (*resetFunc)(void) = 0;

// ==========================================
// 2. КОНСТАНТЫ И ПИНЫ
// ==========================================
const int PIN_DTR = 5;  // Пин управления режимом сна SIM800L (DTR)
const int PIN_LED = A1; // Пин светодиода индикации сброса

const char PHONE_NUMBER[] = "+79277749863";

// Пороговые значения температур для аварийных условий
const float TEMP_BOILER_MIN = 10.0; // Минимальная температура котла (°C)
const float TEMP_BOILER_MAX = 50.0; // Максимальная температура котла (°C)
const float TEMP_HOME_MIN = 5.0;    // Минимальная температура в доме (°C)
const float TEMP_HOME_MAX = 30.0;   // Максимальная температура в доме (°C)

// Таймеры и интервалы (в миллисекундах)
const unsigned long INTERVAL_DAILY = 41546016UL; // Интервал планового отчёта (~12 часов с коррекцией)
const unsigned long INTERVAL_ALARM = 60000UL;    // Интервал проверки аварийных условий (1 минута)
const unsigned long INTERVAL_SMS_POLL = 10000UL; // Интервал опроса входящих SMS (10 секунд)
const unsigned long ALARM_COOLDOWN = 300000UL;   // Период "охлаждения" повторных алармов (5 минут)

// Задержки (в миллисекундах)
const unsigned long DELAY_AT_COMMAND = 2000;      // Задержка после AT-команды
const unsigned long DELAY_LED_ON = 2000;          // Задержка включения светодиода при сбросе
const unsigned long DELAY_AFTER_SMS_SEND = 15000; // Задержка после отправки SMS перед звонком
const unsigned long DELAY_CALL_DURATION = 30000;  // Длительность звонка

// ==========================================
// 3. ТАЙМЕРЫ (ОТСЧЁТ ВРЕМЕНИ)
// ==========================================
unsigned long timerDaily = 0;  // Таймер планового отчёта (~12 часов)
unsigned long timerAlarm = 0;  // Таймер проверки аварийных условий (1 минута)
unsigned long timerSMS = 0;    // Таймер опроса буфера UART на наличие входящих SMS
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
void updateSerial();             // Прозрачный проброс данных между HardwareSerial и SoftwareSerial (для отладки)
void wakeSIM800L();              // Выключаем энергосбережение SIM800L
void sleepSIM800L();             // Включаем энергосбережение SIM800L
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
  // Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4с
  power.autoCalibrate(); // каллибровка WDT
  smsBuffer.reserve(160);
  msg.reserve(160);
  batLevel.reserve(32);
  power.setSleepMode(ADC_SLEEP); // наиболее глубокий сон, отключается всё кроме WDT и внешних прерываний, просыпается от аппаратных (обычных + PCINT) или WDT за 1000 тактов (62 мкс)
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_DTR, OUTPUT);
  sensors.begin();             // включаем датчики температуры
  SerialSIM800L.begin(9600);   // настройка скорости обмена данными с SIM800L
  SerialSIM800L.println("AT"); // установка соединения с SIM800L
  delay(DELAY_AT_COMMAND);
  SerialSIM800L.println("AT+CMGF=1"); // включаем TextMode для SMS
  delay(DELAY_AT_COMMAND);
  SerialSIM800L.println("AT+CNMI=1,2,0,0,0"); // устанавливаем режим обработки поступившие SMS. Данный режим сразу выводит поступившее SMS
  delay(DELAY_AT_COMMAND);
  SerialSIM800L.println("AT+CSCLK=1"); // включаем возможность работы энергосбеирежения
  delay(DELAY_AT_COMMAND);
  sleepSIM800L();
  delay(2000); // Задержка для стабилизации входов, что-бы не срабатывало ложное прерывание

  // Serial.begin(9600); // Для отладки по монитору порта
}

void loop()
{
  if (systemWorking == true)
  {
    daily(); // 12 часовое оповещение
    alarm(); // сигнализация, опрос раз в 1 минуту
  }
  receivingSMS();          // обработка входящих СМС
  power.sleepDelay(10000); // спим 10 секунд
  // Watchdog.reset(); // Переодический сброс watchdog, означающий, что устройство не зависло
}

void daily()
{

  if (millis() - timerDaily >= INTERVAL_DAILY)
  {
    timerDaily += INTERVAL_DAILY;
    if (!gsmLock())
      return;
    getAllTemperature();
    wakeSIM800L();
    getBatLevel();
    constructInfoMessage();
    sendSMS(msg);
    deleteAllSMS();
    sleepSIM800L();
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

    // Отправляем оповещение ТОЛЬКО при смене состояния + соблюдении кулдауна
    if ((homeAlarmState || boilerAlarmState) && (millis() - lastAlarmSentTime >= ALARM_COOLDOWN))
    {
      if (!gsmLock())
        return;
      lastAlarmSentTime = millis();
      wakeSIM800L();
      constructAlarmMessage();
      sendSMS(msg);
      deleteAllSMS();
      makeCall();
      clearBuffer();
      sleepSIM800L();
      gsmUnlock();
    }
  }
}

void makeCall()
{
  delay(1000);
  SerialSIM800L.print("ATD"); // Звоним
  SerialSIM800L.print(PHONE_NUMBER);
  SerialSIM800L.println(";");
  delay(DELAY_CALL_DURATION);
  SerialSIM800L.println("ATH"); // кладём трубку
  delay(DELAY_AT_COMMAND);
}

void receivingSMS()
{
  if (gsmBusy)
    return;
  if (millis() - timerSMS >= INTERVAL_SMS_POLL)
  {
    timerSMS += INTERVAL_SMS_POLL;
    if (SerialSIM800L.available())
    {
      smsBuffer = SerialSIM800L.readString(); // чтение ответа от модуля в переменную smsBuffer
      smsBuffer.replace("\n", "");            // замена символа переноса строки, что бы весь ответ был одной строкой и можно было выполнить её парсинг
      smsBuffer.trim();                       // удаляем пробелы вначале и вконце строки

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
}

void handleInfoCommand()
{
  if (!gsmLock())
    return;
  wakeSIM800L();
  getAllTemperature();
  getBatLevel();
  constructInfoMessage();
  sendSMS(msg);
  deleteAllSMS();
  sleepSIM800L();
  gsmUnlock();
}

void handleStartCommand()
{
  if (!gsmLock())
    return;
  systemWorking = true;
  wakeSIM800L();
  sendSMS("Start OK");
  deleteAllSMS();
  sleepSIM800L();
  gsmUnlock();
}

void handleStopCommand()
{
  if (!gsmLock())
    return;
  systemWorking = false;
  wakeSIM800L();
  sendSMS("Stop OK");
  deleteAllSMS();
  sleepSIM800L();
  gsmUnlock();
}

void wakeSIM800L()
{
  digitalWrite(PIN_DTR, LOW); // выключаем энергосбережение SIM800L
  delay(2000);
  SerialSIM800L.println("AT"); // установка соединения с SIM800L
  delay(DELAY_AT_COMMAND);
  clearBuffer();
}

void sleepSIM800L()
{
  digitalWrite(PIN_DTR, HIGH);
}

void deleteAllSMS()
{
  SerialSIM800L.println("AT+CMGDA=\"DEL ALL\"");
  delay(DELAY_AT_COMMAND);
  clearBuffer();
}

void clearBuffer()
{
  delay(200);
  while (SerialSIM800L.available())
  {
    SerialSIM800L.read();
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
  SerialSIM800L.println("AT+CBC"); // запрос состояния батареи
  delay(DELAY_AT_COMMAND);         // пауза для обработки модулем АТ-комнады
  if (SerialSIM800L.available())
  {                                        // проверка информации в буфере
    batLevel = SerialSIM800L.readString(); // чтение ответа от модуля в переменную batLevel
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
}

void constructInfoMessage()
{
  msg = "";
  msg += "Ambient temp ";
  msg += tAmbient;

  msg += " Home temp ";
  msg += tHome;

  msg += " Boiler temp ";
  msg += tBoiler;

  msg += " Bat ";
  msg += batLevel.substring(16, 18);
  msg += "%";
}

void constructAlarmMessage()
{
  msg = "";
  msg += "Warning!";

  if (boilerAlarmState)
  {
    msg += " Boiler temp ";
    msg += tBoiler;
  }
  if (homeAlarmState)
  {
    msg += " Home temp ";
    msg += tHome;
  }
}

void sendSMS(const String &msg)
{
  SerialSIM800L.print("AT+CMGS=\"");
  SerialSIM800L.print(PHONE_NUMBER);
  SerialSIM800L.println("\"");
  delay(DELAY_AT_COMMAND);
  SerialSIM800L.println(msg);
  delay(DELAY_AT_COMMAND);
  SerialSIM800L.write(26);
  delay(DELAY_AFTER_SMS_SEND);
  clearBuffer();
}

void updateSerial()
{
  // delay(500); // пауза 500 мс
  while (Serial.available())
  {
    SerialSIM800L.write(Serial.read()); // переадресация с последовательного порта SIM800L на последовательный порт Arduino IDE
  }
  while (SerialSIM800L.available())
  {
    Serial.write(SerialSIM800L.read()); // переадресация c Arduino IDE на последовательный порт SIM800L
  }
}
