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
const int PIN_DTR = 5;     // Пин управления режимом сна SIM800L (DTR)
const int PIN_LED = A1;    // Пин светодиода индикации сброса
const int PIN_BTN_RST = 3; // Пин кнопки пробуждения/сброса (с внешним подтягивающим резистором)
const char PHONE_NUMBER[] = "+79277749863";

// Индексы датчиков температуры
const int IDX_SENSOR_AMBIENT = 0; // Уличный датчик
const int IDX_SENSOR_HOME = 1;    // Домашний датчик
const int IDX_SENSOR_BOILER = 2;  // Датчик котла

// Пороговые значения температур для аварийных условий
const float TEMP_BOILER_MIN = 10.0; // Минимальная температура котла (°C)
const float TEMP_BOILER_MAX = 50.0; // Максимальная температура котла (°C)
const float TEMP_HOME_MIN = 5.0;    // Минимальная температура в доме (°C)
const float TEMP_HOME_MAX = 30.0;   // Максимальная температура в доме (°C)

// Таймеры и интервалы (в миллисекундах)
const unsigned long INTERVAL_DAILY = 41546016UL; // Интервал планового отчёта (~12 часов с коррекцией)
const unsigned long INTERVAL_ALARM = 60000UL;    // Интервал проверки аварийных условий (1 минута)
const unsigned long INTERVAL_SMS_POLL = 10000UL; // Интервал опроса входящих SMS (10 секунд)

// Задержки (в миллисекундах)
const unsigned long DELAY_AT_COMMAND = 100;       // Задержка после AT-команды
const unsigned long DELAY_LED_ON = 2000;          // Задержка включения светодиода при сбросе
const unsigned long DELAY_AFTER_SMS_SEND = 15000; // Задержка после отправки SMS перед звонком
const unsigned long DELAY_CALL_DURATION = 10000;  // Длительность звонка

// ==========================================
// 3. ТАЙМЕРЫ (ОТСЧЁТ ВРЕМЕНИ)
// ==========================================
unsigned long timerDaily = 0; // Таймер планового отчёта (~12 часов)
unsigned long timerAlarm = 0; // Таймер проверки аварийных условий (1 минута)
unsigned long timerSMS = 0;   // Таймер опроса буфера UART на наличие входящих SMS

// ==========================================
// 4. СТРОКИ И ТЕКСТЫ СООБЩЕНИЙ
// ==========================================
String txtAmbient = "Ambient temp "; // Префикс уличной температуры
String txtHome = " Home temp ";      // Префикс температуры в доме
String txtBoiler = " Boiler temp ";  // Префикс температуры котла
String txtBat = " Bat ";             // Префикс заряда батареи
String txtWarning = "Warning!";      // Текст аварийного предупреждения
String batLevel = "";                // Буфер для ответа модуля об уровне заряда (+CBC)
String smsBuffer = "";               // Буфер для входящего SMS
String msg = "";                     // Исходящее сообщение

// ==========================================
// 5. ФЛАГИ СОСТОЯНИЯ СИСТЕМЫ
// ==========================================
boolean systemWorking = false;    // Разрешение работы циклов daily() и alarm()
volatile boolean btnFlag = false; // Флаг нажатия кнопки (обработчик прерывания)

// ==========================================
// 6. ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
void button_reset();             // Обработчик внешнего прерывания по кнопке сброса
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

void setup()

{
  // Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4с
  power.autoCalibrate(); // каллибровка WDT
  smsBuffer.reserve(160);
  msg.reserve(160);
  batLevel.reserve(32);
  power.setSleepMode(ADC_SLEEP); // наиболее глубокий сон, отключается всё кроме WDT и внешних прерываний, просыпается от аппаратных (обычных + PCINT) или WDT за 1000 тактов (62 мкс)
  pinMode(PIN_BTN_RST, INPUT_PULLUP);
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
  attachInterrupt(1, button_reset, FALLING);
  // Serial.begin(9600); // Для отладки по монитору порта
}

void button_reset()
{
  btnFlag = 1;
  power.wakeUp();
}

void loop()
{
  if (btnFlag == 1)
  {
    digitalWrite(PIN_LED, HIGH);
    delay(DELAY_LED_ON);
    sensors.requestTemperatures();
    wakeSIM800L();
    getBatLevel();
    constructInfoMessage();
    sendSMS(msg);
    deleteAllSMS();
    sleepSIM800L();
    btnFlag = 0;
    digitalWrite(PIN_LED, LOW);
    resetFunc(); // При нажатии кнопки даёт точку отсчёта времени для выполнения daily();
    // updateSerial(); // Для отладки по монитору порта
  }

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
    sensors.requestTemperatures();
    wakeSIM800L();
    getBatLevel();
    constructInfoMessage();
    sendSMS(msg);
    deleteAllSMS();
    sleepSIM800L();
  }
}

void alarm()
{
  if (millis() - timerAlarm >= INTERVAL_ALARM)
  {
    timerAlarm += INTERVAL_ALARM;
    sensors.requestTemperatures();
    // Проверка выхода температуры за диапазон аварийных значений
    if (sensors.getTempCByIndex(IDX_SENSOR_BOILER) <= TEMP_BOILER_MIN || sensors.getTempCByIndex(IDX_SENSOR_BOILER) >= TEMP_BOILER_MAX || sensors.getTempCByIndex(IDX_SENSOR_HOME) <= TEMP_HOME_MIN || sensors.getTempCByIndex(IDX_SENSOR_HOME) >= TEMP_HOME_MAX)
    {
      wakeSIM800L();
      constructAlarmMessage();
      sendSMS(msg);
      deleteAllSMS();
      makeCall();
      clearBuffer();
      sleepSIM800L();
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
}

void receivingSMS()
{

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
        // Неизвестная команда: просто очищаем память SIM
        deleteAllSMS();
      }
      smsBuffer = ""; // Сброс буфера после обработки
    }
  }
}

void handleInfoCommand()
{
  sensors.requestTemperatures();
  wakeSIM800L();
  getBatLevel();
  constructInfoMessage();
  sendSMS(msg);
  deleteAllSMS();
  sleepSIM800L();
}

void handleStartCommand()
{
  systemWorking = true;
  wakeSIM800L();
  sendSMS("Start OK");
  deleteAllSMS();
  sleepSIM800L();
}

void handleStopCommand()
{
  systemWorking = false;
  wakeSIM800L();
  sendSMS("Stop OK");
  deleteAllSMS();
  sleepSIM800L();
}

void wakeSIM800L()
{
  digitalWrite(PIN_DTR, LOW); // выключаем энергосбережение SIM800L
  delay(100);
  SerialSIM800L.println("AT"); // установка соединения с SIM800L
  clearBuffer();
}

void sleepSIM800L()
{
  digitalWrite(PIN_DTR, HIGH);
}

void deleteAllSMS()
{
  SerialSIM800L.println("AT+CMGDA=\"DEL ALL\"");
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

void constructInfoMessage()
{
  msg = "";
  msg += txtAmbient;
  msg += sensors.getTempCByIndex(IDX_SENSOR_AMBIENT);

  msg += txtHome;
  msg += sensors.getTempCByIndex(IDX_SENSOR_HOME);

  msg += txtBoiler;
  msg += sensors.getTempCByIndex(IDX_SENSOR_BOILER);

  msg += txtBat;
  msg += batLevel.substring(16, 18);
  msg += "%";
}

void constructAlarmMessage()
{
  msg = "";
  msg += txtWarning;

  msg += txtHome;
  msg += sensors.getTempCByIndex(IDX_SENSOR_HOME);

  msg += txtBoiler;
  msg += sensors.getTempCByIndex(IDX_SENSOR_BOILER);
}

void sendSMS(const String &msg)
{
  SerialSIM800L.print("AT+CMGS=\"");
  SerialSIM800L.print(PHONE_NUMBER);
  SerialSIM800L.println("\"");
  clearBuffer();
  SerialSIM800L.println(msg);
  clearBuffer();
  SerialSIM800L.write(26);
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
