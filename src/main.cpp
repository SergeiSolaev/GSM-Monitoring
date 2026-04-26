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
SoftwareSerial mySerial(6, 2);       // RX, TX пины для программного UART с SIM800L
OneWire oneWire(4);                  // Шина данных датчиков температуры
DallasTemperature sensors(&oneWire); // Контроллер датчиков

// Функция программного сброса МК (переход на адрес 0)
void (*resetFunc)(void) = 0;

// ==========================================
// 2. КОНСТАНТЫ И ПИНЫ
// ==========================================
const int PIN_ONEWIRE = 4; // Пин шины данных датчиков температуры
const int PIN_DTR = 5;     // Пин управления режимом сна SIM800L (DTR)
const int PIN_LED = A1;    // Пин светодиода индикации сброса
const int PIN_BTN_RST = 3; // Пин кнопки пробуждения/сброса (с внешним подтягивающим резистором)
const char PHONE_NUMBER[] = "+79277749863";

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

// ==========================================
// 5. ФЛАГИ СОСТОЯНИЯ СИСТЕМЫ
// ==========================================
boolean systemWorking = true;     // Разрешение работы циклов daily() и alarm()
volatile boolean btnFlag = false; // Флаг нажатия кнопки (обработчик прерывания)

// ==========================================
// 6. ПРОТОТИПЫ ФУНКЦИЙ
// ==========================================
void button_reset(); // Обработчик внешнего прерывания по кнопке сброса
void daily();        // Отправка планового отчёта каждые ~12 часов
void alarm();        // Мониторинг аварийных температур, отправка SMS + авто-звонок
void receivingSMS(); // Приём, парсинг и обработка входящих SMS-команд
void sendSMS();      // Формирование и отправка SMS с текущими показаниями
void clearBuffer();  // Полная очистка буфера программного UART
void updateSerial(); // Прозрачный проброс данных между HardwareSerial и SoftwareSerial (для отладки)

void setup()
{

  // Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4с
  power.autoCalibrate();         // каллибровка WDT
  power.setSleepMode(ADC_SLEEP); // наиболее глубокий сон, отключается всё кроме WDT и внешних прерываний, просыпается от аппаратных (обычных + PCINT) или WDT за 1000 тактов (62 мкс)
  pinMode(3, INPUT_PULLUP);      // включаем внутренний подтягивающий резистор для кнопки сброс
  pinMode(A1, OUTPUT);
  pinMode(PIN_DTR, OUTPUT);
  attachInterrupt(1, button_reset, FALLING);
  sensors.begin();        // включаем датчики температуры
  mySerial.begin(9600);   // настройка скорости обмена данными с SIM800L
  mySerial.println("AT"); // установка соединения с SIM800L
  delay(100);
  mySerial.println("AT+CMGF=1"); // включаем TextMode для SMS
  delay(100);
  mySerial.println("AT+CNMI=1,2,0,0,0"); // устанавливаем режим обработки поступившие SMS. Данный режим сразу выводит поступившее SMS
  delay(100);
  mySerial.println("AT+CSCLK=1"); // включаем возможность работы энергосбеирежения
  delay(100);
  digitalWrite(PIN_DTR, HIGH); // включаем энергосбережение SIM800L
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
    delay(2000);
    sendSMS();
    btnFlag = 0;
    digitalWrite(PIN_LED, LOW);
    resetFunc();
  }

  if (systemWorking == true)
  {
    daily(); // 12 часовое оповещение
    alarm(); // сигнализация, опрос раз в 1 минуту
  }
  receivingSMS();          // обработка входящих СМС
  power.sleepDelay(10000); // спим 10 секунд
  //  Watchdog.reset(); // Переодический сброс watchdog, означающий, что устройство не зависло
}

void daily()
{

  if (millis() - timerDaily >= 41546016)
  {                                // установка времени 12 часов 43200000. Погрешность за 12 часов состаляет 33 минуты. Внёс коррекицию.
    timerDaily += 41546016;        // сброс таймера
    sensors.requestTemperatures(); // запрос температуры
    digitalWrite(PIN_DTR, LOW);    // выключаем энергосбережение SIM800L
    delay(100);
    mySerial.println("AT"); // установка соединения с SIM800L
    clearBuffer();
    mySerial.println("AT+CBC"); // запрос состояния батареи
    delay(100);                 // пауза для обработки модулем АТ-комнады
    if (mySerial.available())
    {                                   // проверка информации в буфере
      batLevel = mySerial.readString(); // чтение ответа от модуля в переменную batLevel
    }
    batLevel.replace("\n", ""); // замена символа переноса строки, что бы весь ответ был одной строкой и можно было выполнить её парсинг
    clearBuffer();
    mySerial.print("AT+CMGS=\"");
    mySerial.print(PHONE_NUMBER);
    mySerial.println("\"");
    clearBuffer();
    mySerial.println(txtAmbient + sensors.getTempCByIndex(0) + txtHome + sensors.getTempCByIndex(1) + txtBoiler + sensors.getTempCByIndex(2) + txtBat + batLevel.substring(16, 18) + " %"); // текст SMS
    clearBuffer();
    mySerial.write(26);
    clearBuffer();
    mySerial.println("AT+CMGDA=\"DEL ALL\""); // очистка входящих и исходящих СМС
    clearBuffer();
    digitalWrite(PIN_DTR, HIGH); // включаем энергосбережение SIM800L
  }
}

void alarm()
{

  if (millis() - timerAlarm >= 60000)
  {                                // установка времени 1 минута
    timerAlarm += 60000;           // сброс таймера
    sensors.requestTemperatures(); // запрос температуры

    if (sensors.getTempCByIndex(2) <= 10 || sensors.getTempCByIndex(2) >= 50 || sensors.getTempCByIndex(1) <= 5 || sensors.getTempCByIndex(1) >= 30)
    {                             // boiler temp & home temp
      digitalWrite(PIN_DTR, LOW); // выключаем энергосбережение SIM800L
      delay(100);
      mySerial.println("AT"); // установка соединения с SIM800L
      clearBuffer();
      mySerial.print("AT+CMGS=\"");
      mySerial.print(PHONE_NUMBER);
      mySerial.println("\"");
      clearBuffer();
      mySerial.println(txtWarning + txtBoiler + sensors.getTempCByIndex(2) + txtHome + sensors.getTempCByIndex(1)); // текст SMS
      clearBuffer();
      mySerial.write(26);
      delay(15000);
      clearBuffer();
      mySerial.println("AT+CMGDA=\"DEL ALL\""); // очистка входящих и исходящих СМС
      clearBuffer();
      mySerial.print("ATD");
      mySerial.print(PHONE_NUMBER);
      mySerial.println(";");
      delay(10000);
      mySerial.println("ATH"); // кладём трубку
      clearBuffer();
      digitalWrite(PIN_DTR, HIGH); // включаем энергосбережение SIM800L
    }
  }
}

void receivingSMS()
{

  if (millis() - timerSMS >= 10000)
  { // установка времени 1 минута
    timerSMS += 10000;
    if (mySerial.available())
    {                                    // проверка информации в буфере
      smsBuffer = mySerial.readString(); // чтение ответа от модуля в переменную smsBuffer
      smsBuffer.replace("\n", "");       // замена символа переноса строки, что бы весь ответ был одной строкой и можно было выполнить её парсинг
      smsBuffer.trim();                  // удаляем пробелы вначале и вконце строки
      if (smsBuffer.endsWith("Info"))
      {                             // если строка заканчивается на "GetInfo", то
        digitalWrite(PIN_DTR, LOW); // выключаем энергосбережение SIM800L
        delay(100);
        mySerial.println("AT"); // установка соединения с SIM800L
        clearBuffer();
        mySerial.println("AT+CBC"); // запрос состояния батареи
        delay(100);                 // пауза для обработки модулем АТ-комнады
        if (mySerial.available())
        {                                   // проверка информации в буфере
          batLevel = mySerial.readString(); // чтение ответа от модуля в переменную batLevel
        }
        batLevel.replace("\n", ""); // замена символа переноса строки, что бы весь ответ был одной строкой и можно было выполнить её парсинг
        clearBuffer();
        mySerial.print("AT+CMGS=\"");
        mySerial.print(PHONE_NUMBER);
        mySerial.println("\"");
        clearBuffer();
        mySerial.println(txtAmbient + sensors.getTempCByIndex(0) + txtHome + sensors.getTempCByIndex(1) + txtBoiler + sensors.getTempCByIndex(2) + txtBat + batLevel.substring(16, 18) + " %"); // текст SMS
        clearBuffer();
        mySerial.write(26);
        clearBuffer();
        mySerial.println("AT+CMGDA=\"DEL ALL\""); // ччистка входящих и исходящих СМС
        smsBuffer = "";                           // присваиваем переменной первоначальное пустое значение
        clearBuffer();
        digitalWrite(PIN_DTR, HIGH); // включаем энергосбережение SIM800L
      }
      if (smsBuffer.endsWith("Stop"))
      { // если строка заканчивается на "Stop", то
        systemWorking = false;
        digitalWrite(PIN_DTR, LOW); // выключаем энергосбережение SIM800L
        delay(100);
        mySerial.println("AT"); // установка соединения с SIM800L
        clearBuffer();
        mySerial.print("AT+CMGS=\"");
        mySerial.print(PHONE_NUMBER);
        mySerial.println("\"");
        clearBuffer();
        mySerial.println("Stop OK"); // текст SMS
        clearBuffer();
        mySerial.write(26);
        clearBuffer();
        mySerial.println("AT+CMGDA=\"DEL ALL\""); // очистка входящих и исходящих СМС
        smsBuffer = "";                           // присваиваем переменной первоначальное пустое значение
        clearBuffer();
        digitalWrite(PIN_DTR, HIGH); // включаем энергосбережение SIM800L
      }
      if (smsBuffer.endsWith("Start"))
      { // если строка заканчивается на "Start", то
        systemWorking = true;
        digitalWrite(PIN_DTR, LOW); // выключаем энергосбережение SIM800L
        delay(100);
        mySerial.println("AT"); // установка соединения с SIM800L
        clearBuffer();
        mySerial.print("AT+CMGS=\"");
        mySerial.print(PHONE_NUMBER);
        mySerial.println("\"");
        clearBuffer();
        mySerial.println("Start OK"); // текст SMS
        clearBuffer();
        mySerial.write(26);
        clearBuffer();
        mySerial.println("AT+CMGDA=\"DEL ALL\""); // очистка входящих и исходящих СМС
        smsBuffer = "";                           // присваиваем переменной первоначальное пустое значение
        clearBuffer();
        digitalWrite(PIN_DTR, HIGH); // включаем энергосбережение SIM800L
      }

      if (smsBuffer.startsWith("+CMT:"))
      {                             // если строка начинается на "+CMT:", то
        digitalWrite(PIN_DTR, LOW); // выключаем энергосбережение SIM800L
        delay(100);
        mySerial.println("AT"); // установка соединения с SIM800L
        clearBuffer();
        mySerial.println("AT+CMGDA=\"DEL ALL\""); // очистка входящих и исходящих СМС
        smsBuffer = "";                           // присваиваем переменной первоначальное пустое значение
        clearBuffer();
        digitalWrite(PIN_DTR, HIGH); // включаем энергосбережение SIM800L
      }
    }
  }
}

void sendSMS()
{

  sensors.requestTemperatures(); // запрос температуры
  digitalWrite(PIN_DTR, LOW);    // выключаем энергосбережение SIM800L
  delay(100);
  mySerial.println("AT"); // установка соединения с SIM800L
  clearBuffer();
  mySerial.println("AT+CBC"); // запрос состояния батареи
  delay(100);                 // пауза для обработки модулем АТ-комнады
  if (mySerial.available())
  {                                   // проверка информации в буфере
    batLevel = mySerial.readString(); // чтение ответа от модуля в переменную batLevel
  }
  batLevel.replace("\n", ""); // замена символа переноса строки, что бы весь ответ был одной строкой и можно было выполнить её парсинг
  clearBuffer();
  mySerial.print("AT+CMGS=\"");
  mySerial.print(PHONE_NUMBER);
  mySerial.println("\"");
  clearBuffer();
  mySerial.println(txtAmbient + sensors.getTempCByIndex(0) + txtHome + sensors.getTempCByIndex(1) + txtBoiler + sensors.getTempCByIndex(2) + txtBat + batLevel.substring(16, 18) + " %"); // текст SMS
  clearBuffer();
  mySerial.write(26);
  clearBuffer();
  mySerial.println("AT+CMGDA=\"DEL ALL\""); // очистка входящих и исходящих СМС
  clearBuffer();
  digitalWrite(PIN_DTR, HIGH); // включаем энергосбережение SIM800L
}

void clearBuffer()
{
  delay(30);
  while (mySerial.available())
  {
    mySerial.read();
  }
}

void updateSerial()
{
  delay(500); // пауза 500 мс
  while (Serial.available())
  {
    mySerial.write(Serial.read()); // переадресация с последовательного порта SIM800L на последовательный порт Arduino IDE
  }
  while (mySerial.available())
  {
    Serial.write(mySerial.read()); // переадресация c Arduino IDE на последовательный порт SIM800L
  }
}
