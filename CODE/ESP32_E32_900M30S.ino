/**
 * @file main.cpp
 * @author Ale-maker325 (https://github.com/Ale-maker325/URAD32-LoRa)
 * 
 * @brief Пример работы с модулем E32-900M30S для ESP32. Пример основан на примерах библиотек Adafruit SSD1306 и RadioLib,
 * и рассчитан для применения с дисплеем OLED SSD1306.
 * 
 * Для работы передатчика нужно также определиться, будет он работать как передатчик, либо как приёмник. Для этого необходимо
 * раскомментировать один из дефайнов: #define RECEIVER или #define TRANSMITTER, а второй закомментировать.
 *  
 */

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//#define RECEIVER                //раскомментировать, если модуль будет использоваться как простой приёмник
#define TRANSMITTER             //раскомментировать, если модуль будет использоваться как простой передатчик



#define SCREEN_WIDTH 128              // Ширина дисплея в пикселах
#define SCREEN_HEIGHT 64              // Высота дисплея в пикселах
#define OLED_RESET    -1              // Пин сброса # ( -1 если для сброса используется стандартный пин ардуино)
#define SCREEN_ADDRESS 0x3C           // Стандартный адрес I2C для дисплея (в моём случае такой адрес дал I2C-сканнер)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Создаём объект дисплея

  
//Флаг окончания операции отправки/получения чтобы указать, что пакет был отправлен или получен
volatile bool operationDone = false;

// Эта функция вызывается, когда модуль передает или получает полный пакет
// ВАЖНО: эта функция ДОЛЖНА БЫТЬ 'пуста' типа и НЕ должна иметь никаких аргументов!
IRAM_ATTR void setFlag(void) {
// мы отправили или получили пакет, установите флаг
  operationDone = true;
}

boolean FUN_IS_ON = false;    //Логический флаг включения/отключения вентилятора охлаждения



//Счётчик для сохранения количества отправленных/полученных пакетов
uint64_t count = 0;

// Подключение радиотрансивера SX127.. в соответствии с разводкой  модуля для ESP32:
const uint8_t FUN = 16;       //Пин управлением вентилятором охлаждения
const uint32_t NSS = 17;      
const uint32_t DIO_0 = 26;    
const uint32_t RST = 14;      
const uint32_t DIO_1 = 25;    
const uint32_t RX_EN = 13;    
const uint32_t TX_EN = 12;    


SX1276 radio1 = new Module(NSS, DIO_0, RST, DIO_1); //Инициализируем экземпляр радио



String RSSI = F("RSSI("); //Строка для печати RSSI
String dBm = F(")dBm");   //Строка для печати RSSI

String SNR = F("SNR(");   //Строка для печати SNR
String dB = F(")dB");     //Строка для печати SNR

String FR_ERR = F("F_Err(");  //Строка для печати SNR
String HZ = F(")Hz");         //Строка для печати SNR

String DT_RATE = F("RATE(");  //Строка для печати скорости передачи данных
String BS = F(")B/s");        //Строка для печати скорости передачи данных

String TRANSMIT = F("TRANSMIT: ");  //Строка сообщения для передачи

String RECEIVE = F("RECEIVE: ");  //Строка сообщения для приёма



int state = RADIOLIB_ERR_NONE;; // Переменная, хранящая код состояния передачи/приёма
uint8_t LED_PIN = 27;           // Пин для управления индикацией светодиодом


/**
* @brief Функция инициализации дисплея 
* 
*/
void displayInit()
{

  //Инициализируем дисплей
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {// SSD1306_SWITCHCAPVCC = напряжение дисплея от 3.3V
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Показываем содержимое буфера дисплея, созданное по-умолчанию
  // библиотека по-умолчанию использует эмблему Adafruit.
  //display.display();
  //delay(1000); // Pause for 2 seconds

  //Очищаем буффер дисплея
  display.clearDisplay();

  display.setTextSize(1);                 // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);    // Draw white text
  display.cp437(true);                    // Use full 256 char 'Code Page 437' font
}




/**
* @brief Структура для настройки параметров радиотрансивера
* 
*/
struct LORA_CONFIGURATION
{
  float frequency = 915.0;        //Частота работы передатчика (по-умолчанию 434 MHz)
  float bandwidth = 125.0;        //Полоса пропускания (по-умолчанию 125 килогерц)
  uint8_t spreadingFactor = 9;   //Коэффициент расширения (по-умолчанию 9)
  uint8_t codingRate = 7;         //Скорость кодирования (по-умолчанию 7)
  uint8_t syncWord = 0x18;        //Слово синхронизации (по-умолчанию 0х18). ВНИМАНИЕ! Значение 0x34 зарезервировано для сетей LoRaWAN и нежелательно для использования
  int8_t outputPower = 10;        //Установить выходную мощность (по-умолчанию 10 дБм) (допустимый диапазон -3 - 17 дБм) ПРИМЕЧАНИЕ: значение 20 дБм позволяет работать на большой мощности, но передача рабочий цикл НЕ ДОЛЖЕН ПРЕВЫШАТЬ 1
  uint8_t currentLimit = 80;      //Установить предел защиты по току (по-умолчанию до 80 мА) (допустимый диапазон 45 - 240 мА) ПРИМЕЧАНИЕ: установить значение 0 для отключения защиты от перегрузки по току
  int16_t preambleLength = 8;    //Установить длину преамбулы (по-умолчанию в 8 символов) (допустимый диапазон 6 - 65535)
  uint8_t gain = 0;               //Установить регулировку усилителя (по-умолчанию 1) (допустимый диапазон 1 - 6, где 1 - максимальный рост) ПРИМЕЧАНИЕ: установить значение 0, чтобы включить автоматическую регулировку усиления оставьте в 0, если вы не знаете, что вы делаете

};

//Экземпляр структуры для настройки параметров радиотрансивера 1
LORA_CONFIGURATION config_radio1;



/**
* @brief Функция установки настроек передатчика
* 
* @param radio - экземпляр класса передатчика
* @param config - экземпляр структуры для настройки модуля
*/
void radio_setSettings(SX1276 radio)
{
  Serial.println(F("Set LoRa settings..."));

  // Устанавливаем необходимую нам частоту работы трансивера
  if (radio.setFrequency(config_radio1.frequency) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }
  Serial.print(F("Set frequency = "));
  Serial.println(config_radio1.frequency);


  // установить полосу пропускания до 250 кГц
  if (radio.setBandwidth(config_radio1.bandwidth) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
  }
  Serial.print(F("Set bandWidth = "));
  Serial.println(config_radio1.bandwidth);

  // коэффициент расширения 
  if (radio.setSpreadingFactor(config_radio1.spreadingFactor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }
  Serial.print(F("Set spreadingFactor = "));
  Serial.println(config_radio1.spreadingFactor);

  // установить скорость кодирования
  if (radio.setCodingRate(config_radio1.codingRate) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }
  Serial.print(F("Set codingRate = "));
  Serial.println(config_radio1.codingRate);

  // Устанавливаем слово синхронизации
  if (radio.setSyncWord(config_radio1.syncWord) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Unable to set sync word!"));
    while (true);
  }
  Serial.print(F("Set syncWord = "));
  Serial.println(config_radio1.syncWord);

  // Устанавливаем выходную мощность трансивера
  if (radio.setOutputPower(config_radio1.outputPower) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    Serial.println(F("Selected output power is invalid for this module!"));
    while (true);
  }
  Serial.print(F("Set setOutputPower = "));
  Serial.println(config_radio1.outputPower); 

  // установить предел защиты по току (допустимый диапазон 45 - 240 мА)
  // ПРИМЕЧАНИЕ: установить значение 0 для отключения защиты от перегрузки по току
  if (radio.setCurrentLimit(config_radio1.currentLimit) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
    Serial.println(F("Selected current limit is invalid for this module!"));
    while (true);
  }
  Serial.print(F("Set currentLimit = "));
  Serial.println(config_radio1.currentLimit);

  // установить длину преамбулы (допустимый диапазон 6 - 65535)
  if (radio.setPreambleLength(config_radio1.preambleLength) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
    Serial.println(F("Selected preamble length is invalid for this module!"));
    while (true);
  }
  Serial.print(F("Set preambleLength = "));
  Serial.println(config_radio1.preambleLength);

  // Установить регулировку усилителя (допустимый диапазон 1 - 6, где 1 - максимальный рост)
  // ПРИМЕЧАНИЕ: установить значение 0, чтобы включить автоматическую регулировку усиления
  //   оставьте в 0, если вы не знаете, что вы делаете
  if (radio.setGain(config_radio1.gain) == RADIOLIB_ERR_INVALID_GAIN) {
    Serial.println(F("Selected gain is invalid for this module!"));
    while (true);
  }
  Serial.print(F("Set Gain = "));
  Serial.println(config_radio1.gain);

  Serial.println(F("All settings successfully changed!"));
}







/**
* @brief Функция отправляет данные, выводит на экран информацию об отправке,
* выводит информацию об отправке в сериал-порт
* 
* @param transmit_str - строка для передачи
*/
void transmit_and_print_data(String &transmit_str)
{
  //Посылаем очередной пакет
  Serial.print(F("Send packet ... "));

  state = radio1.startTransmit(transmit_str);

  //Если передача успешна, выводим сообщение в сериал-монитор
  if (state == RADIOLIB_ERR_NONE) {
    //Выводим сообщение об успешной передаче
    Serial.println(F("transmission finished succes!"));

    display.setCursor(0, 0);
    String str1 = TRANSMIT + transmit_str;
    display.print(str1);
                  
    //Выводим в сериал данные отправленного пакета
    Serial.print(F("Data:\t\t"));
    Serial.println(transmit_str);

    //Печатаем RSSI (Received Signal Strength Indicator)
    float rssi_data = radio1.getRSSI();
    String RSSI_DATA = (String)rssi_data;
          
    Serial.print(F("\t\t\t"));
    Serial.print(RSSI);
    Serial.print(RSSI_DATA);
    Serial.println(dBm);
          
    display.setCursor(0, 16);
    display.print(RSSI);
    display.print(RSSI_DATA);
    display.print(dBm);
              

    // печатаем SNR (Signal-to-Noise Ratio)
    float snr_data = radio1.getSNR();
    String SNR_DATA = (String)snr_data;

    Serial.print(F("\t\t\t"));
    Serial.print(SNR);
    Serial.print(SNR_DATA);
    Serial.println(dB);

    display.setCursor(0, 27);
    display.print(SNR);
    display.print(SNR_DATA);
    display.print(dB);


    // печатаем скорость передачи данных последнего пакета (бит в секунду)
    // float data_rate = radio1.getDataRate();
    float data_rate = radio1.getDataRate();
    String DATA_RATE = (String) data_rate;

    Serial.print(F("\t\t\t"));
    Serial.print(DT_RATE);
    Serial.print(DATA_RATE);
    Serial.println(BS);

    display.setCursor(0, 38);
    display.print(DT_RATE);
    display.print(DATA_RATE);
    display.print(BS);

    display.display();
    display.clearDisplay();

    digitalWrite(LED_PIN, LOW);     //Включаем светодиод, сигнализация об передаче/приёма пакета
          
  } else {
    //Если были проблемы при передаче, сообщаем об этом
    Serial.print(F("transmission failed, code = "));
    Serial.println(state);
    display.clearDisplay();
    display.setCursor(0, 10);
    display.print(F("ERROR: "));
    display.print(state);
    display.display();

  }

}
  




/**
* @brief Функция получает данные, выводит на экран информацию о полученном,
* выводит информацию о получении в сериал-порт
* 
*/
//String str;
void receive_and_print_data()
{ 
  String str = " ";
  //можно прочитать полученные данные как строку
  state = radio1.readData(str);

  //Если пакет данных был получен успешно, распечатываем данные
  //в сериал - монитор и на экран
  if (state == RADIOLIB_ERR_NONE) {

    display.setCursor(0, 0); 
    display.print(RECEIVE);
    display.print(str);
                  
    Serial.println(F("Received packet!"));

    // print data of the packet
    Serial.print(F("Data:\t\t"));
    Serial.println(str);

    // print RSSI (Received Signal Strength Indicator)
    float rssi_data = radio1.getRSSI();
    String RSSI_DATA = (String)rssi_data;
          
    Serial.print(F("\t\t\t"));
    Serial.print(RSSI);
    Serial.print(RSSI_DATA);
    Serial.println(dBm);
          
    display.setCursor(0, 16);
    display.print(RSSI);
    display.print(RSSI_DATA);
    display.print(dBm);
              

    // print SNR (Signal-to-Noise Ratio)
    float snr_data = radio1.getSNR();
    String SNR_DATA = (String)snr_data;

    Serial.print(F("\t\t\t"));
    Serial.print(SNR);
    Serial.print(SNR_DATA);
    Serial.println(dB);

    display.setCursor(0, 27);
    display.print(SNR);
    display.print(SNR_DATA);
    display.print(dB);

    // print frequency error
    float freq_error = radio1.getFrequencyError();
    String FREQ_ERROR = (String) freq_error;

    Serial.print(F("\t\t\t"));
    Serial.print(FR_ERR);
    Serial.print(FREQ_ERROR);
    Serial.println(HZ);

    display.setCursor(0, 38);
    display.print(FR_ERR);
    display.print(FREQ_ERROR);
    display.print(HZ);

    display.display();
    display.clearDisplay();

    digitalWrite(LED_PIN, LOW);     //Включаем светодиод, сигнализация об передаче/приёма пакета

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("CRC ERROR!"));
      display.clearDisplay();
      display.setCursor(0, 10);
      display.print(F("CRC ERROR!"));
      display.display();

    } else {
      // some other error occurred
      Serial.print(F("Failed, code "));
      Serial.println(state);
      display.clearDisplay();
      display.setCursor(0, 10);
      display.print(F("ERROR: "));
      display.print(state);
      display.display();
    }
}
  




void setup() {
  //Инициализируем сериал-монитор со скоростью 115200
  Serial.begin(115200);

  //инициализируем дисплей
  Serial.println("Display init....");
  displayInit();

  pinMode(LED_PIN, OUTPUT);      //Контакт управления светодиодом
  pinMode(FUN, OUTPUT);          //Контакт управления вентилятором охлаждения
    
  //Задаём параметры конфигурации радиотрансивера 1
  config_radio1.frequency = 915;
  config_radio1.bandwidth = 125;
  config_radio1.spreadingFactor = 9;
  config_radio1.codingRate = 7;
  config_radio1.syncWord = 0x12;
  config_radio1.outputPower = 3;
  config_radio1.currentLimit = 100;
  config_radio1.preambleLength = 8;
  config_radio1.gain = 0;
  
  //#ifdef TRANSMITTER
    radio1.setRfSwitchPins(RX_EN, TX_EN);         //Назначаем контакты для управлением вкл./выкл. усилителем передатчика
  //#endif

  //Инициализируем радиотрансивер со значениями по-умолчанию
  Serial.println(" ");
  Serial.print(F("Initializing ... "));
  //Инициализируем просто значениями по-умолчанию
  int state = radio1.begin();

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("SUCCES!"));
  } else {
    Serial.print(F("ERROR!  "));
    Serial.println(state);
    
    display.setCursor(0, 10);
    display.print(F("ERROR: "));
    display.print(state);
    display.display();

    while (true);
  }
    
    
  //Устанавливаем наши значения, определённые ранее в структуре config_radio1
  radio_setSettings(radio1);


  #ifdef RECEIVER   //Если определена работа модуля как приёмника

    //Устанавливаем функцию, которая будет вызываться при получении пакета данных
    radio1.setPacketReceivedAction(setFlag);

    //Начинаем слушать есть ли пакеты
    Serial.print(F("[SX1278] Starting to listen ... "));
    state = radio1.startReceive();
  
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
      digitalWrite(LED_PIN, LOW);     //Включаем светодиод, сигнализация об передаче/приёма пакета
    } else {
      Serial.print(F("failed, code: "));
      Serial.println(state);
      while (true);
    }

    //получаем данные
    receive_and_print_data();

    // Если необходимо отключить режим прослушивания, то это можно
    // сделать при помощи следующих методов:
    //
    // radio.standby()
    // radio.sleep()
    // radio.transmit();
    // radio.receive();
    // radio.scanChannel();
    
  #endif


  #ifdef TRANSMITTER   //Если определена работа модуля как передатчика

    //Устанавливаем функцию, которая будет вызываться при отправке пакета данных
    radio1.setPacketSentAction(setFlag);

    //Начинаем передачу пакетов
    Serial.println(F("Sending first packet ... "));

    String str = F("START!");
    transmit_and_print_data(str);
    digitalWrite(LED_PIN, LOW);     //Включаем светодиод, сигнализация об передаче/приёма пакета
    delay(2000);

  #endif
  

  Serial.println(" ");

  digitalWrite(LED_PIN, HIGH);      //Выключаем светодиод, сигнализация об окончании передачи/приёма пакета

  //Если мощность усилителя передатчика E32 900M30S больше 200 милливат (вы можете установить своё значение),
  // и вентилятор охлаждения не включен, то включаем вентилятор охлаждения
  if(config_radio1.outputPower > 4 && FUN_IS_ON != true)
  {
    //отмечаем вентилятор как включенный
    FUN_IS_ON = true;
    //и включаем его
    digitalWrite(FUN, HIGH);
  }
  
  Serial.println(" ");

  
}




void loop() {

  delay(1000);
  digitalWrite(LED_PIN, HIGH); //Выключаем светодиод, сигнализация об окончании передачи/приёма пакета

  #ifdef RECEIVER   //Если определен модуль как приёмник
    //проверяем, была ли предыдущая передача успешной
    Serial.println("..................................................");
    if(operationDone) {
      //Сбрасываем сработавший флаг прерывания
      operationDone = false;
      receive_and_print_data();
    }
  #endif


  #ifdef TRANSMITTER   //Если определен модуль как передатчик
    //проверяем, была ли предыдущая передача успешной
    Serial.println("..................................................");
    if(operationDone) {
      
      //Сбрасываем сработавший флаг прерывания
      operationDone = false;

      //готовим строку для отправки
      String str = "#" + String(count++);

      transmit_and_print_data(str);
      
      
    }
  #endif


}



