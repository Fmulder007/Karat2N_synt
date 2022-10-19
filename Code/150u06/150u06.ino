char ver[ ] = "150u06";
/*
   Для однодиапазонной переделки Карат-2Н.
   ВНИМАНИЕ!!! Применять ядро от AlexGyver: https://github.com/AlexGyver/GyverCore
   Добавлен Канальный режим. В нем недоступны "лишние" настройки
   Включение с нажатым кнобом переводит в режим инжменю.
   Добавлен цифровой фильтр для вольтметра
*/

//#define SI_OVERCLOCK 750000000L
#define ENCODER_OPTIMIZE_INTERRUPTS

#define crcmod 7// поправка расчета CRC для НЕ СОВМЕСТИМОСТИ со старыми/другими прошивками

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_I2C_ADRESS 0x3C // Display I2c adress

#define max_number_of_bands	30 // Максимальное количество диапазонов.
#define Si_Xtall_Freq 27000000UL // Частота кварца si5351, Гц.
#define si_cload SI5351_CRYSTAL_LOAD_10PF// 
#define lo_freq 500000UL // Частота опоры 500 КГц.
#define min_hardware_freq 10 // *100KHz Минимальный железный предел частоты диапазона VFO
#define max_hardware_freq 99 // *100KHz Максимальный железный предел частоты диапазона VFO
#define ONE_WIRE_BUS 14 // Порт датчика температуры
#define myEncBtn 4 // Порт нажатия кноба.
#define fwdpin 15 // Порт fwd показометра мощности. А0

#define mybattpin 21 // Порт датчика АКБ
#define txsenspin 17 //Порт датчика ТХ



#include "Adafruit_SSD1306.h" // Use version 1.2.7!!!
#include "si5351a.h"
#include "Wire.h"
#include "Encoder.h"
#include "Eeprom24C32_64.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "DS1307RTC.h"

//Общие настройки
struct general_set {
  uint8_t stp = 3; //Начальный шаг настройки.
  uint8_t band_set = 0; // Стартовый диапазон.
  uint8_t number_of_bands_set = 0; // Количество диапазонов.
  bool cmode = false; // Канальный режим.
  int16_t lo_cal_freq_set  = 0; // калибровка опоры 500кГц.
  bool EMF_HI = 0; // Верхний ЭМФ - 1, нижний ЭМФ = 0.
  int16_t Si_Xtall_calFreq_set = 5850; // Начальная частота калибровки кварца, Гц.
  uint8_t batt_cal_set = 208; // Начальная калибровка вольтметра.
  bool reverse_encoder_set = false; //Реверс энкодера.
  uint8_t mem_enc_div = 4; // Делитель импульсов энкодера
  int8_t temp_cal = 0; //Калибровка термометра
} general_setting;

#define stp general_setting.stp
#define band general_setting.band_set
#define number_of_bands general_setting.number_of_bands_set
#define cmode general_setting.cmode
#define lo_cal_freq general_setting.lo_cal_freq_set
#define Si_Xtall_calFreq general_setting.Si_Xtall_calFreq_set
#define batt_cal general_setting.batt_cal_set
#define reverse_encoder general_setting.reverse_encoder_set
#define mem_enc_div general_setting.mem_enc_div
#define temp_cal general_setting.temp_cal
#define EMF_HI general_setting.EMF_HI


// Диапазонные настройки
struct band_set {
  bool mode_set = 0; // LSB=0, USB=1.
  uint32_t vfo_freq_set = 3650000UL; // Начальная частота VFO при первом включении.
  uint8_t min_freq_set = 35; // *100KHz Минимальный предел частоты диапазона VFO.
  uint8_t max_freq_set = 38; // *100KHz Максимальный предел частоты диапазона VFO.
} band_setting;
#define mode band_setting.mode_set
#define vfo_freq band_setting.vfo_freq_set
#define min_freq band_setting.min_freq_set
#define max_freq band_setting.max_freq_set

//
//
//

uint8_t menu = 0; //Начальное положение меню.
uint16_t arraystp[] = {1, 10, 50, 100, 500, 1000, 10000}; //шаги настройки * 10 герц.

uint8_t enc_div = 4;

uint16_t fwdpower = 0;

uint8_t mybatt = 0;
int8_t temperature = 0;
uint16_t screenstep = 1000;
uint16_t rawbatt = 0;
bool toneen = false;

long oldPosition  = 0;
bool actencf = false;
bool txen = false;
bool knobup = true;
bool exitmenu = false;
bool reqtemp = false;
bool timesetup = false;
bool actfmenuf = false;


uint32_t previousdsp = 0;
uint32_t previoustemp = 0;
uint32_t previoustime = 0;
uint32_t knobMillis = 0;
uint32_t actenc = 0;



static Eeprom24C32_64 AT24C32(0x50);
Si5351 si;
Encoder myEnc(2, 3); //порты подключения енкодера.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
tmElements_t tm;


void setup() {

  // other pin`s setup
  pinMode(myEncBtn, INPUT);
  pinMode(fwdpin, INPUT);
  digitalWrite(myEncBtn, HIGH);
  analogReference(INTERNAL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADRESS);
  display.clearDisplay();
  display.display();
  sensors.begin();
  memread();
  enc_div = mem_enc_div;
  si5351init();
  si5351correction();
  vfosetup();
  battmeter();
  powermeter();
  tempsensor ();
  timenow ();
  if (!digitalRead(myEncBtn)) menu = 100;
  versionprint ();
  mainscreen();

}

void loop() { // Главный цикл

  pushknob();
  if (!cmode) readencoder(); //Если не в канальном режиме - считать енкодер
  if (cmode && menu) readencoder(); //Если в канальном режиме и не на главном экране - считать енкодер
  txsensor();
  battmeter();
  if (!menu) {
    if (txen) screenstep = 50;
    else screenstep = 1000;
    if (millis() - previousdsp > screenstep) {
      storetomem();
      powermeter();
      tempsensor();
      timenow();
      mainscreen();
      previousdsp = millis();
    }
  }
}

void timenow () {
  if (timesetup) {
    tm.Second = 0;
    RTC.write(tm);
    timesetup = false;
  }
  else {
    if (millis() - previoustime > 1000 || !previoustime) {
      previoustime = millis();
      if (!RTC.read(tm)) {
        if (RTC.chipPresent()) {
          tm.Hour = 0;  tm.Minute = 0; tm.Second = 0;
          RTC.write(tm);
        }
      }
    }
  }
}

void tempsensor () { //Измерение температуры
  if (millis() - previoustemp > 5000 && !reqtemp) {
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    sensors.setWaitForConversion(true);
    reqtemp = true;
  }
  if (millis() - previoustemp > 8000 && reqtemp) {
    temperature = (int8_t)(0.5 + sensors.getTempCByIndex(0));
    temperature = temperature + temp_cal;
    previoustemp = millis();
    reqtemp = false;
  }
}

void txsensor () {
  bool txsens = digitalRead(txsenspin);
  //Если радио на приеме и нажали ТХ то txen = true
  if (txsens && !txen) {
    txen = true;
    mainscreen();
  }
  //Если радио на передаче и отпустили ТХ то txen = false
  if (!txsens && txen) {
    txen = false;
    mainscreen();
  }
}

void pushknob () {  // Обработка нажатия на кноб

  bool knobdown = digitalRead(myEncBtn);   //Читаем состояние кноба
  if (!knobdown && knobup) {   //Если кноб был отпущен, но нажат сейчас
    knobup = false;   // отмечаем флаг что нажат
    knobMillis = millis();  // запускаем таймер для антидребезга
  }

  if (knobdown && !knobup) { //Если кноб нажат
    knobup = true; // отмечаем флаг что кноб отпущен
    long knobupmillis = millis();
    if (knobupmillis - knobMillis >= 1000) { //Если длительное нажатие
      if (menu == 0) {
        if (cmode) menu = 23;
        else menu = 20;// Если долгое нажатие на главном экране, то перейти в юзерменю
      }
      else if (menu != 0) menu = 0; // Если долгое нажатие не на главном экране, то перейти на главный экран
    }

    if (knobupmillis - knobMillis < 1000 && knobupmillis - knobMillis > 100) { //Если кноб отпущен и был нажат и времени от таймера прошло 100Мс
      if (menu < 4 && menu > 0 && actfmenuf) //Если 0<меню<4 и крутили енкодер в быстром меню, то выйти на главный экран
      {
        actfmenuf = false;
        menu = 0;
      }
      else {
        menu ++; //Переходим на меню дальше
        if (menu == 4) menu = 0; //Если меню 5 выйти на главный экран
        if (menu > 26 && menu < 100) menu = 20; //Если меню > 30 но < 100 перейти на меню 20
        if (menu > 105) menu = 100; //Если меню больше 105 перейти на меню 100
      }
      if (!number_of_bands && (menu == 1 || menu == 100)) menu++; // Если каналы не настроены, то нет меню 1 и 100
      if (cmode && number_of_bands) {                             //Если в канальном режиме, то пропускать меню 2,3,20,21,22,25,26
        switch (menu) {
          case 2:
          case 3: menu = 0;
            break;
          case 20:
          case 21:
          case 22:
          case 25:
          case 26: menu = 23;
            break;
        }
      }
    }
    mainscreen();
  }
}

void storetomem() { // Если крутили енкодер, то через 10 секунд все сохранить

  if ((millis() - actenc > 10000UL) && actencf) {
    actenc = millis();
    actencf = false;
    memwrite ();
  }
}

void readencoder() { // работа с енкодером
  long newPosition;
  newPosition = myEnc.read() / enc_div;
  if (reverse_encoder) newPosition *= (-1);
  if (newPosition != oldPosition && digitalRead(myEncBtn)) { // ЕСЛИ КРУТИЛИ энкодер

    if (menu > 0 && menu < 5) actfmenuf = true; // Если крутили энкодер в быстром меню - флаг вверх!
    switch (menu) {

      case 0: //Основная настройка частоты
        if (newPosition > oldPosition && vfo_freq <= max_freq * 100000UL) {
          if (vfo_freq % (arraystp[stp] * 10UL)) {
            vfo_freq = vfo_freq + (arraystp[stp] * 10UL) - (vfo_freq % (arraystp[stp] * 10UL));
          }
          else {
            vfo_freq = vfo_freq + (arraystp[stp] * 10UL);
          }
        }
        if (newPosition < oldPosition && vfo_freq >= min_freq * 100000UL) {
          if (vfo_freq % (arraystp[stp] * 10UL)) {
            vfo_freq = vfo_freq - (vfo_freq % (arraystp[stp] * 10UL));
          }
          else {
            vfo_freq = vfo_freq - (arraystp[stp] * 10UL);
          }
        }
        if (vfo_freq < min_freq * 100000UL) vfo_freq = min_freq * 100000UL;
        if (vfo_freq > max_freq * 100000UL) vfo_freq = max_freq * 100000UL;
        vfosetup();
        break;

      case 1: //Переключение каналов
        if (newPosition > oldPosition && band < number_of_bands) band++;
        if (newPosition < oldPosition && band > 0) band--;
        //if (band > number_of_bands) band = number_of_bands;
        band = constrain(band, 0, number_of_bands);
        band_memread();
        vfosetup();
        break;

      case 2: //Настройка ШАГА настройки
        if (newPosition > oldPosition && stp < (sizeof(arraystp) / sizeof(arraystp[0]) - 1)) stp++;
        if (newPosition < oldPosition && stp > 0) stp--;
        //if (stp < 10) stp = 10;
        if (stp > (sizeof(arraystp) / sizeof(arraystp[0]) - 1)) stp = (sizeof(arraystp) / sizeof(arraystp[0]) - 1);
        break;

      case 3: //Переключение LSB|USB.
        mode = !mode;
        vfosetup();
        break;

      case 20: //Настройка min_freq
        if (newPosition > oldPosition && min_freq <= max_hardware_freq) min_freq++;
        if (newPosition < oldPosition && min_freq >= min_hardware_freq) min_freq--;
        min_freq = constrain(min_freq, min_hardware_freq, max_freq - 1);
        break;

      case 21: //Настройка maxfreq
        if (newPosition > oldPosition && max_freq <= max_hardware_freq) max_freq++;
        if (newPosition < oldPosition && max_freq >= min_hardware_freq) max_freq--;
        max_freq = constrain(max_freq, min_freq + 1, max_hardware_freq);
        break;

      case 22: // Настройка количества каналов
        if (newPosition > oldPosition && number_of_bands < max_number_of_bands) number_of_bands++;
        if (newPosition < oldPosition && number_of_bands > 0) number_of_bands--;
        if (number_of_bands > max_number_of_bands) number_of_bands = max_number_of_bands;
        if (band > number_of_bands) band = number_of_bands;
        break;

      case 23: //Настройка Часов
        if (newPosition > oldPosition && tm.Hour < 24) tm.Hour++;
        if (newPosition < oldPosition && tm.Hour > 0) tm.Hour--;
        if (tm.Hour > 23) tm.Hour = 23;
        timesetup = true;
        break;

      case 24: //Настройка Минут
        if (newPosition > oldPosition && tm.Minute < 60) tm.Minute++;
        if (newPosition < oldPosition && tm.Minute > 0) tm.Minute--;
        if (tm.Minute > 59) tm.Minute = 59;
        timesetup = true;
        break;

      case 25: // Настройка делителя импульсов энкодера
        if (newPosition > oldPosition && mem_enc_div < 255) mem_enc_div++;
        if (newPosition < oldPosition && mem_enc_div > 1) mem_enc_div--;
        mem_enc_div = constrain(mem_enc_div, 1, 255);
        break;

      case 26: //Инверсия энкодера.
        if (reverse_encoder) {
          reverse_encoder = false;
        }
        else {
          reverse_encoder = true;
        }
        newPosition *= (-1);
        break;

      case 100: //Канальный режим.
        cmode = !cmode;
        break;

      case 101: //Настройка калибровки по питанию
        if (newPosition > oldPosition && batt_cal <= 254) batt_cal++;
        if (newPosition < oldPosition && batt_cal >= 100) batt_cal--;
        if (batt_cal > 254) batt_cal = 254;
        if (batt_cal < 100) batt_cal = 100;
        break;

      case 102: //Калибровка термодатчика
        if (newPosition > oldPosition && temp_cal <= 30) temp_cal++;
        if (newPosition < oldPosition && temp_cal >= - 30) temp_cal--;
        temp_cal = constrain(temp_cal, -30, 30);
        break;

      case 103: //Частота кварца синтезатора
        if (newPosition > oldPosition && Si_Xtall_calFreq <= 30000) Si_Xtall_calFreq += arraystp[stp];
        if (newPosition < oldPosition && Si_Xtall_calFreq >= - 30000) Si_Xtall_calFreq -= arraystp[stp];
        Si_Xtall_calFreq = constrain(Si_Xtall_calFreq, -30000, 30000);
        si5351correction();
        vfosetup();
        break;

      case 104: //1пч
        if (newPosition > oldPosition) lo_cal_freq += arraystp[stp];
        if (newPosition < oldPosition) lo_cal_freq -= arraystp[stp];
        lo_cal_freq = constrain(lo_cal_freq, (-1000), 1000);
        vfosetup();
        break;

      case 105: //Верхний ЭМФ.
        EMF_HI = !EMF_HI;
        break;


    }
    actenc = millis();
    actencf = true;
    mainscreen();
    oldPosition = newPosition;
  }
}

void powermeter () { // Измеритель уровня выхода
  /*
   * filt = (A * filt + B * signal) >> k;
   * Коэффициенты у этого фильтра выбираются следующим образом:    
   * k = 1, 2, 3…   
   * A + B = 2^k
   * Чем больше А, тем плавнее фильтр (отношение A/B)
   * Чем больше k, тем более плавным можно сделать фильтр. Но больше 5 уже нет смысла, т.к. A=31, B=1 уже 
   * очень плавно, а при увеличении может переполниться int и придётся использовать 32-х битные типы данных.
   * Результат умножения не должен выходить за int, иначе придётся преобразовывать к long
   * Более подробно о выборе коэффициентов читайте в статье, ссылка выше
   * Особенность: если “ошибка” сигнала (signal – filter) меньше 2^k – фильтрованное 
   * значение меняться не будет. Для повышения “разрешения” фильтра можно домножить (или сдвинуть) 
   * переменную фильтра, то есть фильтровать в бОльших величинах
  */
  fwdpower = (12 * fwdpower + 4 * (analogRead(fwdpin))) >> 4;
}

void battmeter () { // Измеритель напряжения питания
  //rawbatt = analogRead(mybattpin);
  rawbatt = (14 * rawbatt + 2 * (analogRead(mybattpin))) >> 4; // Крутой фильтр для усреднения показаний вольтметра!!!
  mybatt = map(rawbatt, 0, 1023, 0, batt_cal);
}

void mainscreen() { //Процедура рисования главного экрана
  int mypower = (map(fwdpower, 0, 1023, 0, 567));
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.setTextSize(3);
  switch (menu) {

    case 0: //Если не в меню, то рисовать главный экран
      //Вывод частоты на дисплей
      if ((vfo_freq / 1000000) < 10) display.print(" ");
      display.print(vfo_freq / 1000000);//Вывод МГц
      display.setCursor(display.getCursorX() + 5, display.getCursorY()); //Переводим курсор чуть правее текущего положения
      if ((vfo_freq % 1000000) / 1000 < 100) display.print("0");
      if ((vfo_freq % 1000000) / 1000 < 10) display.print("0");
      display.print((vfo_freq % 1000000) / 1000); //Выводим КГц
      display.setTextSize(2); // Для сотен и десятков герц делаем шрифт поменьше
      display.setCursor(display.getCursorX() + 5, 7); //Переводим курсор чуть ниже текущего положения
      if ((vfo_freq % 1000) / 10 < 10) display.print("0"); //Если герц <10 то выводим "0" перед ними.
      display.println((vfo_freq % 1000) / 10);

      //Выводим вторую строку на дисплей
      display.setTextSize(1);// Ставим маленький шрифт
      if (mybatt - 100 < 0) display.print("0");
      display.print(mybatt / 10);
      display.print(".");
      display.print(mybatt % 10);
      display.print("v ");


      if (txen) {//Если передача, то вывод показометра мощности
        display.print(map(fwdpower, 0, 1023, 0, 99));
        if (mypower > 9) display.fillRect(64, 23, mypower / 9, 9, WHITE);
        display.fillRect(64 + mypower / 9, 23, 1, mypower % 9, WHITE);
      }
      else {// Если прием, то рисовать температуру часы, полосу и диапазон
        //char ddot
        if (temperature >= -50 && temperature <= 50) {
          if (temperature >= 0) display.print(" ");
          display.print(temperature);
          display.print((char)247);
          display.print("C");
        }
        else {
          display.print(" err ");
        }
        if (actencf) {
          display.print(" ");
        }
        else {
          display.print(".");
        }
        if (tm.Hour < 10) display.print(" ");
        display.print(tm.Hour);
        if (tm.Second % 2) {
          display.print(":");
        }
        else {
          display.print(" ");
        }
        if (tm.Minute < 10) display.print("0");
        display.print(tm.Minute);
        if (mode) {
          display.print(" U");
        }
        else {
          display.print(" L");
        }
        if (band < 10) display.print(" ");
        display.print(band);
      }
      break;

    case 1: //Меню 1 - канал
      display.println(band);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Ch from 0 to ");
      display.print(number_of_bands);
      break;

    case 2: //Меню 2 - шаг настройки
      display.println(arraystp[stp] * 10UL);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Step Hz");
      break;

    case 3: //Меню 3 - LSB|USB
      if (mode) {
        display.println("USB");
      }
      else
      {
        display.println("LSB");
      }
      display.setTextSize(1);
      display.print(menu);
      display.println("  LSB|USB Switch");
      break;
    //-----------------------------USER MENU Display-------------------------//
    case 20: //Настройка min_freq
      display.println(min_freq * 100);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Min Freq ");
      display.print((char)240);
      display.print("kHz");
      break;

    case 21: //Настройка maxfreq
      display.println(max_freq * 100);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Max Freq ");
      display.print((char)240);
      display.print("kHz");
      break;

    case 22: //Количество каналов
      display.println(number_of_bands);
      display.setTextSize(1);
      display.print(menu);
      display.print("  MAX Num Ch");
      break;


    case 23: //Меню 12 - Настройка Часов
      if (tm.Hour < 10) display.print("0");
      display.println(tm.Hour);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Hour");
      break;

    case 24: //Меню 13 - Настройка Минут
      if (tm.Minute < 10) display.print("0");
      display.println(tm.Minute);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Min");
      break;


    case 25: //Меню 15 - Encoder Divider
      display.println(mem_enc_div);
      display.setTextSize(1);
      display.print(menu);
      display.print(" !Enc Divider!");
      break;

    case 26: //Меню 14 - Reverse Encoder
      if (reverse_encoder) {
        display.println("Yes");
      }
      else
      {
        display.println("NO");
      }
      display.setTextSize(1);
      display.print(menu);
      display.println("  Reverse Enc");
      break;

    //-------------------------------------SETUP MENU DISPLAY--------------------------------------//

    case 100: //Channel mode
      if (cmode) {
        display.println("Yes");
      }
      else
      {
        display.println("NO");
      }
      display.setTextSize(1);
      display.print(menu);
      display.println("  CHannel MODE");
      break;


    case 101: //Меню 10 - Настройка калибровки по питанию
      display.println(batt_cal);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Batt ");
      if (mybatt - 100 < 0) display.print("0");
      display.print(mybatt / 10);
      display.print(".");
      display.print(mybatt % 10);
      display.print("v");
      break;

    case 102: //Калибровка термодатчика
      display.println(temp_cal);
      display.setTextSize(1);
      display.print(menu);
      display.print(" Temp CAL ");
      display.print((char)240);
      display.print((char)247);
      display.print("C");
      break;

    case 103: //Настройка калибровки кварца
      display.println(Si_Xtall_calFreq);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Xtal Cal ");
      display.print((char)240);
      display.print("Hz");
      break;

    case 104: //Настройка опорного гетеродина 500кГц
      display.println(lo_cal_freq);
      display.setTextSize(1);
      display.print(menu);
      display.print("  500kHz cal");
      display.print((char)240);
      display.print("Hz");
      break;

    case 105: //Верхний ЭМФ.
      if (EMF_HI) {
        display.println("500B");
      }
      else
      {
        display.println("500H");
      }
      display.setTextSize(1);
      display.print(menu);
      display.println("  EMF Type");
      break;

  }
  display.display();
  //debug();
}



void vfosetup() {
  if (mode) {
    si.set_freq((vfo_freq + lo_freq + lo_cal_freq), 0, 0);
  }
  else {
    si.set_freq((vfo_freq - lo_freq + lo_cal_freq), 0, 0);
  }
}

void si5351init() {
  si.setup(0, 0, 0);
  si.cload(si_cload);
}

void si5351correction() {
  si.set_xtal_freq(Si_Xtall_Freq + Si_Xtall_calFreq);
  si.update_freq(0);
  si.update_freq(2);
}

void memwrite () { //Запись general_setting
  int16_t crc = 0;
  uint8_t i = 0;
  uint8_t * adr;
  adr =  (uint8_t*)(& general_setting);
  while (i < (sizeof(general_setting)))
  {
    crc += *(adr + i);
    i++;
  }
  AT24C32.writeEE(2, general_setting);
  AT24C32.writeEE(0, (crc + crcmod));

  // Запись band_setting
  crc = 0;
  i = 0;
  adr =  (uint8_t*)(& band_setting);
  while (i < (sizeof(band_setting)))
  {
    crc += *(adr + i);
    i++;
  }
  AT24C32.writeEE(sizeof(general_setting) + 2 + ((sizeof(band_setting) + 2)*band) + 2, band_setting);
  AT24C32.writeEE(sizeof(general_setting) + 2 + ((sizeof(band_setting) + 2)*band), (crc + crcmod));
}

void memread() {
  int16_t crc = 0;
  int16_t crcrom = 0;
  uint8_t i = 0;

  // Чтение general_setting
  AT24C32.readEE (0, crc);
  while (i < (sizeof(general_setting)))
  {
    crcrom += AT24C32.readByte ((i + 2));
    i++;
  }
  if (crc == (crcrom + crcmod)) {
    AT24C32.readEE (2, general_setting);
  }
  else {
    memwrite ();
  }
  band_memread();
}

void band_memread() {

  int16_t crc = 0;
  int16_t crcrom = 0;
  uint8_t i = 0;
  AT24C32.readEE (sizeof(general_setting) + 2 + ((sizeof(band_setting) + 2) * band), crc);
  while (i < (sizeof(band_setting)))
  {
    crcrom += AT24C32.readByte ((i + sizeof(general_setting) + 2 + ((sizeof(band_setting) + 2) * band)) + 2);
    i++;
  }
  if (crc == (crcrom + crcmod)) {
    AT24C32.readEE (sizeof(general_setting) + 2 + ((sizeof(band_setting) + 2)*band) + 2, band_setting);
  }
  else {
    memwrite ();
  }
}

void versionprint() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.setTextSize(3);
  if (menu == 100) {
    display.println("Setup");
    display.display();
    while (!digitalRead(myEncBtn));
  }
  else {
    display.println(ver);
    display.setTextSize(1);
    display.println("    From UD0CAJ");
    display.display();
  }
  delay(1000);
}
