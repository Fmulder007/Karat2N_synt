/*
  UD0CAJ Karat2_sintez
  7-7,2 MHz
  500N
  496.570 KHz Lo freq
  Eeprom24C32 memory using
  
*/
#include "Adafruit_SSD1306.h" // Use version 1.2.7!!!
#include "si5351.h"
#include "Wire.h"
#include "Encoder.h"
#include "Eeprom24C32_64.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "DS1307RTC.h"



#define ENCODER_OPTIMIZE_INTERRUPTS
//#define ENCODER_DO_NOT_USE_INTERRUPTS

char ver[ ] = "v 1.2.0";

byte ONE_WIRE_BUS = 12; // Порт датчика температуры
byte myEncBtn = 4;  // Порт нажатия кноба.
byte mypowerpin = 14; // Порт показометра мощности. А0
byte mybattpin = 15; // Порт датчика АКБ А1
byte txpin = 5; //Порт датчика ТХ.
byte menu = 0; //Начальное положение меню.
byte arraystp[] = {1, 10, 50, 100}; //шаги настройки * 10 герц.

byte mypower;
byte mybatt;
byte temperature;
int screenstep = 1000;

long oldPosition  = 0;
boolean actencf = false;
boolean txen = false;
int RXifshift = 0;
boolean knobup = true;
boolean exitmenu = false;
boolean reqtemp = false;
boolean timesetup = false;


struct var {
  byte stp = 0;
  int battcal = 253;
  unsigned long freq = 7090000UL; // Начальная частота при первом включении.
  unsigned long lofreq = 496570UL; // Начальная ПЧ при первом включении.
  int calibration = 2277; // Начальная калибровка при первом включении.
  int ifshift = 0; // Начальный сдвиг ПЧ при первом включении.
  byte minfreq = 70; // *100KHz Минимальный предел частоты
  byte maxfreq = 72; // *100KHz Максимальный предел частоты
} varinfo;


// unsigned long previousMillis = 0;
unsigned long previousdsp = 0;
unsigned long previoustemp = 0;
unsigned long previoustime = 0;
unsigned long knobMillis = 0;
unsigned long actenc = 0;


static Eeprom24C32_64 AT24C32(0x50);
Si5351 si5351;
Encoder myEnc(3, 2); //порты подключения енкодера.
Adafruit_SSD1306 display(4);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
tmElements_t tm;


void setup() {
  pinMode(myEncBtn, INPUT);
  pinMode(mypowerpin, INPUT);
  digitalWrite(myEncBtn, HIGH);
  analogReference(INTERNAL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  sensors.begin();
  memread();
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  si5351.set_correction(varinfo.calibration * 100L, SI5351_PLL_INPUT_XO);
  losetup();
  vfosetup();
  battmeter();
  powermeter();
  tempsensor ();
  timenow ();
  versionprint ();
  mainscreen();
}

void loop() { // Главный цикл
  pushknob ();
  readencoder();
  txsensor ();
  if (!menu) {
    if (txen) screenstep = 100;
    else screenstep = 1000;
    if (millis() - previousdsp > screenstep) {
      storetomem();
      battmeter();
      powermeter();
      tempsensor ();
      timenow ();
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

void tempsensor () {
  if (millis() - previoustemp > 5000 && !reqtemp) {
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    sensors.setWaitForConversion(true);
    reqtemp = true;
  }
  if (millis() - previoustemp > 8000 && reqtemp) {
    temperature = (byte)(0.5 + sensors.getTempCByIndex(0));
    previoustemp = millis();
    reqtemp = false;
  }
}

void txsensor () {
  boolean txsens = digitalRead(txpin);
  //Если радио на приеме и нажали ТХ то обнулить IF-SHIFT
  if (txsens && !txen) {
    txen = true;
    RXifshift = 0;
    vfosetup();
    losetup();
  }
  //Если радио на передаче и отпустили ТХ то прописать IF-SHIFT
  if (!txsens && txen) {
    txen = false;
    RXifshift = varinfo.ifshift;
    vfosetup();
    losetup();
    mainscreen();
  }
  //Если радио на приеме и НЕ нажали ТХ и RXifshift не равно varinfo.ifshift то прописать IF-SHIFT
  if (!txsens && !txen && RXifshift != varinfo.ifshift) {
    txen = false;
    RXifshift = varinfo.ifshift;
    vfosetup();
    losetup();
  }
}

void pushknob () {  // Обработка нажатия на кноб

  boolean knobdown = digitalRead(myEncBtn);   //Читаем состояние кноба
  if (!knobdown && knobup) {   //Если кноб был отпущен, но нажат сейчас
    knobup = false;   // отмечаем флаг что нажат
    knobMillis = millis();  // запускаем таймер для антидребезга
  }

  if (knobdown && !knobup) { //Если кноб отпущен и был нажат
    knobup = true; // отмечаем флаг что кноб отпущен
    long knobupmillis = millis();
    if (knobupmillis - knobMillis >= 1000) { //Если длительное нажатие
      if (menu == 0) menu = 3;
      else if (menu != 0) menu = 0;
    }

    if (knobupmillis - knobMillis < 1000 && knobupmillis - knobMillis > 100) { //Если кноб отпущен и был нажат и времени от таймера прошло 100Мс
      menu ++; //Переходим на меню дальше
      if (menu == 3) menu = 0; //Если меню 3 выйти на главный экран
      if (menu > 9) menu = 3; //Если меню больше 9 перейти на меню 3
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
  long newPosition = myEnc.read() / 4;
  if (newPosition != oldPosition) {
    switch (menu) {

      case 0: //Основная настройка частоты
        if (newPosition > oldPosition && varinfo.freq <= varinfo.maxfreq * 100000UL) {
          if (varinfo.freq % (arraystp[varinfo.stp] * 10)) {
            varinfo.freq = varinfo.freq + (arraystp[varinfo.stp] * 10) - (varinfo.freq % (arraystp[varinfo.stp] * 10));
          }
          else {
            varinfo.freq = varinfo.freq + (arraystp[varinfo.stp] * 10);
          }
        }
        if (newPosition < oldPosition && varinfo.freq >= varinfo.minfreq * 100000UL) {
          if (varinfo.freq % (arraystp[varinfo.stp] * 10)) {
            varinfo.freq = varinfo.freq - (varinfo.freq % (arraystp[varinfo.stp] * 10));
          }
          else {
            varinfo.freq = varinfo.freq - (arraystp[varinfo.stp] * 10);
          }
        }
        if (varinfo.freq < varinfo.minfreq * 100000UL) varinfo.freq = varinfo.minfreq * 100000UL;
        if (varinfo.freq > varinfo.maxfreq * 100000UL) varinfo.freq = varinfo.maxfreq * 100000UL;
        vfosetup();
        break;

      case 1: //Настройка ШАГА настройки
        if (newPosition > oldPosition && varinfo.stp < (sizeof(arraystp) / sizeof(arraystp[0]) - 1)) varinfo.stp = varinfo.stp + 1;
        if (newPosition < oldPosition && varinfo.stp > 0) varinfo.stp = varinfo.stp - 1;
        //if (varinfo.stp < 10) varinfo.stp = 10;
        if (varinfo.stp > (sizeof(arraystp) / sizeof(arraystp[0]) - 1)) varinfo.stp = (sizeof(arraystp) / sizeof(arraystp[0]) - 1);
        break;

      case 2: //Настройка IF-SHIFT
        if (newPosition > oldPosition && varinfo.ifshift <= 3000) varinfo.ifshift = varinfo.ifshift + 50;
        if (newPosition < oldPosition && varinfo.ifshift >= -3000) varinfo.ifshift = varinfo.ifshift - 50;
        if (varinfo.ifshift > 3000) varinfo.ifshift = 3000;
        if (varinfo.ifshift < -3000) varinfo.ifshift = - 3000;
        losetup();
        vfosetup();
        break;

      case 3: //Настройка опорного гетеродина
        if (newPosition > oldPosition && varinfo.lofreq <= 550000) varinfo.lofreq = varinfo.lofreq + arraystp[varinfo.stp];
        if (newPosition < oldPosition && varinfo.lofreq >= 450000) varinfo.lofreq = varinfo.lofreq - arraystp[varinfo.stp];
        if (varinfo.lofreq < 450000) varinfo.lofreq = 450000;
        if (varinfo.lofreq > 550000) varinfo.lofreq = 550000;
        losetup();
        vfosetup();
        break;

      case 4: //Настройка калибровки по питанию
        if (newPosition > oldPosition && varinfo.battcal <= 254) varinfo.battcal = varinfo.battcal + 1;
        if (newPosition < oldPosition && varinfo.battcal >= 100) varinfo.battcal = varinfo.battcal - 1;
        if (varinfo.battcal > 254) varinfo.battcal = 254;
        if (varinfo.battcal < 100) varinfo.battcal = 100;
        break;

      case 5: //Настройка калибровки PLL
        if (newPosition > oldPosition && varinfo.calibration <= 30000) varinfo.calibration = varinfo.calibration + arraystp[varinfo.stp];
        if (newPosition < oldPosition && varinfo.calibration >= - 30000) varinfo.calibration = varinfo.calibration - arraystp[varinfo.stp];
        if (varinfo.calibration > 30000) varinfo.calibration = 30000;
        if (varinfo.calibration <  - 30000) varinfo.calibration =  - 30000;
        si5351.set_correction(varinfo.calibration * 100L, SI5351_PLL_INPUT_XO);
        losetup();
        vfosetup();
        break;

      case 6: //Настройка minfreq
        if (newPosition > oldPosition && varinfo.minfreq <= 100) varinfo.minfreq = varinfo.minfreq + 1;
        if (newPosition < oldPosition && varinfo.minfreq >= 10) varinfo.minfreq = varinfo.minfreq - 1;
        if (varinfo.minfreq < 10) varinfo.minfreq = 10;
        if (varinfo.minfreq >= varinfo.maxfreq) varinfo.minfreq = varinfo.maxfreq - 1;
        break;

      case 7: //Настройка maxfreq
        if (newPosition > oldPosition && varinfo.maxfreq <= 100) varinfo.maxfreq = varinfo.maxfreq + 1;
        if (newPosition < oldPosition && varinfo.maxfreq >= 10) varinfo.maxfreq = varinfo.maxfreq - 1;
        if (varinfo.maxfreq <= varinfo.minfreq) varinfo.maxfreq = varinfo.minfreq + 1;
        if (varinfo.maxfreq > 100) varinfo.maxfreq = 100;
        break;

      case 8: //Настройка Часов
        if (newPosition > oldPosition) tm.Hour++;
        if (newPosition < oldPosition) tm.Hour--;
        if (tm.Hour < 0) tm.Hour = 23;
        if (tm.Hour > 23) tm.Hour = 0;
        timesetup = true;
        break;

      case 9: //Настройка Минут
        if (newPosition > oldPosition) tm.Minute++;
        if (newPosition < oldPosition) tm.Minute--;
        if (tm.Minute < 0) tm.Minute = 59;
        if (tm.Minute > 59) tm.Minute = 0;
        timesetup = true;
        break;
    }
    actenc = millis();
    actencf = true;
    mainscreen();
    oldPosition = newPosition;
  }
}

void powermeter () { // Измеритель уровня выхода
  int rawpower = analogRead(mypowerpin);
  mypower = map(rawpower, 0, 1023, 0, 100);
}

void battmeter () { // Измеритель напряжения питания
  int rawbatt = analogRead(mybattpin);
  mybatt = map(rawbatt, 0, 1023, 0, varinfo.battcal);
}

void mainscreen() { //Процедура рисования главного экрана
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.setTextSize(3);
  switch (menu) {

    case 0: //Если не в меню, то рисовать главный экран
      display.println(varinfo.freq / 1000.0);
      display.setTextSize(1);
      if (mybatt - 100 < 0) display.print("0");
      display.print(mybatt / 10.0);
      display.print("v ");
      if (txen) {
        display.print("PWR ");
        display.fillRect(64, 23, mypower, 9, WHITE);
      }
      else {
        char ddot;
        display.print(temperature);
        display.print((char)247);
        display.print("C ");
        if (tm.Hour < 10) display.print(" ");
        display.print(tm.Hour);
        display.print(":");
        if (tm.Minute < 10) display.print("0");
        display.print(tm.Minute);
        display.print(":");
        if (tm.Second < 10) display.print("0");
        display.print(tm.Second);
      }
      break;

    case 1: //Меню 1 - шаг настройки
      display.println(arraystp[varinfo.stp] * 10);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Step");
      break;

    case 2: //Меню 2 - IF-SHIFT
      display.println(varinfo.ifshift);
      display.setTextSize(1);
      display.print(menu);
      display.println("  IF - Shift");
      break;

    case 3: //Меню 3 - Настройка опорного гетеродина
      display.println(varinfo.lofreq);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Lo Cal Hz");
      break;


    case 4: //Меню 4 - Настройка калибровки по питанию
      display.println(varinfo.battcal);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Batt Cal");
      break;

    case 5: //Меню 5 - Настройка калибровки кварца
      display.println(varinfo.calibration);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Xtal Cal Hz");
      break;


    case 6: //Меню 8 - Настройка minfreq
      display.println(varinfo.minfreq * 100);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Min Freq kHz");
      break;

    case 7: //Меню 9 - Настройка maxfreq
      display.println(varinfo.maxfreq * 100);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Max Freq kHz");
      break;

    case 8: //Меню 10 - Настройка Часов
      if (tm.Hour < 10) display.print("0");
      display.println(tm.Hour);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Hour");
      break;

    case 9: //Меню 11 - Настройка Минут
      if (tm.Minute < 10) display.print("0");
      display.println(tm.Minute);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Minute");
      break;
  }
  display.display();
}


void vfosetup() {
  si5351.set_freq(((varinfo.freq + varinfo.lofreq + RXifshift) * 100), SI5351_CLK0);
}


void losetup() {
  si5351.set_freq(((varinfo.lofreq + RXifshift) * 100), SI5351_CLK1);
}

void memwrite () {
  int crc = 0;
  byte i = 0;
  byte * adr;
  adr =  (byte*)(& varinfo);
  while (i < (sizeof(varinfo)))
  {
    crc += *(adr + i);
    i++;
  }
  AT24C32.writeEE(2, varinfo);
  AT24C32.writeEE(0, crc);
}

void memread() {
  int crc = 0;
  int crcrom = 0;
  byte i = 0;
  AT24C32.readEE (0, crc);
  while (i < (sizeof(varinfo)))
  {
    crcrom += AT24C32.readByte ((i + 2));
    i++;
  }
  if (crc == crcrom) {
    AT24C32.readEE (2, varinfo);
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
  display.println(ver);
  display.display();
  delay(500);
}
