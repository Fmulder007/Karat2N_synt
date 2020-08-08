/*
  UD0CAJ Karat2_sintez
  Si5351 CLK0 for VFO and CLK1 for LO!!!
  7-7,2 MHz
  500Hz step add
  Add Eeprom24C32 memory using
  Using si5351a lib from Andrew Bilokon, UR5FFR
  minimal I2C library from from Andrew Bilokon, UR5FFR
*/
char ver[ ] = "121u01";
#define SI_OVERCLOCK 750000000L
#define ENCODER_OPTIMIZE_INTERRUPTS

#define start_freq 7090000UL // Начальная частота VFO при первом включении.
#define start_lo_freq 496003UL // Начальная частота опоры при первом включении.
#define Si_Xtall_Freq 25000000UL // Частота кварца si5351, Гц
#define Si_Xtall_calFreq 2277 // Начальная частота калибровки кварца, Гц
#define lo_max_freq 550000UL // Максимальная частота опоры, Гц
#define lo_min_freq 450000UL // Минимальная частота опоры, Гц
#define start_min_freq 70 // *100KHz Минимальный предел частоты диапазона VFO
#define start_max_freq 72 // *100KHz Максимальный предел частоты диапазона VFO
#define min_hardware_freq 10 // *100KHz Минимальный железный предел частоты диапазона VFO
#define max_hardware_freq 250 // *100KHz Максимальный железный предел частоты диапазона VFO
#define start_batt_cal 253 // Начальная калибровка вольтметра
#define ONE_WIRE_BUS 12 // Порт датчика температуры
#define myEncBtn 4 // Порт нажатия кноба.
#define mypowerpin 14 // Порт показометра мощности. А0
#define mybattpin 15 // Порт датчика АКБ А1
#define txpin 5 //Порт датчика ТХ.


byte menu = 0; //Начальное положение меню.
byte arraystp[] = {1, 10, 50, 100}; //шаги настройки * 10 герц.

#include "Adafruit_SSD1306.h" // Use version 1.2.7!!!
#include "si5351a.h"
#include "Wire.h"
#include "Encoder.h"
#include "Eeprom24C32_64.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "DS1307RTC.h"



//#define ENCODER_DO_NOT_USE_INTERRUPTS



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
  int battcal = start_batt_cal;
  unsigned long freq = start_freq;
  unsigned long lofreq = start_lo_freq;
  int calibration = Si_Xtall_calFreq;
  int ifshift = 0;
  byte minfreq = start_min_freq;
  byte maxfreq = start_max_freq;
} varinfo;


// unsigned long previousMillis = 0;
unsigned long previousdsp = 0;
unsigned long previoustemp = 0;
unsigned long previoustime = 0;
unsigned long knobMillis = 0;
unsigned long actenc = 0;


static Eeprom24C32_64 AT24C32(0x50);
Si5351 si;
Encoder myEnc(3, 2); //порты подключения енкодера.
Adafruit_SSD1306 display(4);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
tmElements_t tm;


void setup() {
  Serial.begin(57600);
  pinMode(myEncBtn, INPUT);
  pinMode(mypowerpin, INPUT);
  digitalWrite(myEncBtn, HIGH);
  analogReference(INTERNAL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  sensors.begin();
  memread();
  si5351init();
  si5351correction();
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
  }
  //Если радио на передаче и отпустили ТХ то прописать IF-SHIFT
  if (!txsens && txen) {
    txen = false;
    RXifshift = varinfo.ifshift;
    vfosetup();
    mainscreen();
  }
  //Если радио на приеме и НЕ нажали ТХ и RXifshift не равно varinfo.ifshift то прописать IF-SHIFT
  if (!txsens && !txen && RXifshift != varinfo.ifshift) {
    txen = false;
    RXifshift = varinfo.ifshift;
    vfosetup();
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
        vfosetup();
        break;

      case 3: //Настройка опорного гетеродина
        if (newPosition > oldPosition && varinfo.lofreq <= lo_max_freq) varinfo.lofreq = varinfo.lofreq + arraystp[varinfo.stp];
        if (newPosition < oldPosition && varinfo.lofreq >= lo_min_freq) varinfo.lofreq = varinfo.lofreq - arraystp[varinfo.stp];
        if (varinfo.lofreq < lo_min_freq) varinfo.lofreq = lo_min_freq;
        if (varinfo.lofreq > lo_max_freq) varinfo.lofreq = lo_max_freq;
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
        if (varinfo.calibration <  -30000) varinfo.calibration =  -30000;
        si5351correction();
        vfosetup();
        break;

      case 6: //Настройка minfreq
        if (newPosition > oldPosition && varinfo.minfreq <= max_hardware_freq) varinfo.minfreq = varinfo.minfreq + 1;
        if (newPosition < oldPosition && varinfo.minfreq >= min_hardware_freq) varinfo.minfreq = varinfo.minfreq - 1;
        if (varinfo.minfreq < min_hardware_freq) varinfo.minfreq = min_hardware_freq;
        if (varinfo.minfreq >= varinfo.maxfreq) varinfo.minfreq = varinfo.maxfreq - 1;
        break;

      case 7: //Настройка maxfreq
        if (newPosition > oldPosition && varinfo.maxfreq <= max_hardware_freq) varinfo.maxfreq = varinfo.maxfreq + 1;
        if (newPosition < oldPosition && varinfo.maxfreq >= min_hardware_freq) varinfo.maxfreq = varinfo.maxfreq - 1;
        if (varinfo.maxfreq <= varinfo.minfreq) varinfo.maxfreq = varinfo.minfreq + 1;
        if (varinfo.maxfreq > max_hardware_freq) varinfo.maxfreq = max_hardware_freq;
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
  si.set_freq((varinfo.freq + varinfo.lofreq + RXifshift), 0, (varinfo.lofreq + RXifshift));
}

void si5351init() {
  si.setup();
}

void si5351correction() {
  si.set_xtal_freq(Si_Xtall_Freq + varinfo.calibration);
  si.update_freq(0);
  si.update_freq(2);
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
  delay(1000);
}
