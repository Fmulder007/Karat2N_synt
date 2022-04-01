//#include <avr/eeprom.h>
#include "tiny5351.h"
#include "slimmath.h"

uint32_t SI_XTAL_FREQ = 27018405UL; // Measured crystal frequency of XTAL2 for CL = 10pF
uint8_t SI_outPWR = 0;  //3 - 8mA, 2 - 6mA, 1 - 4mA 0 -2mA
uint32_t frequency = 3100000UL;
//uint8_t ch = 0;
uint16_t prv_ch = 0;
uint8_t count = 0;



void setup() {
  DDRB &= ~(1 << PB3);      // For attiny it is PB3 for adc input
  ADMUX |= (0 << REFS0) | (1 << MUX1) | (1 << MUX0); // set reference and channel
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN); //set prescaller to 128 and enable ADC
  sei();
  if (!ReadADC()) prv_ch++;
}

void loop() {
  uint16_t ch = ReadADC();
  if (ch == ReadADC()) {
    count++;
  }
  else {
    count = 0;
  }
  if (prv_ch != ch && count >= 24) {
    //si5351_freq(frequency + (ch << 8) , 0);
    si5351_freq(frequency + tmultiply(ch, 50) , 0);
    prv_ch = ch;
    count = 0;
  }
}

uint16_t ReadADC() {
  ADCSRA |= (1 << ADSC);       // start conversion
  while (ADCSRA & (1 << ADSC)) {} // wait for conversion complete
  return ADC;
}
