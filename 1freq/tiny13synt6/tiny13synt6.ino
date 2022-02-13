#include "tiny5351.h"
uint8_t prv_ch = 48;

void setup() {
  DDRB &= ~(1 << PB3);      // For attiny it is PB3 for adc input
  ADMUX |= (0 << REFS0) | (1 << MUX1) | (1 << MUX0); // set reference and channel
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN); //set prescaller to 128 and enable ADC
  sei();
}

void loop() {
  uint8_t ch = (ReadADC() >> 6);
  if (prv_ch != ch && (ch == (ReadADC() >> 6))) {
    si5351_freq(ch);
    prv_ch = ch;
  }
}

uint16_t ReadADC() {
  ADCSRA |= (1 << ADSC);       // start conversion
  while (ADCSRA & (1 << ADSC)) {} // wait for conversion complete
  return ADC;
}
