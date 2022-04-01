#include <inttypes.h>
#include <stdlib.h>
#include "i2cmaster.h"
#include "tiny5351.h"
#include "slimmath.h"


#define I2C_WRITE 0b11000000
#define I2C_READ  0b11000001


uint8_t i2cSendRegister(uint8_t reg, uint8_t data)
{
  i2c_start_wait(I2C_WRITE);
  i2c_write(reg);
  i2c_write(data);
  i2c_stop();
  return 0;
}

static void setupPLL( uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom, uint8_t rDiv)
{
  uint32_t P1;
  uint32_t P2;
  uint32_t P3;

  uint32_t mulresult = num << 7;
  div_result output = tdivide(mulresult, denom);
  uint32_t term =  output.quot;
  uint32_t mulresultmix = tmultiply(denom, term);
  P2 = mulresult - mulresultmix;
  mulresult = mult << 7;
  P1 = mulresult + term - 512;
  P3 = denom;
  i2cSendRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 1, (P3 & 0x000000FF));
  if (rDiv != R_DIV_NA)
  {
    i2cSendRegister(pll + 2,   ((P1 & 0x00030000) >> 16) | rDiv);
  }
  else
  {
    i2cSendRegister(pll + 2,   ((P1 & 0x00030000) >> 16));
  }
  i2cSendRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 4, (P1 & 0x000000FF));
  i2cSendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  i2cSendRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 7, (P2 & 0x000000FF));
}

void si5351_freq(uint32_t freq, uint8_t clk) //, uint8_t i, uint8_t q
{
  uint8_t si5351_mult;
  uint64_t pll_freq;
  //uint8_t r_div = 1;
  i2c_init();
  setupPLL(SI_SYNTH_PLL_A, 19, 0, 1, R_DIV_NA);
  //setupPLL(SI_SYNTH_PLL_B, 30, 0, 1,R_DIV_NA);
  pll_freq = (SI_XTAL_FREQ * 19);
  div_result output = tdivide(pll_freq, freq);
  si5351_mult = output.quot;
  uint32_t l =  output.remainder << 10;
  output = tdivide(l, freq);
  l = output.quot;
  l = l << 10;
  uint32_t num = l;
  const uint32_t denom = 0xFFFFF;
  setupPLL((SI_SYNTH_MS_0 + (8 * clk)), si5351_mult, num, denom, SI_R_DIV_1);
  i2cSendRegister((SI_CLK0_CONTROL + clk), (0x4C + SI_outPWR) | SI_CLK_SRC_PLL_A);
  i2cSendRegister(SI_PLL_RESET, 0xA0);
  i2cSendRegister(SI_CLK_OE,  ~(1 << clk)); // Enable
  i2c_exit();						// Exit I2C
}
