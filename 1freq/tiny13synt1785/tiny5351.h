#ifndef TINY5351_H
#define TINY5351_H

#include <inttypes.h>

/* Borrowed from https://github.com/threeme3/QCX-SSB */

#define SI_XTAL_FREQ  27003660UL // Measured crystal frequency of XTAL2 for CL = 10pF
//extern uint32_t SI_XTAL_FREQ;
//extern uint8_t SI_outPWR;
#define SI_CLK_OE 3     // Register definitions
#define SI_CLK0_PHOFF 165
#define SI_CLK1_PHOFF 166
#define SI_CLK2_PHOFF 167



#define SI_CLK0_CONTROL	16			// Register definitions
#define SI_CLK1_CONTROL	17
#define SI_CLK2_CONTROL	18
#define SI_SYNTH_PLL_A	26
#define SI_SYNTH_PLL_B	34
#define SI_SYNTH_MS_0		42
#define SI_SYNTH_MS_1		50
#define SI_SYNTH_MS_2		58
#define SI_PLL_RESET		177

#define SI_R_DIV_1		0b00000000			// R-division ratio definitions
#define SI_R_DIV_2		0b00010000
#define SI_R_DIV_4		0b00100000
#define SI_R_DIV_8		0b00110000
#define SI_R_DIV_16		0b01000000
#define SI_R_DIV_32		0b01010000
#define SI_R_DIV_64		0b01100000
#define SI_R_DIV_128	0b01110000
#define R_DIV_NA		0b11111111

#define SI_CLK_SRC_PLL_A	0b00000000
#define SI_CLK_SRC_PLL_B	0b00100000

//#define XTAL_FREQ	27000000
#ifdef __cplusplus
extern "C" {
#endif			// Crystal frequency
void si5351_freq(uint32_t freq, uint8_t clk, uint8_t res);
#ifdef __cplusplus
}
#endif
#endif //TINY5351_H
