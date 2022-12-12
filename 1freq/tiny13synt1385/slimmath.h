#ifndef _SLIMMATH_H
#define _SLIMMATH_H
#endif


typedef struct div_result { 
    uint32_t remainder; 
	uint32_t quot;
} div_result;

#ifdef __cplusplus
extern "C"{
#endif

uint64_t tmultiply(uint32_t x, uint32_t y);
extern div_result tdivide(uint32_t divid,uint32_t divisor);

#ifdef __cplusplus
}
#endif
