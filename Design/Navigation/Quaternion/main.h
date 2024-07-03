#ifndef MAIN_H_
#define MAIN_H_
// https://www.youtube.com/watch?v=i1phJl-0v54&t=1220s
#include <avr/io.h>
#include <HardwareSerial.h>
#include "matrix_math.h"

#define FP_INT_TYPE         int32_t
#define FP_INT_XL_TYPE      int64_t
typedef FP_INT_TYPE         fpint_t;
typedef FP_INT_XL_TYPE      fpintxl_t;
// Bit shifting left by num multiplies the bit shifted number by pow(2,num)
// Bit shifting right by num divides the bit shifted number by pow(2,num)
#define FP_NUM_BITS         (sizeof(fpint_t) << 3) 
// Bits assigned to whole number
#define FP_INT_BITS         (14) //2^14 = 16,384
// Bits assigned to fractional number
#define FP_FRAC_BITS        (FP_NUM_BITS-FP_INT_BITS)

#define FP_SCALE            (FP_FRAC_BITS)        
#define FP_SCALE_FACTOR     (1L<<FP_SCALE)

float fp_to_float (fpint_t a){
    float x = (float)a/FP_SCALE_FACTOR;
    return x;
}

fpint_t float_to_fp (float a){
    fpint_t x = a*FP_SCALE_FACTOR;
    return x;
}

fpint_t fp_mult(fpint_t a, fpint_t b){
    return (((fpintxl_t)a*b)>>FP_SCALE);
}

fpint_t fp_div(fpint_t a, fpint_t b){
    return ((fpintxl_t)a<<FP_SCALE)/b;
}

#endif
