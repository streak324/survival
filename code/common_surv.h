#include <stdint.h>

#ifndef COMMON_SURV_H
#define COMMON_SURV_H

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t  i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef float f32;
typedef double f64;

typedef i32 b32;

#if DEBUG_SURV
#define Assert(expression) {if(!(expression)) {InvalidCodePath;}}
#else
#define Assert(expression)
#endif

#define InvalidCodePath (* (int*) 0 = 0)

#endif
