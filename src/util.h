#pragma once
#include <stdio.h>
#include <string>
#include <cassert>

#define null nullptr
#define print_error(...) fprintf(stderr, __VA_ARGS__)

using u8 = unsigned char;
using u16 = unsigned short;
using u32 = unsigned int;
using u64 = unsigned long int;
using f32 = float;

#define F32_INF (f32)(1.0f / 0.0f)
#define F32_NAN (f32)(0.0f / 0.0f)

inline bool f32_eq(f32 a, f32 b, float epsilon = 1e-4f) {
    return std::abs(a - b) < epsilon;
}