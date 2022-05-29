/**
 * @file	efilib.h
 *
 * @date Feb 21, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include <stdint.h>

int minI(int i1, int i2);

#ifdef __cplusplus

// C++ helpers go here
namespace efi
{
    template <typename T, size_t N>
    constexpr size_t size(const T(&)[N]) {
        return N;
    }
}

#endif // __cplusplus
