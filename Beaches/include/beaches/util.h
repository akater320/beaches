#pragma once
#ifndef BEACHES_UTIL_H
#define BEACHES_UTIL_H

namespace beaches {
    //Sign may not be preserved.
    template<int numberBytes, typename T>
    void PackIntoBytes(uint8_t* startByte, T value) {
        for (int i = 0; i < numberBytes; i++) {
            startByte[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xff);
        }
    }

    template<int numberBytes, typename T>
    T UnpackFromBytes(const uint8_t* startByte) {
        T value = startByte[numberBytes - 1];
        for (int i = numberBytes - 2; i >= 0; i--) {
            value <<= 8;
            value |= static_cast<T>(startByte[i]);
        }

        return value;
    }
}

#endif // !BEACHES_UTIL_H
