
#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>

inline uint8_t crc8(const uint8_t* data, uint32_t len) {
    uint8_t crc = 0x00; // Init value

    for (uint32_t i = 0; i < len; ++i) {
        crc ^= data[i]; // XOR input byte
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc; // XorOut is 0x00
}

#endif
