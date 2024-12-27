#pragma once
#include <cstdint>
#include "nes6502.h"
#include <array>

class Bus {
public:
        Bus();
        ~Bus();

public: // devices
        nes6502 cpu;
        std::array<uint8_t, 64 * 1024> ram;

public: 
        uint8_t read(uint16_t addr, bool bReadOnly = false);
        void write(uint16_t addr, uint8_t data);
};
