#include "nes6502.h"
#include "bus.h"
#include <cstdint>
#include <ctime>

nes6502::nes6502() {
    // assembles the instruction lookup table
    using a = nes6502;
    lookup = {
        { "BRK", &a::BRK, &a::IMM, 7 },{ "ORA", &a::ORA, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "ORA", &a::ORA, &a::ZP0, 3 },
        { "ASL", &a::ASL, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PHP", &a::PHP, &a::IMP, 3 },{ "ORA", &a::ORA, &a::IMM, 2 },{ "ASL", &a::ASL, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },
        { "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ABS, 4 },{ "ASL", &a::ASL, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "BPL", &a::BPL, &a::REL, 2 },{ "ORA", &a::ORA, &a::IZY, 5 },
        { "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ZPX, 4 },{ "ASL", &a::ASL, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },
        { "CLC", &a::CLC, &a::IMP, 2 },{ "ORA", &a::ORA, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ABX, 4 },
        { "ASL", &a::ASL, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },{ "JSR", &a::JSR, &a::ABS, 6 },{ "AND", &a::AND, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },
        { "BIT", &a::BIT, &a::ZP0, 3 },{ "AND", &a::AND, &a::ZP0, 3 },{ "ROL", &a::ROL, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PLP", &a::PLP, &a::IMP, 4 },{ "AND", &a::AND, &a::IMM, 2 },
        { "ROL", &a::ROL, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "BIT", &a::BIT, &a::ABS, 4 },{ "AND", &a::AND, &a::ABS, 4 },{ "ROL", &a::ROL, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
        { "BMI", &a::BMI, &a::REL, 2 },{ "AND", &a::AND, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "AND", &a::AND, &a::ZPX, 4 },
        { "ROL", &a::ROL, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SEC", &a::SEC, &a::IMP, 2 },{ "AND", &a::AND, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },
        { "???", &a::NOP, &a::IMP, 4 },{ "AND", &a::AND, &a::ABX, 4 },{ "ROL", &a::ROL, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },{ "RTI", &a::RTI, &a::IMP, 6 },{ "EOR", &a::EOR, &a::IZX, 6 },
        { "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "EOR", &a::EOR, &a::ZP0, 3 },{ "LSR", &a::LSR, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },
        { "PHA", &a::PHA, &a::IMP, 3 },{ "EOR", &a::EOR, &a::IMM, 2 },{ "LSR", &a::LSR, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "JMP", &a::JMP, &a::ABS, 3 },{ "EOR", &a::EOR, &a::ABS, 4 },
        { "LSR", &a::LSR, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "BVC", &a::BVC, &a::REL, 2 },{ "EOR", &a::EOR, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },
        { "???", &a::NOP, &a::IMP, 4 },{ "EOR", &a::EOR, &a::ZPX, 4 },{ "LSR", &a::LSR, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLI", &a::CLI, &a::IMP, 2 },{ "EOR", &a::EOR, &a::ABY, 4 },
        { "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "EOR", &a::EOR, &a::ABX, 4 },{ "LSR", &a::LSR, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
        { "RTS", &a::RTS, &a::IMP, 6 },{ "ADC", &a::ADC, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "ADC", &a::ADC, &a::ZP0, 3 },
        { "ROR", &a::ROR, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PLA", &a::PLA, &a::IMP, 4 },{ "ADC", &a::ADC, &a::IMM, 2 },{ "ROR", &a::ROR, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },
        { "JMP", &a::JMP, &a::IND, 5 },{ "ADC", &a::ADC, &a::ABS, 4 },{ "ROR", &a::ROR, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "BVS", &a::BVS, &a::REL, 2 },{ "ADC", &a::ADC, &a::IZY, 5 },
        { "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "ADC", &a::ADC, &a::ZPX, 4 },{ "ROR", &a::ROR, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },
        { "SEI", &a::SEI, &a::IMP, 2 },{ "ADC", &a::ADC, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "ADC", &a::ADC, &a::ABX, 4 },
        { "ROR", &a::ROR, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 2 },{ "STA", &a::STA, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 6 },
        { "STY", &a::STY, &a::ZP0, 3 },{ "STA", &a::STA, &a::ZP0, 3 },{ "STX", &a::STX, &a::ZP0, 3 },{ "???", &a::XXX, &a::IMP, 3 },{ "DEY", &a::DEY, &a::IMP, 2 },{ "???", &a::NOP, &a::IMP, 2 },
        { "TXA", &a::TXA, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "STY", &a::STY, &a::ABS, 4 },{ "STA", &a::STA, &a::ABS, 4 },{ "STX", &a::STX, &a::ABS, 4 },{ "???", &a::XXX, &a::IMP, 4 },
        { "BCC", &a::BCC, &a::REL, 2 },{ "STA", &a::STA, &a::IZY, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "STY", &a::STY, &a::ZPX, 4 },{ "STA", &a::STA, &a::ZPX, 4 },
        { "STX", &a::STX, &a::ZPY, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "TYA", &a::TYA, &a::IMP, 2 },{ "STA", &a::STA, &a::ABY, 5 },{ "TXS", &a::TXS, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 5 },
        { "???", &a::NOP, &a::IMP, 5 },{ "STA", &a::STA, &a::ABX, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "LDY", &a::LDY, &a::IMM, 2 },{ "LDA", &a::LDA, &a::IZX, 6 },
        { "LDX", &a::LDX, &a::IMM, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "LDY", &a::LDY, &a::ZP0, 3 },{ "LDA", &a::LDA, &a::ZP0, 3 },{ "LDX", &a::LDX, &a::ZP0, 3 },{ "???", &a::XXX, &a::IMP, 3 },
        { "TAY", &a::TAY, &a::IMP, 2 },{ "LDA", &a::LDA, &a::IMM, 2 },{ "TAX", &a::TAX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "LDY", &a::LDY, &a::ABS, 4 },{ "LDA", &a::LDA, &a::ABS, 4 },
        { "LDX", &a::LDX, &a::ABS, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "BCS", &a::BCS, &a::REL, 2 },{ "LDA", &a::LDA, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 5 },
        { "LDY", &a::LDY, &a::ZPX, 4 },{ "LDA", &a::LDA, &a::ZPX, 4 },{ "LDX", &a::LDX, &a::ZPY, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "CLV", &a::CLV, &a::IMP, 2 },{ "LDA", &a::LDA, &a::ABY, 4 },
        { "TSX", &a::TSX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 4 },{ "LDY", &a::LDY, &a::ABX, 4 },{ "LDA", &a::LDA, &a::ABX, 4 },{ "LDX", &a::LDX, &a::ABY, 4 },{ "???", &a::XXX, &a::IMP, 4 },
        { "CPY", &a::CPY, &a::IMM, 2 },{ "CMP", &a::CMP, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "CPY", &a::CPY, &a::ZP0, 3 },{ "CMP", &a::CMP, &a::ZP0, 3 },
        { "DEC", &a::DEC, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "INY", &a::INY, &a::IMP, 2 },{ "CMP", &a::CMP, &a::IMM, 2 },{ "DEX", &a::DEX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },
        { "CPY", &a::CPY, &a::ABS, 4 },{ "CMP", &a::CMP, &a::ABS, 4 },{ "DEC", &a::DEC, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "BNE", &a::BNE, &a::REL, 2 },{ "CMP", &a::CMP, &a::IZY, 5 },
        { "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "CMP", &a::CMP, &a::ZPX, 4 },{ "DEC", &a::DEC, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },
        { "CLD", &a::CLD, &a::IMP, 2 },{ "CMP", &a::CMP, &a::ABY, 4 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "CMP", &a::CMP, &a::ABX, 4 },
        { "DEC", &a::DEC, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },{ "CPX", &a::CPX, &a::IMM, 2 },{ "SBC", &a::SBC, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },
        { "CPX", &a::CPX, &a::ZP0, 3 },{ "SBC", &a::SBC, &a::ZP0, 3 },{ "INC", &a::INC, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "INX", &a::INX, &a::IMP, 2 },{ "SBC", &a::SBC, &a::IMM, 2 },
        { "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::SBC, &a::IMP, 2 },{ "CPX", &a::CPX, &a::ABS, 4 },{ "SBC", &a::SBC, &a::ABS, 4 },{ "INC", &a::INC, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
        { "BEQ", &a::BEQ, &a::REL, 2 },{ "SBC", &a::SBC, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "SBC", &a::SBC, &a::ZPX, 4 },
        { "INC", &a::INC, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SED", &a::SED, &a::IMP, 2 },{ "SBC", &a::SBC, &a::ABY, 4 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },
        { "???", &a::NOP, &a::IMP, 4 },{ "SBC", &a::SBC, &a::ABX, 4 },{ "INC", &a::INC, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
    };
}

nes6502::~nes6502() {}

/*-------------------------------------------
 *            BUS CONNECTIVITY
 * ----------------------------------------*/

uint8_t nes6502::read(uint16_t a) 
{
        return bus->read(a, false);
}

void nes6502::write(uint16_t a, uint8_t d) 
{
        bus->write(a, d);
}

/*-------------------------------------------
 *            EXTERNAL INPUTS
 * ----------------------------------------*/

void nes6502::reset() 
{
        // reset internal registers
        a = 0;
        x = 0;
        y = 0;
        stkp = 0xFD;
        status = 0x00 | U;

        // set pc
        addr_abs = 0xFFFC;
        uint16_t lo = read(addr_abs + 0);
        uint16_t hi = read(addr_abs + 1);
        pc = (hi << 8) | lo;

        // clear internal pointers
        addr_abs = 0x0000;
        addr_rel = 0x0000;
        fetched = 0x00;

        cycles = 8;
}

void nes6502::irq() 
{
        if (GetFlag(I) == 0) {
                // write pc to stack
                write(0x0100 + stkp, (pc >> 8) & 0x00FF); // hi bytes in 8-bit form
                stkp--;
                write(0x0100 + stkp, pc & 0x00FF); // lo bytes
                stkp--;

                // push status to stack
                SetFlag(B, 0); SetFlag(U, 1); SetFlag(I, 1);
                write(0x0100 + stkp, status);
                stkp--;

                // read new pc
                addr_abs = 0xFFFE;
                uint16_t lo = read(addr_abs + 0);
                uint16_t hi = read(addr_abs + 1);
                pc = (hi << 8) | lo;

                cycles = 7;
        }
}

void nes6502::nmi() 
{
        // write pc to stack
        write(0x0100 + stkp, (pc >> 8) & 0x00FF); // hi bytes in 8-bit form
        stkp--;
        write(0x0100 + stkp, pc & 0x00FF); // lo bytes
        stkp--;

        // push status to stack
        SetFlag(B, 0); SetFlag(U, 1); SetFlag(I, 1);
        write(0x0100 + stkp, status);
        stkp--;

        // read new pc
        addr_abs = 0xFFFA;
        uint16_t lo = read(addr_abs + 0);
        uint16_t hi = read(addr_abs + 1);
        pc = (hi << 8) | lo;

        cycles = 8;
}

void nes6502::clock() 
{
        if (cycles == 0) {
                opcode = read(pc);
                pc++;
                cycles = lookup[opcode].cycles;

                // calls address mode & operator functions
                // returns the number of additional cycles needed
                uint8_t addrmode_addt_cycles = (this->*lookup[opcode].addrmode)();
                uint8_t operate_addt_cycles = (this->*lookup[opcode].operate)();

                cycles += (addrmode_addt_cycles & operate_addt_cycles);
        }
        cycles--;
}

/*-------------------------------------------
 *             FLAG FUNCTIONS
 * ----------------------------------------*/

// returns the value of the f flag in status (either 0 or 1)
uint8_t nes6502::GetFlag(FLAGS6502 f) 
{
        return ((status & f) > 0) ? 1 : 0;
}

// if v, set flag f in status, otherwise clear flag f
void nes6502::SetFlag(FLAGS6502 f, bool v) 
{
        if (v) 
                status |= f;
        else 
                status &= ~f;
}

/*-------------------------------------------
 *            ADDRESSING MODES
 * ----------------------------------------*/

// implied
uint8_t nes6502::IMP() 
{
        fetched = a;
        return 0;
}

// immediate
uint8_t nes6502::IMM() 
{
        addr_abs = pc++;
        return 0;
}

// zero page
uint8_t nes6502::ZP0() 
{
        addr_abs = read(pc);
        pc++;
        addr_abs &= 0x00FF;
        return 0;
}

// zero page with x offset
uint8_t nes6502::ZPX() 
{
        addr_abs = (read(pc) + x);
        pc++;
        addr_abs &= 0x00FF;
        return 0;
}

// zero page with y offset
uint8_t nes6502::ZPY() 
{
        addr_abs = (read(pc) + y);
        pc++;
        addr_abs &= 0x00FF;
        return 0;
}

// relative
uint8_t nes6502::REL() 
{
        addr_rel = read(pc);
        pc++;
        if (addr_rel & 0x80) { // if addr_rel negative (8 bit two's complement)
                addr_rel |= 0xFF00; // convert to same number in 16 bit two's complement
        }
        return 0;
}

// absolute
uint8_t nes6502::ABS() 
{
        uint16_t lo = read(pc++);
        uint16_t hi = read(pc++);
        addr_abs = (hi << 8) | lo;
        return 0;
}

// absolute with x offset
uint8_t nes6502::ABX() 
{
        uint16_t lo = read(pc++);
        uint16_t hi = read(pc++);
        addr_abs = (hi << 8) | lo;
        addr_abs += x;
        if ((addr_abs & 0xFF00) != (hi << 8))
                return 1; // additional clock cycle needed since we crossed a page boundary
        else
                return 0;
}

// absolute with x offset
uint8_t nes6502::ABY() 
{
        uint16_t lo = read(pc++);
        uint16_t hi = read(pc++);
        addr_abs = (hi << 8) | lo;
        addr_abs += y;
        if ((addr_abs & 0xFF00) != (hi << 8))
                return 1; // additional clock cycle needed since we crossed a page boundary
        else
                return 0;
}

// indirect 
uint8_t nes6502::IND() 
{
        uint16_t lo = read(pc++);
        uint16_t hi = read(pc++);
        uint16_t ptr = (hi << 8) | lo;
        if (lo == 0x00FF)
                addr_abs = (read(ptr & 0xFF00) << 8) | read(ptr + 0); 
        else
                addr_abs = (read(ptr + 1) << 8) | read(ptr + 0); 
        return 0;
}

// indexed indirect (ind, x) 
uint8_t nes6502::IZX() 
{
        uint16_t ptr = read(pc++);
        uint16_t lo = read((uint16_t)(ptr + (uint16_t)x) & 0x00FF);
        uint16_t hi = read((uint16_t)(ptr + (uint16_t)x + 1) & 0x00FF);
        addr_abs = (hi << 8) | lo;
        return 0;
}

// indirect indexed (ind, y) 
uint8_t nes6502::IZY() 
{
        uint16_t ptr = read(pc++);
        uint16_t lo = read(ptr & 0x00FF);
        uint16_t hi = read((ptr + 1) & 0x00FF);
        addr_abs = (hi << 8) | lo;
        addr_abs += y;
        if ((addr_abs & 0xFF00) != (hi << 8))
                return 1;
        else
                return 0;
}

/*-------------------------------------------
 *             INSTRUCTIONS
 * ----------------------------------------*/

uint8_t nes6502::fetch() 
{
        // in implied mode, just fetch from accumulator
        // otherwise, find the respective location in memory
        if (!(lookup[opcode].addrmode == &nes6502::IMP))
                fetched = read(addr_abs);
        return fetched;
}

// Add with Carry: A = A + M + C
// flags: C, V, N, Z
uint8_t nes6502::ADC() 
{
        fetch();
        temp = (uint16_t) a + (uint16_t) fetched + (uint16_t) GetFlag(C);
        SetFlag(C, temp > 255);
        SetFlag(Z, (temp & 0x00FF) == 0);
        // MSB of A and M are the same but different from R (temp)
        // i.e. P + P = N as well as N + N = P cause overflows
        SetFlag(V, (~((uint16_t) a ^ (uint16_t) fetched) & ((uint16_t) a ^ (uint16_t) temp)) & 0x0080);
        SetFlag(N, temp & 0x0080);
        a = temp & 0x00FF;
        return 1;
}

// Subtract with Carry: A = A - M - (~C) = A - M - (1 - C) = A + (-M + 1) + C
// flags: C, V, N, Z
uint8_t nes6502::SBC() 
{
        fetch();
        uint16_t inv_m = ((uint16_t) fetched) ^ 0x00FF; // note that in two's complement, the inversion of M is the same as -M + 1
        temp = (uint16_t) a + inv_m + (uint16_t) GetFlag(C);
        SetFlag(C, temp & 0xFF00);
        SetFlag(Z, ((temp & 0x00FF) == 0));
        SetFlag(V, (temp ^ (uint16_t) a) & (temp ^ inv_m) & 0x0080); // set V if MSB of R is diff from A and (-M+1) --note: equiv. logic as in ADC()
        SetFlag(N, temp & 0x0080);
        a = temp & 0x00FF;
        return 1;
}

// bitwise AND: A = A & M
// flags: N, Z
uint8_t nes6502::AND() 
{
        fetch();
        a = a & fetched;
        SetFlag(Z, a == 0x00);
        SetFlag(N, a & 0x80);
        return 1;
}

// Arithmetic Shift Left
// flags: N, Z, C
uint8_t nes6502::ASL() 
{
        fetch();
        temp = (uint16_t) fetched << 1;
        SetFlag(C, (temp & 0xFF00) > 0);
        SetFlag(Z, (temp & 0x00FF) == 0);
        SetFlag(N, temp & 0x80);
        if (lookup[opcode].addrmode == &nes6502::IMP)
                a = temp & 0x00FF;
        else
                write(addr_abs, temp & 0x00FF);
        return 0;
}

// Branch if Carry Clear: if (C == 0) then pc = address
uint8_t nes6502::BCC() 
{
        if (GetFlag(C) == 0) {
                cycles++;
                addr_abs = pc + addr_rel;
                if ((addr_abs & 0xFF00) != (pc & 0xFF00))
                        cycles++; // add cycle if we cross a page boundary
                pc = addr_abs;
        }
        return 0;
}

// Branch if Carry Set: if (C == 1) then pc = address
uint8_t nes6502::BCS() 
{
        if (GetFlag(C) == 1) {
                cycles++;
                addr_abs = pc + addr_rel;
                if ((addr_abs & 0xFF00) != (pc & 0xFF00))
                        cycles++;
                pc = addr_abs;
        }
        return 0;
}

// Branch if Equal: (if Z == 1) then pc = address
uint8_t nes6502::BEQ() 
{
        if (GetFlag(Z) == 1) {
                cycles++;
                addr_abs = pc + addr_rel;
                if ((addr_abs & 0xFF00) != (pc & 0xFF00))
                        cycles++;
                pc = addr_abs;
        }
        return 0;
}

// Test Bits in Memory with Accumulator
uint8_t nes6502::BIT() 
{
        fetch();
        temp = a & fetched;
        SetFlag(Z, (temp & 0x00FF) == 0x00);
        SetFlag(N, fetched & (1 << 7));
        SetFlag(V, fetched & (1 << 6));
        return 0;
}

// Branch if Result Minus: (if N == 1) then pc = address
uint8_t nes6502::BMI() 
{
        if (GetFlag(N) == 1) {
                cycles++;
                addr_abs = pc + addr_rel;
                if ((addr_abs & 0xFF00) != (pc & 0xFF00)) {
                        cycles++;
                }
                pc = addr_abs;
        }
        return 0;
}

// Branch if Not Equal: (if Z == 0) then pc = address
uint8_t nes6502::BNE() 
{
        if (GetFlag(Z) == 0) {
                cycles++;
                addr_abs = pc + addr_rel;
                if ((addr_abs & 0xFF00) != (pc & 0xFF00))
                        cycles++;
                pc = addr_abs;
        }
        return 0;
}

// Branch if Result Plus: (if N == 0) then pc = address
uint8_t nes6502::BPL() 
{
        if (GetFlag(N) == 0) {
                cycles++;
                addr_abs = pc + addr_rel;
                if ((addr_abs & 0xFF00) != (pc & 0xFF00))
                        cycles++;
                pc = addr_abs;
        }
        return 0;
}

// Break: software interrupt
uint8_t nes6502::BRK() 
{
        // add extra byte for spacing
        pc++; // TODO: maybe increment by 2?

        // write pc to stack
        SetFlag(I, 1);
        write(0x0100 + stkp, (pc >> 8) & 0x00FF); // hi bytes (in 8 bits)
        stkp--;
        write(0x0100 + stkp, pc & 0x00FF); // lo bytes
        stkp--;

        // push status to stack
        SetFlag(B, 1);
        write(0x0100 + stkp, status);
        stkp--;
        SetFlag(B, 0);

        // set pc to interrupt vector
        pc = (uint16_t) read(0xFFFE) | ((uint16_t) read(0xFFFF) << 8);

        return 0;
}

// Branch if Overflow Clear: (if V == 0) then pc = address
uint8_t nes6502::BVC() 
{
        if (GetFlag(V) == 0) {
                cycles++;
                addr_abs = pc + addr_rel;
                if ((addr_abs & 0xFF00) != (pc & 0xFF00))
                        cycles++;
                pc = addr_abs;
        }
        return 0;
}

// Branch if Overflow Set: (if V == 1) then pc = address
uint8_t nes6502::BVS() 
{
        if (GetFlag(V) == 1) {
                cycles++;
                addr_abs = pc + addr_rel;
                if ((addr_abs & 0xFF00) != (pc & 0xFF00))
                        cycles++;
                pc = addr_abs;
        }
        return 0;
}

// Clear Carry Flag
uint8_t nes6502::CLC() 
{
        SetFlag(C, false);
        return 0;
}

// Clear Decimal Flag
uint8_t nes6502::CLD() 
{
        SetFlag(D, false);
        return 0;
}

// Clear Disable Interrupts Flag
uint8_t nes6502::CLI() 
{
        SetFlag(C, false);
        return 0;
}

// Clear Overflow Flag
uint8_t nes6502::CLV() 
{
        SetFlag(V, false);
        return 0;
}

// Compare with Accumulator
uint8_t nes6502::CMP()
{
        fetch();
        temp = (uint16_t) a - (uint16_t) fetched; 
        SetFlag(C, a >= fetched);
        SetFlag(Z, (temp & 0x00FF) == 0x0000);
        SetFlag(N, temp & 0x0080);
        return 1;
}

// Compare with X
uint8_t nes6502::CPX()
{
        fetch();
        temp = (uint16_t) x - (uint16_t) fetched; 
        SetFlag(C, x >= fetched);
        SetFlag(Z, (temp & 0x00FF) == 0x0000);
        SetFlag(N, temp & 0x0080);
        return 0;
}

// Compare with Y
uint8_t nes6502::CPY()
{
        fetch();
        temp = (uint16_t) y - (uint16_t) fetched; 
        SetFlag(C, y >= fetched);
        SetFlag(Z, (temp & 0x00FF) == 0x0000);
        SetFlag(N, temp & 0x0080);
        return 0;
}

// Decrement Memory
uint8_t nes6502::DEC()
{
        fetch();
        temp = fetched - 1;
        write(addr_abs, temp & 0x00FF);
        SetFlag(Z, (temp & 0x00FF) == 0x0000);
        SetFlag(N, temp & 0x0080);
        return 0;
}

// Decrement X
uint8_t nes6502::DEX()
{
        x--;
        SetFlag(Z, x == 0x00);
        SetFlag(N, x & 0x80);
        return 0;
}

// Decrement Memory
uint8_t nes6502::DEY()
{
        y--;
        SetFlag(Z, y == 0x00);
        SetFlag(N, y & 0x80);
        return 0;
}

// Exclusive OR
uint8_t nes6502::EOR()
{
        fetch();
        a = a ^ fetched;
        SetFlag(Z, a == 0x00);
        SetFlag(N, a & 0x80);
        return 1;
}

// Increment Memory
uint8_t nes6502::INC()
{
        fetch();
        temp = fetched + 1;
        write(addr_abs, temp & 0x00FF);
        SetFlag(Z, (temp & 0x00FF) == 0x0000);
        SetFlag(N, temp & 0x0080);
        return 0;
}

// Increment X
uint8_t nes6502::INX()
{
        x++;
        SetFlag(Z, x == 0x00);
        SetFlag(N, x & 0x80);
        return 0;
}

// Increment Y
uint8_t nes6502::INY()
{
        y++;
        SetFlag(Z, y == 0x00);
        SetFlag(N, y & 0x80);
        return 0;
}

// Jump
uint8_t nes6502::JMP()
{
        pc = addr_abs;
        return 0;
}

// Jump to Subroutine: push current pc to stack, set pc to address
uint8_t nes6502::JSR()
{
        pc--;

        write(0x0100 + stkp, (pc >> 8) & 0x00FF);
        stkp--;
        write(0x0100 + stkp, pc & 0x00FF);
        stkp--;

        pc = addr_abs;
        return 0;
}

// Load Accumulator
uint8_t nes6502::LDA()
{
        fetch();
        a = fetched;
        SetFlag(Z, a == 0x00);
        SetFlag(N, a & 0x80);
        return 1;
}

// Load X
uint8_t nes6502::LDX()
{
        fetch();
        x = fetched;
        SetFlag(Z, x == 0x00);
        SetFlag(N, x & 0x80);
        return 1;
}

// Load Y
uint8_t nes6502::LDY()
{
        fetch();
        y = fetched;
        SetFlag(Z, y == 0x00);
        SetFlag(N, y & 0x80);
        return 1;
}

// Logical Shift Right
uint8_t nes6502::LSR()
{
        fetch();
        SetFlag(C, fetched & 0x0001);
        temp = fetched >> 1;
        SetFlag(Z, (temp & 0x00FF) == 0x0000);
        SetFlag(N, temp & 0x0080);
        if (lookup[opcode].addrmode == &nes6502::IMP)
                a = temp & 0x00FF;
        else
                write(addr_abs, temp & 0x00FF);
        return 0;
}

// No Operation
// https://www.nesdev.org/wiki/CPU_unofficial_opcodes
uint8_t nes6502::NOP()
{
        switch (opcode) {
        case 0x1C:
        case 0x3C:
        case 0x5C:
        case 0x7C:
        case 0xDC:
        case 0xFC:
                return 1;
                break;
        }
        return 0;
}

// OR with Accumulator
uint8_t nes6502::ORA()
{
        fetch();
        a = a | fetched;
        SetFlag(Z, a == 0x00);
        SetFlag(N, a & 0x80);
        return 1;
}

// Push Accumulator on Stack
uint8_t nes6502::PHA() 
{
        write(0x0100 + stkp, a);
        stkp--;
        return 0;
}

// Push Processor Status
uint8_t nes6502::PHP()
{
        write(0x0100 + stkp, status | B | U);
        SetFlag(B, 0);
        SetFlag(U, 0);
        stkp--;
        return 0;
}

// Pull Accumulator on Stack
uint8_t nes6502::PLA() 
{
        stkp++;
        a = read(0x0100 + stkp);
        SetFlag(Z, a == 0x00);
        SetFlag(N, a == 0x80);
        return 0;
}

// Pull Processor Status
uint8_t nes6502::PLP()
{
        stkp++;
        status = read(0x0100 + stkp);
        SetFlag(U, 1);
        return 0;
}

// Return from Interrupt
// restore pc, status (ignoring B & U)
uint8_t nes6502::RTI() 
{
        stkp++;
        status = read(0x0100 + stkp);
        status &= ~B;
        status &= ~U;

        stkp++;
        pc = (uint16_t) read(0x0100 + stkp);
        stkp++;
        pc |= (uint16_t) read(0x0100 + stkp) << 8;
        return 0;
}
