#pragma once
#include <iostream>
#include <cassert>
#include <array>

namespace Luna {
using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

struct Label {
    u32 addr;
    std::string name;
};

enum Distance {
    Far = 0,
    Near
};

enum Displacement {
    Rel8, Rel16, Rel32, Automatic
};

enum Condition {
    o   = 0, 
    no  = 1, 
    b   = 2, nae = 2, c =  2,
    nb  = 3, ae  = 3, nc = 3,
    e   = 4, z  = 4,
    ne  = 5, nz = 5,
    na  = 6, be = 6,
    a   = 7, nbe = 7,
    s   = 8,
    ns  = 9,
    p   = 10, pe = 10,
    np  = 11, po = 11,
    l   = 12, nge = 12,
    nl  = 13, ge = 13,
    ng  = 14, le = 14,
    g   = 15, nle = 15
};

struct SIB {
    u8 scale, index, base;
    u32 displacement;
};

SIB currentSIB;

enum R64 {
    rax = 0, rcx, rdx, rbx, rsp, rbp, rsi, rdi, r8, r9, r10, r11, r12, r13, r14, r15
};

SIB& operator+(R64 base, R64 index) { // used for the [r64 + r64] syntax
    currentSIB.base = (u8) base;
    currentSIB.index = (u8)index;
    currentSIB.scale = 0;

    return currentSIB;
}

SIB& operator*(R64 index, int multiplier) { // used for the [r64 + r64 * multipler] syntax
    assert (multiplier == 1 || multiplier == 2 || multiplier == 4 || multiplier == 8);
    constexpr std::array <u8, 9> scales = {0, 0, 1, 0, 2, 0, 0, 0, 3};

    currentSIB.base = 0;
    currentSIB.index = (u8) index;
    currentSIB.scale = scales[multiplier];

    return currentSIB;
}

SIB& operator+(R64 base, SIB& sib) { // used for the [r64 + r64 * multipler] syntax
    currentSIB.base = (u8) base;
    return currentSIB;
}

SIB& operator+(SIB& sib, u32 displacement) { // used for the [r64 + r64 * multiplier + disp] syntax
    currentSIB.displacement = displacement;
    return currentSIB;
}

enum R32 {
    eax = 0, ecx, edx, ebx, esp, ebp, esi, edi, r8d, r9d, r10d, r11d, r12d, r13d, r14d, r15d
};

enum R16 {
    ax = 0, cx, dx, bx, sp, bp, si, di, r8w, r9w, r10w, r11w, r12w, r13w, r14w, r15w
};

enum R8 {
    al = 0, cl, dl, bl, ah, ch, dh, bh, r8b, r9b, r10b, r11b, r12b, r13b, r14b, r15b, 
    spl = 0x14, bpl, sil, dil // Special REX 8-bit registers
};

enum Segment {
    cs, ss, ds, es, fs, gs
};

class QWord {
public:
    R64* operator[](size_t idx) { return (R64*) (idx & 15); }
    constexpr const SIB& operator[](const SIB& sib) { return sib; } // syntax sugar, used for the r64, [r64 + r64 * multiplier + disp] syntax
};
} // end namespace Luna