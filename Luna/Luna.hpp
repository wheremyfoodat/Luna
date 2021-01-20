#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <cstring>
#include <cstdlib>
#include <cassert>
#include <fstream>

#if defined(_WIN32) || defined(WIN32)
#include <windows.h>
uint8_t* getExecutableMemory (size_t size) {
    const auto buffer = static_cast <uint8_t*> (VirtualAlloc(nullptr, size, MEM_COMMIT, PAGE_EXECUTE_READWRITE));
    if (!buffer)
        printf ("Executable memory allocation failed\n");
    return buffer;
}

#else
#include <sys/mman.h>
uint8_t* getExecutableMemory (size_t size) {
    auto buffer = static_cast<uint8_t*> (mmap(nullptr, size, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
    if (!buffer)
        printf ("Couldn't allocate executable memory! :(\n");
    return buffer;
}
#endif


namespace Luna {
using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

struct Label {
    u32 addr;
    std::string name;
};

const auto REX   = 0b01000000; 
const auto REX_W = 0b01001000; // Promotes some instructions to 64 bits
const auto REX_R = 0b01000100; // Acts as the 4th bit in a register number (for accessing r8/9/10 etc)
const auto REX_B = 0b01000001; // Same as REX_R

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

enum R64 {
    rax = 0, rcx, rdx, rbx, rsp, rbp, rsi, rdi, r8, r9, r10, r11, r12, r13, r14, r15
};

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

const std::array <const R64*, 16> qword = {(R64*) 0, (R64*) 1, (R64*) 2, (R64*) 3, (R64*) 4, (R64*) 5, (R64*) 6, (R64*) 7, (R64*) 8, (R64*) 9, (R64*) 10, (R64*) 11, (R64*) 12, (R64*) 13, (R64*) 14, (R64*) 15};   // Used for memory accesses. Eg access [rax] by doing qword [rax]

class Generator {
    static const size_t kilobyte = 1024;
    u32 bufferPtr = 0;
    u8* buffer;
    std::vector <Label> labels;

public:
  Generator (size_t codeSize = kilobyte) {
    buffer = getExecutableMemory(codeSize);
  }
  
  template <typename T>
  constexpr void write (T val) {
      std::memcpy((void*) &buffer[bufferPtr], (void*) &val, sizeof(T));
      bufferPtr += sizeof(T);
  }
  
  template <Distance d = Distance::Near>
  constexpr void ret() { 
    if constexpr (d == Distance::Far) // ret far
        write <u8> (0xCB);
    else // ret near
        write <u8> (0xC3);
  }

  template <u16 opcode>
  constexpr void movx (R16 dest, R8 src) {
      write <u8> (0x66); // size override prefix
      movx <opcode> ((R32) dest, src);
  }

  template <u16 opcode>
  constexpr void movx (R32 dest, R8 src) {
    auto rex = 0;
    if (dest >= R32::r8d)
        rex |= REX_R;

    if (src >= R8::spl) // handle the "special" REX behavior with 8-bit regs
        rex |= REX;
    else if (src >= R8::r8b)
        rex |= REX_B;

    if (rex) // REX
        write <u8> (rex);

    write <u16> (opcode); // opcode
    write <u8> (((dest & 7) << 3) | (src & 7) | (0b11 << 6)); // mod r/m   
  }
    
  template <u16 opcode>
  constexpr void movx (R64 dest, R8 src) {
    auto rex = REX_W;
    if (dest >= R64::r8)
        rex |= REX_R;

    if (src >= R8::r8b && src <= R8::spl)
        rex |= REX_B;

    write <u8> (rex); // REX prefix
    write <u16> (opcode); // opcode
    write <u8> (((dest & 7) << 3) | (src & 7) | (0b11 << 6)); // mod r/m   
  }

  template <u16 opcode>
  constexpr void movx (R32 dest, R16 src) {
    auto rex = 0;
      
    if (dest >= R32::r8d)
        rex |= REX_R;

    if (src >= R16::r8w)
        rex |= REX_B;
    
    if (rex)
        write <u8> (rex); // REX prefix
    write <u16> (opcode); // opcode
    write <u8> (((dest & 7) << 3) | (src & 7) | (0b11 << 6)); // mod r/m   
  }

  template <u16 opcode> 
  constexpr void movx (R64 dest, R16 src) {
    auto rex = REX_W;
      
    if (dest >= R64::r8)
        rex |= REX_R;

    if (src >= R16::r8w)
        rex |= REX_B;
    
    if (rex)
        write <u8> (rex); // REX prefix
    write <u16> (opcode); // opcode
    write <u8> (((dest & 7) << 3) | (src & 7) | (0b11 << 6)); // mod r/m   
  }
  
  template <u16 opcode, u16 extension> 
  constexpr void op_r32 (R32 reg) {
    if (reg >= R32::r8d) {
        write <u8> (REX_B); // REX prefix
    }

    write <u8> (opcode); // opcode
    write <u8> ((reg & 7) | (extension << 3) | 0b11000000); // mod r/m
  }
  
  template <u16 opcode, u16 extension> 
  constexpr void op_r64 (R64 reg) {
    auto rex = REX_W;
    if (reg >= R64::r8)
        rex |= REX_B;

    write <u16> ((opcode << 8) | rex); // REX prefix + opcode
    write <u8> ((reg & 7) | (extension << 3) | 0b11000000); // mod r/m
  }
  
  template <u16 opcode>
  constexpr void op_r8_r8 (R8 dest, R8 src) {
    auto rex = 0;
    if (dest >= R8::spl) // handle the "special" REX behavior with 8-bit regs
        rex |= REX;
    else if (dest >= R8::r8b)
        rex |= REX_R;

    if (src >= R8::spl) // handle the "special" REX behavior with 8-bit regs
        rex |= REX;
    else if (src >= R8::r8b)
        rex |= REX_B;

    if (rex) // REX
        write <u8> (rex);

    write <u8> (opcode); // opcode
    write <u8> (((dest & 7) << 3) | (src & 7) | (0b11 << 6)); // mod r/m   
  }
  
  template <u16 opcode>
  constexpr void op_r16_r16 (R16 dest, R16 src) {
    auto rex = 0;
    if (dest >= R16::r8w)
        rex |= REX_R;
    if (src >= R16::r8w)
        rex |= REX_B;

    write <u8> (0x66); // size override prefix
    if (rex) // REX
        write <u8> (rex);

    write <u8> (opcode); // opcode
    write <u8> (((dest & 7) << 3) | (src & 7) | (0b11 << 6)); // mod r/m   
  }
  
  template <u16 opcode> 
  constexpr void op_r32_r32 (R32 dest, R32 src) {
    auto rex = 0;
    if (dest >= R32::r8d)
        rex |= REX_R;
    if (src >= R32::r8d)
        rex |= REX_B;

    if (rex) // REX
        write <u8> (rex);

    write <u8> (opcode); // opcode
    write <u8> (((dest & 7) << 3) | (src & 7) | (0b11 << 6)); // mod r/m   
  }

  template <u16 opcode> 
  constexpr void op_r64_r64 (R64 dest, R64 src) {
    auto rex = REX_W;
    if (dest >= R64::r8)
        rex |= REX_R;
    if (src >= R64::r8)
        rex |= REX_B; 

    write <u16> ((opcode << 8) | rex); // REX + opcode
    write <u8> (((dest & 7) << 3) | (src & 7) | (0b11 << 6)); // mod r/m   
  }

  template <u16 opcode>
  constexpr void op_m64_r64 (const R64* destPtr, R64 src) {
    auto rex = REX_W;
    auto dest = (R64) (uintptr_t) destPtr;
    auto mod_rm = ((dest & 7) | ((src & 7) << 3));

    if (dest >= R64::r8)
        rex |= REX_B;
    if (src >= R64::r8)
        rex |= REX_R;
        
    write <u16> ((opcode << 8) | rex); // REX + opcode

    switch (dest & 7) { // What the fuck are these edge cases?
        case R64::rsp: write <u8> (mod_rm); write <u8> (0x24); return; // insert mod r/m and SIB for RBP/R12
        case R64::rbp: write <u8> (mod_rm | 0x40); write <u8> (0x00); return; // Add a 1-byte displacement of 00 for RSP/R13
    }

    write <u8> (mod_rm); // mod r/m 
  }

  template <u16 opcode>
  constexpr void op_r64_m64 (R64 dest, const R64* srcPtr) {
    auto rex = REX_W;
    auto src = (R64) (uintptr_t) srcPtr;
    auto mod_rm = ((dest & 7) << 3) | (src & 7);

    if (dest >= R64::r8)
        rex |= REX_R;
    if (src >= R64::r8)
        rex |= REX_B;
        
    write <u16> ((opcode << 8) | rex); // REX + opcode

    switch (src & 7) { // What the fuck are these edge cases?
        case R64::rsp: write <u8> (mod_rm); write <u8> (0x24); return; // insert mod r/m and SIB for RBP/R12
        case R64::rbp: write <u8> (mod_rm | 0x40); write <u8> (0x00); return; // Add a 1-byte displacement of 00 for RSP/R13
    }

    write <u8> (mod_rm); // mod r/m 
  }

  template <u16 opcode, u16 extension>
  constexpr void op_r64_i32 (R64 reg, u32 immediate) {
      auto rex = REX_W;
      if (reg >= R64::r8)
        rex |= REX_B;

      write <u16> ((opcode << 8) | rex); // REX prefix + opcode
      write <u8>  ((reg & 7) | (extension << 3) | (0b11 << 6)); // mod r/m
      write <u32> (immediate); // immediate
  }

  constexpr void nop() { // single byte NOP
      write <u8> (0x90);
  }

  constexpr void mov (R32 reg, u32 immediate) { // mov r32, imm
      assert(reg < R32::r8d);
      auto opcode = 0xB8 + ((int) reg & 7);
      write <u8> (opcode);
      write <u32> (immediate); // immediate  
  }

  constexpr void mov (R64 reg, u64 immediate) { // movabs r64, imm
      auto rex = REX_W;
      if (reg >= R64::r8)
        rex |= REX_B;

      auto opcode = 0xB8 + ((u16) reg & 7);
      write <u16> ((opcode << 8) | rex); // REX prefix + opcode
      write <u64> (immediate); // immediate
  }

  // call displacement
  template <Displacement displacementType = Displacement::Rel32> 
  void call (void* function) {
      assert (displacementType == Displacement::Rel32);
      auto currAddr = (uintptr_t) &buffer[bufferPtr + 5]; // Pointer to the end of the CALL
      long long  disp = (uintptr_t) function - currAddr; // calculate the distance we need to jump
      assert (disp <= 0xFFFF'FFFF); // assert it can be jumped to using a relative jump
      write <u8> (0xE8);
      write <u32> ((u32) disp);
  }

  constexpr void add (R64 reg, u32 immediate) { op_r64_i32 <0x81, 0> (reg, immediate); }   // add r64, imm32
  constexpr void OR  (R64 reg, u32 immediate) { op_r64_i32 <0x81, 1> (reg, immediate); }   // or  r64, imm32
  constexpr void adc (R64 reg, u32 immediate) { op_r64_i32 <0x81, 2> (reg, immediate); }   // adc r64, imm32
  constexpr void sbb (R64 reg, u32 immediate) { op_r64_i32 <0x81, 3> (reg, immediate); }   // sbb r64, imm32
  constexpr void AND (R64 reg, u32 immediate) { op_r64_i32 <0x81, 4> (reg, immediate); }   // and r64, imm32
  constexpr void sub (R64 reg, u32 immediate) { op_r64_i32 <0x81, 5> (reg, immediate); }   // sub r64, imm32
  constexpr void XOR (R64 reg, u32 immediate) { op_r64_i32 <0x81, 6> (reg, immediate); }   // xor r64, imm32
  constexpr void cmp (R64 reg, u32 immediate) { op_r64_i32 <0x81, 7> (reg, immediate); }   // cmp r64, imm32

  constexpr void add (R64 dest, R64 src) { op_r64_r64 <0x03> (dest, src); } // add r64, r64
  constexpr void OR  (R64 dest, R64 src) { op_r64_r64 <0x0B> (dest, src); } // or  r64, r64
  constexpr void adc (R64 dest, R64 src) { op_r64_r64 <0x13> (dest, src); } // adc r64, r64
  constexpr void sbb (R64 dest, R64 src) { op_r64_r64 <0x1B> (dest, src); } // sbb r64, r64
  constexpr void AND (R64 dest, R64 src) { op_r64_r64 <0x23> (dest, src); } // and r64, r64
  constexpr void sub (R64 dest, R64 src) { op_r64_r64 <0x2B> (dest, src); } // sub r64, r64
  constexpr void XOR (R64 dest, R64 src) { op_r64_r64 <0x33> (dest, src); } // xor r64, r64
  constexpr void cmp (R64 dest, R64 src) { op_r64_r64 <0x3B> (dest, src); } // cmp r64, r64
  constexpr void test(R64 op1, R64 op2)  { op_r64_r64 <0x85> (op2, op1); } // test r64, r64 
  constexpr void mov (R64 dest, R64 src) { op_r64_r64 <0x8B> (dest, src); } // mov r64, r64

  constexpr void add (R32 dest, R32 src) { op_r32_r32 <0x03> (dest, src); } // add r32, r32
  constexpr void OR  (R32 dest, R32 src) { op_r32_r32 <0x0B> (dest, src); } // or  r32, r32
  constexpr void adc (R32 dest, R32 src) { op_r32_r32 <0x13> (dest, src); } // adc r32, r32
  constexpr void sbb (R32 dest, R32 src) { op_r32_r32 <0x1B> (dest, src); } // sbb r32, r32
  constexpr void AND (R32 dest, R32 src) { op_r32_r32 <0x23> (dest, src); } // and r32, r32
  constexpr void sub (R32 dest, R32 src) { op_r32_r32 <0x2B> (dest, src); } // sub r32, r32
  constexpr void XOR (R32 dest, R32 src) { op_r32_r32 <0x33> (dest, src); } // xor r32, r32
  constexpr void cmp (R32 dest, R32 src) { op_r32_r32 <0x3B> (dest, src); } // cmp r32, r32
  constexpr void mov (R32 dest, R32 src) { op_r32_r32 <0x8B> (dest, src); } // mov r32, r32
  
  constexpr void add (R16 dest, R16 src) { op_r16_r16 <0x03> (dest, src); } // add r16, r16
  constexpr void OR  (R16 dest, R16 src) { op_r16_r16 <0x0B> (dest, src); } // or  r16, r16
  constexpr void adc (R16 dest, R16 src) { op_r16_r16 <0x13> (dest, src); } // adc r16, r16
  constexpr void sbb (R16 dest, R16 src) { op_r16_r16 <0x1B> (dest, src); } // sbb r16, r16
  constexpr void AND (R16 dest, R16 src) { op_r16_r16 <0x23> (dest, src); } // and r16, r16
  constexpr void sub (R16 dest, R16 src) { op_r16_r16 <0x2B> (dest, src); } // sub r16, r16
  constexpr void XOR (R16 dest, R16 src) { op_r16_r16 <0x33> (dest, src); } // xor r16, r16
  constexpr void cmp (R16 dest, R16 src) { op_r16_r16 <0x3B> (dest, src); } // cmp r16, r16
  constexpr void mov (R16 dest, R16 src) { op_r16_r16 <0x8B> (dest, src); } // mov r16, r16
  
  constexpr void add (R8 dest, R8 src) { op_r8_r8 <0x02> (dest, src); } // add r8, r8
  constexpr void OR  (R8 dest, R8 src) { op_r8_r8 <0x0A> (dest, src); } // or  r8, r8
  constexpr void adc (R8 dest, R8 src) { op_r8_r8 <0x12> (dest, src); } // adc r8, r8
  constexpr void sbb (R8 dest, R8 src) { op_r8_r8 <0x1A> (dest, src); } // sbb r8, r8
  constexpr void AND (R8 dest, R8 src) { op_r8_r8 <0x22> (dest, src); } // and r8, r8
  constexpr void sub (R8 dest, R8 src) { op_r8_r8 <0x2A> (dest, src); } // sub r8, r8
  constexpr void XOR (R8 dest, R8 src) { op_r8_r8 <0x32> (dest, src); } // xor r8, r8
  constexpr void cmp (R8 dest, R8 src) { op_r8_r8 <0x3A> (dest, src); } // cmp r8, r8
  constexpr void mov (R8 dest, R8 src) { op_r8_r8 <0x8A> (dest, src); } // mov r8, r8

  constexpr void movzx (R16 dest, R8 src) { movx <0xB60F> (dest, src); } // movzx r16, r8
  constexpr void movzx (R32 dest, R8 src) { movx <0xB60F> (dest, src); } // movzx r32, r8
  constexpr void movzx (R64 dest, R8 src) { movx <0xB60F> (dest, src); } // movzx r64, r8
  constexpr void movzx (R32 dest, R16 src){ movx <0xB70F> (dest, src); } // movzx r32, r16
  constexpr void movzx (R64 dest, R16 src){ movx <0xB70F> (dest, src); } // movzx r64, r16

  constexpr void movsx (R16 dest, R8 src) { movx <0xBE0F> (dest, src); } // movsx r16, r8
  constexpr void movsx (R32 dest, R8 src) { movx <0xBE0F> (dest, src); } // movsx r32, r8
  constexpr void movsx (R64 dest, R8 src) { movx <0xBE0F> (dest, src); } // movsx r64, r8
  constexpr void movsx (R32 dest, R16 src){ movx <0xBF0F> (dest, src); } // movsx r32, r16 
  constexpr void movsx (R64 dest, R16 src){ movx <0xBF0F> (dest, src); } // movzx r64, r16

  constexpr void add (const R64* destPtr, R64 src) { op_m64_r64 <0x01> (destPtr, src); } // add [r64], r64
  constexpr void OR  (const R64* destPtr, R64 src) { op_m64_r64 <0x09> (destPtr, src); } // or  [r64], r64
  constexpr void adc (const R64* destPtr, R64 src) { op_m64_r64 <0x11> (destPtr, src); } // adc [r64], r64
  constexpr void sbb (const R64* destPtr, R64 src) { op_m64_r64 <0x19> (destPtr, src); } // sbb [r64], r64
  constexpr void AND (const R64* destPtr, R64 src) { op_m64_r64 <0x21> (destPtr, src); } // and [r64], r64
  constexpr void sub (const R64* destPtr, R64 src) { op_m64_r64 <0x29> (destPtr, src); } // sub [r64], r64
  constexpr void XOR (const R64* destPtr, R64 src) { op_m64_r64 <0x31> (destPtr, src); } // xor [r64], r64
  constexpr void cmp (const R64* destPtr, R64 src) { op_m64_r64 <0x39> (destPtr, src); } // cmp [r64], r64
  constexpr void mov (const R64* destPtr, R64 src) { op_m64_r64 <0x89> (destPtr, src); } // mov [r64], r64

  constexpr void add (R64 dest, const R64* srcPtr) { op_r64_m64 <0x03> (dest, srcPtr); } // add r64, [r64]
  constexpr void OR  (R64 dest, const R64* srcPtr) { op_r64_m64 <0x0B> (dest, srcPtr); } // or  r64, [r64]
  constexpr void adc (R64 dest, const R64* srcPtr) { op_r64_m64 <0x13> (dest, srcPtr); } // adc r64, [r64]
  constexpr void sbb (R64 dest, const R64* srcPtr) { op_r64_m64 <0x1B> (dest, srcPtr); } // sbb r64, [r64]
  constexpr void AND (R64 dest, const R64* srcPtr) { op_r64_m64 <0x23> (dest, srcPtr); } // and r64, [r64]
  constexpr void sub (R64 dest, const R64* srcPtr) { op_r64_m64 <0x2B> (dest, srcPtr); } // sub r64, [r64]
  constexpr void XOR (R64 dest, const R64* srcPtr) { op_r64_m64 <0x33> (dest, srcPtr); } // xor r64, [r64]
  constexpr void cmp (R64 dest, const R64* srcPtr) { op_r64_m64 <0x3B> (dest, srcPtr); } // cmp r64, [r64]
  constexpr void mov (R64 dest, const R64* srcPtr) { op_r64_m64 <0x8B> (dest, srcPtr); } // mov r64, [r64]

  constexpr void inc (R64 reg) { op_r64 <0xFF, 0> (reg); } // inc r64
  constexpr void dec (R64 reg) { op_r64 <0xFF, 1> (reg); } // dec r64
  constexpr void call(R64 reg) { op_r64 <0xFF, 2> (reg); } // call r64
  constexpr void jmp (R64 reg) { op_r64 <0xFF, 4> (reg); } // jmp r64

  constexpr void inc (R32 reg) { op_r32 <0xFF, 0> (reg); } // inc r32
  constexpr void dec (R32 reg) { op_r32 <0xFF, 1> (reg); } // dec r32
  constexpr void call(R32 reg) { op_r32 <0xFF, 2> (reg); } // call r32
  constexpr void jmp (R32 reg) { op_r32 <0xFF, 4> (reg); } // jmp r32

  // push r64
  void push (R64 reg) {
      auto rex = REX_W;
      if (reg >= R64::r8)
        rex |= REX_B;

      auto opcode = 0x50 + (int) (reg & 7);
      write <u16> ((opcode << 8) | rex); // REX prefix + opcode
  }

  // push imm32
  void push (u32 immediate) {
      write <u8> (0x68); // opcode
      write <u32> (immediate);
  }

  constexpr void pushf() { write <u8> (0x9C); } // pushf
  constexpr void popf()  { write <u8> (0x9D); } // popf
  constexpr void sahf()  { write <u8> (0x9E); } // sahf
  constexpr void lahf()  { write <u8> (0x9F); } // lahf

  // jcc displacement
  template <Condition cond, Displacement displacementType = Displacement::Automatic>
  constexpr void jcc (int disp) {
      assert (displacementType == Displacement::Automatic);
      if (disp >= -128 && disp <= 127) {
          write <u16> (((u16) disp << 8) | (0x70 + cond));
      }

      else {
          printf ("Can't assemble jcc offset :(\n");
      }
  }

  template <Condition cond, Displacement displacementType = Displacement::Automatic>
  void jcc (std::string label) {
      for (auto i : labels) {
          if (!i.name.compare(label)) {
              int displacement = i.addr - (bufferPtr + 2); // The +2 accounts for the fact the instruction is 2 bytes long
              assert (displacement >= -128 && displacement <= 127); // check if it can fit in a near jump
              jcc <cond> (displacement);
              return;
          }
      }
      
      printf ("No label found!\n");
      jcc <cond> (0);
  }

  template <Condition cond> 
  void set (R8 reg) {
    auto rex = 0;
    if (reg >= R8::spl) // handle the "special" REX behavior with 8-bit regs
        write <u8> (REX);
    else if (reg >= R8::r8b)
        write <u8> (REX_B);

    write <u16> (((0x90 + (int) cond) << 8) | 0x0F); // opcode = 0x0F{(90+cc)}
    write <u8> (0b11000000 | reg); // mod r/m
  }

  // Misc control operations
  constexpr void ud2() { write <u16> (0x0B0F); } // UD2
  constexpr void cpuid() { write <u16> (0xA20F); } // CPUID
  constexpr void loop (u8 disp) { write <u16> ((disp << 8) | 0xE2); } // LOOP
  constexpr void loope (u8 disp) { write <u16> ((disp << 8) | 0xE1); } // LOOPE
  constexpr void loopne (u8 disp) { write <u16> ((disp << 8) | 0xE0); } // LOOPNE
  
  // Flag operations
  constexpr void cmc() { write <u8> (0xF5); } // CMC (Complement carry)
  constexpr void clc() { write <u8> (0xF8); } // CLC
  constexpr void stc() { write <u8> (0xF9); } // STC
  constexpr void cli() { write <u8> (0xFA); } // CLI
  constexpr void sti() { write <u8> (0xFB); } // STI
  constexpr void cld() { write <u8> (0xFC); } // CLD
  constexpr void std() { write <u8> (0xFD); } // STD (Use protection)

  // VM operations
  constexpr void vmcall()  { write <u8> (0x0F); write <u16> (0xC101); } // VMCALL
  constexpr void vmlaunch() { write <u8> (0x0F); write <u16> (0xC201); } // VMLAUNCH
  constexpr void vmresume() { write <u8> (0x0F); write <u16> (0xC301); } // VMRESUME
  constexpr void vmxoff() { write <u8> (0x0F); write <u16> (0xC401); } // VMCALL

  // Segment operations (does not work in 64-bit mode)  
  constexpr void push (Segment segment) {
      switch (segment) {
          case es: write <u16> (0x0666); break;
          case cs: write <u16> (0x0E66); break;
          case ss: write <u16> (0x1666); break;
          case ds: write <u16> (0x1E66); break;
          case gs: write <u16> (0xA80F); break;
          case fs: write <u16> (0xA00F); break;
          default: printf ("Invalid segment: %d, Ignoring\n", segment);
      }
  }

  constexpr void pop (Segment segment) {
      switch (segment) {
          case cs: printf ("[Luna] Can't pop a value into CS!\n"); break;
          case es: write <u16> (0x0766); break;
          case ss: write <u16> (0x1767); break;
          case ds: write <u16> (0x1F67); break;
          case gs: write <u16> (0xA90F); break;
          case fs: write <u16> (0xA10F); break;
          default: printf ("Invalid segment: %d, Ignoring\n", segment);
      }
  }

  void dump() {
      std::ofstream file ("output.exe", std::ios::binary);
      file.write ((const char*) buffer, bufferPtr);
      // printf ("Dumped %d bytes\n", bufferPtr);
  }

  void flush() { // reset buffer
      //for (int i = 0; i < bufferPtr; i++) {
      //    buffer[i] = 0;
      //}

      bufferPtr = 0;
  }

  void addLabel (std::string name) {
      Label label;
      label.addr = bufferPtr;
      label.name = name;

      labels.push_back(label);
  }
 
  /// Get a pointer to the emitter buffer
  u8* data() {
      return buffer;
  }

  /// Set a user-defined buffer for the emitter
  void setBuffer (u8* buffer) {
      this->buffer = buffer;
  }

  void setBufferIndex (u32 index) {
      bufferPtr = index;
  }
};
} // namespace Emitter