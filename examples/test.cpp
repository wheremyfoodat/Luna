#include <iostream>
#include "../Luna/Luna.hpp"
using namespace Luna;
typedef int (*FunctionPointer)();

void helloWorld() {
    Generator code;
    long long variable = 0;
    code.mov (rcx, (u64) &variable); // ptr in rcx
    code.mov (rax, 10);
    code.sub (qword [rcx], rax);
    code.ret();

    auto foo = (FunctionPointer) code.data();
    (*foo)();

    printf ("%lld\n", variable); // Should print 1
}

int main() {
    Generator code;

    code.ret();
    code.ret <Distance::Far>();
    code.nop();

    code.mov (rax, 0x123456789ABCDEF0);
    code.mov (rbx, 1);
    code.ud2();
    code.push (rdx);
    code.push (rsi);
    code.push (0x100);
    code.push (0x69420);
    code.pushf();
    code.popf();
    code.lahf();
    code.sahf();
    code.mov (edx, 0xC0FE'BABE);

    code.addLabel ("Test");
    code.mov (r15, 10);
    code.jcc <nz> ("Test");
    code.cpuid();
    code.mov (rcx, r15);
    code.mov (rax, r9);
    code.mov (rbx, rsp);
    code.call (rax);
    code.call (r12);
    code.add (r12, r9);
    code.XOR (rax, 1);
    code.sbb (rax, 1);
    code.sub (rax, 1);
    code.adc (rax, 1);
    code.sbb (rax, 1);
    code.OR (rax, 1);
    code.add (rax, 1);

    code.AND (r10, 0x10);
    code.AND (rax, rsi);
    code.AND (r9b, al);
    code.add (r9b, cl);
    code.sub (r9b, dl);
    code.sbb (r9b, sil);
    code.adc (r9b, bpl);
    code.XOR (r9b, spl);
    code.OR  (r9b, bl);
    code.cmp (r9b, r10b);

    code.mov (rax, r11);
    code.mov (ebx, r12d);
    code.mov (bx, r9w);
    code.mov (spl, r9b);

    code.movzx (ax, r9b);
    code.movzx (sp, spl);
    code.movzx (r9w, r9b);
    code.movzx (ecx, r9b);
    code.movzx (edx, r10b);
    code.movzx (rax, dil);  

    code.movsx (r9w, al);
    code.movsx (eax, bpl);
    code.movsx (rax, dil);

    code.loop(255);
    code.loope(10);
    code.loopne(30);

    code.cli();
    code.sti();
    code.cld();
    code.std();

    code.vmcall();
    code.vmlaunch();
    code.vmresume();
    code.vmxoff();

    code.mov (rax, qword [rcx]);
    code.mov (rcx, qword [r10]);
    code.mov (r10, qword [rbp]);
    code.mov (r14, qword [rsp]);

    code.mov (qword [rsp], rax);
    code.mov (qword [rbp], rdx);
    code.mov (qword [r12], r13);

    code.cmp (qword [r12], r13);
    code.cmp (r13, qword [r12]);
    code.OR  (qword [r13], r12);
    code.add (qword [rsp], r11);
    code.cmp (qword [rax], r13);
    code.add (r12, qword [rsp]);
    code.XOR (r10, qword [rbp]);
    code.sbb (rax, qword [rax]);
    code.push (r15);
    code.test (rax, r11);
    code.cmc();

    code.movzx (rax, r12w);
    code.movsx (r12d, ax);
    code.inc (rsp);
    code.dec (rbp);
    code.call (rax);
    code.jmp (rsp);
    
    code.inc (esp);
    code.dec (ebp);
    code.call (ebx);
    code.jmp (r11d);

    code.set <z> (r12b);
    code.set <e> (bl);
    code.set <o> (spl);

    code.push (cs);
    code.pop (es);

    code.dump();

    helloWorld();
}