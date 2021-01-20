#include <iostream>
#include "../Luna/Luna.hpp"
using namespace Luna;
typedef void (*FunctionPointer)();

void inc (u64* var, u64 amount) {
    Generator code;
    code.mov (rcx, (u64) var); // pointer to var in RCX
    code.mov (rdx, amount); // amount to increment in rdx
    code.add (qword[rcx], rdx); // increment
    code.ret();      // return

    auto foo = (FunctionPointer) code.data(); // function pointer to the generated function
    (*foo)(); // call the function
}

int main() {
    u64 var = 0;
    inc (&var, 420); // increment var by 10
    printf ("Var is: %lld", var);
}