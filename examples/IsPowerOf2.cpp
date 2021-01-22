#include <iostream>
#include "../Luna/Luna.hpp"
using namespace Luna;
typedef bool (*FunctionPointer)();

bool isPowerOfTwo (u64 var) {
    Generator code;
    code.mov (rax, var); // var in RCX
    code.popcnt (rax, rax); // check num of bits
    code.cmp (rax, 1); // if 1 then it's a power of 2
    code.set <z> (al);
    code.movzx (eax, al);
    code.ret(); // return

    auto foo = (FunctionPointer) code.data(); // function pointer to the generated function
    return (*foo)(); // call the function
}

int main() {
    u64 var = 31;
    printf ("The variable is %sa power of 2", isPowerOfTwo (var) ? "" : "not ");
}