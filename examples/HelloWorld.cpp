#include <iostream>
#include "../Luna/Luna.hpp"
using namespace Luna;
typedef void (*FunctionPointer)();

int main() {
    const char* str = "Hello World!\n";
    Generator code;

    code.mov (rcx, (u64) str); // pointer to the string in rcx
    code.mov (rax, (u64) &printf); // pointer to printf in rax
    code.call (rax); // call printf
    code.ret();      // return

    auto foo = (FunctionPointer) code.data(); // function pointer to the generated function
    (*foo)(); // call the function
}