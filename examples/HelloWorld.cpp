#include <iostream>
#include "../Luna/Luna.hpp"
using namespace Luna;
typedef void (*FunctionPointer)();

int main() {
    const char* str = "Hello World!\n";
    Generator code;

    auto stackOffset = 8;  // The offset needed to align the stack.
#ifdef _WIN32
        stackOffset += 32; // Windows needs an extra 32 bytes because of the "shadow" stack space
#endif

    code.mov (rcx, (u64) str); // pointer to the string in rcx
    code.mov (rax, (u64) &printf); // pointer to printf in rax
    code.sub(rsp, stackOffset); //align stack to 16 byte boundary
    code.call (rax); // call printf
    code.add(rsp, stackOffset); //return stack to original position
    code.ret();      // return

    auto foo = (FunctionPointer) code.data(); // function pointer to the generated function
    (*foo)(); // call the function
}