# Luna
Work-in-progress x64 dynamic assembler/emitter for entertainment uses. <br>
Aims to provide a decently user-friendly interface, using a similar syntax to NASM/Xbyak.

**Simple "Hello World!" example**
```cpp
#include <iostream>
#include "../Luna/Luna.hpp"
using namespace Luna;
typedef void (*FunctionPointer)();

int main() {
    const char* str = "Hello World!\n"; // the string we'll print
    Generator code; // code emitter object
    auto stackOffset = 40;  // The offset needed to align the stack. Windows needs an extra 32 bytes because of the "shadow" stack space

    code.mov (rcx, (u64) str); // pointer to the string in rcx
    code.mov (rax, (u64) &printf); // pointer to printf in rax
    code.sub(rsp, stackOffset); //align stack to 16 byte boundary
    code.call (rax); // call printf
    code.add(rsp, stackOffset); //return stack to original position
    code.ret();      // return

    auto foo = (FunctionPointer) code.data(); // function pointer to the generated function
    (*foo)(); // call the function
}
```
