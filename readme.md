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

    code.mov (rcx, (u64) str); // pointer to the string in rcx
    code.mov (rax, (u64) &printf); // pointer to printf in rax
    code.call (rax); // call printf
    code.ret();      // return

    auto foo = (FunctionPointer) code.data(); // function pointer to the generated function
    (*foo)(); // call the function
}
```
