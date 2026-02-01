# Dawn EMulator
Dawn EMulator is a hardware emulation test for checking correctness in [dawn](https://github.com/sivansh11/dawn).

# Why ?
- Emulating Linux was never the primary goal of this project. However, booting a kernel serves as an exhaustive correctness test. 
- The complexities of Linux boot sequence helped me identify and resolve numerous edge-case bugs in instruction execution that I would have missed otherwise.

# How ?
- Since dawn doesnt have a supervisor mode, and doesnt have a mmu to provide memory protection, booting an off the shelf linux kernel is out of the question.
- Regular linux heavily relies on fork() to create new user processes, which isnt possible in a no mmu system. 
- Instead, I am running a specialized configuration of linux called uCLinux, which is written to run on systems without an mmu, here, rather than running ELF binaries, it runs a different BFLT binary, and rather than fork(), it uses vfork(), clone(), posix_spawn(). 
- In this, the host must provide emulated hardware to dawn, for example uart, framebuffer etc. I use libfdt library to generate a device tree while initializing dawn.
- See, [src/main.cpp](https://github.com/sivansh11/dem/blob/main/src/main.cpp) for more information

# Build instructions
```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/dem ./Image ./rootfs.cpio     # To run dem
```

# Filesystem
To simplify rootfs modification, I provide 2 scripts
- ./scripts/unpack.sh (extracts rootfs.cpio to rootfs/)
- ./scripts/pack.sh (rebuilds the rootfs.cpio from rootfs/)

# Running custom BFLT binaries inside emulated linux
- Firstly you will need a riscv cross compiler, I recommend using buildroot for getting it.
- To compile your code to a valid bflt binary, you need to pass `-pie -fPIC -Wl,-elf2flt=-r -static`.
```
riscv64-linux-gcc -g -O0 -pie -fPIC -Wl,-elf2flt=-r -static main.c 
```
- move the compiled binary to the rootfs and run ./scripts/pack.sh
