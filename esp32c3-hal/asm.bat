riscv32-esp-elf-gcc -ggdb3  -c -mabi=ilp32 -march=rv32i asm.S -o bin/esp32c3asm.o
riscv32-esp-elf-ar crs bin/asm_riscv32i-unknown-none-elf.a bin/esp32c3asm.o
del bin\esp32c3asm.o
