#pio debug --interface=gdb -- -x gdbinit

#pio debug --interface=gdb -- -x .pioinit

gdb-multiarch -q --command=debug.gdb --args ".pio/build/stm32/firmware.elf"
