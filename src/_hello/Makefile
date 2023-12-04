CC      = pdm run python -m ziglang cc -target riscv32-freestanding
OBJCOPY = pdm run python -m ziglang objcopy

ASFLAGS = -mcpu=baseline_rv32-a-c-d-f # restrict to RV32IM
LDFLAGS = -nostdlib -Tlink.ld

all: hello.bin

hello.bin: hello
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f hello.bin hello
