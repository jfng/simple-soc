OUTPUT_FORMAT("elf32-littleriscv")
OUTPUT_ARCH("riscv")
ENTRY(_start)

MEMORY
{
	rom : ORIGIN = 0x00000000, LENGTH = 0x00001000
}

SECTIONS
{
	. = 0x00000000;
	.text :
	{
		*(.text) *(.text.*)
	} > rom
	.rodata :
	{
		*(.rodata) *(.rodata.*)
	} > rom
}

PROVIDE(_serial_txs = 0x8000000c);
PROVIDE(_serial_txd = 0x80000010);
