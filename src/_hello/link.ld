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

PROVIDE(_serial_tx_info = 0x80000003);
PROVIDE(_serial_tx_data = 0x80000004);
