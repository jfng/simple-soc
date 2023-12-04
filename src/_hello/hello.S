	.section .rodata
str:
	.asciz "hello, world!\n"

	.section .text
	.global _start
_start:
	la a0, str
	la a1, _serial_tx_info
	la a2, _serial_tx_data

1:	// poll tx_info.rdy
	lb t0, 0(a1)
	andi t0, t0, 1
	beqz t0, 1b

	// write to tx_data
	lb t0, 0(a0)
	sb t0, 0(a2)
	addi a0, a0, 1
	bnez t0, 1b

2:	j 2b
