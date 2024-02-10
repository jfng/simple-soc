	.section .rodata
str:
	.asciz "hello, world!\n"

	.section .text
	.global _start
_start:
	la a0, str
	la a1, _serial_txs
	la a2, _serial_txd

1:	// poll TXS.rdy
	lb t0, 0(a1)
	andi t0, t0, 1
	beqz t0, 1b

	// write to TXD
	lb t0, 0(a0)
	sb t0, 0(a2)
	addi a0, a0, 1
	bnez t0, 1b

2:	j 2b
