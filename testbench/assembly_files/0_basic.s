// Test 0: basic pipelining
//
// A sequence of non-branch, non-memory, independent instructions
// Minimum required IPC: 1.0

addi s0, zero, 8
addi s1, zero, 0x555

addi s2,  zero, 0x9B8 # -1608
addi s6,  zero, 0x70F # 1807
addi s9,  zero, 0x70F
addi s10, zero, 0x70F

add   s2,  s2,  s0 		# expect 0xFFFFF9C0
xori  s6,  s6,  0x3C 	# expect 0x733
ori   s9,  s9,  0x3C 	# expect 0x73F
andi  s10, s10, 0x3C 	# expect 0x00C

sub  s2,  s2,  s1 		# expect 0xFFFFF46B
xor  s6,  s6,  s1  		# expect 0x266
or   s9,  s9,  s1 		# expect 0x77F
measure_end:
and  s10, s10, s1 		# expect 0x004

nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop

// Results section for simulator
.org 0x800
.dw 0x0	// start PC of measurement region
.dw measure_end		// end PC
.dw 14	// number of instructions
#ifdef L7
.dw 128	// minimum IPC * 128
#else
.dw 25	// minimum IPC * 128
#endif
// which regs to check
.db 0b00000000
.db 0b00000011 // s1-s0
.db 0b01000100 // s7-s2
.db 0b00000110 // s10-s8
.org 0x820
// correct reg values
.dw 0, 0, 0, 0, 0, 0, 0, 0
.dw 8, 0x555, 0, 0, 0, 0, 0, 0
.dw 0, 0, 0xFFFFF46B, 0xFF9B8000, 1, 0, 0x266, 0x000FFFFF
.dw 0xFFFFFFFF, 0x77F, 0x004, 0, 0, 0, 0, 0
