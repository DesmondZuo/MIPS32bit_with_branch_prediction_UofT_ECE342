// Test 1: dependent arithmetic
//
// A sequence of dependent arithmetic instructions
// Minimum required IPC: 1.0


// Test forwarding to second following instruction

addi a0, zero, 4
addi t0, zero, 2
add  a1, a0, a0 	// a1 = 8
addi t0, t0, -3 	// t0 = -1
sub  a1, a0, a1 	// a1 = -4
addi t0, t0, 0x3FD 	// t0 = 0x3FC
addi a1, a1, 2 		// a1 = -2
addi t0, t0, 0x3FC 	// t0 = 0x07F8

// Test next instr forwarding

lui  t3, 0xFEDCC 	// t3 = 0xFEDCC000
addi t3, t3, 0xA00
addi t3, t3, 0x90
addi t3, t3, 0x8
add  t4, t3, zero 	// t4 = 0xFEDCBA98

// test lui, auipc

lui  a4, 0xBEEFF 	// a4 = 0xBEEFF000
addi a5, a4, 0xD 	// a5 = 0xBEEFF00D
sub  a4, a5, a4 	// a4 = 0xD

auipc_tag:
auipc a6, 0x123 	// a6 = 0x123000 + auipc_tag

lui tp 0x123
measure_end:
sub gp, a6, tp 		// gp = auipc_tag

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
.dw 19	// number of instructions
#ifdef L7
.dw 128	// minimum IPC * 128
#else
.dw 25	// minimum IPC * 128
#endif
// which regs to check
.db 0b00101000 // t0, tp, gp
.db 0b11001100 // a5-a0
.db 0b00000000
.db 0b00100000 // t4-t3
.org 0x820
// correct reg values
.dw 0, 0, 0, auipc_tag, 0, 0x07F8, 0, 0
.dw 0, 0, 4, -2, 13, 21, 0xD, 0xBEEFF00D
.dw 0, 0, 0, 0, 0, 0, 0, 0
.dw 0, 0, 0, 0, 0xFEDCB000, 0xFEDCBA98, 0, 0

