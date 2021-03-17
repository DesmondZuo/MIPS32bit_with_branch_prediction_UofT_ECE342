// Test 2: branches not taken
//
// Tests conditional branches that are not taken (PC=PC+4 assumption is always right)
// Minimum required IPC: 1.0

// temp vars
addi s2, zero, -4
addi s3, zero, -1
addi s4, zero, 0
addi s5, zero, 2
addi s6, zero, 3

// Error flag
addi t3, zero, 0


// Test beq
beq s2, s3, error
beq s2, s4, error
beq s2, s6, error
beq s4, s6, error
beq s5, s6, error

// Test blt
blt s3, s2, error
blt s4, s2, error
blt s6, s2, error
blt s6, s4, error
blt s6, s5, error
blt s2, s2, error
blt s4, s4, error
blt s6, s6, error

measure_end:
jal zero, measure_end

error:
addi t3, zero, 1
jal t4, measure_end



// Results section for simulator
.org 0x800
.dw 0x0	// start PC of measurement region
.dw measure_end		// end PC
.dw 20	// number of instructions
#ifdef L7
.dw 128	// minimum IPC * 128
#else
.dw 25	// minimum IPC * 128
#endif
.db 0b00000000	// which regs to check
.db 0b00000000
.db 0b00000000
.db 0b00110000 // t4-t3
.org 0x820
.dw 0, 0, 0, 0, 0, 0, 0, 0	// correct reg values
.dw 0, 0, 0, 0, 0, 0, 0, 0
.dw 0, 0, 0, 0, 0, 0, 0, 0
.dw 0, 0, 0, 0, 0, 0, 0, 0

