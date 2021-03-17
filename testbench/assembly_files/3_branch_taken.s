// Test 3: branches taken
//
// Tests branches, conditional or not, that are taken.
// Correctness is tested via making sure that the instructions
// behind the branch get squashed properly.
// Minimum required IPC: 0.5

// Error flags - these should remain at 0
// If they get incremented, your CPU failed to squash either of the
// two instructions in the pipeline behind the branch
addi s5, s6, 0
addi s6, s6, 0

// Test jal instruction
jal ra, testfunc
addi s5, s5, 1	// these two should be skipped over when returning
addi s6, s6, 1
jal zero, part2	// testfunc returns here

addi s5, s5, 1
addi s6, s6, 1

testfunc:
	// when we return, skip two instructions
	jalr zero, ra, 8	// we get to test jalr here

addi s5, s5, 1
addi s6, s6, 1

// Test jalr similarly
part2:
addi s7, zero, testfunc
jalr ra, s7, 0
addi s5, s5, 1
addi s6, s6, 1
jal zero, part3

addi s5, s5, 1
addi s6, s6, 1

// Test branches
part3:
addi a2, zero, -4
addi a3, zero, -1
addi a4, zero, 0
addi a5, zero, 2
addi a6, zero, 3

// beq
beq1:
beq a2, a2, beq2
addi s5, s5, 1
addi s6, s6, 1
beq2:
beq a2, a2, beq3
addi s5, s5, 1
addi s6, s6, 1
beq3:
beq a2, a2, blt1
addi s5, s5, 1
addi s6, s6, 1

// blt
blt1:
blt a2, a3, blt2
addi s5, s5, 1
addi s6, s6, 1
blt2:
blt a2, a4, blt3
addi s5, s5, 1
addi s6, s6, 1
blt3:
blt a2, a6, blt4
addi s5, s5, 1
addi s6, s6, 1
blt4:
blt a4, a6, blt5
addi s5, s5, 1
addi s6, s6, 1
blt5:
blt a5, a6, measure_end
addi s5, s5, 1
addi s6, s6, 1

measure_end:
jal ra, measure_end
end:

// Results section for simulator
.org 0x800
.dw 0x0	// start PC of measurement region
.dw measure_end		// end PC
.dw 24		// number of instructions
#ifdef L7
.dw 64	// minimum IPC * 128
#else
.dw 25	// minimum IPC * 128
#endif
.db 0b00000010	// which regs to check
.db 0b00000000
.db 0b01100000
.db 0b00000000
.org 0x820
.dw 0, end, 0, 0, 0, 0, 0, 0	// correct reg values
.dw 0, 0, 0, 0, 0, 0, 0, 0
.dw 0, 0, 0, 0, 0, 0, 0, 0
.dw 0, 0, 0, 0, 0, 0, 0, 0

