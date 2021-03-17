// Test 4: memory dependence and forwarding
//
// Tests load and store forwarding
// Minimum required IPC: 1.0

// error flag
addi a2, zero, 0

// pointer chase - test load to load forwarding
addi s3, zero, array2
addi s2, zero, pointer1
lw s2, s2, 0 // s2 = array1
lw s4, s2, 0 // s4 = array1[0]
addi s4, s4, 0x102 // s4 = 0xFFFF0000
sw s3, s4, 0

// test load to store forwarding
lw s4, s2, 4
sw s3, s4, 4
sw s3, s4, 8
sw s3, s4, 12

lw s5, s3, 0
lw s6, s3, 4
add s5, s6, s5
lw s6, s3, 8
add s5, s6, s5
lw s6, s3, 12
add s5, s6, s5

// check that s5 = 0x0368CCFD
lui s6, 0x0368D
addi s6, s6, 0xCFD
bne s5, s6, error

measure_end:
jal zero, measure_end

error:
addi a2, a2, 1
jal zero, measure_end

pointer1:
.dw array1

array1:
.dw 0xFFFEFEFE
.dw 0xABCDEEFF
.dw 0

array2:
.dw 0 // end result: 0xFFFF0000
.dw 0 // end result: 0xABCDEEFF
.dw 0 // end result: 0xABCDEEFF
.dw 0 // end result: 0xABCDEEFF

// Results section for simulator
.org 0x800
.dw 0x0	// start PC of measurement region
.dw measure_end		// end PC
.dw 22	// n instructions
#ifdef L7
.dw 128	// minimum IPC * 128
#else
.dw 25	// minimum IPC * 128
#endif
.db 0b00000000	// which regs to check
.db 0b00010000  // a2
.db 0b00000000
.db 0b00000000
.org 0x820
.dw 0, 0, 0, 0, 0, 0, 0, 0	// correct reg values
.dw 0, 0, 0, 0, 0, 0, 0, 0
.dw 0, 0, 0, 0, 0, 0, 0, 0
.dw 0, 0, 0, 0, 0, 0, 0, 0


