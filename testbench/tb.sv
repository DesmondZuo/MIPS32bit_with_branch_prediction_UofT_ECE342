// `define COUNT_INSTRS

module tb();

// Change these paths if needed to point to the testbench files
localparam string PROGRAMS[] = 
{
    "0_basic-L5.hex",
    "1_arithdep-L5.hex",
    "2_branch_nottaken-L5.hex",
    "3_branch_taken-L5.hex",
    "4_memdep-L5.hex"
};

localparam IW = 32;   // instr width
localparam REGS = 32; // number of registers

// Create a 100MHz clock
logic clk;
initial clk = '0;
always #5 clk = ~clk;

// Create the reset signal 
logic reset;


// Declare the bus signals, using the CPU's names for them
logic [IW-1:0] o_pc_addr;
logic o_pc_rd;
logic [IW-1:0] i_pc_rddata;
logic [3:0] o_pc_byte_en;
logic [IW-1:0] o_ldst_addr;
logic o_ldst_rd;
logic o_ldst_wr;
logic [IW-1:0] i_ldst_rddata;
logic [IW-1:0] o_ldst_wrdata;
logic [3:0] o_ldst_byte_en;
logic [IW-1:0] o_tb_regs [0:REGS-1];

// Instantiate the processor and hook up signals.
// Since the cpu's ports have the same names as the signals
// in the testbench, we can use the .* shorthand to automatically match them up
cpu dut(.*);

// Create a 512KB memory, dual-ported
localparam MEM_WORDS = 131072;
logic [IW-1:0] mem [0:MEM_WORDS-1];


// Define memory functionality.
always_ff @ (posedge clk) begin
    //
    // PORT 1 (pc read)
    //

    // Read logic.
    // For extra compliance, fill readdata with garbage unless
    // rd enable is actually used.
    if (o_pc_rd) begin
        case(o_pc_byte_en)
            4'b0001: begin
                case(o_pc_addr[1:0])
                2'h0: i_pc_rddata <= {24'hx, mem[o_pc_addr[IW-1:2]][7:0]};
                2'h1: i_pc_rddata <= {24'hx, mem[o_pc_addr[IW-1:2]][15:8]};
                2'h2: i_pc_rddata <= {24'hx, mem[o_pc_addr[IW-1:2]][23:16]};
                2'h3: i_pc_rddata <= {24'hx, mem[o_pc_addr[IW-1:2]][31:24]};
                endcase
            end
            4'b0011: begin
                case(o_pc_addr[1])
                1'h0: i_pc_rddata <= {16'hx, mem[o_pc_addr[IW-1:2]][15:0]};
                1'h1: i_pc_rddata <= {16'hx, mem[o_pc_addr[IW-1:2]][31:16]};
                endcase
            end
            default: i_pc_rddata <= mem[o_pc_addr[IW-1:2]];
        endcase
    end
    else begin 
        i_pc_rddata <= {IW{1'bx}};
    end
    
    //
    // PORT 2 (loads and stores)
    //
    
    // Read logic.
    if (o_ldst_rd) begin
        case(o_ldst_byte_en)
            4'b0001: begin
                case(o_ldst_addr[1:0])
                2'h0: i_ldst_rddata <= {24'hx, mem[o_ldst_addr[IW-1:2]][7:0]};
                2'h1: i_ldst_rddata <= {24'hx, mem[o_ldst_addr[IW-1:2]][15:8]};
                2'h2: i_ldst_rddata <= {24'hx, mem[o_ldst_addr[IW-1:2]][23:16]};
                2'h3: i_ldst_rddata <= {24'hx, mem[o_ldst_addr[IW-1:2]][31:24]};
                endcase
            end
            4'b0011: begin
                case(o_ldst_addr[1])
                1'h0: i_ldst_rddata <= {16'hx, mem[o_ldst_addr[IW-1:2]][15:0]};
                1'h1: i_ldst_rddata <= {16'hx, mem[o_ldst_addr[IW-1:2]][31:16]};
                endcase
            end
            default: i_ldst_rddata <= mem[o_ldst_addr[IW-1:2]];
        endcase
    end
    else begin 
        i_ldst_rddata <= {IW{1'bx}};
    end
    
    // Write logic
    if (o_ldst_wr) begin
        case (o_ldst_addr)
        32'h1000: begin
            // Writing a number to 0x1000 will display it
            $display("Integer result: %h", o_ldst_wrdata);
        end
        
        32'h1002: begin
            // Writing an address to 0x1002 will print the null-terminated string
            // at that address, as long as it's up to 512 characters long.
            int rd_addr;
            string str;
            int str_len;
            
            // Allocate a verilog string with 512 characters (we can't expand strings apparently).
            // rd_addr points to the string to print out, and the CPU gave this to us
            str = {512{" "}};
            str_len = 0;
            rd_addr = o_ldst_wrdata;

            while (str_len < 512) begin
                logic [7:0] rd_val;
                rd_val = mem[rd_addr >> 2] >> 8*rd_addr[1:0];
                
                if (rd_val == 8'hxx) begin
                    $display("Bad string result: got xx at address %h", rd_addr);
                end
                
                // Add the ASCII character to the string
                str.putc(str_len, rd_val);
                
                // Got null terminator, we're done
                if (rd_val == 8'd0) 
                    break;
                
                // Advance memory read position (by 1 char) and string write position
                rd_addr++;
                str_len++;
            end
            
            // Ran out of string room?
            if (str_len == 512) begin
                $display("Bad string result: no null terminator found after 512 chars");
                $stop();
            end
        
            // Got string, display it
            $display("String result: %s", str.substr(0, str_len-1));
        end
        default: begin
            // don't overwrite testbench stats
            if(o_ldst_addr >= 32'h800 && o_ldst_addr <= 32'h900) begin
                $display("Bad write: tried to overwrite testbench statistics.");
            end
            else begin
                // All other addresses: just write to memory
                case(o_ldst_byte_en)
                4'b0001: begin
                    case(o_ldst_addr[1:0])
                        2'h0: mem[o_ldst_addr[IW-1:2]][7:0] <= o_ldst_wrdata[7:0];
                        2'h1: mem[o_ldst_addr[IW-1:2]][15:8] <= o_ldst_wrdata[7:0];
                        2'h2: mem[o_ldst_addr[IW-1:2]][23:16] <= o_ldst_wrdata[7:0];
                        2'h3: mem[o_ldst_addr[IW-1:2]][31:24] <= o_ldst_wrdata[7:0];
                    endcase
                end
                4'b0011: begin
                    case(o_ldst_addr[1])
                        1'h0: mem[o_ldst_addr[IW-1:2]][15:0] <= o_ldst_wrdata[15:0];
                        1'h1: mem[o_ldst_addr[IW-1:2]][31:16] <= o_ldst_wrdata[15:0];
                    endcase
                end
                4'b1111: begin
                    mem[o_ldst_addr[IW-1:2]] <= o_ldst_wrdata;
                end
                endcase
            end
        end
        endcase
    end
end

// Test State
struct
{
    // Starting and ending PCs (inclusive) of IPC measurement region
    bit [IW-1:0] measure_start;
    bit [IW-1:0] measure_end;
    // Number of instructions in test (manually entered)
    integer n_instrs;
    // Minimum expected IPC
    real min_ipc;
    // Bitmask of which registers need validation
    bit [REGS-1:0] check_regs;
    // The expected valid values of said registers
    bit [IW-1:0] reg_vals [0:REGS-1];
    
    // Cycle counters for measurement
    bit measure_enable;
    integer measure_cycles;
`ifdef COUNT_INSTRS
    integer measure_instrs; // used for verification
`endif
} tstate;

// Snoop on the CPU's instruction fetching.
always_ff @ (posedge clk) begin
    // Once it fetches the instruction at the measure_start address,
    // then this testbench is now in measuring mode.
    if (o_pc_rd && o_pc_addr == tstate.measure_start) begin
        tstate.measure_enable = '1;
    end
    
    // Count every cycle in which we're in measuring mode.
    if (tstate.measure_enable) begin
        tstate.measure_cycles++;
        
`ifdef COUNT_INSTRS
        if(dut.instr_will_commit)
            tstate.measure_instrs++;
`endif
    end
    
    // Once the CPU fetches the instruction at PC=measure_end,
    // that's the last instruction in the measurement region.
    // Turn off measuring mode now.
    if (o_pc_rd && o_pc_addr == tstate.measure_end) begin
        tstate.measure_enable = '0;
    end
end

task reset_cpu();
    reset = '1;
    @(posedge clk);
    @(posedge clk);
    reset = '0;
endtask

// Load the test metadata section from byte address 0x800
task load_metadata();
    tstate.measure_start = mem[30'h200]; // address=0x800
    tstate.measure_end = mem[30'h201];
    tstate.n_instrs = mem[30'h202];
    tstate.min_ipc = real'(mem[30'h203]) / 128.0;
    tstate.check_regs = mem[30'h204];
    for (integer i = 0; i < REGS; i++) begin
        tstate.reg_vals[i] = mem[30'h208 + i];
    end
endtask

// Validate correctness of benchmark
task check_regs();
    bit all_passed;
    string detailed_info;
    all_passed = '1;
    
    // Benchmark has no correctness conditions? Exit quietly
    if (tstate.check_regs == '0)
        return;
    
    // First, check the registers
    detailed_info = "REG\tEXPECTED   ACTUAL     RESULT\n";
    for (integer i = 0; i < REGS; i++) begin
        logic [IW-1:0] expected;
        logic [IW-1:0] actual;
        bit pass;
        string detailed_line;
        
        // Does the benchmark care about this register?
        if (!tstate.check_regs[i]) continue;
        
        expected = tstate.reg_vals[i];
        actual = o_tb_regs[i];
        pass = expected == actual;
        
        detailed_line = $sformatf("x%0d\t0x%8x\t0x%8x\t%s\n",
            i,
            expected,
            actual,
            pass? "Pass" : "FAIL"
        );
        
        detailed_info = {detailed_info, detailed_line};
        all_passed &= pass;
    end
    
    $display("Functional correctness: %s",
        all_passed? "Pass" : "FAIL");
    
    if (!all_passed)
        $display(detailed_info);
endtask

// Validate performance of benchmark
task check_ipc();
    real ipc;
`ifdef COUNT_INSTRS
    real committed_ipc;
`endif

    ipc = tstate.measure_cycles > 0 ?
        real'(tstate.n_instrs) / real'(tstate.measure_cycles) : 0;
    
    $display("Instructions/cycles: %0d/%0d",
        tstate.n_instrs,
        tstate.measure_cycles
    );
    
    $display("IPC expected/achieved: %02f/%02f",
        tstate.min_ipc, ipc);
        
    $display("Performance result: %s",
        ipc >= tstate.min_ipc ? "Pass" : "FAIL");

`ifdef COUNT_INSTRS
    $display("Committed instructions: %0d",
        tstate.measure_instrs
    );
    committed_ipc = tstate.measure_cycles > 0 ?
        real'(tstate.measure_instrs) / real'(tstate.measure_cycles) : 0;
    $display("Committed IPC: %02f", committed_ipc);
`endif

endtask

task do_test(integer test_no);
    string hex_file;
    integer fp;
    integer i;
    // temporary variable: we read from file as bytes
    logic [7:0] tmp_mem [0:4*MEM_WORDS-1];

    // Reset the CPU
    reset_cpu();
    
    // Load the program
    hex_file = PROGRAMS[test_no];
    $display("================================");
    $display("Starting test %0d (%s)", test_no, hex_file);
    // Extra check to see if file exists
    fp = $fopen(hex_file, "r");
    if (fp == 0) begin
        $error("Couldn't open %s", hex_file);
        $stop;
    end
    $fclose(fp);

    // convert input hex file (in bytes) to 32b words
    $readmemh(hex_file, tmp_mem);
    for(i = 0; i < MEM_WORDS; i++)
        mem[i] = {tmp_mem[4*i+3], tmp_mem[4*i+2], tmp_mem[4*i+1], tmp_mem[4*i+0]};
    
    // Parse metadata section
    load_metadata();
    
    // Init test counters
    tstate.measure_enable = 0;
    tstate.measure_cycles = 0;
`ifdef COUNT_INSTRS
    tstate.measure_instrs = 0;
`endif
    // Wait for the program to do its thing and for
    // measurement to be done
    wait(tstate.measure_enable == 1);
    $display("Entered test region PC=%0x", tstate.measure_start);
    wait(tstate.measure_enable == 0);
    $display("Exited test region PC=%0x", tstate.measure_end);
    
    // Let the pipeline settle
    @(posedge clk); 
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk); 
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk); 
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    
    // Check results
    check_regs();   
    check_ipc();
    //$display("Measured instrs: %0d", tstate.measure_instrs);
    $display("================================");
endtask

initial begin
    // Comment these out to run only certain tests.
    do_test(0);
    do_test(1);
    do_test(2);
    do_test(3);
    do_test(4);
   
    $stop;
end

endmodule
