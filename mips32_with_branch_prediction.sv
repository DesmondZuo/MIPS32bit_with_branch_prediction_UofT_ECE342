`define THRESHOLD 128
`define RANGE     10
`define BPC_ENTRY 32


module cpu (
	input clk,
	input reset,
	
	// read only port
	output [31:0] o_pc_addr,
	output 	      o_pc_rd,
	input  [31:0] i_pc_rddata,
	output [3:0]  o_pc_byte_en,
	
	// read/write port
	output [31:0] o_ldst_addr,
	output 		  o_ldst_rd,
	output 		  o_ldst_wr,
	input  [31:0] i_ldst_rddata,
	output [31:0] o_ldst_wrdata,
	output [3:0]  o_ldst_byte_en,
	
	output [31:0] o_tb_regs [0:31]
);

typedef struct packed {
	reg        flush;
	reg [31:0] pc;
	reg 	   hit;
	reg        taken;
	reg 	   jump;
} PIPREG_IF_ID;

typedef struct packed {
	logic        flush;
	logic [31:0] pc;
	logic 		 hit;
	logic        taken;
	logic 		 jump;
} W_PIPREG_IF_ID;

typedef struct packed {
	reg [31:0]  pc;
	reg [4:0] 	rd;
	reg [4:0] 	rs1;
	reg [4:0] 	rs2;
	reg [31:0]  ra;
	reg [31:0]  rb;
	reg [31:0]  imval;
	reg [3:0]   funct3;
	reg [6:0]   funct7;
	reg [6:0]   opcode;

	reg 	   hit;
	reg        taken;
	reg 	   jump;
} PIPREG_ID_EX;

typedef struct packed {
	logic [31:0]  pc;
	logic [4:0]   rd;
	logic [4:0]   rs1;
	logic [4:0]   rs2;
	logic [31:0]  ra;
	logic [31:0]  rb;
	logic [31:0]  imval;
	logic [3:0]   funct3;
	logic [6:0]   funct7;
	logic [6:0]   opcode;

	logic 		 hit;
	logic        taken;
	logic 		 jump;
} W_PIPREG_ID_EX;

typedef struct packed {
	reg [4:0]  rd;
	reg [31:0] alu_out;
	reg [3:0]  funct3;
	reg [6:0]  opcode;
} PIPREG_EX_WR;

typedef struct packed {
	logic [4:0]  rd;
	logic [31:0] alu_out;
	logic [3:0]  funct3;
	logic [6:0]  opcode;
} W_PIPREG_EX_WR;

typedef struct packed {
	logic        wr_en;   
	logic [4:0]  addr_a;  
	logic [4:0]  addr_b;  
	logic [4:0]  addr_c;  
	logic [31:0] rd_a;    
	logic [31:0] rd_b;    
	logic [31:0] wr_c;    
} RF_WIRES;

W_PIPREG_IF_ID w_preg_if_id;
W_PIPREG_ID_EX w_preg_id_ex;
W_PIPREG_EX_WR w_preg_ex_wr;

PIPREG_IF_ID   preg_if_id;
PIPREG_ID_EX   preg_id_ex;
PIPREG_EX_WR   preg_ex_wr;

RF_WIRES	 wire_rf;

logic [31:0] w_pc_writeback;
logic        w_flush;
logic        w_mux_pc;
logic [31:0] w_forward;

logic[31:0] rf_readout [31:0];

assign o_tb_regs = rf_readout;

logic [1:0]  w_write_hit_flags;

logic [31:0] w_lookup_pc;
logic [31:0] w_prev_pc;        
logic [31:0] w_branched_pc;    
logic        w_predict_result; 
logic        w_bpc_jump;       
logic        w_bpc_write_en;   
logic [31:0] w_hit_pc;         
logic [1:0]  w_hit_flags;      

logic        w_is_jump;

branch_prediction_cache bpc (
	.clk            (clk),
	.reset          (reset),
	.lookup_pc      (w_lookup_pc),
	.prev_pc        (w_prev_pc),
	.branched_pc    (w_branched_pc),
	.predict_result (w_predict_result),
	.bpc_jump       (w_bpc_jump),
	.bpc_write_en   (w_bpc_write_en),
	.is_jump        (w_is_jump),
	.write_hit_flags(w_write_hit_flags),
	.hit_pc         (w_hit_pc),
	.hit_flags      (w_hit_flags)
);

state_0 s0 (
	.clk             (clk),
	.reset           (reset),
	.mux_pc			 (w_mux_pc),
	.i_flush     	 (w_flush),
	.pc_writeback    (w_pc_writeback),
	.o_pip_pctemp    (w_preg_if_id.pc),
	.o_pc_addr       (o_pc_addr),
	.o_pc_rd         (o_pc_rd),
	.o_pc_byte_en    (o_pc_byte_en),
	.lookup_pc       (w_lookup_pc),
	.bpc_pc_input    (w_hit_pc),
	.hit_flags       (w_hit_flags),
	.is_jump     	 (w_is_jump),
	.o_pip_hit  	 (w_preg_if_id.hit),
	.o_pip_taken	 (w_preg_if_id.taken),
	.o_pip_jump 	 (w_preg_if_id.jump),
	.flush 			 (w_preg_if_id.flush)
);

assign w_preg_id_ex.hit   = preg_if_id.hit;
assign w_preg_id_ex.taken = preg_if_id.taken;
assign w_preg_id_ex.jump  = preg_if_id.jump;

state_1 s1 (
	.clk             (clk),
	.reset           (reset),
	.flush           (w_flush),
	.second_flush    (preg_if_id.flush),
	.i_pc_rddata     (i_pc_rddata),
	.i_pip_pctemp    (preg_if_id.pc),
	.i_data_rs1      (wire_rf.rd_a),
	.i_data_rs2      (wire_rf.rd_b),
	.o_addr_rs1      (wire_rf.addr_a),
	.o_addr_rs2      (wire_rf.addr_b),
	.o_pip_pctemp    (w_preg_id_ex.pc),
	.o_pip_rd        (w_preg_id_ex.rd),
	.o_pip_rs1       (w_preg_id_ex.rs1),
	.o_pip_rs2       (w_preg_id_ex.rs2),
	.o_pip_ra        (w_preg_id_ex.ra),
	.o_pip_rb        (w_preg_id_ex.rb),
	.o_pip_funct3    (w_preg_id_ex.funct3),
	.o_pip_funct7    (w_preg_id_ex.funct7),
	.o_pip_opcode    (w_preg_id_ex.opcode),
	.o_pip_imval     (w_preg_id_ex.imval)
);

state_2 s2 (
	.clk           	 (clk),
	.reset         	 (reset),
	.flush         	 (w_flush),
	.i_pip_pctemp  	 (preg_id_ex.pc),
	.i_pip_rd      	 (preg_id_ex.rd),
	.i_pip_rs1     	 (preg_id_ex.rs1),
	.i_pip_rs2     	 (preg_id_ex.rs2),
	.i_pip_ra      	 (preg_id_ex.ra),
	.i_pip_rb      	 (preg_id_ex.rb),
	.i_pip_funct3  	 (preg_id_ex.funct3),
	.i_pip_funct7  	 (preg_id_ex.funct7),
	.i_pip_opcode  	 (preg_id_ex.opcode),
	.i_pip_imval   	 (preg_id_ex.imval),
	.i_prev_rd     	 (preg_ex_wr.rd),
	.i_forward     	 (w_forward),
	.o_ldst_addr   	 (o_ldst_addr),
	.o_ldst_rd     	 (o_ldst_rd),
	.o_ldst_wr     	 (o_ldst_wr),
	.o_ldst_wrdata 	 (o_ldst_wrdata),
	.o_ldst_byte_en	 (o_ldst_byte_en),
	.o_pip_rd      	 (w_preg_ex_wr.rd),
	.o_pip_alu_out 	 (w_preg_ex_wr.alu_out),
	.o_pc_writeback	 (w_pc_writeback),
	.mux_pc        	 (w_mux_pc),

	.i_pip_hit		 (preg_id_ex.hit),
	.i_pip_taken	 (preg_id_ex.taken),
	.i_pip_jump		 (preg_id_ex.jump),

	.write_hit_flags (w_write_hit_flags),
	.prev_pc       	 (w_prev_pc),
	.branched_pc   	 (w_branched_pc),
	.predict_result	 (w_predict_result),
	.bpc_jump      	 (w_bpc_jump),
	.bpc_write_en  	 (w_bpc_write_en)
);

assign w_preg_ex_wr.funct3 = preg_id_ex.funct3;
assign w_preg_ex_wr.opcode = preg_id_ex.opcode;

state_3 s3 (
	.clk             (clk),
	.reset           (reset),

	.i_ldst_addr     (o_ldst_addr),
	.i_ldst_byte_en  (o_ldst_byte_en),
	.i_pip_funct3    (preg_ex_wr.funct3),
	.i_pip_opcode    (preg_ex_wr.opcode),
	.i_ldst_rddata   (i_ldst_rddata),
	.i_pip_rd        (preg_ex_wr.rd),
	.i_pip_alu_out   (preg_ex_wr.alu_out),
	.o_rf_wraddr     (wire_rf.addr_c),
	.o_rf_wrdata     (wire_rf.wr_c),
	.o_rf_wren       (wire_rf.wr_en),
	.o_forward       (w_forward)
);

register_file_32x32 rf (
	.clk      		 (clk),
	.reset    		 (reset),
	.wr_en    		 (wire_rf.wr_en),
	.addr_a   		 (wire_rf.addr_a),
	.addr_b   		 (wire_rf.addr_b),
	.addr_c   		 (wire_rf.addr_c),
	.rd_a     		 (wire_rf.rd_a),
	.rd_b     		 (wire_rf.rd_b),
	.wr_c     		 (wire_rf.wr_c),
	.copy_all 		 (rf_readout)
);

// Pipline register follow on posedge

always_ff @ (posedge clk or posedge reset) begin
	if (reset) begin
		preg_if_id.flush  <= 0;
		preg_if_id.pc     <= 0;
		preg_if_id.hit 	  <= 0;
		preg_if_id.taken  <= 0;
		preg_if_id.jump   <= 0;

		preg_id_ex.pc     <= 0;
		preg_id_ex.rd     <= 0;
		preg_id_ex.rs1    <= 0;
		preg_id_ex.rs2    <= 0;
		preg_id_ex.ra     <= 0;
		preg_id_ex.rb     <= 0;
		preg_id_ex.imval  <= 0;
		preg_id_ex.funct3 <= 0;
		preg_id_ex.funct7 <= 0;
		preg_id_ex.opcode <= 0;
		preg_id_ex.hit 	  <= 0;
		preg_id_ex.taken  <= 0;
		preg_id_ex.jump   <= 0;

		preg_ex_wr.rd     <= 0;
		preg_ex_wr.alu_out<= 0;
		preg_ex_wr.funct3 <= 0;
		preg_ex_wr.opcode <= 0;
	end else begin
		preg_if_id.flush  <= w_preg_if_id.flush  ;
		preg_if_id.pc     <= w_preg_if_id.pc     ;
		preg_if_id.hit 	  <= w_preg_if_id.hit 	 ;
		preg_if_id.taken  <= w_preg_if_id.taken  ;
		preg_if_id.jump   <= w_preg_if_id.jump   ;

		preg_id_ex.pc     <= w_preg_id_ex.pc     ;
		preg_id_ex.rd     <= w_preg_id_ex.rd     ;
		preg_id_ex.rs1    <= w_preg_id_ex.rs1    ;
		preg_id_ex.rs2    <= w_preg_id_ex.rs2    ;
		preg_id_ex.ra     <= w_preg_id_ex.ra     ;
		preg_id_ex.rb     <= w_preg_id_ex.rb     ;
		preg_id_ex.imval  <= w_preg_id_ex.imval  ;
		preg_id_ex.funct3 <= w_preg_id_ex.funct3 ;
		preg_id_ex.funct7 <= w_preg_id_ex.funct7 ;
		preg_id_ex.opcode <= w_preg_id_ex.opcode ;
		preg_id_ex.hit 	  <= w_preg_id_ex.hit 	 ;
		preg_id_ex.taken  <= w_preg_id_ex.taken  ;
		preg_id_ex.jump   <= w_preg_id_ex.jump   ;

		preg_ex_wr.rd     <= w_preg_ex_wr.rd     ;
		preg_ex_wr.alu_out<= w_preg_ex_wr.alu_out;
		preg_ex_wr.funct3 <= w_preg_ex_wr.funct3 ;
		preg_ex_wr.opcode <= w_preg_ex_wr.opcode ;
	end
end

endmodule : cpu

module branch_prediction_cache (
	input 		        clk,
	input 		        reset,
	input 		 [31:0] lookup_pc,
	input 		 [31:0] prev_pc,
	input 		 [31:0] branched_pc,
	input 		 		predict_result,
	input 		 		bpc_jump,
	input		 		bpc_write_en,

	output logic        is_jump,
	output logic [1:0]	write_hit_flags,
	output logic [31:0] hit_pc,
	output logic [1:0]  hit_flags // 0-miss, 1-hit, taken, 2-hit,nottaken
);
	reg evict;
	logic input_hit;
	logic [5:0] input_hit_index;
	
	logic [73:0] bpc_write_entry;
	reg   [5:0]  bpc_write_pointer;
	reg   [5:0]  bpc_evict_pointer;

	reg   [73:0] bpc [0:`BPC_ENTRY-1];
	
	// Input Encode to bpc entry
	always_comb begin
		bpc_write_entry[31:0] = prev_pc;
		bpc_write_entry[63:32]= branched_pc;
		bpc_write_entry[72:65]= 8'd128;
		bpc_write_entry[73] = 1;
		if (bpc_jump) begin
			bpc_write_entry[64] = 1;
		end else begin
			bpc_write_entry[64] = 0;
		end
	end
	logic break_logic_1;
	logic break_logic_2;
	
	integer j;
	integer k;
	always_comb begin
		break_logic_1 = 0;
		
		for (j = 0; j < `BPC_ENTRY-1 && break_logic_1 == 0; j=j+1) begin
			// (for WRITING to BPC) Iterate through the bpc entry to find hit or miss 
			if (bpc_write_en && bpc[j][73] == 1 && bpc[j][31:0] == bpc_write_entry[31:0]) begin
				// hit
				input_hit = 1;
				input_hit_index = j;
				if (bpc[j][64]) begin
					// if jump, hit taken.
					write_hit_flags = 2'b01;
				end else begin
					// if Branch, determine taken or not taken.
					write_hit_flags = (bpc[j][72:65] > `THRESHOLD) ? 2'b01 : 2'b10;
				end
				break_logic_1 = 1;
			end else begin
				input_hit = 0;
				input_hit_index = 0;
				write_hit_flags = 0;
			end
		end
		break_logic_2 = 0;
		
		for (k = 0; k < `BPC_ENTRY-1 && break_logic_2 == 0; k=k+1) begin
			// (for READING to BPC) Iterate through the bpc entry to find hit or miss
			if (bpc[k][73] == 1 && bpc[k][31:0] == lookup_pc[31:0]) begin
				if (bpc[k][64]) begin
					// if jump, hit taken.
					is_jump = 1;
					hit_flags = 2'b01;
				end else begin
					// if Branch, determine taken or not taken.
					is_jump = 0;
					hit_flags = (bpc[k][72:65] > `THRESHOLD) ? 2'b01 : 2'b10;
				end
				// either way, hit_pc will be the branch pc of bpc.
				hit_pc    = bpc[k][63:32];
				break_logic_2 = 1;
			end else begin
				// miss, output hit_flags.
				is_jump   = 0;
				hit_pc    = 0;
				hit_flags = 0;
			end
			
		end
		
	end

	integer m;
	always @ (posedge clk or posedge reset) begin
		if (reset) begin
			
			for (m = 0; m < `BPC_ENTRY-1; m=m+1) begin
				bpc[m] <= 74'b0;
			end
			evict             <= 0;
			bpc_write_pointer <= 0;
			bpc_evict_pointer <= 0;
		end

		else begin
			m = 0;
			// Write only on Not hit and write_en (incoming data misses)
			if (input_hit == 0 && bpc_write_en && evict) begin
				// Write address will be the evict pointer.
				bpc[bpc_evict_pointer] <= bpc_write_entry;
				// Evict handler
				if (bpc_evict_pointer == `BPC_ENTRY-1) begin
					// If bpc_evict_pointer reached 64, reset it.
					bpc_evict_pointer <= 0;
				end else begin
					// Increment the evict pointer
					bpc_evict_pointer <= bpc_evict_pointer + 1;
				end
			end
			// Evict if BPC miss and BPC full.
			else if (input_hit == 0 && bpc_write_en && !evict) begin
				// Write address will be the write pointer.
				bpc[bpc_write_pointer] <= bpc_write_entry;
				// Evict handler
				if (bpc_write_pointer == `BPC_ENTRY-1 && !evict) begin
					// Reset bpc_write_pointer and enable evict
					bpc_write_pointer <= 0;
					evict <= 1;
				end else begin
					bpc_write_pointer <= bpc_write_pointer + 1;
				end
			end
			// Hit, increment or decrement the counter!
			else if (input_hit == 1 && bpc_write_en && bpc[input_hit_index][64] == 0) begin
				if (predict_result == 0 && write_hit_flags == 1 && $unsigned(bpc[input_hit_index][72:65]) < $unsigned(`THRESHOLD + `RANGE)) begin
					// Correct prediction for taken.
					bpc[input_hit_index][72:65] <= bpc[input_hit_index][72:65] + 1;
				end else if (predict_result == 0 && write_hit_flags == 2 && $unsigned(bpc[input_hit_index][72:65]) > $unsigned(`THRESHOLD - `RANGE))  begin
					// Correct prediction for not taken
					bpc[input_hit_index][72:65] <= bpc[input_hit_index][72:65] - 1;
				end else if (predict_result == 1 && write_hit_flags == 1 && $unsigned(bpc[input_hit_index][72:65]) > $unsigned(`THRESHOLD - `RANGE)) begin
					// Incorrect prediction for taken
					bpc[input_hit_index][72:65] <= bpc[input_hit_index][72:65] - 1;
				end else if (predict_result == 1 && write_hit_flags == 2 && $unsigned(bpc[input_hit_index][72:65]) < $unsigned(`THRESHOLD + `RANGE)) begin
					// Incorrect prediction for not taken
					bpc[input_hit_index][72:65] <= bpc[input_hit_index][72:65] + 1;
				end
			end
		end
	end



endmodule : branch_prediction_cache

module state_0 (
	input clk,
	input reset,
	input mux_pc,
	input i_flush,
	// pc flush
	input [31:0] pc_writeback,
	output[31:0] o_pip_pctemp,
	// pc_mem
	output [31:0] o_pc_addr,
	output 	      o_pc_rd,
	output [3:0]  o_pc_byte_en,
	// Branch Prediction Cache
	output logic [31:0] lookup_pc,
	input        [31:0] bpc_pc_input,
	input        [1:0]  hit_flags,
	input 				is_jump,

	output logic  o_pip_hit,
	output logic  o_pip_taken,
	output logic  o_pip_jump,

	output logic  flush
);
	reg  [31:0] reg_pc;

	assign o_pip_hit   = (hit_flags == 1 || hit_flags == 2) ? 1 : 0;
	assign o_pip_taken = (hit_flags == 1) ? 1 : 0;
	assign o_pip_jump  = is_jump;

	always_ff @ (posedge clk or posedge reset) begin
		if (reset) reg_pc <= 0;
		else 	   reg_pc <= mux_pc ? pc_writeback : (o_pip_taken ? bpc_pc_input : reg_pc + 4);
	end

	assign lookup_pc        = reg_pc;

	assign o_pip_pctemp     = reg_pc;
	assign flush            = mux_pc;

	assign o_pc_addr        = reg_pc;
	assign o_pc_rd 			= 1;
	assign o_pc_byte_en		= 4'b1111;

endmodule : state_0

module state_1 (
	input 		 clk,
	input 		 reset,
	input		 flush,
	input		 second_flush,

	input [31:0] i_pc_rddata,

	input [31:0] i_pip_pctemp,

	input [31:0] i_data_rs1,
	input [31:0] i_data_rs2,
	output[4:0]  o_addr_rs1,
	output[4:0]  o_addr_rs2,

	output[31:0] o_pip_pctemp,
	output[4:0]  o_pip_rd,
	output[4:0]  o_pip_rs1,
	output[4:0]  o_pip_rs2,
	output[31:0] o_pip_ra,
	output[31:0] o_pip_rb,
	output[3:0]  o_pip_funct3,
	output[6:0]  o_pip_funct7,
	output[6:0]  o_pip_opcode,
	output[31:0] o_pip_imval
);
	wire [4:0]   w_rd;
	wire [4:0]   w_rs1;  
	wire [4:0]   w_rs2;  
	wire [31:0]  w_ra;   
	wire [31:0]  w_rb;
	wire [3:0]   w_funct3;
	wire [6:0]   w_funct7;
	wire [6:0]   w_opcode;
	wire [31:0]  w_imval;

	ir_decoder ird (
		.ir    	(i_pc_rddata),
		.imval 	(w_imval),
		.rs1   	(w_rs1),
		.rs2   	(w_rs2),
		.rd    	(w_rd),
		.funct3	(w_funct3),
		.funct7	(w_funct7),
		.opcode	(w_opcode)
	);

	assign w_ra         = i_data_rs1;
	assign w_rb         = i_data_rs2;
	assign o_addr_rs1   = w_rs1;
	assign o_addr_rs2   = w_rs2;

	assign o_pip_pctemp = (flush || second_flush) ? 0 : i_pip_pctemp;
	assign o_pip_rd     = (flush || second_flush) ? 0 : w_rd;
	assign o_pip_rs1    = (flush || second_flush) ? 0 : w_rs1;
	assign o_pip_rs2    = (flush || second_flush) ? 0 : w_rs2;
	assign o_pip_ra     = (flush || second_flush) ? 0 : w_ra;
	assign o_pip_rb     = (flush || second_flush) ? 0 : w_rb;
	assign o_pip_funct3 = (flush || second_flush) ? 0 : w_funct3;
	assign o_pip_funct7 = (flush || second_flush) ? 0 : w_funct7;
	assign o_pip_opcode = (flush || second_flush) ? 0 : w_opcode;
	assign o_pip_imval  = (flush || second_flush) ? 0 : w_imval;

endmodule : state_1

module state_2 (
	input        		 clk,
	input        		 reset,
	output logic 		 flush,
 	
 	input [31:0] 		 i_pip_pctemp,
	input [4:0]  		 i_pip_rd,
	input [4:0]  		 i_pip_rs1,
	input [4:0]  		 i_pip_rs2,
	input [31:0] 		 i_pip_ra,
	input [31:0] 		 i_pip_rb,
	input [3:0]  		 i_pip_funct3,
	input [6:0]  		 i_pip_funct7,
	input [6:0]  		 i_pip_opcode,
	input [31:0] 		 i_pip_imval,
  
	input [4:0]  		 i_prev_rd,
	input [31:0] 		 i_forward,

	output logic [31:0]  o_ldst_addr,
	output logic         o_ldst_rd,
	output logic         o_ldst_wr,
	output logic [31:0]  o_ldst_wrdata,
	output logic [3:0]   o_ldst_byte_en,

	output logic [4:0]   o_pip_rd,
	output logic [31:0]  o_pip_alu_out,
 
	output logic [31:0]  o_pc_writeback,
	output logic 		 mux_pc,

	input 			     i_pip_hit,	
	input 			     i_pip_taken,
	input 			     i_pip_jump,	

	input  		 [1:0]	 write_hit_flags,
	output logic [31:0]  prev_pc,
	output logic [31:0]  branched_pc,
	output logic         predict_result,
	output logic		 bpc_jump,
	output logic         bpc_write_en
);
	
	logic [3:0]  w_alu_sel;
	logic [31:0] w_alu_a;
	logic [31:0] w_alu_b;
	logic [31:0] w_alu_rs1;
	logic [31:0] w_alu_rs2;
	logic [31:0] w_alu_out;
	logic [5:0]  w_flag;
	logic 		 branch_result;

	alu_32b alu (
		.sel  (w_alu_sel),
		.rs1  (w_alu_a),
		.rs2  (w_alu_b),
		.out  (w_alu_out)
	);

	flag_alu aluf (
		.rs1  (w_alu_rs1),
		.rs2  (w_alu_rs2),
		.flag (w_flag)
	);

	assign o_pip_rd = i_pip_rd;
	assign o_pip_alu_out = w_alu_out;
	// ALU data and control logics
	// ALU Sel logic
	always_comb begin
		if (i_pip_opcode == 7'b0110011 || i_pip_opcode == 7'b0010011) begin
			/* add, sub, xor, or, and */ /* addi, xori, ori, andi */
			if      (i_pip_opcode == 7'b0110011 && i_pip_funct3 == 0 && i_pip_funct7 == 7'h00) w_alu_sel = 0; // add
			else if (i_pip_opcode == 7'b0110011 && i_pip_funct3 == 0 && i_pip_funct7 == 7'h20) w_alu_sel = 1; // sub
			else if (i_pip_opcode != 7'b0110011 && i_pip_funct3 == 0) w_alu_sel = 0; // addi
			else if (i_pip_funct3 == 4) w_alu_sel = 2; // xor, xori
			else if (i_pip_funct3 == 6) w_alu_sel = 3; // or, ori
			else if (i_pip_funct3 == 7) w_alu_sel = 4; // and, andi
			/* sll,  srl,  sra */ /* slli, srli, srai */
			else if (i_pip_funct3 == 1) w_alu_sel = 5; // sll, slli
			else if (i_pip_funct3 == 5 && i_pip_funct7 == 7'h00) w_alu_sel = 6; // srl, srli
			else if (i_pip_funct3 == 5 && i_pip_funct7 == 7'h20) w_alu_sel = 7; // sra, srai
			/* slt,  sltu */ /* slti, sltui */
			else if (i_pip_funct3 == 2) w_alu_sel = 8; // slt, slti
			else if (i_pip_funct3 == 3) w_alu_sel = 9; // slti, sltui
			else w_alu_sel = 0; // add
		end
		
		else begin
			w_alu_sel = 0; // add
		end
	end

	// ALU input logic
	always_comb begin
		// R instr
		if (i_pip_opcode == 7'b0110011) begin
			w_alu_a = w_alu_rs1;
			if (i_pip_funct3 == 1 || i_pip_funct3 == 5) begin
				w_alu_b = {{27{1'b0}},{w_alu_rs2[4:0]}};
			end else begin
				w_alu_b = w_alu_rs2;
			end
		end
		// I (ALU) instr
		else if (i_pip_opcode == 7'b0010011) begin
			w_alu_a = w_alu_rs1;
			if (i_pip_funct3 == 1 || i_pip_funct3 == 5) begin
				w_alu_b = {{27{1'b0}},{i_pip_imval[4:0]}};
			end else begin
				w_alu_b = i_pip_imval;
			end
		end
		// I (LOAD) instr; S (STORE) instr
		else if (i_pip_opcode == 7'b0000011 || i_pip_opcode == 7'b0100011) begin
			w_alu_a = w_alu_rs1;
			w_alu_b = i_pip_imval;
		end
		// Load Upper Immediate
		else if (i_pip_opcode == 7'b0110111) begin
			w_alu_a = 0;
			w_alu_b = i_pip_imval;
		end
		// Add Upper IMM to PC
		else if (i_pip_opcode == 7'b0010111) begin
			w_alu_a = i_pip_pctemp;
			w_alu_b = i_pip_imval;
		end
		// Jalr & Jal
		else if (i_pip_opcode == 7'b1100111 || i_pip_opcode == 7'b1101111) begin
			w_alu_a = i_pip_pctemp;
			w_alu_b = 4;
		end

		else begin
			w_alu_a = 0;
			w_alu_b = 0;
		end
	end

	// Forward Controller
	always_comb begin
		if (i_prev_rd != 0 && (i_prev_rd == i_pip_rs1)) begin
			w_alu_rs1 = i_forward;
		end else begin
			w_alu_rs1 = i_pip_ra;
		end

		if (i_prev_rd != 0 && (i_prev_rd == i_pip_rs2)) begin
			w_alu_rs2 = i_forward;
		end else begin
			w_alu_rs2 = i_pip_rb;
		end
	end

	// Memory write / read handler
	always_comb begin
		if (i_pip_opcode == 7'b0100011) begin
			case (i_pip_funct3)
				0: begin 
					o_ldst_wrdata  = {{24{1'b0}}, {w_alu_rs2 [7:0]}};
					o_ldst_byte_en = 4'b0001;
				end
				1: begin 
					o_ldst_wrdata  = {{16{1'b0}}, {w_alu_rs2 [15:0]}};
					o_ldst_byte_en = 4'b0011;
				end
				2: begin 
					o_ldst_wrdata  = w_alu_rs2;
					o_ldst_byte_en = 4'b1111;
				end
				default: begin
					o_ldst_wrdata  = 0;
					o_ldst_byte_en = 0;
				end
			endcase // i_pip_funct3
			o_ldst_addr   = w_alu_out;
			o_ldst_wr     = 1;
			o_ldst_rd 	  = 0;
		end

		else if (i_pip_opcode == 7'b0000011) begin
			o_ldst_wrdata = 0;
			case (i_pip_funct3)
				0, 4: begin
					o_ldst_byte_en = 4'b0001;
				end

				1, 5: begin
					o_ldst_byte_en = 4'b0011;
				end

				2: begin
					o_ldst_byte_en = 4'b1111;
				end

				default: begin
					o_ldst_byte_en = 4'b1111;
				end
			endcase // i_pip_funct3
			o_ldst_addr   = w_alu_out;
			o_ldst_wr     = 0;
			o_ldst_rd 	  = 1;
		end

		else begin
			o_ldst_wrdata  = 0;
			o_ldst_byte_en = 4'b1111;
			o_ldst_addr    = 0;
			o_ldst_wr      = 0;
			o_ldst_rd 	   = 0;
		end
	end

	always_comb begin
		// get the flush flags.
		case (i_pip_funct3)
			0: branch_result = w_flag[0];
			1: branch_result = w_flag[1];
			4: branch_result = w_flag[2];
			5: branch_result = w_flag[3];
			6: branch_result = w_flag[4];
			7: branch_result = w_flag[5];
			default: branch_result = 0;
		endcase // i_pip_funct3
		// Predicted taken, actually should not be taking.
	end

	// Flush PC handler
	always_comb begin
		// jalr
		if (i_pip_opcode == 7'b1100111) begin
			// PC = PC + rs1
			o_pc_writeback = w_alu_rs1 + i_pip_imval;
			// Must flush.
			flush  = 1;
			mux_pc = 1;
			// Don't write anything!
			prev_pc        = 0;
			branched_pc    = 0;
			predict_result = 0;
			bpc_jump       = 0;
			bpc_write_en   = 0;
		end

		// jal
		else if (i_pip_opcode == 7'b1101111) begin
			// PC = PC + imval
			o_pc_writeback = i_pip_imval + i_pip_pctemp;
			// If it was a hit, don't flush
			if (i_pip_hit) begin
				flush  = 0;
				mux_pc = 0;
			end else begin
				flush  = 1;
				mux_pc = 1;
			end
			// Write to BPC
			prev_pc         = i_pip_pctemp;
			branched_pc     = o_pc_writeback;
			predict_result  = 0; // Doesn't matter!
			bpc_jump        = 1;
			bpc_write_en    = 1;
		end

		//Branching
		else if (i_pip_opcode == 7'b1100011) begin
			if (i_pip_taken && !branch_result) begin
				// PC = PC(false prediction) + 4
				o_pc_writeback  = i_pip_pctemp + 4;
				prev_pc         = i_pip_pctemp;
				branched_pc     = i_pip_pctemp + i_pip_imval;
				// Flush.
				flush   = 1;
				mux_pc  = 1;
				// False prediction, write to BPC.
				predict_result  = 1;
				bpc_jump        = 0;
				bpc_write_en    = 1;
			end
			// Predicted taken and correct, Predicted not taken and correct
			else if ((i_pip_taken && branch_result) || (!i_pip_taken && !branch_result))begin
				o_pc_writeback  = i_pip_pctemp + i_pip_imval;
				prev_pc         = i_pip_pctemp;
				branched_pc     = i_pip_pctemp + i_pip_imval;
				// No flush.
				flush   = 0;
				mux_pc  = 0;
				// Correct prediction, write to BPC.
				predict_result  = 0;
				bpc_jump        = 0;
				bpc_write_en    = 1;
			end
			// OR predicted not taken, actually should be taking.
			else if (!i_pip_taken && branch_result) begin
				// PC = PC(false prediction) + imval
				o_pc_writeback  = i_pip_pctemp + i_pip_imval;
				prev_pc         = i_pip_pctemp;
				branched_pc     = i_pip_pctemp + i_pip_imval;
				// Flush.
				flush   = 1;
				mux_pc  = 1;
				// False prediction, write to BPC.
				predict_result  = 1;
				bpc_jump        = 0;
				bpc_write_en    = 1;
			end
			// Void case, will not be happening.
			else begin
				// PC = PC(false prediction) + 4
				o_pc_writeback  = i_pip_pctemp + 4;
				// No flush.
				flush   = 0;
				mux_pc  = 0;
				// Don't write anything!
				prev_pc         = 0;
				branched_pc     = 0;
				predict_result  = 0;
				bpc_jump        = 0;
				bpc_write_en    = 0;
			end
		end

		// Irrelavent instructions
		else begin
			// PC = PC(false prediction) + 4
			o_pc_writeback  = i_pip_pctemp + 4;
			// No flush.
			flush   = 0;
			mux_pc  = 0;
			// Don't write anything!
			prev_pc         = 0;
			branched_pc     = 0;
			predict_result  = 0;
			bpc_jump        = 0;
			bpc_write_en    = 0;
		end
	end

endmodule : state_2


module state_3 (
	input 		  		clk,
	input 		  		reset,

	input        [31:0] i_ldst_addr,
	input        [3:0]	i_ldst_byte_en,
	input  		 [3:0]  i_pip_funct3,
	input  		 [6:0]  i_pip_opcode,
	input  		 [31:0] i_ldst_rddata,
	input  		 [4:0]  i_pip_rd,
	input  		 [31:0] i_pip_alu_out,
	output logic [4:0]  o_rf_wraddr,
	output logic [31:0] o_rf_wrdata,
	output logic 		o_rf_wren,

	output logic [31:0] o_forward
);
	

	// rd enable logic
	always_comb begin
		if (i_pip_opcode == 7'b0100011 || i_pip_opcode == 7'b1100011) begin
			o_rf_wren = 0;
		end else begin
			o_rf_wren = 1;
		end
	end

	// load byte/halfword/word logic
	always_comb begin
		if (i_pip_opcode == 7'b0000011) begin
			case (i_pip_funct3)
				0: begin
					o_rf_wrdata = {{24{i_ldst_rddata[7]}}, {i_ldst_rddata[7:0]}};
				end
				1: begin
					o_rf_wrdata = {{16{i_ldst_rddata[15]}}, {i_ldst_rddata[15:0]}};
				end
				2: begin
					o_rf_wrdata = i_ldst_rddata[31:0];
				end
				4: begin
					o_rf_wrdata = {{24{1'b0}}, {i_ldst_rddata[7:0]}};
				end
				5: begin
					o_rf_wrdata = {{16{1'b0}}, {i_ldst_rddata[15:0]}};
				end
				default: o_rf_wrdata = 0;
			endcase // i_pip_funct3
		end else begin
			o_rf_wrdata = i_pip_alu_out;
		end
		o_rf_wraddr = i_pip_rd;
	end

	assign o_forward = o_rf_wrdata;

endmodule : state_3


module ir_decoder (
	input 		 [31:0] ir,

	output logic [31:0] imval,
	output logic [4:0]  rs1,
	output logic [4:0]  rs2,
	output logic [4:0]  rd,
	output logic [3:0]  funct3,
	output logic [6:0]  funct7,
	output logic [6:0]  opcode
);
	always_comb begin
		case (ir[6:0])
			7'b0110011: 		   /*R*/ begin
				imval       = 0;
				rs1			= ir[19:15];
				rs2			= ir[24:20];
				rd 			= ir[11:7];
				funct3 		= ir[14:12];
				funct7 		= ir[31:25];
				opcode 		= ir[6:0];
			end

			7'b0010011, 7'b0000011, 7'b1100111: /*I*/ begin
				imval [11:0]= ir[31:20];
				imval[31:12]= {20{imval[11]}}; // imval msb extended
				rs1	   		= ir[19:15];
				rs2         = 0;
				rd 	   		= ir[11:7];
				funct3 		= ir[14:12];
				funct7 		= ir[31:25];
				opcode 		= ir[6:0];
			end

			7'b0100011: 		   /*S*/ begin
				opcode      = ir[6:0];
				rs1    		= ir[19:15];
				rs2    		= ir[24:20];
				rd 			= 0;
				imval[4:0]  = ir[11:7];
				funct3      = ir[14:12];
				funct7 		= 0;
				imval[11:5] = ir[31:25];
				// imval msb extended
				imval[31:12]= {20{imval[11]}};
			end

			7'b1100011: 		   /*B*/ begin
				opcode      = ir[6:0];
				rs1   		= ir[19:15];
				rs2   		= ir[24:20];
				rd 			= 0;
				funct3      = ir[14:12];
				funct7 		= 0;
				{imval[4:1] , imval[11]  } = ir[11:7];
				{imval[12]  , imval[10:5]} = ir[31:25];
				imval[0]    = 1'b0;
				// imval msb extended
				imval[31:13]= {19{imval[11]}};
			end

			7'b0110111, 7'b0010111: 		   /*U*/ begin
				opcode		= ir[6:0];
				rs1 		= 0;
				rs2 		= 0;
				rd			= ir[11:7];
				funct3 		= 0;
				funct7 		= 0;
				imval[31:12]= ir[31:12];
				// use zero to fill the remaining part.
				imval[11:0] = {12{1'b0}};
			end

			7'b1101111: 		   /*J*/ begin
				opcode 		= ir[6:0];
				rd 			= ir[11:7];
				rs1 		= 0;
				rs2 		= 0;
				funct3 		= 0;
				funct7 		= 0;
				{imval[20], imval[10:1], imval[11], imval[19:12]} = ir[31:12];
				// use zero to fill the remaining part.
				imval[0]    = 1'b0;
				// imval msb extended
				imval[31:21]= {11{imval[20]}};
			end
			default: begin
				opcode = 0;
				rd = 0;
				rs1 = 0;
				rs2 = 0;
				funct3 = 0;
				funct7 = 0;
				imval = 0;
			end
		endcase // ir[6:0]
	end
endmodule : ir_decoder


module register_file_32x32 (
	input  	      clk,
	input		  reset,
	input		  wr_en,
	input  [4:0]  addr_a,
	input  [4:0]  addr_b,
	input  [4:0]  addr_c,
	output [31:0] rd_a,
	output [31:0] rd_b,
	input  [31:0] wr_c,
	output [31:0] copy_all [0:31]
);

	reg [31:0] reg_file [0:31];

	assign copy_all = reg_file;
	assign rd_a = reg_file[addr_a];
	assign rd_b = reg_file[addr_b];

	integer i;
	always @ (posedge reset or negedge clk) begin
		if (reset) begin
			for (i = 0; i < 32; i=i+1) begin
				reg_file[i] <= 32'b0;
			end
		end else begin
			i = 0;
			if (wr_en) begin
				if (addr_c != 0) reg_file[addr_c] <= wr_c;
			end
		end
	end

endmodule : register_file_32x32


module flag_alu (
	input [31:0] rs1,
	input [31:0] rs2,
	output logic [5:0] flag
);
	parameter        EQ   =  3'd0,   GE   =  3'd3,
			         NE   =  3'd1,   LTU  =  3'd4,
			         LT   =  3'd2,   GEU  =  3'd5;

	always_comb begin
		flag[EQ]  = (rs1 ==  rs2) ?  1 : 0;
		flag[NE]  = (rs1 !=  rs2) ?  1 : 0;
		flag[LT]  = ($signed(rs1) <   $signed(rs2)) ?  1 : 0;
		flag[GE]  = ($signed(rs1) <   $signed(rs2)) ?  0 : 1;
		flag[LTU] = ($unsigned(rs1) <  $unsigned(rs2)) ? 1 : 0;
		flag[GEU] = ($unsigned(rs1) <  $unsigned(rs2)) ? 0 : 1;
	end

endmodule : flag_alu

module alu_32b (
	input  		 [3:0]  sel,  // {add, sub, xor, or, and, sll, srl, sra, slt, sltu}
	input  		 [31:0] rs1,
	input  		 [31:0] rs2,
	output logic [31:0] out
);

	parameter        ADD  =  4'd0,   SLL  =  4'd5,
			         SUB  =  4'd1,   SRL  =  4'd6,
			         XOR  =  4'd2,   SRA  =  4'd7,
			         OR   =  4'd3,   SLT  =  4'd8,
			         AND  =  4'd4,   SLTU =  4'd9;

	always_comb begin
		case (sel)
			ADD   :  out  =  rs1 +   rs2;
			SUB   :  out  =  rs1 -   rs2;
			XOR   :  out  =  rs1 ^   rs2;
			OR    :  out  =  rs1 |   rs2;
			AND   :  out  =  rs1 &   rs2;
			SLL   :  out  =  rs1 <<  rs2;
			SRL   :  out  =  rs1 >>  rs2;
			SRA   :  out  =  $signed(rs1) >>> rs2;
			SLT   :  out  = (rs1 >   rs2)   ?  1 : 0;
			SLTU  :  out  = ($unsigned(rs1) > $unsigned(rs2)) ?  0 : 1;
			default: begin
				out = 0;
			end
		endcase // sel
	end

endmodule : alu_32b
