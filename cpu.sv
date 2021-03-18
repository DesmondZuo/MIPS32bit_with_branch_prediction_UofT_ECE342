module cpu # (
	parameter IW = 32, // instr width
	parameter REGS = 32 // number of registers
)(
	input clk,
	input reset,
	
	// read only port
	output [IW-1:0] o_pc_addr,
	output 			o_pc_rd,
	input  [IW-1:0] i_pc_rddata,
	output [3:0]    o_pc_byte_en,
	
	// read/write port
	output [IW-1:0] o_ldst_addr,
	output 			o_ldst_rd,
	output 			o_ldst_wr,
	input  [IW-1:0] i_ldst_rddata,
	output [IW-1:0] o_ldst_wrdata,
	output [3:0]    o_ldst_byte_en,
	
	output [IW-1:0] o_tb_regs [0:REGS-1]
);
	wire 		w_rs2_cast_l5;
	wire 		w_imm_cast_l5;
	wire   		w_imm_sll_12;
	wire 		w_mem_wr_en;
	wire 		w_mem_rd_en;
	wire 		w_rf_wr_en;    
	wire 		w_ir_wr_en;    
	wire 		w_ir_bypass_en;
	wire 		w_ra_bypass_en;
	wire 		w_rb_bypass_en;
	wire 		w_rz_bypass_en;
	wire 		w_ry_bypass_en;
	wire 		w_rm_bypass_en;
	wire 		w_pc_wr_en;   
	wire 		w_alu_flag_en; 
	wire 		w_alu_flag_bp;
	wire 		w_sel_mux_a;
	wire [1:0]  w_sel_mux_b;
	wire [1:0]  w_sel_mux_c;   
	wire 		w_sel_mux_pc; 
	wire 		w_sel_mux_inc; 
	wire 		w_sel_mux_ma;  
	wire [1:0]  w_sel_mux_y;   
	wire [3:0]	w_sel_alu;     
	wire [31:0] w_mem_in;      
	wire [31:0] w_mem_out;
	wire [32:0] w_mem_addr;    
	wire [5:0]  w_flag_alu;
	wire [6:0]	w_opcode;
	wire [3:0]	w_funct3;
	wire [6:0]	w_funct7;
	wire [4:0]	w_rs1;
	wire [4:0]	w_rs2;

	wire [31:0] w_pc_in;
	wire 		w_pc_rd_en;
	wire [31:0] w_pc_addr;

	wire [3:0] w_ldst_byte;
	wire       w_ldst_sign;

	wire [31:0] w_pc_test;
	wire [31:0] w_ir_test;
	wire [31:0] w_imval_test;
	wire [31:0] w_write_test;
	wire [31:0] w_reg_a_test;
	wire [31:0] w_reg_b_test;
	wire [31:0] w_reg_z_test;
	wire [31:0] w_reg_y_test;
	wire [31:0] w_reg_m_test;

	wire [2:0]	state;

	/* Output Assignments */
	assign o_pc_rd = w_pc_rd_en;
	assign o_pc_addr = w_pc_addr;
	assign w_pc_in = i_pc_rddata;

	assign o_ldst_byte_en = w_ldst_byte;
	assign o_ldst_rd = w_mem_rd_en;
	assign o_ldst_wr = w_mem_wr_en;

	assign w_mem_in = i_ldst_rddata;
    assign o_ldst_wrdata = w_mem_out;
    assign o_ldst_addr   = w_mem_addr;

	datapath dp (
		.clk         (clk),
		.reset       (reset),
		.pc_test	 (w_pc_test),
		.ir_test	 (w_ir_test),
		.rs2_l5 	 (w_rs2_cast_l5),
		.imm_l5      (w_imm_cast_l5),
		.imm_sll_12  (w_imm_sll_12),
		.mem_wr_en   (w_mem_wr_en),
		.mem_rd_en   (w_mem_rd_en),
		.rf_wr_en    (w_rf_wr_en),
		.ir_wr_en    (w_ir_wr_en),
		.ir_bypass_en(w_ir_bypass_en),
		.ra_bypass_en(w_ra_bypass_en),
		.rb_bypass_en(w_rb_bypass_en),
		.rz_bypass_en(w_rz_bypass_en),
		.ry_bypass_en(w_ry_bypass_en),
		.rm_bypass_en(w_rm_bypass_en),
		.pc_wr_en    (w_pc_wr_en),
		.alu_flag_en (w_alu_flag_en),
		.alu_flag_bp (w_alu_flag_bp),
		.sel_mux_a   (w_sel_mux_a),
		.sel_mux_b   (w_sel_mux_b), 
		.sel_mux_c   (w_sel_mux_c), 
		.sel_mux_pc  (w_sel_mux_pc),
		.sel_mux_inc (w_sel_mux_inc),
		.sel_mux_ma  (w_sel_mux_ma),
		.sel_mux_y   (w_sel_mux_y),
		.sel_alu     (w_sel_alu),
		.pc_rd_in	 (w_pc_in),	
		.mem_in      (w_mem_in),
		.mem_out     (w_mem_out),
		.ldst_byte   (w_ldst_byte),
		.ldst_sign   (w_ldst_sign),
		.pc_rd_addr	 (w_pc_addr),
		.output_port (o_tb_regs), // output port.
		.mem_addr    (w_mem_addr),
		.flag_alu    (w_flag_alu),
		.opcode		 (w_opcode),
		.funct3		 (w_funct3),
		.funct7		 (w_funct7),
		.rs1		 (w_rs1),
		.rs2		 (w_rs2),
		.imval_test	 (w_imval_test),
		.write_test  (w_write_test),
		.w_reg_a_test(w_reg_a_test),
		.w_reg_b_test(w_reg_b_test),
		.w_reg_z_test(w_reg_z_test),
		.w_reg_y_test(w_reg_y_test),
		.w_reg_m_test(w_reg_m_test)

	);

	ctrlpath cp (
		.clk         (clk),
		.reset       (reset),
		.curstate	 (state),
		.flag_alu    (w_flag_alu),
		.opcode      (w_opcode),
		.funct3      (w_funct3),
		.funct7      (w_funct7),
		.rs1         (w_rs1),
		.rs2         (w_rs2),
		.rs2_cast_l5 (w_rs2_cast_l5),
		.imm_cast_l5 (w_imm_cast_l5),
		.imm_sll_12  (w_imm_sll_12),
		.mem_wr_en   (w_mem_wr_en),
		.mem_rd_en   (w_mem_rd_en),
		.rf_wr_en    (w_rf_wr_en),
		.ir_wr_en    (w_ir_wr_en),
		.ir_bypass_en(w_ir_bypass_en),
		.ra_bypass_en(w_ra_bypass_en),
		.rb_bypass_en(w_rb_bypass_en),
		.rz_bypass_en(w_rz_bypass_en),
		.ry_bypass_en(w_ry_bypass_en),
		.rm_bypass_en(w_rm_bypass_en),
		.pc_wr_en    (w_pc_wr_en),
		.pc_rd_en 	 (w_pc_rd_en),
		.pc_rd_byte  (o_pc_byte_en),
		.ldst_byte   (w_ldst_byte),
		.ldst_sign   (w_ldst_sign),
		.alu_flag_en (w_alu_flag_en),
		.alu_flag_bp (w_alu_flag_bp),
		.sel_mux_a   (w_sel_mux_a),
		.sel_mux_b   (w_sel_mux_b),
		.sel_mux_c   (w_sel_mux_c),
		.sel_mux_pc  (w_sel_mux_pc),
		.sel_mux_inc (w_sel_mux_inc),
		.sel_mux_ma  (w_sel_mux_ma),
		.sel_mux_y   (w_sel_mux_y),
		.sel_alu     (w_sel_alu)
	);


endmodule : cpu

module datapath (
	input 		  clk,
	input 		  reset,
	output [31:0] pc_test,
	output [31:0] ir_test,
	input		  rs2_l5,
	input		  imm_l5,
	input		  imm_sll_12,
	input  		  mem_wr_en,
	input  		  mem_rd_en,
	input		  rf_wr_en,
	input		  ir_wr_en,
	input		  ir_bypass_en,
	input		  ra_bypass_en,
	input		  rb_bypass_en,
	input		  rz_bypass_en,
	input		  ry_bypass_en,
	input		  rm_bypass_en,
	input		  pc_wr_en,
	input		  alu_flag_en,
	input		  alu_flag_bp,
	input		  sel_mux_a,
	input  [1:0]  sel_mux_b,
	input  [1:0]  sel_mux_c,  
	input  		  sel_mux_pc,
	input  		  sel_mux_inc,
	input  		  sel_mux_ma,
	input  [1:0]  sel_mux_y,
	input  [3:0]  sel_alu,
	input  [31:0] pc_rd_in,
	input		  ldst_sign,
	input  [3:0]  ldst_byte,
	input  [31:0] mem_in,
	output [31:0] mem_out,
	output [31:0] pc_rd_addr,
	output [31:0] output_port [0:31],
	output [32:0] mem_addr,
	output [5:0]  flag_alu,
	output [6:0]  opcode,
	output [3:0]  funct3,
	output [6:0]  funct7,
	output [4:0]  rs1,
	output [4:0]  rs2,
	output [31:0] imval_test,
	output [31:0] write_test,

	output [31:0] w_reg_a_test,
	output [31:0] w_reg_b_test,
	output [31:0] w_reg_z_test,
	output [31:0] w_reg_y_test,
	output [31:0] w_reg_m_test
);
	/* IR related wires */
	wire   [4:0]  w_rs1;
	wire   [4:0]  w_rs2;
	wire   [4:0]  w_rd;
	logic  [31:0] w_imval_casted;
	wire   [31:0] w_imval;
	logic  [31:0] w_imval_final;
	wire   [3:0]  w_funct3;
	wire   [6:0]  w_funct7;
	wire   [6:0]  w_opcode;
	/* Registers */
	reg    [31:0] reg_a;
	reg    [31:0] reg_b;
	reg    [31:0] reg_z;
	reg    [31:0] reg_y;
	reg    [31:0] reg_m;
	/* Interconnection wires */
	// wire in
	wire   [31:0] wi_reg_a;
	wire   [31:0] wi_reg_b;
	wire   [31:0] wi_reg_y;
	wire   [31:0] wi_reg_z;
	wire   [31:0] wi_reg_m;
	// wire out
	wire   [31:0] wo_reg_a;
	wire   [31:0] wo_reg_b;
	wire   [31:0] wo_reg_y;
	wire   [31:0] wo_reg_z;
	wire   [31:0] wo_reg_m;
	wire   [31:0] w_pc;
	wire   [31:0] w_pc_temp;
	wire   [31:0] wo_mux_a;
	wire   [31:0] wo_mux_b;
	wire   [31:0] wo_mux_c;
	// ldst_cast_wire
	logic   [31:0] w_ld_cast;
	logic   [31:0] w_st_cast;
	// rs2 cast
	logic	[31:0] wi_reg_b_final;

	/* Cast imval */
	assign w_imval_casted[4:0]	= w_imval[4:0];
	assign w_imval_casted[31:5] = {27{1'b0}};
	assign write_test = wo_reg_y;

	always @ (*) begin
		if (imm_l5) w_imval_final <= w_imval_casted;
		else if (imm_sll_12) w_imval_final <= (w_imval << 0);
		else w_imval_final <= w_imval;

		if (rs2_l5) begin
			wi_reg_b_final[4:0]  <= wi_reg_b[4:0];
			wi_reg_b_final[31:5] <= {27{1'b0}};
		end
		else wi_reg_b_final <= wi_reg_b;
	end

	assign w_reg_a_test = wo_reg_a;
	assign w_reg_b_test = wo_reg_b;
	assign w_reg_z_test = wo_reg_z;
	assign w_reg_y_test = wo_reg_y;
	assign w_reg_m_test = wo_reg_m;

	/* Assign Debug test signals */
	assign pc_test = w_pc;
	assign imval_test = w_imval;
	/* Register Bypassing Switches */
	assign wo_reg_a = (ra_bypass_en) ? wi_reg_a : reg_a;
	assign wo_reg_b = (rb_bypass_en) ? wi_reg_b_final : reg_b;
	assign wo_reg_z = (rz_bypass_en) ? wi_reg_z : reg_z;
	assign wo_reg_y = (ry_bypass_en) ? wi_reg_y : reg_y;
	assign wo_reg_m = (rm_bypass_en) ? wi_reg_m : reg_m;

	assign opcode   =  w_opcode;
	assign funct3   =  w_funct3;
	assign funct7   =  w_funct7;
	assign rs1  	=  w_rs1;
	assign rs2  	=  w_rs2;

	always @ (*) begin
		case(ldst_byte)
			4'b1111: begin
				w_ld_cast[31:0] <= mem_in[31:0];

				w_st_cast[31:0] <= wo_reg_b[31:0];
			end
			4'b0011: begin
				w_ld_cast[15:0] <= mem_in[15:0];
				w_ld_cast[31:16]<= ldst_sign ? {16{1'b0}} : {16{w_ld_cast[15]}};

				w_st_cast[15:0] <= wo_reg_b[15:0];
				w_st_cast[31:16]<= {16{w_st_cast[15]}};
			end
			4'b0001: begin
				w_ld_cast[7:0]  <= mem_in[7:0];
				w_ld_cast[31:8] <= ldst_sign ? {24{1'b0}} : {24{w_ld_cast[7]}};

				w_st_cast[7:0] <= wo_reg_b[7:0];
				w_st_cast[31:8]<= {24{w_st_cast[7]}};
			end
		endcase

	end

	assign wi_reg_m = w_st_cast;
	assign mem_out  = wo_reg_m;

	/* MUX Y */
	assign wi_reg_y  = sel_mux_y[1] ? (w_pc_temp) : (sel_mux_y[0] ? w_ld_cast : wo_reg_z);

	/* MUX MA*/
	assign pc_rd_addr= sel_mux_ma ? wo_reg_z : w_pc;

	/* MUX A */
	assign wo_mux_a  = sel_mux_a ? w_pc : wo_reg_a;

	/* MUX B */
	assign wo_mux_b  = sel_mux_b[1] ? 
					  (sel_mux_b[0] ? w_rs2 : w_rs2) : (sel_mux_b[0] ? w_imval_final : wo_reg_b);

	/* MUX C */
	assign wo_mux_c  = sel_mux_c[1] ? (32'b0) : (sel_mux_c[0] ? w_imval_final : wo_reg_y);

	/* Mem addr */
	assign mem_addr = (mem_wr_en || mem_rd_en) ? wo_reg_z : 32'b0;


	/* Register taking input */
	always @ (posedge clk) begin
		if (reset) begin
			reg_a <= 32'b0;
			reg_b <= 32'b0;
			reg_z <= 32'b0;
			reg_y <= 32'b0;
			reg_m <= 32'b0;
		end
		else begin
			reg_a <= wi_reg_a;
			reg_b <= wi_reg_b_final;
			reg_z <= wi_reg_y;
			reg_y <= wi_reg_z;
			reg_m <= wi_reg_m;
		end
	end

	pc program_counter (
		.clk        (clk),
		.reset      (reset),
		.pc_sel     (sel_mux_pc),
		.inc_sel    (sel_mux_inc),
		.pc_en      (pc_wr_en),
		.in_ra      (wi_reg_a),
		.imval      (w_imval),
		.out_pc     (w_pc),
		.out_pc_temp(w_pc_temp)
	);

	ir instr_register (
		.clk   		(clk),
		.reset 		(reset),
		.ir_en 		(ir_wr_en),
		.ir_bp_en   (ir_bypass_en),
		.in_ir 		(pc_rd_in),
		.ir_test    (ir_test),
		.imval 		(w_imval),
		.rs1   		(w_rs1),
		.rs2   		(w_rs2),
		.rd    		(w_rd),
		.funct3		(w_funct3),
		.funct7		(w_funct7),
		.opcode		(w_opcode)
	);

	alu_32b ALU (
		.clk    	(clk),
		.sel 		(sel_alu),
		.flag_en	(alu_flag_en),
		.alu_flag_bp(alu_flag_bp),
		.rs1 		(wo_mux_a),
		.rs2 		(wo_mux_b),
		.out 		(wi_reg_z),
		.flag		(flag_alu)
	);

	register_file_32x32 RF (
		.clk		(clk),
		.reset  	(reset),
		.wr_en  	(rf_wr_en),
		.addr_a 	(w_rs1),
		.addr_b 	(w_rs2),
		.addr_c 	(w_rd),
		.rd_a   	(wi_reg_a),
		.rd_b   	(wi_reg_b),
		.wr_c		(wo_mux_c),
		.copy_all	(output_port)
	);

endmodule : datapath

module ctrlpath (
	input 		  	   clk,
	input 		  	   reset,
	input  [2:0]  	   curstate,
	input  [5:0]  	   flag_alu,
	input  [6:0]  	   opcode,
	input  [3:0]  	   funct3,
	input  [6:0]  	   funct7,
	input  [4:0]  	   rs1,
	input  [4:0]  	   rs2,
	output logic	   rs2_cast_l5,
	output logic	   imm_cast_l5,
	output logic	   imm_sll_12,
	output logic  	   mem_wr_en,
	output logic  	   mem_rd_en,
	output logic 	   rf_wr_en,
	output logic 	   ir_wr_en,
	output logic 	   ir_bypass_en,
	output logic 	   ra_bypass_en,
	output logic 	   rb_bypass_en,
	output logic 	   rz_bypass_en,
	output logic 	   ry_bypass_en,
	output logic 	   rm_bypass_en,
	output logic 	   pc_wr_en,
	output logic 	   pc_rd_en,
	output logic [3:0] pc_rd_byte,
	output logic [3:0] ldst_byte,
	output logic       ldst_sign,
	output logic 	   alu_flag_en,
	output logic	   alu_flag_bp,
	output logic 	   sel_mux_a,
	output logic [1:0] sel_mux_b,
	output logic [1:0] sel_mux_c,
	output logic  	   sel_mux_pc,
	output logic  	   sel_mux_inc,
	output logic  	   sel_mux_ma,
	output logic [1:0] sel_mux_y,
	output logic [3:0] sel_alu
);

enum int unsigned
{
	S_0,
	S_1,
	S_2,
	S_3,
	S_4
} state, nextstate;

assign curstate = state;


// Clocked always block for making state registers
always_ff @ (posedge clk or posedge reset) begin
	if (reset) begin
		state <= S_0;
	end

	else state <= nextstate;
end

always_comb begin
	imm_sll_12  = 0;
	imm_cast_l5 = 0;
	rs2_cast_l5 = 0;
	mem_wr_en   = 0;
	mem_rd_en   = 0;
	rf_wr_en  	= 0;
	ir_wr_en  	= 0;
	ir_bypass_en= 0;
	ra_bypass_en= 0;
	rb_bypass_en= 0;
	rz_bypass_en= 0;
	ry_bypass_en= 0;
	rm_bypass_en= 0;
	pc_wr_en 	= 0;
	pc_rd_en    = 0;
	pc_rd_byte	= 0;
	ldst_byte   = 0;
	ldst_sign   = 0;
	alu_flag_en = 0;
	alu_flag_bp = 0;
	sel_mux_a   = 0;
	sel_mux_b 	= 0;
	sel_mux_c   = 0;
	sel_mux_pc 	= 0;
	sel_mux_inc = 0;
	sel_mux_ma 	= 0;
	sel_mux_y 	= 0;
	sel_alu 	= 0;

	if (state == S_0) begin // Stage 1: Tell the memory to give instructions @PC.
		ir_bypass_en = 1;
		pc_rd_en     = 1;		// Request IR output @ addr provided by pc.
		pc_rd_byte   = 4'b1111; // Full word access.
		sel_mux_ma   = 0; 	    // set mem addr to pc
		sel_mux_pc   = 0;
		nextstate    = S_1;
		ry_bypass_en = 1;
	end
	else begin
		case (opcode)
			7'b0110011, 7'b0010011: /*R, I (Arithmetic)*/ begin
				case(state)
					S_1: begin
						// We want to write to the ir and the register file.
						rf_wr_en     = 1;
						ir_wr_en     = 1;
						// We want to bypass all the intermediate registers.
						ir_bypass_en = 1;
						ra_bypass_en = 1;
						rb_bypass_en = 1;
						rz_bypass_en = 1;
						ry_bypass_en = 1;

						/* MUX B control logic */
						if (opcode == 7'b0110011) begin // R instr
							if (funct3 == 1 || funct3 == 5) rs2_cast_l5 = 1;
							sel_mux_b 	= 0; // add, sub, xor, or, and, slt, sltu
						end
						else begin // I instr
							if (funct3 == 1 || funct3 == 5) imm_cast_l5 = 1; // use only [4:0] of the imval
							sel_mux_b 	= 1; // sll, srl, sra
						end
						/* ALU   control logic */
						/* add, sub, xor, or, and */ /* addi, xori, ori, andi */
						if      (opcode == 7'b0110011 && funct3 == 0 && funct7 == 7'h00) sel_alu = 0; // add
						else if (opcode == 7'b0110011 && funct3 == 0 && funct7 == 7'h20) sel_alu = 1; // sub
						else if (opcode != 7'b0110011 && funct3 == 0) sel_alu = 0; // addi
						else if (funct3 == 4) sel_alu = 2; // xor, xori
						else if (funct3 == 6) sel_alu = 3; // or, ori
						else if (funct3 == 7) sel_alu = 4; // and, andi
						/* sll,  srl,  sra */ /* slli, srli, srai */
						else if (funct3 == 1) sel_alu = 5; // sll, slli
						else if (funct3 == 5 && funct7 == 7'h00) sel_alu = 6; // srl, srli
						else if (funct3 == 5 && funct7 == 7'h20) sel_alu = 7; // sra, srai
						/* slt,  sltu */ /* slti, sltui */
						else if (funct3 == 2) sel_alu = 8; // slt, slti
						else if (funct3 == 3) sel_alu = 9; // slti, sltui

						// increment the pc
						pc_wr_en    = 1; // allow the write to pc.
						sel_mux_inc = 0; // select the increment # to be 4.
						sel_mux_pc  = 1; // select the pc input to the feedback loop.
						nextstate   = S_0;
					end
				endcase // state
			end
			7'b0000011: /* I (Load)*/ begin
				case(state)
					S_1: begin // Give the address to the memory
						// Write to the instr register.
						ir_wr_en     = 1;
						// We want to bypass all the intermediate registers.
						ir_bypass_en = 1;
						ra_bypass_en = 1;
						rb_bypass_en = 1;
						rz_bypass_en = 1;
						ry_bypass_en = 1;
						rm_bypass_en = 1;
	
						/* MUX B control logic */
						sel_mux_b 	 = 1; // imval
						/* ALU   control logic */
						sel_alu      = 0; // add
						/* MUX MA control logic */
						sel_mux_ma   = 0; // Mux MA output regz value.
						/* Memory enable read */
						mem_rd_en    = 1;

						sel_mux_pc   = 0;
	
						case(funct3)
							0:	ldst_byte = 4'b0001; // unsigned load byte
							1:	ldst_byte = 4'b0011; // unsigned load half word
							2:	ldst_byte = 4'b1111; // unsigned load word
							4:	begin
								ldst_byte = 4'b0001;
								ldst_sign = 1; // signed load byte
							end
							5:	begin
								ldst_byte = 4'b0011;
								ldst_sign = 1; // signed load half word
							end
						endcase // funct3
						nextstate = S_2;
					end

					S_2: begin // Save the memory input to rd
						/* MUX Y control logic */
						sel_mux_y    = 1;
						// Bypass ry to save cycles
						ry_bypass_en = 1;
						// Write to the register file.
						rf_wr_en     = 1;

						case(funct3)
							0:	ldst_byte = 4'b0001; // unsigned load byte
							1:	ldst_byte = 4'b0011; // unsigned load half word
							2:	ldst_byte = 4'b1111; // unsigned load word
							4:	begin
								ldst_byte = 4'b0001;
								ldst_sign = 1; // signed load byte
							end
							5:	begin
								ldst_byte = 4'b0011;
								ldst_sign = 1; // signed load half word
							end
						endcase // funct3

						// increment the pc
						pc_wr_en    = 1; // allow the write to pc.
						sel_mux_inc = 0; // select the increment # to be 4.
						sel_mux_pc  = 1; // select the pc input to the feedback loop.
						nextstate   = S_0;
					end
				endcase // state
			end

			7'b0100011: 		   /*S*/ begin
				case(state)
					S_1: begin // Fetch and give data to the memory @ addr.
						// Write to the instr register.
						ir_wr_en     = 1;
						// We want to bypass all the intermediate registers.
						ir_bypass_en = 1;
						ra_bypass_en = 1;
						rb_bypass_en = 1;
						rz_bypass_en = 1;
						ry_bypass_en = 1;
						rm_bypass_en = 1;
	
						/* MUX B control logic */
						sel_mux_b 	 = 1; // imval
						/* ALU   control logic */
						sel_alu      = 0; // addi
						/* MUX MA control logic */
						sel_mux_ma   = 0; // Mux MA output regz value.
						/* Memory enable write */
						mem_wr_en    = 1;
	
						case(funct3)
							0:	ldst_byte = 4'b0001; // unsigned load byte
							1:	ldst_byte = 4'b0011; // unsigned load half word
							2:	ldst_byte = 4'b1111; // unsigned load word
						endcase // funct3
	
						// increment the pc
						pc_wr_en    = 1; // allow the write to pc.
						sel_mux_inc = 0; // select the increment # to be 4.
						sel_mux_pc  = 1; // select the pc input to the feedback loop.
						nextstate   = S_0;
					end
				endcase // funct3
			end
			7'b1100011: 		   /*B*/ begin
				case(state)
					S_1: begin // load imm<<12 to rd
						// Bypass and Write to the instr register.
						ir_wr_en     = 1;
						ir_bypass_en = 1;
						// Enable and bypass the flag register.
						alu_flag_bp = 1;
						alu_flag_en = 1;
						// Bypass register A and B.
						ra_bypass_en = 1;
						rb_bypass_en = 1;

						// Now we have the ALU flag signal.
						case(funct3)
							0: sel_mux_inc = flag_alu[0];
							1: sel_mux_inc = flag_alu[1];
							4: sel_mux_inc = flag_alu[2];
							5: sel_mux_inc = flag_alu[3];
							6: sel_mux_inc = flag_alu[4];
							7: sel_mux_inc = flag_alu[5];
						endcase // funct3

						// increment the pc
						pc_wr_en    = 1; // allow the write to pc.
						sel_mux_pc  = 1; // select the pc input to the feedback loop.
						nextstate   = S_0;
					end
				endcase // state
			end
			7'b0110111:        /* lui */ begin
				case(state)
					S_1: begin // load imm<<12 to rd
						/* Write and bypass the instr register. */
						ir_wr_en     = 1;
						ir_bypass_en = 1;
						/* Write to the register file. */
						rf_wr_en     = 1;
						/* MUX C control logic*/
						sel_mux_c    = 1;
						// increment the pc
						pc_wr_en    = 1; // allow the write to pc.
						sel_mux_inc = 0; // select the increment # to be 4.
						sel_mux_pc  = 1; // select the pc input to the feedback loop.
						nextstate   = S_0;
					end
				endcase // state
			end
			7'b0010111:        /*auipc*/ begin
				case(state)
					S_1: begin // branch.
						/* Write and bypass the instr register. */
						ir_wr_en     = 1;
						ir_bypass_en = 1;
						/* Write to the register file. */
						rf_wr_en     = 1;
						/* MUX A control logic*/
						sel_mux_a    = 1; // PC
						/* MUX B control logic*/
						sel_mux_b    = 1; // upper imval
						/* MUX C control logic*/
						sel_mux_c    = 0;
						/* MUX Y control logic*/
						sel_mux_y    = 0;
						/* Bypassing a, b, y, z */
						ra_bypass_en = 1;
						rb_bypass_en = 1;
						rz_bypass_en = 1;
						ry_bypass_en = 1;

						// increment the pc
						pc_wr_en    = 1; // allow the write to pc.
						sel_mux_inc = 0; // select the increment # to be 4.
						sel_mux_pc  = 1; // select the pc input to the feedback loop.
						nextstate   = S_0;
					end
				endcase // state
			end
			7'b1101111: /*Jal*/ begin
				case(state)
					S_1: begin // jump and link
						/* Write and bypass the instr register. */
						ir_wr_en     = 1;
						ir_bypass_en = 1;
						
						sel_mux_pc   = (opcode == 7'b1101111) ? 1 : 0;
						sel_mux_inc  = 1; // imval
						sel_mux_y    = 2;
						ry_bypass_en = 1;
						pc_wr_en    = 1; // allow the write to pc.
						/* Write to the register file. */
						rf_wr_en     = 1;
						nextstate   = S_0;
					end
				endcase // state
			end
			7'b1100111: /*Jalr*/ begin
				case(state)
					S_1: begin // jump and link reg
						/* Write and bypass the instr register. */
						ir_wr_en     = 1;
						ir_bypass_en = 1;
						/* Write the RA to PC */
						pc_wr_en    = 1; // allow the write to pc.
						sel_mux_pc  = 0;
						sel_mux_inc  = 1; // imval
						sel_mux_y    = 2; // pc temp
						ry_bypass_en = 1;
						/* Write PC + 4 (PC_temp) to the register file. */
						rf_wr_en     = 1;
						nextstate   = S_2;
					end
					S_2: begin
						/* Write and bypass the instr register. */
						ir_wr_en     = 1;
						ir_bypass_en = 1;
						/* Write PC += imm */
						sel_mux_pc   = 1; // feedback
						sel_mux_inc  = 1; // imval
						pc_wr_en     = 1; // allow the write to pc.
						nextstate    = S_0;
					end
				endcase // state
			end
			default: begin // bootstrap
				ir_bypass_en = 1;
				pc_rd_en     = 1;		// Request IR output @ addr provided by pc.
				pc_rd_byte   = 4'b1111; // Full word access.
				sel_mux_ma   = 0; 	    // set mem addr to pc
				nextstate    = S_1;
			end
		endcase
	end
end



endmodule : ctrlpath


// pc_sel : {1: adder feedback;  0: in_ra}
// inc_sel: {1: imval;  0: 4}
module pc (
	input			  clk,
	input			  reset,
	input             pc_sel,
	input			  inc_sel,
	input			  pc_en,
	input      [31:0] in_ra,
	input      [31:0] imval,
	output reg [31:0] out_pc,
	output reg [31:0] out_pc_temp
);
	wire [31:0] addero_muxpc;
	wire [31:0] adderi_muxinc;

	assign addero_muxpc  = out_pc  + adderi_muxinc;
	assign adderi_muxinc = inc_sel ? imval : 32'd4;

	always @ (posedge clk) begin
		if (reset) begin
			out_pc <= 0;
		end
		else if (pc_en) begin
			out_pc <= (pc_sel ? addero_muxpc : in_ra);
		end
		out_pc_temp <= (out_pc + 4);
	end
endmodule : pc

module ir (
	input				clk,
	input				reset,
	input				ir_en,
	input				ir_bp_en,
	input        [31:0] in_ir,
	output       [31:0] ir_test,
	output logic [31:0] imval,
	output logic [4:0]  rs1,
	output logic [4:0]  rs2,
	output logic [4:0]  rd,
	output logic [3:0]  funct3,
	output logic [6:0]  funct7,
	output logic [6:0]  opcode
);
	reg  [31:0] inst_reg;
	wire [31:0] w_inst_reg;

	assign w_inst_reg = (ir_bp_en) ? in_ir : inst_reg;
	assign ir_test = inst_reg;

	always @ (posedge clk) begin
		if (reset) begin
			inst_reg <= 32'b0;
		end
		else if (ir_en) begin
			inst_reg <= in_ir;
		end
	end

	always_comb begin
		case (w_inst_reg[6:0])
			7'b0110011: 		   /*R*/ begin
				opcode 		= w_inst_reg[6:0];
				rs1			= w_inst_reg[19:15];
				rs2			= w_inst_reg[24:20];
				rd 			= w_inst_reg[11:7];
				funct3 		= w_inst_reg[14:12];
				funct7 		= w_inst_reg[31:25];
			end

			7'b0010011, 7'b0000011, 7'b1100111: /*I*/ begin
				opcode 		= w_inst_reg[6:0];
				rs1	   		= w_inst_reg[19:15];
				rd 	   		= w_inst_reg[11:7];
				funct3 		= w_inst_reg[14:12];
				funct7 		= w_inst_reg[31:25];
				imval [11:0]= w_inst_reg[31:20];
				// imval msb extended
				imval[31:12]= {20{imval[11]}};
			end

			7'b0100011: 		   /*S*/ begin
				opcode      = w_inst_reg[6:0];
				rs1    		= w_inst_reg[19:15];
				rs2    		= w_inst_reg[24:20];
				imval[4:0]  = w_inst_reg[11:7];
				funct3      = w_inst_reg[14:12];
				imval[11:5] = w_inst_reg[31:25];
				// imval msb extended
				imval[31:12]= {20{imval[11]}};
			end

			7'b1100011: 		   /*B*/ begin
				opcode      = w_inst_reg[6:0];
				rs1   		= w_inst_reg[19:15];
				rs2   		= w_inst_reg[24:20];
				funct3      = w_inst_reg[14:12];
				{imval[4:1] , imval[11]  } = w_inst_reg[11:7];
				{imval[12]  , imval[10:5]} = w_inst_reg[31:25];
				imval[0]    = 1'b0;
				// imval msb extended
				imval[31:13]= {19{imval[11]}};
			end

			7'b0110111, 7'b0010111: 		   /*U*/ begin
				opcode		= w_inst_reg[6:0];
				rd			= w_inst_reg[11:7];
				imval[31:12]= w_inst_reg[31:12];
				// use zero to fill the remaining part.
				imval[11:0] = {12{1'b0}};
			end

			7'b1101111: 		   /*J*/ begin
				opcode 		= w_inst_reg[6:0];
				rd 			= w_inst_reg[11:7];
				{imval[20], imval[10:1], imval[11], imval[19:12]} = w_inst_reg[31:12];
				// use zero to fill the remaining part.
				imval[0]    = 1'b0;
				// imval msb extended
				imval[31:21]= {11{imval[20]}};
			end
		endcase // in_ir[6:0]
	end
endmodule : ir

module alu_32b (
	input				clk,
	input  		 [3:0]  sel,  // {add, sub, xor, or, and, sll, srl, sra, slt, sltu}
	input				flag_en,
	input 				alu_flag_bp,
	input  		 [31:0] rs1,
	input  		 [31:0] rs2,
	output logic [31:0] out,
	output       [5:0]  flag // {EQ, NE, LT, GE, LTU, GEU}
);
	logic [5:0] w_flag;
	reg   [5:0] reg_flag;

	assign flag = alu_flag_bp ?  w_flag : reg_flag;

	parameter        ADD  =  4'd0,   SLL  =  4'd5,
			         SUB  =  4'd1,   SRL  =  4'd6,
			         XOR  =  4'd2,   SRA  =  4'd7,
			         OR   =  4'd3,   SLT  =  4'd8,
			         AND  =  4'd4,   SLTU =  4'd9;
			             
	parameter        EQ   =  3'd0,   GE   =  3'd3,
			         NE   =  3'd1,   LTU  =  3'd4,
			         LT   =  3'd2,   GEU  =  3'd5;

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
		endcase // sel
	end

	always_comb begin
		w_flag[EQ]  = (rs1 ==  rs2) ?  1 : 0;
		w_flag[NE]  = (rs1 !=  rs2) ?  1 : 0;
		w_flag[LT]  = ($signed(rs1) <   $signed(rs2)) ?  1 : 0;
		w_flag[GE]  = ($signed(rs1) <   $signed(rs2)) ?  0 : 1;
		w_flag[LTU] = ($unsigned(rs1) <  $unsigned(rs2)) ? 1 : 0;
		w_flag[GEU] = ($unsigned(rs1) <  $unsigned(rs2)) ? 0 : 1;
	end

	always @ (posedge clk) begin
		if (flag_en) begin
			reg_flag[EQ]  <= (rs1 ==  rs2) ?  1 : 0;
			reg_flag[NE]  <= (rs1 !=  rs2) ?  1 : 0;
			reg_flag[LT]  <= ($signed(rs1) <   $signed(rs2)) ?  1 : 0;
			reg_flag[GE]  <= ($signed(rs1) <   $signed(rs2)) ?  0 : 1;
			reg_flag[LTU] <= ($unsigned(rs1) <  $unsigned(rs2)) ? 1 : 0;
			reg_flag[GEU] <= ($unsigned(rs1) <  $unsigned(rs2)) ? 0 : 1;
		end
	end


endmodule : alu_32b

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
	always @ (posedge clk) begin
		if (reset) begin
			for (i = 0; i < 32; i=i+1) begin
				reg_file[i] <= 32'b0;
			end
		end
		else if (wr_en) begin
			if (addr_c != 0) reg_file[addr_c] <= wr_c;
		end
	end
endmodule : register_file_32x32
