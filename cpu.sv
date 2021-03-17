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
	wire [1:0]  w_sel_mux_b;   
	wire 		w_sel_mux_pc; 
	wire 		w_sel_mux_inc; 
	wire 		w_sel_mux_ma;  
	wire [1:0]  w_sel_mux_y;   
	wire 		w_sel_alu;     
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

	/* Output Assignments */
	assign o_pc_rd = w_pc_rd_en;
	assign o_pc_addr = w_pc_addr;
	assign w_pc_in = i_pc_rddata;

	datapath dp (
		.clk         (clk),
		.reset       (reset),
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
		.sel_mux_b   (w_sel_mux_b),  
		.sel_mux_pc  (w_sel_mux_pc),
		.sel_mux_inc (w_sel_mux_inc),
		.sel_mux_ma  (w_sel_mux_ma),
		.sel_mux_y   (w_sel_mux_y),
		.sel_alu     (w_sel_alu),
		.pc_rd_in	 (w_pc_in),	
		.mem_in      (w_mem_in),
		.mem_out     (w_mem_out),
		.pc_rd_addr	 (w_pc_addr),
		.output_port (o_tb_regs), // output port.
		.mem_addr    (w_mem_addr),
		.flag_alu    (w_flag_alu),
		.opcode		 (w_opcode),
		.funct3		 (w_funct3),
		.funct7		 (w_funct7),
		.rs1		 (w_rs1),
		.rs2		 (w_rs2)
	);

	ctrlpath cp (
		.clk         (clk),
		.reset       (reset),
		.flag_alu    (w_flag_alu),
		.opcode      (w_opcode),
		.funct3      (w_funct3),
		.funct7      (w_funct7),
		.rs1         (w_rs1),
		.rs2         (w_rs2),
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
		.pc_rd_byte  (),
		.alu_flag_en (w_alu_flag_en),
		.sel_mux_b   (w_sel_mux_b),
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
	input  [1:0]  sel_mux_b,  
	input  		  sel_mux_pc,
	input  		  sel_mux_inc,
	input  		  sel_mux_ma,
	input  [1:0]  sel_mux_y,
	input  		  sel_alu,
	input  [31:0] pc_rd_in,
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
	output [4:0]  rs2
);
	/* IR related wires */
	wire   [4:0]  w_rs1;
	wire   [4:0]  w_rs2;
	wire   [4:0]  w_rd;
	wire   [31:0] w_imval;
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
	wire   [31:0] wo_mux_b;

	/* Register Bypassing Switches */
	assign wo_reg_a = (ra_bypass_en) ? wi_reg_a : reg_a;
	assign wo_reg_b = (rb_bypass_en) ? wi_reg_b : reg_b;
	assign wo_reg_y = (rz_bypass_en) ? wi_reg_y : reg_z;
	assign wo_reg_z = (ry_bypass_en) ? wi_reg_z : reg_y;
	assign wo_reg_m = (rm_bypass_en) ? wi_reg_m : reg_m;

	assign opcode   =  w_opcode;
	assign funct3   =  w_funct3;
	assign funct7   =  w_funct7;
	assign rs1  	=  w_rs1;
	assign rs2  	=  w_rs2;

	/* MUX Y */
	assign wi_reg_y = sel_mux_y[1] ? (w_pc_temp) : (sel_mux_y[0] ? mem_in : reg_z);

	/* MUX MA*/
	assign pc_rd_addr= sel_mux_ma ? reg_z : w_pc;

	/* MUX B */
	assign wo_mux_b = sel_mux_b[1] ? (sel_mux_b[0] ? w_rs2 : w_rs2) : (sel_mux_b[0] ? w_imval : wo_reg_b);

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
			reg_b <= wi_reg_b;
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
		.in_ra      (reg_a),
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
		.rs1 		(wo_reg_a),
		.rs2 		(w_rs2),
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
		.wr_c		(reg_y),
		.copy_all	(output_port)
	);

endmodule : datapath

module ctrlpath (
	input 		  clk,
	input 		  reset,
	input  [5:0]  flag_alu,
	input  [6:0]  opcode,
	input  [3:0]  funct3,
	input  [6:0]  funct7,
	input  [4:0]  rs1,
	input  [4:0]  rs2,
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
	output logic       pc_rd_byte,
	output logic 	   alu_flag_en,
	output logic [1:0] sel_mux_b,  
	output logic  	   sel_mux_pc,
	output logic  	   sel_mux_inc,
	output logic  	   sel_mux_ma,
	output logic [1:0] sel_mux_y,
	output logic  	   sel_alu
);

enum int unsigned
{
	S_0,
	S_1,
	S_2,
	S_3,
	S_4
} state, nextstate;

// Clocked always block for making state registers
always_ff @ (posedge clk or posedge reset) begin
	if (reset) state <= S_0;
	else state <= nextstate;
end

always_comb begin
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
	alu_flag_en = 0;
	sel_mux_b 	= 0;
	sel_mux_pc 	= 0;
	sel_mux_inc = 0;
	sel_mux_ma 	= 0;
	sel_mux_y 	= 0;
	sel_alu 	= 0;

	case (opcode)
		7'b0110011: /*R*/ begin
			case(state)
				S_0: begin // fetch the instructions
					pc_rd_en   = 1;		  // Request IR output @ addr provided by pc.
					pc_rd_byte = 4'b1111; // Full word access.
					sel_mux_ma = 0; 	  // set mem addr to pc
					nextstate  = S_1;
				end
				S_1: begin
					// We want to write to the register file.
					rf_wr_en  	= 1;
					// We want to bypass all the intermediate registers.
					ir_bypass_en= 1;
					ra_bypass_en= 1;
					rb_bypass_en= 1;
					rz_bypass_en= 1;
					ry_bypass_en= 1;

					/* MUX B control logic */
					if (funct3 == 1 || funct3 == 5) sel_mux_b 	= 2; // sll, srl, sra
					else sel_mux_b 	= 0; // add, sub, xor, or, and, slt, sltu

					/* ALU   control logic */
					/* add, sub, xor, or, and */
					if (funct3 == 0 && funct7 == 7'h00) sel_alu = 0;
					else if (funct3 == 0 && funct7 == 7'h20) sel_alu = 1;
					else if (funct3 == 4) sel_alu = 2;
					else if (funct3 == 6) sel_alu = 3;
					else if (funct3 == 7) sel_alu = 4;
					/* sll, srl, sra */
					else if (funct3 == 1) sel_alu = 5;
					else if (funct3 == 5 && funct7 == 7'h00) sel_alu = 6;
					else if (funct3 == 5 && funct7 == 7'h20) sel_alu = 7;
					/* slt, sltu */
					else if (funct3 == 2) sel_alu = 8;
					else if (funct3 == 3) sel_alu = 9;
					/* next state */
					nextstate  = S_2;
				end
				S_2: begin // increment the pc
					pc_wr_en    = 1; // allow the write to pc.
					sel_mux_inc = 0; // select the increment # to be 4.
					sel_mux_pc  = 1; // select the pc input to the feedback loop.
					nextstate   = S_0;
				end
			endcase
			end
		7'b00?0011, 7'b110011: /*I*/ begin
			//case (funct3)
			nextstate  = S_2;
			//endcase	
		end
		7'b0100011: 		   /*S*/ begin
			//case (funct3)
			nextstate  = S_2;
			//endcase
		end
		7'b1100011: 		   /*B*/ begin
			//case (funct3)
			nextstate  = S_2;
			//endcase
		end
		7'b0?10111: 		   /*U*/ begin
			//case (funct3)
			nextstate  = S_2;
			//endcase
		end
		7'b1101111: 		   /*J*/ begin
			//case (funct3)
			nextstate  = S_2;
			//endcase
		end
	endcase
end



endmodule : ctrlpath


// pc_sel : {1: in_ra;  0: adder feedback}
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

	assign addero_muxpc = out_pc + adderi_muxinc;
	assign adderi_muxinc = inc_sel ? 32'd4 : imval;

	always @ (posedge clk) begin
		if (reset) begin
			out_pc <= 0;
		end
		else if (pc_en) begin
			out_pc <= pc_sel ? in_ra : addero_muxpc;
		end
		out_pc_temp <= out_pc;
	end
endmodule : pc

module ir (
	input				clk,
	input				reset,
	input				ir_en,
	input				ir_bp_en,
	input        [31:0] in_ir,
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

	always @ (posedge clk) begin
		if (reset) begin
			inst_reg <= 32'b0;
		end
		else if (ir_en) begin
			inst_reg <= in_ir;
		end
	end

	always_comb begin
		casez (inst_reg[6:0])
			7'b0110011: 		   /*R*/ begin
				opcode 		= w_inst_reg[6:0];
				rs1			= w_inst_reg[19:15];
				rs2			= w_inst_reg[24:20];
				rd 			= w_inst_reg[11:7];
				funct3 		= w_inst_reg[14:12];
				funct7 		= w_inst_reg[31:25];
			end

			7'b00?0011, 7'b110011: /*I*/ begin
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

			7'b0?10111: 		   /*U*/ begin
				opcode		= w_inst_reg[6:0];
				rd			= w_inst_reg[11:7];
				imval[31:12]= w_inst_reg[31:12];
				// use zero to fill the remaining part.
				imval[11:0] = 12'b0;
			end

			7'b1101111: 		   /*J*/ begin
				opcode 		= w_inst_reg[6:0];
				rd 			= w_inst_reg[11:7];
				{imval[20], imval[10:1], imval[11], imval[19:12]} = w_inst_reg[31:12];
				// use zero to fill the remaining part.
				imval[0]    = 1'b0;
				// imval msb extended
				imval[31:21]= {11{imval[11]}};
			end
		endcase // in_ir[6:0]
	end
endmodule : ir

module alu_32b (
	input				clk,
	input  		 [3:0]  sel,  // {add, sub, xor, or, and, sll, srl, sra, slt, sltu}
	input				flag_en,
	input  		 [31:0] rs1,
	input  		 [31:0] rs2,
	output logic [31:0] out,
	output reg   [5:0]  flag // {EQ, NE, LT, GE, LTU, GEU}
);
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
			XOR   :  out  =  rs1 ^~  rs2;
			OR    :  out  =  rs1 |   rs2;
			AND   :  out  =  rs1 &   rs2;
			SLL   :  out  =  rs1 <<  rs2;
			SRL   :  out  =  rs1 >>  rs2;
			SRA   :  out  =  $signed(rs1) >>> rs2;
			SLT   :  out  = (rs1 >   rs2)   ?  1 : 0;
			SLTU  :  out  = ($unsigned(rs1) > $unsigned(rs2)) ?  1 : 0;
		endcase // sel
	end

	always @ (posedge clk) begin
		if (flag_en) begin
			flag[EQ]  <= (rs1 ==  rs2) ?  1 : 0;
			flag[NE]  <= (rs1 !=  rs2) ?  1 : 0;
			flag[LT]  <= (rs1 <   rs2) ?  1 : 0;
			flag[GE]  <= (rs1 <  rs2)  ?  0 : 1;
			flag[LTU] <= ($unsigned(rs1) <  $unsigned(rs2)) ? 1 : 0;
			flag[GEU] <= ($unsigned(rs1) <  $unsigned(rs2)) ? 0 : 1;
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
			reg_file[addr_c] <= wr_c;
		end
	end
endmodule : register_file_32x32
