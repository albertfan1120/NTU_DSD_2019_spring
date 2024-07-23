// For MIPS RTL debug
// Support: add, addi, sub, slt, nop, lw, sw, beq, bne, j, jal, jr, jalr
// New Support: mult, div, mfhi, mflo
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;

//=========================================
	// Note that the overall design of your MIPS includes:
	// 1. pipelined MIPS processor
	// 2. data cache
	// 3. instruction cache


	RISCV_Pipeline i_RISCV(
		// control interface
		clk, 
		rst_n,
		//----------I cache interface-------		
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_wdata,
		ICACHE_stall,
		ICACHE_rdata,
		//----------D cache interface-------
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata,
	);
	
	cache D_cache(
		clk,
		~rst_n,
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata,
		mem_read_D,
		mem_write_D,
		mem_addr_D,
		mem_rdata_D,
		mem_wdata_D,
		mem_ready_D
	);

	cache I_cache(
		clk,
		~rst_n,
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_wdata,
		ICACHE_stall,
		ICACHE_rdata,
		mem_read_I,
		mem_write_I,
		mem_addr_I,
		mem_rdata_I,
		mem_wdata_I,
		mem_ready_I
	);
endmodule

//	TODO:
//	generate for dff
//	lw stall =>PC & IR only 							(done)

// jal  store:PC+4	jumpto: PC+imm(in ID)
// jalr store:PC+4	jumpto: rs+imm(in ID)
// branch no store  jumpto: PC+imm(in ID)

`define ALU_OP_WIDTH 4

`define ALU_OP_ADD  `ALU_OP_WIDTH'd0
`define ALU_OP_SUB  `ALU_OP_WIDTH'd1
`define ALU_OP_XOR  `ALU_OP_WIDTH'd2
`define ALU_OP_OR   `ALU_OP_WIDTH'd3
`define ALU_OP_AND  `ALU_OP_WIDTH'd4
`define ALU_OP_SLL  `ALU_OP_WIDTH'd5
`define ALU_OP_SRL  `ALU_OP_WIDTH'd6
`define ALU_OP_SRA  `ALU_OP_WIDTH'd7
`define ALU_OP_SLT  `ALU_OP_WIDTH'd8
`define ALU_OP_SLTU `ALU_OP_WIDTH'd9

module RISCV_Pipeline (
		// control interface
		clk, 
		rst_n,
//----------I cache interface-------		
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_wdata,
		ICACHE_stall,
		ICACHE_rdata,
//----------D cache interface-------
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata
	);

//==== in/out declaration =================================
	input  clk, rst_n;

	input  [31:0] ICACHE_rdata;
	input  		  ICACHE_stall;
	output 		  ICACHE_ren, ICACHE_wen;
	output [29:0] ICACHE_addr;
	output [31:0] ICACHE_wdata;

	input  [31:0] DCACHE_rdata;
	input  		  DCACHE_stall;
	output 		  DCACHE_ren, DCACHE_wen;
	output [29:0] DCACHE_addr;
	output [31:0] DCACHE_wdata;
			
//==== reg/wire declaration ========================================
	wire   [31:0] IR_r; 
	wire   [31:0] IR_w;

	wire   [4:0]  rv32_rd    = IR_r[11:7];
	wire   [4:0]  rv32_rs1   = IR_r[19:15];
	wire   [4:0]  rv32_rs2   = IR_r[24:20];
	wire   [2:0]  rv32_func3 = IR_r[14:12];
	wire   [6:0]  rv32_func7 = IR_r[31:25];
	wire   [6:0]  rv32_op    = IR_r[6:0];
	wire          rv32_Rtype = rv32_op == 7'b0110011;
	wire          rv32_Itype = rv32_op == 7'b0010011;
	wire          rv32_Btype = rv32_op == 7'b1100011;
	wire          rv32_store = rv32_op == 7'b0100011;
	wire          rv32_load  = rv32_op == 7'b0000011;
	wire 		  rv32_jal   = rv32_op == 7'b1101111;
	wire 		  rv32_jalr  = rv32_op == 7'b1100111;
	wire 		  rv32_Jtype = rv32_jal|rv32_jalr;
//	wire 		  rv32_lui   = rv32_op == 7'b0110111;
//	wire 		  rv32_auipc = rv32_op == 7'b0010111;
//	wire 		  rv32_Utype = rv32_lui|rv32_auipc;

    wire          CACHE_stall = ICACHE_stall|DCACHE_stall;
   	wire          lw_stall    ;
   	wire 		  flush       ;
   	wire          Branch_jump ;

    reg    [18:0] ctrl_signal_w1;
	wire   [18:0] ctrl_signal_r1;
	reg    [7:0]  ctrl_signal_w2;
	wire   [7:0]  ctrl_signal_r2;
	reg    [5:0]  ctrl_signal_w3;
	wire   [5:0]  ctrl_signal_r3;


	dff #(.DW(19)) d1(.clk(clk), .rst_n(rst_n), .d(ctrl_signal_w1), .q(ctrl_signal_r1));
	dff #(.DW(8 )) d2(.clk(clk), .rst_n(rst_n), .d(ctrl_signal_w2), .q(ctrl_signal_r2));
	dff #(.DW(6 )) d3(.clk(clk), .rst_n(rst_n), .d(ctrl_signal_w3), .q(ctrl_signal_r3));
	
	wire   [31:0] rv32_i_imm = {
							{20{IR_r[31]}},
							IR_r[31:20]
							};
	wire   [31:0] rv32_j_imm = {
							{12{IR_r[31]}},
							IR_r[19:12],
							IR_r[20],
							IR_r[30:25],
							IR_r[24:21],
							1'b0
							};
	wire   [31:0] rv32_b_imm = {
							{20{IR_r[31]}},
							IR_r[7],
							IR_r[30:25],
							IR_r[11:8],
							1'b0
							};
	wire   [31:0] rv32_s_imm = {
							{21{IR_r[31]}},
							IR_r[30:25],
							IR_r[11:7]
							};
	wire   [31:0] rv32_u_imm = {
							IR_r[31:12],
							12'd0
							};					

	wire   [31:0] rv32_imm;
//==== IF part =====================================================
	wire   [31:0] PC_w;
	wire   [31:0] PC_r;
	wire   [31:0] PC_IF_w; 
	wire   [31:0] PC_IF_r;
	wire   [31:0] PC_ID_w; 
	wire   [31:0] PC_ID_r;
	wire   [31:0] PC_EX_w; 
	wire   [31:0] PC_EX_r;
	wire   [31:0] PC_MEM_w; 
	wire   [31:0] PC_MEM_r;

//  control signal
	wire 		  PCSel;
	wire  		  BrResult;
	wire 		  RegWen;
	wire   [1:0]  WBSel;
	wire   		  sel_imm;
	
	wire   [31:0] read_data1, read_data2, write_data;
	wire   [4:0]  read_register1, read_register2;
//  register file
	reg    [31:0] read_data1_w, read_data2_w;
	wire   [31:0] read_data1_r, read_data2_r;
	wire   [31:0] read_data2_store_w, read_data2_store_r;  //for sw
	reg    [31:0] read_data2f; //forward
	wire   [31:0] read_data2_EX_w, read_data2_EX_r; 

	wire   [31:0] write_data_WB_w, write_data_WB_r;
    
	wire   [ 4:0] write_register_ID_w, write_register_ID_r;
	wire   [ 4:0] write_register_EX_w, write_register_EX_r;
	wire   [ 4:0] write_register_MEM_w, write_register_MEM_r;
//  forward ALU  
	wire   [1:0]  Forwardrs1, Forwardrs2;
	wire   [31:0] ALUin1;
	wire   [31:0] ALUin2;
	wire   [31:0] ALUout;
	wire   [31:0] ALUout_EX_w , ALUout_EX_r;
	wire   [31:0] ALUout_MEM_w, ALUout_MEM_r;

	reg    [31:0] DCACHE_rdata_w;
	wire   [31:0] DCACHE_rdata_r;

//  PC jump, branch
	wire   [31:0] jump_address;
	//wire   [31:0] jump_address_imm;
	//wire   [31:0] jump_address_imm_ID_r,jump_address_imm_ID_w;
	//wire   [31:0] jump_address_ID_r, jump_address_ID_w;
	wire   [31:0] rv32_imm_ID_r, rv32_imm_ID_w;
	
	assign Branch_jump = BrResult&ctrl_signal_r1[10];
	assign PC_w  = 	(CACHE_stall|lw_stall)  ?PC_r:
					 flush|Branch_jump ?jump_address+rv32_imm_ID_r:
					PC_r+4;//ALUout == jalr(rs1+imm) or Br(PC+imm) or jal(PC+imm)

	assign        PC_IF_w = (CACHE_stall|lw_stall)? PC_IF_r : PC_r;
	assign 		  PC_ID_w = CACHE_stall? PC_ID_r : PC_IF_r;
	assign 		  PC_EX_w = CACHE_stall? PC_EX_r : PC_ID_r;
	assign 		  PC_MEM_w =CACHE_stall? PC_MEM_r: PC_EX_r;

	assign jump_address          = ctrl_signal_r1[18]? read_data1_r : PC_ID_r;
//	assign jump_address_imm      = jump_address+rv32_imm;
//	assign jump_address_imm_ID_w = (CACHE_stall|lw_stall)? jump_address_imm_ID_r:jump_address_imm; 
//	assign jump_address_ID_w     = (CACHE_stall|lw_stall)? jump_address_ID_r: jump_address;
	assign rv32_imm_ID_w         = (CACHE_stall|lw_stall)? rv32_imm_ID_r    : rv32_imm;

	assign  ICACHE_ren = rst_n? 1:0; //avoid timing violation
	assign  ICACHE_wen = 0;
	assign  ICACHE_addr = PC_r[31:2]; 
	assign  ICACHE_wdata = 0;

	assign IR_w     = CACHE_stall?  IR_r: 
					  flush|(Branch_jump)?       32'hd://addi x0 0
					  {ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};

	dff #(.DW(32)) d5 (.clk(clk), .rst_n(rst_n), .d(PC_w), .q(PC_r));
	dff #(.DW(32)) d6 (.clk(clk), .rst_n(rst_n), .d(PC_IF_w), .q(PC_IF_r));
	dff #(.DW(32)) d7 (.clk(clk), .rst_n(rst_n), .d(PC_ID_w), .q(PC_ID_r));
	dff #(.DW(32)) d8 (.clk(clk), .rst_n(rst_n), .d(PC_EX_w), .q(PC_EX_r));
//	dff #(.DW(32)) d9 (.clk(clk), .rst_n(rst_n), .d(PC_MEM_w), .q(PC_MEM_r));
	dff #(.DW(32)) d10(.clk(clk), .rst_n(rst_n), .d(IR_w), .q(IR_r));	
	//dff #(.DW(32)) d21(.clk(clk), .rst_n(rst_n), .d(jump_address_imm_ID_w), .q(jump_address_imm_ID_r));	
	//dff #(.DW(32)) d22(.clk(clk), .rst_n(rst_n), .d(jump_address_ID_w), .q(jump_address_ID_r));	
	dff #(.DW(32)) d23(.clk(clk), .rst_n(rst_n), .d(rv32_imm_ID_w), .q(rv32_imm_ID_r));	

//=====================ID_part======================================

	Forward forward1(.ERegWrite(ctrl_signal_r1[2]), 
			.MRegWrite(ctrl_signal_r2[2]),
			.WRegWrite(ctrl_signal_r3[2]), 
			.ERegRd(write_register_ID_r), 
			.MRegRd(write_register_EX_r), 
			.WRegRd(write_register_MEM_r), 
			.IRegRs1(read_register1), 
			.IRegRs2(read_register2), 
			.Forwardrs1(Forwardrs1), 
			.Forwardrs2(Forwardrs2)
			);

	assign  write_data =  //ctrl_signal_r3[0]?DCACHE_rdata_r:
						  write_data_WB_r;
						  //ctrl_signal_r3[1]?PC_MEM_r+4:
						  //ALUout_MEM_r;
	assign  read_register1 = rv32_rs1;
	assign  read_register2 = rv32_rs2;
	assign  sel_imm= ~(rv32_Rtype|rv32_Btype|rv32_Jtype);
	assign  rv32_imm=
			({32{rv32_Itype|rv32_load|rv32_jalr}} & rv32_i_imm)
		|	({32{rv32_store}} & rv32_s_imm)
		|	({32{rv32_Btype}} & rv32_b_imm)
//		|	({32{rv32_Utype}} & rv32_u_imm)
		|	({32{rv32_jal}} & rv32_j_imm);

	always@(*) begin
		if(CACHE_stall)
			read_data1_w = read_data1_r;
		else begin
			case(Forwardrs1)
				2'b00: read_data1_w = read_data1;
				2'b01: read_data1_w = write_data;
				2'b10: read_data1_w = ctrl_signal_r2[7]? DCACHE_rdata_w:ALUout_MEM_w;
				2'b11: read_data1_w = ALUout; //PC?
			endcase
		end
	end
	always@(*) begin
		if(CACHE_stall)
			read_data2_w = read_data2_r;
		else if(sel_imm)
			read_data2_w = rv32_imm;
		else begin
			read_data2_w = read_data2f;
		end
	end

	always@(*)begin
		case(Forwardrs2)
			2'b00: read_data2f = read_data2;
			2'b01: read_data2f = write_data;
			2'b10: read_data2f = ctrl_signal_r2[7]? DCACHE_rdata_w:ALUout_MEM_w;
			2'b11: read_data2f = ALUout; //PC?
		endcase
	end
	assign  read_data2_store_w= CACHE_stall? read_data2_store_r: read_data2f;
	assign  read_data2_EX_w = CACHE_stall? read_data2_EX_r :read_data2_store_r;

	assign  write_register_ID_w = CACHE_stall? write_register_ID_r: rv32_rd;
	assign  write_register_EX_w = CACHE_stall? write_register_EX_r: write_register_ID_r;
	assign  write_register_MEM_w= CACHE_stall? write_register_MEM_r: write_register_EX_r;

	dff #(.DW(32)) d11 (.clk(clk), .rst_n(rst_n), .d(read_data1_w), .q(read_data1_r));	
	dff #(.DW(32)) d12 (.clk(clk), .rst_n(rst_n), .d(read_data2_w), .q(read_data2_r));	
	dff #(.DW(32)) d20 (.clk(clk), .rst_n(rst_n), .d(read_data2_store_w), .q(read_data2_store_r));	
	dff #(.DW(32)) d13 (.clk(clk), .rst_n(rst_n), .d(read_data2_EX_w), .q(read_data2_EX_r));	
	dff #(.DW(32)) d25 (.clk(clk), .rst_n(rst_n), .d(write_data_WB_w), .q(write_data_WB_r));	
	
	dff #(.DW(5)) d14 (.clk(clk), .rst_n(rst_n), .d(write_register_ID_w), .q(write_register_ID_r));	
	dff #(.DW(5)) d15 (.clk(clk), .rst_n(rst_n), .d(write_register_EX_w), .q(write_register_EX_r));	
	dff #(.DW(5)) d16 (.clk(clk), .rst_n(rst_n), .d(write_register_MEM_w), .q(write_register_MEM_r));	
	
// Control
	wire [3:0] ALU_OP;
	wire 		MemRen;
	wire 		MemWen;
	Control Control1(
	.rv32_func7(rv32_func7[5]),
	.rv32_func3(rv32_func3),
	.rv32_Rtype(rv32_Rtype),
	.rv32_Itype(rv32_Itype),
	.rv32_store(rv32_store),
	.rv32_load (rv32_load),
	.rv32_jal  (rv32_jal),
	.rv32_jalr (rv32_jalr),
	.rv32_lui  (rv32_lui),
	.rv32_auipc(rv32_auipc),
	.ALU_OP    (ALU_OP),
	.PCSel     (PCSel),
	.RegWen    (RegWen),
	.MemRen    (MemRen),
	.MemWen    (MemWen),
	.WBSel     (WBSel)
	);

//======================EX_part=====================================
//ALU
	assign  ALUin1 = read_data1_r;
	assign  ALUin2 = read_data2_r;
	assign  ALUout_EX_w  = CACHE_stall? ALUout_EX_r : ctrl_signal_r1[17]? PC_ID_r+4 : ALUout;
	assign  ALUout_MEM_w = CACHE_stall? ALUout_MEM_r: ALUout_EX_r;
	
	dff #(.DW(32)) d17 (.clk(clk), .rst_n(rst_n), .d(ALUout_EX_w), .q(ALUout_EX_r));	
	dff #(.DW(32)) d18 (.clk(clk), .rst_n(rst_n), .d(ALUout_MEM_w), .q(ALUout_MEM_r));	

//====================MEM_part=============================
	dff #(.DW(32)) d19 (.clk(clk), .rst_n(rst_n), .d(DCACHE_rdata_w), .q(DCACHE_rdata_r));	

	assign  DCACHE_wen = ctrl_signal_r2[6];   //sw
	assign  DCACHE_ren = ctrl_signal_r2[7];   //lw
	assign  DCACHE_addr = ALUout_EX_r[31:2];  //rs1+imm
	assign  DCACHE_wdata = ctrl_signal_r2[3]? {read_data2_EX_r[7:0],read_data2_EX_r[15:8],{16{read_data2_EX_r[15]}}}: //TODO  // rv32_func3[0]
						   ctrl_signal_r2[4]? {read_data2_EX_r[7:0],read_data2_EX_r[15:8],read_data2_EX_r[23:16],read_data2_EX_r[31:24]} // rv32_func3[1]
						                     :{read_data2_EX_r[7:0],{24{read_data2_EX_r[7]}}};

//====================combinational_part===================
	always@(*)begin
		if(CACHE_stall)begin
		ctrl_signal_w1 = ctrl_signal_r1;
		ctrl_signal_w2 = ctrl_signal_r2;
		ctrl_signal_w3 = ctrl_signal_r3;
		end
		else begin
		if(lw_stall|(Branch_jump)|flush)
			ctrl_signal_w1 = 19'd0;
		else
			ctrl_signal_w1 ={rv32_jalr,rv32_Jtype, ALU_OP, rv32_Rtype, rv32_Itype, rv32_Btype, rv32_func7[5],PCSel,MemRen,MemWen,rv32_func3,RegWen,WBSel} ;
		ctrl_signal_w2 = ctrl_signal_r1[7:0];
		ctrl_signal_w3 = ctrl_signal_r2[5:0];
		end
	end

	always@(*) begin
		if(CACHE_stall) begin
			DCACHE_rdata_w = DCACHE_rdata_r;
		end
		else begin
		case(ctrl_signal_r2[6:3])
			3'b000: DCACHE_rdata_w = {DCACHE_rdata[7:0],{24{DCACHE_rdata[7]}}};  //TODO!
			3'b001: DCACHE_rdata_w = {DCACHE_rdata[7:0],DCACHE_rdata[15:8],{16{DCACHE_rdata[15]}}};
			3'b010: DCACHE_rdata_w = {DCACHE_rdata[7:0],DCACHE_rdata[15:8],DCACHE_rdata[23:16],DCACHE_rdata[31:24]};
			3'b100: DCACHE_rdata_w = {DCACHE_rdata[7:0],24'b0};
			3'b101: DCACHE_rdata_w = {DCACHE_rdata[7:0],DCACHE_rdata[15:8],16'b0};
			default: DCACHE_rdata_w = DCACHE_rdata_r;
		endcase 
		end
	end
	assign write_data_WB_w = CACHE_stall? write_data_WB_r:
							 ctrl_signal_r2[0]?DCACHE_rdata_w:
							 //ctrl_signal_r2[1]?PC_MEM_w+4:
							 ALUout_MEM_w;
//		assign  write_data =  ctrl_signal_r3[0]?DCACHE_rdata_r:
//						  ctrl_signal_r3[1]?PC_MEM_r+4
//						  :ALUout_MEM_r;
BranchComp BrCmp(.in1(read_data1_r), .in2(read_data2_r), .rv32_func3(ctrl_signal_r1[5:3]), .BrResult(BrResult));
RISC_ALU   ALU(.ALU_OP(ctrl_signal_r1[16:13]), .in1(ALUin1), .in2(ALUin2), .out(ALUout));
register   regi(
		   .Clk(clk),
		   .Rst(rst_n),
		   .i_Reg_write(ctrl_signal_r3[2]),
		   .i_read_register1(rv32_rs1),
		   .i_read_register2(rv32_rs2),
		   .i_write_data(write_data),
		   .i_write_register(write_register_MEM_r),
		   .o_read_data1(read_data1),
		   .o_read_data2(read_data2)
			);

assign   lw_stall    = ctrl_signal_r1[7] & (write_register_ID_r==read_register1 || write_register_ID_r==read_register2);
assign	 flush       = ctrl_signal_r1[8];// | (BrResult&rv32_Btype);

endmodule


// ALU

`define ALU_OP_WIDTH 4

`define ALU_OP_ADD  `ALU_OP_WIDTH'd0
`define ALU_OP_SUB  `ALU_OP_WIDTH'd1
`define ALU_OP_XOR  `ALU_OP_WIDTH'd2
`define ALU_OP_OR   `ALU_OP_WIDTH'd3
`define ALU_OP_AND  `ALU_OP_WIDTH'd4
`define ALU_OP_SLL  `ALU_OP_WIDTH'd5
`define ALU_OP_SRL  `ALU_OP_WIDTH'd6
`define ALU_OP_SRA  `ALU_OP_WIDTH'd7
`define ALU_OP_SLT  `ALU_OP_WIDTH'd8
`define ALU_OP_SLTU `ALU_OP_WIDTH'd9

module RISC_ALU(ALU_OP, in1, in2, out);
	input 		 [3:0]  ALU_OP;
	input signed [31:0] in1, in2;
	wire         [ 4:0] shamt;        //rs2%32
	assign              shamt = in2[4:0];
	output reg   [31:0] out;

	always@(*) begin
	case(ALU_OP)
		`ALU_OP_ADD : out = in1 + in2;
        `ALU_OP_SUB : out = in1 - in2;
        `ALU_OP_XOR : out = in1 ^ in2;
        `ALU_OP_OR  : out = in1 | in2;
        `ALU_OP_AND : out = in1 & in2;
        `ALU_OP_SLL : out = in1 << shamt;
        `ALU_OP_SRL : out = in1 >> shamt;
        `ALU_OP_SRA : out = $signed(in1) >>> shamt;
        `ALU_OP_SLT : out = {31'b0, $signed(in1) < $signed(in2)};
        `ALU_OP_SLTU : out = {31'b0, in1 < in2};
        default: out = 0;
    endcase
	end
endmodule

//ppt p.86
module Forward(ERegWrite, MRegWrite,WRegWrite, ERegRd, MRegRd, WRegRd, IRegRs1, IRegRs2, Forwardrs1, Forwardrs2);
	input 		 ERegWrite, MRegWrite, WRegWrite;
	input  [4:0] ERegRd, MRegRd, WRegRd, IRegRs1, IRegRs2;
	output [1:0] Forwardrs1, Forwardrs2;

	reg    [1:0] Forwardrs1, Forwardrs2;

//IERegRs,IERegRt is in ID
always@(*)begin
	if( (ERegWrite)&(ERegRd!=0)&(ERegRd==IRegRs1))
		Forwardrs1=2'b11;
	else if( (MRegWrite)&(MRegRd!=0)&(MRegRd==IRegRs1))
		Forwardrs1=2'b10;
	else if( (WRegWrite)&(WRegRd!=0)&(WRegRd==IRegRs1))
		Forwardrs1=2'b01;
	else
		Forwardrs1=2'b00;

	if( (ERegWrite)&(ERegRd!=0)&(ERegRd==IRegRs2))
		Forwardrs2=2'b11;
	else if( (MRegWrite)&(MRegRd!=0)&(MRegRd==IRegRs2))
		Forwardrs2=2'b10;
	else if( (WRegWrite)&(WRegRd!=0)&(WRegRd==IRegRs2))
		Forwardrs2=2'b01;
	else
		Forwardrs2=2'b00;
end		
endmodule

module BranchComp (
	in1,
	in2,
	rv32_func3,
	BrResult
);
	input [31:0] in1;
	input [31:0] in2;
	input [2:0]  rv32_func3;
	output reg   BrResult;	//jump==1 not jump==0

	always@(*)begin
		case(rv32_func3)
			3'b000:	BrResult = in1 == in2;
			3'b001:	BrResult = in1 != in2;
			3'b100:	BrResult = $signed(in1) < $signed(in2); 
			3'b101:	BrResult = $signed(in1) >= $signed(in2);
			3'b110:	BrResult = in1 < in2;
			3'b111:	BrResult = in1 >= in2;
		default: BrResult = 0;
		endcase // rv32_func3
	end

endmodule

module Control(
	rv32_func7,
	rv32_func3,
	rv32_Rtype,
	rv32_Itype,
	rv32_load,
	rv32_store,
	rv32_jal ,
	rv32_jalr,
	rv32_lui ,
	rv32_auipc,
	ALU_OP,
	PCSel,
	RegWen,
	MemRen,
	MemWen,
	WBSel
	);

	input  rv32_func7;
	input  [2:0] rv32_func3;
	input  rv32_Rtype;
	input  rv32_Itype;
	input  rv32_load;
	input  rv32_store;
	input  rv32_jal ;
	input  rv32_jalr;
	input  rv32_lui ;
	input  rv32_auipc;
	
	output reg [3:0] ALU_OP;
	output PCSel;		
	output RegWen;
	output MemRen;
	output MemWen;
	output [1:0] WBSel;
	
	assign PCSel = rv32_jal | rv32_jalr;
	assign RegWen= rv32_Rtype | rv32_Itype | rv32_load | rv32_jal | rv32_jalr | rv32_lui | rv32_auipc;
	assign MemRen= rv32_load;
	assign MemWen= rv32_store;
	assign WBSel[1] = rv32_jal | rv32_jalr;
	assign WBSel[0] = rv32_load;
	always@(*)begin
		if(~(rv32_Rtype|rv32_Itype))
			ALU_OP = `ALU_OP_ADD;
		else begin
		case (rv32_func3)
			3'b000: begin if(rv32_func7&rv32_Rtype) ALU_OP = `ALU_OP_SUB;
						  else 			 ALU_OP = `ALU_OP_ADD;end
			3'b001:	ALU_OP=`ALU_OP_SLL;
			3'b010:	ALU_OP=`ALU_OP_SLT;
			3'b011:	ALU_OP=`ALU_OP_SLTU;
			3'b100:	ALU_OP=`ALU_OP_XOR;
			3'b101:	if(rv32_func7) ALU_OP=`ALU_OP_SRA; else ALU_OP=`ALU_OP_SRL;
			3'b110:	ALU_OP=`ALU_OP_OR;
			3'b111:	ALU_OP=`ALU_OP_AND;
			endcase
		end
	end
endmodule

// ============== register ================================
module register(
	Clk,
    Rst,
    i_Reg_write,
    i_read_register1,
    i_read_register2,
    i_write_data,
    i_write_register,
    o_read_data1,
    o_read_data2
);
	input         Clk, Rst, i_Reg_write;
	input  [ 4:0] i_read_register1, i_read_register2, i_write_register;
	input  [31:0] i_write_data;
	output [31:0] o_read_data1, o_read_data2;

	reg [31:0] r_w [31:0];
	reg [31:0] r_r [31:0];
	
	integer i;

	always@(*) begin
		for (i = 1; i < 32; i = i + 1)
			r_w[i] = r_r[i];
		if(i_Reg_write == 1)
			r_w[i_write_register] = i_write_data;
	end

	assign o_read_data1 = r_r[i_read_register1];
	assign o_read_data2 = r_r[i_read_register2];

	always@(posedge Clk) begin
		if (!Rst) begin
			for(i = 0; i < 32; i = i+1)
				r_r[i] <= 0;
		end
		else begin
			r_r[0] <= 0;
			for(i = 1; i < 32; i = i+1)
				r_r[i] <= r_w[i];
		end
	end
endmodule

module dff #(
	parameter DW = 32
)(
	clk,
	rst_n,
	d,
	q
);

	input clk, rst_n;
	input [DW-1:0] d;
	output [DW-1:0] q;

	reg [DW-1:0] q_r;
	assign q = q_r;

	always@(posedge clk) begin
		if (!rst_n)  
			q_r <= {DW{1'b0}};
		else
			q_r <= d;
	end
endmodule


module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_wdata,
    proc_stall,
    proc_rdata,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    reg         proc_stall;
    reg  [31:0] proc_rdata;
    reg         mem_read, mem_write;
    reg  [27:0] mem_addr;
    reg [127:0] mem_wdata;

    // Finite State Machine
    parameter Idle       = 2'b00;
    parameter Write_Back = 2'b01;
    parameter Allocate   = 2'b10;

    reg [1:0] current_state, next_state;
    reg valid, hit, dirty;

    // cache data
    reg [127:0] cache_data0, cache_data1, cache_data2, cache_data3,
                cache_data4, cache_data5, cache_data6, cache_data7;
    reg [127:0] cache_data0_nxt, cache_data1_nxt, cache_data2_nxt, cache_data3_nxt,
                cache_data4_nxt, cache_data5_nxt, cache_data6_nxt, cache_data7_nxt;
    // cache tag
    reg  [24:0] cache_tag0, cache_tag1, cache_tag2, cache_tag3,
                cache_tag4, cache_tag5, cache_tag6, cache_tag7;
    reg  [24:0] cache_tag0_nxt, cache_tag1_nxt, cache_tag2_nxt, cache_tag3_nxt,
                cache_tag4_nxt, cache_tag5_nxt, cache_tag6_nxt, cache_tag7_nxt;
    // cache valid, dirty
    reg   [0:7] cache_valid, cache_dirty,
                cache_valid_nxt, cache_dirty_nxt;
    // cache local write enable
    

    reg   [0:7] write_from_mem;
    reg   [0:7] write_from_proc0, write_from_proc1, write_from_proc2, write_from_proc3;
    reg   [0:7] set_valid; // reset only at initialization (asynchronous)
    reg   [0:7] set_dirty;
    reg   [0:7] reset_dirty;

    // Finite State Machine
    // next state logic
    always @(*) begin
        case (current_state)
            Idle: begin
            // If CPU request, change to Different Types of Compare
                if (proc_read | proc_write) begin
                    case ({valid, hit, dirty})
                        3'b000: next_state = Allocate;
                        3'b001: next_state = Allocate;
                        3'b010: next_state = Allocate;
                        3'b011: next_state = Allocate;
                        3'b100: next_state = Allocate;
                        3'b101: next_state = Write_Back;
                        3'b110: next_state = Idle;
                        3'b111: next_state = Idle;
                    endcase
                end
                else next_state = Idle;
            end
            Write_Back: begin
            // wait for memory ready
                case (mem_ready)
                    1'b0: next_state = Write_Back;
                    1'b1: next_state = Allocate;
                endcase
            end
            Allocate: begin
            // allocate new data
                case (mem_ready)
                    1'b0: next_state = Allocate;
                    1'b1: next_state = Idle;
                endcase
            end
            default: next_state = Idle;
        endcase
    end
    // update sequential logic
    always@( posedge clk or posedge proc_reset ) begin
        if( proc_reset ) begin
            current_state <= Idle;
        end
        else begin
            current_state <= next_state;
        end
    end
    // interface control signals
    always @(*) begin
        case(current_state)
            Idle: begin
                if (proc_read | proc_write) begin
                    case ({valid, hit, dirty})
                        3'b000: proc_stall = 1'b1;
                        3'b001: proc_stall = 1'b1;
                        3'b010: proc_stall = 1'b1;
                        3'b011: proc_stall = 1'b1;
                        3'b100: proc_stall = 1'b1;
                        3'b101: proc_stall = 1'b1;
                        3'b110: proc_stall = 1'b0;
                        3'b111: proc_stall = 1'b0;
                    endcase 
                end
                else proc_stall = 1'b0;
                mem_read   = 1'b0;
                mem_write  = 1'b0;
            end
            Write_Back: begin
                proc_stall = 1'b1;
                mem_read   = 1'b0;
                mem_write  = 1'b1;
            end
            Allocate: begin
                proc_stall = 1'b1;
                mem_read   = 1'b1;
                mem_write  = 1'b0;
            end
            default: begin
                proc_stall = 1'b0;
                mem_read   = 1'b0;
                mem_write  = 1'b0;
            end
        endcase
    end
    // interface data and address
    always @(*) begin
        // memory interface
        if (mem_write) begin
            case (proc_addr[4:2])
                3'd0: mem_addr = {cache_tag0, proc_addr[4:2]};
                3'd1: mem_addr = {cache_tag1, proc_addr[4:2]};
                3'd2: mem_addr = {cache_tag2, proc_addr[4:2]};
                3'd3: mem_addr = {cache_tag3, proc_addr[4:2]};
                3'd4: mem_addr = {cache_tag4, proc_addr[4:2]};
                3'd5: mem_addr = {cache_tag5, proc_addr[4:2]};
                3'd6: mem_addr = {cache_tag6, proc_addr[4:2]};
                3'd7: mem_addr = {cache_tag7, proc_addr[4:2]};
            endcase
        end
        else mem_addr = proc_addr[29:2];

        case (proc_addr[4:2])
            3'd0: mem_wdata = cache_data0;
            3'd1: mem_wdata = cache_data1;
            3'd2: mem_wdata = cache_data2;
            3'd3: mem_wdata = cache_data3;
            3'd4: mem_wdata = cache_data4;
            3'd5: mem_wdata = cache_data5;
            3'd6: mem_wdata = cache_data6;
            3'd7: mem_wdata = cache_data7;
        endcase
        case (proc_addr[4:2]) // find where data stores
            3'd0: 
                case (proc_addr[1:0])
                    2'b00: proc_rdata = cache_data0[31:0];
                    2'b01: proc_rdata = cache_data0[63:32];
                    2'b10: proc_rdata = cache_data0[95:64];
                    2'b11: proc_rdata = cache_data0[127:96];
                endcase
            3'd1: 
                case (proc_addr[1:0])
                    2'b00: proc_rdata = cache_data1[31:0];
                    2'b01: proc_rdata = cache_data1[63:32];
                    2'b10: proc_rdata = cache_data1[95:64];
                    2'b11: proc_rdata = cache_data1[127:96];
                endcase
            3'd2: 
                case (proc_addr[1:0])
                    2'b00: proc_rdata = cache_data2[31:0];
                    2'b01: proc_rdata = cache_data2[63:32];
                    2'b10: proc_rdata = cache_data2[95:64];
                    2'b11: proc_rdata = cache_data2[127:96];
                endcase
            3'd3: 
                case (proc_addr[1:0])
                    2'b00: proc_rdata = cache_data3[31:0];
                    2'b01: proc_rdata = cache_data3[63:32];
                    2'b10: proc_rdata = cache_data3[95:64];
                    2'b11: proc_rdata = cache_data3[127:96];
                endcase
            3'd4: 
                case (proc_addr[1:0])
                    2'b00: proc_rdata = cache_data4[31:0];
                    2'b01: proc_rdata = cache_data4[63:32];
                    2'b10: proc_rdata = cache_data4[95:64];
                    2'b11: proc_rdata = cache_data4[127:96];
                endcase
            3'd5: 
                case (proc_addr[1:0])
                    2'b00: proc_rdata = cache_data5[31:0];
                    2'b01: proc_rdata = cache_data5[63:32];
                    2'b10: proc_rdata = cache_data5[95:64];
                    2'b11: proc_rdata = cache_data5[127:96];
                endcase
            3'd6: 
                case (proc_addr[1:0])
                    2'b00: proc_rdata = cache_data6[31:0];
                    2'b01: proc_rdata = cache_data6[63:32];
                    2'b10: proc_rdata = cache_data6[95:64];
                    2'b11: proc_rdata = cache_data6[127:96];
                endcase
            3'd7: 
                case (proc_addr[1:0])
                    2'b00: proc_rdata = cache_data7[31:0];
                    2'b01: proc_rdata = cache_data7[63:32];
                    2'b10: proc_rdata = cache_data7[95:64];
                    2'b11: proc_rdata = cache_data7[127:96];
                endcase
        endcase
    end
    // detection unit
    always @(*) begin
        case (proc_addr[4:2]) // find where data stores
            3'd0: begin
                valid = cache_valid[0];
                hit = (proc_addr[29:5] == cache_tag0);
                dirty = cache_dirty[0];
            end
            3'd1: begin 
                valid = cache_valid[1];
                hit = (proc_addr[29:5] == cache_tag1);
                dirty = cache_dirty[1];
            end
            3'd2: begin 
                valid = cache_valid[2];
                hit = (proc_addr[29:5] == cache_tag2);
                dirty = cache_dirty[2];
            end
            3'd3: begin 
                valid = cache_valid[3];
                hit = (proc_addr[29:5] == cache_tag3);
                dirty = cache_dirty[3];
            end
            3'd4: begin 
                valid = cache_valid[4];
                hit = (proc_addr[29:5] == cache_tag4);
                dirty = cache_dirty[4];
            end
            3'd5: begin 
                valid = cache_valid[5];
                hit = (proc_addr[29:5] == cache_tag5);
                dirty = cache_dirty[5];
            end
            3'd6: begin 
                valid = cache_valid[6];
                hit = (proc_addr[29:5] == cache_tag6);
                dirty = cache_dirty[6];
            end
            3'd7: begin 
                valid = cache_valid[7];
                hit = (proc_addr[29:5] == cache_tag7);
                dirty = cache_dirty[7];
            end
        endcase
    end
    // cache write enable control
    always @(*) begin
        case (current_state)
            Idle: begin
                if (proc_write && (!proc_stall)) begin
                    write_from_mem = 8'b0;
                    case (proc_addr[1:0])
                        2'b00: begin
                            case (proc_addr[4:2])
                                3'd0: write_from_proc0 = 8'b1000_0000;
                                3'd1: write_from_proc0 = 8'b0100_0000;
                                3'd2: write_from_proc0 = 8'b0010_0000;
                                3'd3: write_from_proc0 = 8'b0001_0000;
                                3'd4: write_from_proc0 = 8'b0000_1000;
                                3'd5: write_from_proc0 = 8'b0000_0100;
                                3'd6: write_from_proc0 = 8'b0000_0010;
                                3'd7: write_from_proc0 = 8'b0000_0001;
                            endcase
                            write_from_proc1 = 8'b0;
                            write_from_proc2 = 8'b0;
                            write_from_proc3 = 8'b0;
                        end
                        2'b01: begin
                            write_from_proc0 = 8'b0;
                            case (proc_addr[4:2])
                                3'd0: write_from_proc1 = 8'b1000_0000;
                                3'd1: write_from_proc1 = 8'b0100_0000;
                                3'd2: write_from_proc1 = 8'b0010_0000;
                                3'd3: write_from_proc1 = 8'b0001_0000;
                                3'd4: write_from_proc1 = 8'b0000_1000;
                                3'd5: write_from_proc1 = 8'b0000_0100;
                                3'd6: write_from_proc1 = 8'b0000_0010;
                                3'd7: write_from_proc1 = 8'b0000_0001;
                            endcase
                            write_from_proc2 = 8'b0;
                            write_from_proc3 = 8'b0;
                        end
                        2'b10: begin
                            write_from_proc0 = 8'b0;
                            write_from_proc1 = 8'b0;
                            case (proc_addr[4:2])
                                3'd0: write_from_proc2 = 8'b1000_0000;
                                3'd1: write_from_proc2 = 8'b0100_0000;
                                3'd2: write_from_proc2 = 8'b0010_0000;
                                3'd3: write_from_proc2 = 8'b0001_0000;
                                3'd4: write_from_proc2 = 8'b0000_1000;
                                3'd5: write_from_proc2 = 8'b0000_0100;
                                3'd6: write_from_proc2 = 8'b0000_0010;
                                3'd7: write_from_proc2 = 8'b0000_0001;
                            endcase
                            write_from_proc3 = 8'b0;
                        end
                        2'b11: begin
                            write_from_proc0 = 8'b0;
                            write_from_proc1 = 8'b0;
                            write_from_proc2 = 8'b0;
                            case (proc_addr[4:2])
                                3'd0: write_from_proc3 = 8'b1000_0000;
                                3'd1: write_from_proc3 = 8'b0100_0000;
                                3'd2: write_from_proc3 = 8'b0010_0000;
                                3'd3: write_from_proc3 = 8'b0001_0000;
                                3'd4: write_from_proc3 = 8'b0000_1000;
                                3'd5: write_from_proc3 = 8'b0000_0100;
                                3'd6: write_from_proc3 = 8'b0000_0010;
                                3'd7: write_from_proc3 = 8'b0000_0001;
                            endcase
                        end
                    endcase
                    case (proc_addr[4:2])
                        3'd0: set_valid = 8'b1000_0000;
                        3'd1: set_valid = 8'b0100_0000;
                        3'd2: set_valid = 8'b0010_0000;
                        3'd3: set_valid = 8'b0001_0000;
                        3'd4: set_valid = 8'b0000_1000;
                        3'd5: set_valid = 8'b0000_0100;
                        3'd6: set_valid = 8'b0000_0010;
                        3'd7: set_valid = 8'b0000_0001;
                    endcase
                    case (proc_addr[4:2])
                        3'd0: set_dirty = 8'b1000_0000;
                        3'd1: set_dirty = 8'b0100_0000;
                        3'd2: set_dirty = 8'b0010_0000;
                        3'd3: set_dirty = 8'b0001_0000;
                        3'd4: set_dirty = 8'b0000_1000;
                        3'd5: set_dirty = 8'b0000_0100;
                        3'd6: set_dirty = 8'b0000_0010;
                        3'd7: set_dirty = 8'b0000_0001;
                    endcase
                    reset_dirty      = 8'b0;                    
                end 
                else begin
                    write_from_mem   = 8'b0;
                    write_from_proc0 = 8'b0;
                    write_from_proc1 = 8'b0;
                    write_from_proc2 = 8'b0;
                    write_from_proc3 = 8'b0;
                    set_valid        = 8'b0;
                    set_dirty        = 8'b0;
                    reset_dirty      = 8'b0;
                end
            end
            Write_Back: begin
                write_from_mem   = 8'b0;
                write_from_proc0 = 8'b0;
                write_from_proc1 = 8'b0;
                write_from_proc2 = 8'b0;
                write_from_proc3 = 8'b0;
                set_valid        = 8'b0;
                set_dirty        = 8'b0;
                case (proc_addr[4:2])
                    3'd0: reset_dirty = 8'b1000_0000;
                    3'd1: reset_dirty = 8'b0100_0000;
                    3'd2: reset_dirty = 8'b0010_0000;
                    3'd3: reset_dirty = 8'b0001_0000;
                    3'd4: reset_dirty = 8'b0000_1000;
                    3'd5: reset_dirty = 8'b0000_0100;
                    3'd6: reset_dirty = 8'b0000_0010;
                    3'd7: reset_dirty = 8'b0000_0001;
                endcase
            end
            Allocate: begin
                case (proc_addr[4:2])
                    3'd0: write_from_mem = 8'b1000_0000;
                    3'd1: write_from_mem = 8'b0100_0000;
                    3'd2: write_from_mem = 8'b0010_0000;
                    3'd3: write_from_mem = 8'b0001_0000;
                    3'd4: write_from_mem = 8'b0000_1000;
                    3'd5: write_from_mem = 8'b0000_0100;
                    3'd6: write_from_mem = 8'b0000_0010;
                    3'd7: write_from_mem = 8'b0000_0001;
                endcase
                write_from_proc0 = 8'b0;
                write_from_proc1 = 8'b0;
                write_from_proc2 = 8'b0;
                write_from_proc3 = 8'b0;
                case (proc_addr[4:2])
                    3'd0: set_valid = 8'b1000_0000;
                    3'd1: set_valid = 8'b0100_0000;
                    3'd2: set_valid = 8'b0010_0000;
                    3'd3: set_valid = 8'b0001_0000;
                    3'd4: set_valid = 8'b0000_1000;
                    3'd5: set_valid = 8'b0000_0100;
                    3'd6: set_valid = 8'b0000_0010;
                    3'd7: set_valid = 8'b0000_0001;
                endcase
                set_dirty   = 8'b0;
                reset_dirty = 8'b0;   
            end
            default: begin
                write_from_mem   = 8'b0;
                write_from_proc0 = 8'b0;
                write_from_proc1 = 8'b0;
                write_from_proc2 = 8'b0;
                write_from_proc3 = 8'b0;
                set_valid        = 8'b0;
                set_dirty        = 8'b0;
                reset_dirty      = 8'b0;
            end
        endcase
    end
    // cache write access
    always @(*) begin
        cache_valid_nxt = set_valid | cache_valid;
        cache_dirty_nxt = (~reset_dirty) & (set_dirty | cache_dirty);

        if (write_from_mem[0]) cache_data0_nxt = mem_rdata;
        else begin
            if (write_from_proc0[0]) cache_data0_nxt  [31:0] = proc_wdata;
            else                     cache_data0_nxt  [31:0] = cache_data0[31:0];
            if (write_from_proc1[0]) cache_data0_nxt [63:32] = proc_wdata;
            else                     cache_data0_nxt [63:32] = cache_data0 [63:32];
            if (write_from_proc2[0]) cache_data0_nxt [95:64] = proc_wdata;
            else                     cache_data0_nxt [95:64] = cache_data0 [95:64];
            if (write_from_proc3[0]) cache_data0_nxt[127:96] = proc_wdata;
            else                     cache_data0_nxt[127:96] = cache_data0[127:96];
        end
        if (write_from_mem[1]) cache_data1_nxt = mem_rdata;
        else begin
            if (write_from_proc0[1]) cache_data1_nxt  [31:0] = proc_wdata;
            else                     cache_data1_nxt  [31:0] = cache_data1[31:0];
            if (write_from_proc1[1]) cache_data1_nxt [63:32] = proc_wdata;
            else                     cache_data1_nxt [63:32] = cache_data1 [63:32];
            if (write_from_proc2[1]) cache_data1_nxt [95:64] = proc_wdata;
            else                     cache_data1_nxt [95:64] = cache_data1 [95:64];
            if (write_from_proc3[1]) cache_data1_nxt[127:96] = proc_wdata;
            else                     cache_data1_nxt[127:96] = cache_data1[127:96];
        end
        if (write_from_mem[2]) cache_data2_nxt = mem_rdata;
        else begin
            if (write_from_proc0[2]) cache_data2_nxt  [31:0] = proc_wdata;
            else                     cache_data2_nxt  [31:0] = cache_data2[31:0];
            if (write_from_proc1[2]) cache_data2_nxt [63:32] = proc_wdata;
            else                     cache_data2_nxt [63:32] = cache_data2 [63:32];
            if (write_from_proc2[2]) cache_data2_nxt [95:64] = proc_wdata;
            else                     cache_data2_nxt [95:64] = cache_data2 [95:64];
            if (write_from_proc3[2]) cache_data2_nxt[127:96] = proc_wdata;
            else                     cache_data2_nxt[127:96] = cache_data2[127:96];
        end
        if (write_from_mem[3]) cache_data3_nxt = mem_rdata;
        else begin
            if (write_from_proc0[3]) cache_data3_nxt  [31:0] = proc_wdata;
            else                     cache_data3_nxt  [31:0] = cache_data3[31:0];
            if (write_from_proc1[3]) cache_data3_nxt [63:32] = proc_wdata;
            else                     cache_data3_nxt [63:32] = cache_data3 [63:32];
            if (write_from_proc2[3]) cache_data3_nxt [95:64] = proc_wdata;
            else                     cache_data3_nxt [95:64] = cache_data3 [95:64];
            if (write_from_proc3[3]) cache_data3_nxt[127:96] = proc_wdata;
            else                     cache_data3_nxt[127:96] = cache_data3[127:96];
        end
        if (write_from_mem[4]) cache_data4_nxt = mem_rdata;
        else begin
            if (write_from_proc0[4]) cache_data4_nxt  [31:0] = proc_wdata;
            else                     cache_data4_nxt  [31:0] = cache_data4[31:0];
            if (write_from_proc1[4]) cache_data4_nxt [63:32] = proc_wdata;
            else                     cache_data4_nxt [63:32] = cache_data4 [63:32];
            if (write_from_proc2[4]) cache_data4_nxt [95:64] = proc_wdata;
            else                     cache_data4_nxt [95:64] = cache_data4 [95:64];
            if (write_from_proc3[4]) cache_data4_nxt[127:96] = proc_wdata;
            else                     cache_data4_nxt[127:96] = cache_data4[127:96];
        end
        if (write_from_mem[5]) cache_data5_nxt = mem_rdata;
        else begin
            if (write_from_proc0[5]) cache_data5_nxt  [31:0] = proc_wdata;
            else                     cache_data5_nxt  [31:0] = cache_data5[31:0];
            if (write_from_proc1[5]) cache_data5_nxt [63:32] = proc_wdata;
            else                     cache_data5_nxt [63:32] = cache_data5 [63:32];
            if (write_from_proc2[5]) cache_data5_nxt [95:64] = proc_wdata;
            else                     cache_data5_nxt [95:64] = cache_data5 [95:64];
            if (write_from_proc3[5]) cache_data5_nxt[127:96] = proc_wdata;
            else                     cache_data5_nxt[127:96] = cache_data5[127:96];
        end
        if (write_from_mem[6]) cache_data6_nxt = mem_rdata;
        else begin
            if (write_from_proc0[6]) cache_data6_nxt  [31:0] = proc_wdata;
            else                     cache_data6_nxt  [31:0] = cache_data6[31:0];
            if (write_from_proc1[6]) cache_data6_nxt [63:32] = proc_wdata;
            else                     cache_data6_nxt [63:32] = cache_data6 [63:32];
            if (write_from_proc2[6]) cache_data6_nxt [95:64] = proc_wdata;
            else                     cache_data6_nxt [95:64] = cache_data6 [95:64];
            if (write_from_proc3[6]) cache_data6_nxt[127:96] = proc_wdata;
            else                     cache_data6_nxt[127:96] = cache_data6[127:96];
        end
        if (write_from_mem[7]) cache_data7_nxt = mem_rdata;
        else begin
            if (write_from_proc0[7]) cache_data7_nxt  [31:0] = proc_wdata;
            else                     cache_data7_nxt  [31:0] = cache_data7[31:0];
            if (write_from_proc1[7]) cache_data7_nxt [63:32] = proc_wdata;
            else                     cache_data7_nxt [63:32] = cache_data7 [63:32];
            if (write_from_proc2[7]) cache_data7_nxt [95:64] = proc_wdata;
            else                     cache_data7_nxt [95:64] = cache_data7 [95:64];
            if (write_from_proc3[7]) cache_data7_nxt[127:96] = proc_wdata;
            else                     cache_data7_nxt[127:96] = cache_data7[127:96];
        end

        if (write_from_mem[0]) cache_tag0_nxt = proc_addr[29:5]; else cache_tag0_nxt = cache_tag0;
        if (write_from_mem[1]) cache_tag1_nxt = proc_addr[29:5]; else cache_tag1_nxt = cache_tag1;
        if (write_from_mem[2]) cache_tag2_nxt = proc_addr[29:5]; else cache_tag2_nxt = cache_tag2;
        if (write_from_mem[3]) cache_tag3_nxt = proc_addr[29:5]; else cache_tag3_nxt = cache_tag3;
        if (write_from_mem[4]) cache_tag4_nxt = proc_addr[29:5]; else cache_tag4_nxt = cache_tag4;
        if (write_from_mem[5]) cache_tag5_nxt = proc_addr[29:5]; else cache_tag5_nxt = cache_tag5;
        if (write_from_mem[6]) cache_tag6_nxt = proc_addr[29:5]; else cache_tag6_nxt = cache_tag6;
        if (write_from_mem[7]) cache_tag7_nxt = proc_addr[29:5]; else cache_tag7_nxt = cache_tag7;
    end
    // sequential cache data
    always@( posedge clk or posedge proc_reset ) begin
        if( proc_reset ) begin
            cache_data0 <= 128'b0; cache_data1 <= 128'b0; cache_data2 <= 128'b0; cache_data3 <= 128'b0;
            cache_data4 <= 128'b0; cache_data5 <= 128'b0; cache_data6 <= 128'b0; cache_data7 <= 128'b0;
            cache_tag0 <= 25'b0; cache_tag1 <= 25'b0; cache_tag2 <= 25'b0; cache_tag3 <= 25'b0;
            cache_tag4 <= 25'b0; cache_tag5 <= 25'b0; cache_tag6 <= 25'b0; cache_tag7 <= 25'b0;
            cache_valid <= 8'b0; cache_dirty <= 8'b0;
        end
        else begin
            cache_data0 <= cache_data0_nxt; cache_data1 <= cache_data1_nxt; cache_data2 <= cache_data2_nxt; cache_data3 <= cache_data3_nxt;
            cache_data4 <= cache_data4_nxt; cache_data5 <= cache_data5_nxt; cache_data6 <= cache_data6_nxt; cache_data7 <= cache_data7_nxt;
            cache_tag0 <= cache_tag0_nxt; cache_tag1 <= cache_tag1_nxt; cache_tag2 <= cache_tag2_nxt; cache_tag3 <= cache_tag3_nxt;
            cache_tag4 <= cache_tag4_nxt; cache_tag5 <= cache_tag5_nxt; cache_tag6 <= cache_tag6_nxt; cache_tag7 <= cache_tag7_nxt;
            cache_valid <= cache_valid_nxt; cache_dirty <= cache_dirty_nxt;
        end
    end

endmodule