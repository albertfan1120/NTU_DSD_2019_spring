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