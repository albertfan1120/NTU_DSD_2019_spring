`timescale 1 ns/10 ps
`define TestPort    30'hFF
`define BeginSymbol 32'h00000168
`define EndSymbol   32'hFFFFFD5D
`define CheckNum    7'd81

module	TestBed(
	clk,
	rst,
	addr,
	data,
	wen,
	error_num,
	duration,
	finish
);
	input			clk, rst;
	input	[29:0]	addr;
	input	[31:0]	data;
	input			wen;

	output	[7:0]	error_num;
	output	[15:0]	duration;
	output			finish;
	reg		[7:0]	error_num;
	reg		[15:0]	duration;
	reg				finish;
	
	reg		[31:0]	answer;

	reg		[1:0]	curstate;
	reg		[1:0]	nxtstate;
	reg		[6:0]	curaddr;
	reg		[6:0]	nxtaddr;
	reg		[7:0]	nxt_error_num;
	reg		[15:0]	nxtduration;
	
	reg				state,state_next;

	wire    [31:0]  data_modify;
		
	parameter	state_idle 	= 2'b00;
	parameter	state_check = 2'b01;
	parameter	state_report= 2'b10;	

	assign data_modify = {data[7:0],data[15:8],data[23:16],data[31:24]}; // convert little-endian format to readable format
		
	always@( posedge clk or negedge rst )						// State-DFF
	begin
		if( ~rst )
		begin
			curstate <= state_idle;
			curaddr  <= 0;
			duration <= 0;
			error_num <= 8'd255;
			
			state <= 0;
		end
		else
		begin
			curstate <= nxtstate;
			curaddr  <= nxtaddr;
			duration <= nxtduration;
			error_num <= nxt_error_num;
			
			state <= state_next;
		end
	end
			
	always@(*)	// FSM for test
	begin
		finish = 1'b0;
		case( curstate )
		state_idle: 	begin
							nxtaddr = 0;
							nxtduration = 0;
							nxt_error_num = 255;	
							if( addr==`TestPort && data_modify==`BeginSymbol && wen )
							begin
								nxt_error_num = 0;
								nxtstate = state_check;
							end	 	
							else nxtstate = state_idle;
						end
		state_check:	begin
							nxtduration = duration + 1;
							nxtaddr = curaddr;						
							nxt_error_num = error_num;	
							if( addr==`TestPort && wen && state==0 )
							begin
								nxtaddr = curaddr + 1;
								if( data_modify != answer )
									nxt_error_num = error_num + 8'd1;
							end
							nxtstate = curstate;
							if( curaddr==`CheckNum )	
								nxtstate = state_report;
						end
		state_report:	begin
							finish = 1'b1;
							nxtaddr = curaddr;
							nxtstate = curstate;		
							nxtduration = duration;
							nxt_error_num = error_num;	
						end				
		endcase	
	end

	always@( negedge clk )						
	begin
		if(curstate == state_report) begin
			$display("--------------------------- Simulation FINISH !!---------------------------");
			if (error_num) begin 
				$display("============================================================================");
				$display("\n (T_T) FAIL!! The simulation result is FAIL!!! there were %d errors at all.\n", error_num);
				$display("============================================================================");
			end
			 else begin 
				$display("============================================================================");
				$display("\n \\(^o^)/ CONGRATULATIONS!!  The simulation result is PASS!!!\n");
				$display("============================================================================");
			end
		end
	end
	
	always@(*)begin//sub-FSM (avoid the Dcache stall condition)
		case(state)
			1'b0:begin
				if(wen)
					state_next=1;
				else
					state_next=state;				
			end
			1'b1:begin
				if(!wen)
					state_next=0;
				else
					state_next=state;	
			end
		endcase
	end
	
	
	always@(*)	// ROM for correct result
	begin
		answer = 0;
		case( curaddr )
		7'd0   : answer = 32'd0     ;
		7'd1   : answer = 32'd1     ;
		7'd2   : answer = 32'd1     ;
		7'd3   : answer = 32'd2     ;
		7'd4   : answer = 32'd3     ;
		7'd5   : answer = 32'd5     ;
		7'd6   : answer = 32'd8     ;
		7'd7   : answer = 32'd13    ;
		7'd8   : answer = 32'd21    ;
		7'd9   : answer = 32'd34    ;
		7'd10  : answer = 32'd55    ;
		7'd11  : answer = 32'd89    ;
		7'd12  : answer = 32'd144   ;
		7'd13  : answer = 32'd233   ;
		7'd14  : answer = 32'd377   ;
		7'd15  : answer = 32'd610   ;
		7'd16  : answer = 32'd987   ;
		7'd17  : answer = 32'd1597  ;
		7'd18  : answer = 32'd2584  ;
		7'd19  : answer = 32'd4181  ;
		7'd20  : answer = 32'd6765  ;
		7'd21  : answer = 32'd10946 ;
		7'd22  : answer = 32'd17711 ;
		7'd23  : answer = 32'd28657 ;
		7'd24  : answer = 32'd46368 ;
		7'd25  : answer = 32'd75025 ;
		7'd26  : answer = 32'd121393;
		7'd27  : answer = 32'd196418;
		7'd28  : answer = 32'd317811;
		7'd29  : answer = 32'd514229;
		7'd30  : answer = 32'd832040;
		7'd31  : answer = 32'd1346269;
		7'd32  : answer = 32'd2178309;
		7'd33  : answer = 32'd3524578;
		7'd34  : answer = 32'd5702887;
		7'd35  : answer = 32'd9227465;
		7'd36  : answer = 32'd14930352;
		7'd37  : answer = 32'd24157817;
		7'd38  : answer = 32'd39088169;
		7'd39  : answer = 32'd63245986;
		7'd40  : answer = 32'd63245986;
		7'd41  : answer = 32'd39088169;
		7'd42  : answer = 32'd24157817;
		7'd43  : answer = 32'd14930352;
		7'd44  : answer = 32'd9227465;
		7'd45  : answer = 32'd5702887;
		7'd46  : answer = 32'd3524578;
		7'd47  : answer = 32'd2178309;
		7'd48  : answer = 32'd1346269;
		7'd49  : answer = 32'd832040;
		7'd50  : answer = 32'd514229;
		7'd51  : answer = 32'd317811;
		7'd52  : answer = 32'd196418;
		7'd53  : answer = 32'd121393;
		7'd54  : answer = 32'd75025 ;
		7'd55  : answer = 32'd46368 ;
		7'd56  : answer = 32'd28657 ;
		7'd57  : answer = 32'd17711 ;
		7'd58  : answer = 32'd10946 ;
		7'd59  : answer = 32'd6765  ;
		7'd60  : answer = 32'd4181  ;
		7'd61  : answer = 32'd2584  ;
		7'd62  : answer = 32'd1597  ;
		7'd63  : answer = 32'd987   ;
		7'd64  : answer = 32'd610   ;
		7'd65  : answer = 32'd377   ;
		7'd66  : answer = 32'd233   ;
		7'd67  : answer = 32'd144   ;
		7'd68  : answer = 32'd89    ;
		7'd69  : answer = 32'd55    ;
		7'd70  : answer = 32'd34    ;
		7'd71  : answer = 32'd21    ;
		7'd72  : answer = 32'd13    ;
		7'd73  : answer = 32'd8     ;
		7'd74  : answer = 32'd5     ;
		7'd75  : answer = 32'd3     ;
		7'd76  : answer = 32'd2     ;
		7'd77  : answer = 32'd1     ;
		7'd78  : answer = 32'd1     ;
		7'd79  : answer = 32'd0     ;
		7'd80  : answer = `EndSymbol;
		endcase
	end
endmodule