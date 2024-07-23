module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
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
    always@( posedge clk )begin//or negedge proc_reset ) begin
        if( !proc_reset ) begin
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
    always@( posedge clk )begin//or posedge proc_reset ) begin
        if( !proc_reset ) begin
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
