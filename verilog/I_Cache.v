module    I_Cache(
    // control signal
    input                        CLK,
    input                        RESET,
    // DRAM side
    input                  [31:0]DRAM_data,
    input                        DRAM_valid,
    output reg                   DRAM_req,
    output reg             [31:0]DRAM_req_addr,
    // IF side
    input                  [31:0]IF_address,         //addr
    input                        Instr_req,          // instruction req
    output reg             [31:0]instuction,         // instruction for IF
    output                       hit,                //determine if hit in combination with Instr_req 
    output                       rom_abort
    );
    
    reg                    [31:0]counter;
    reg                    [31:0]IF_address_dly;
    reg                          Instr_req_dly;
    reg                          DRAM_req_dly;
    reg                    [31:0]DRAM_data_shift[7:0];
    reg                   [274:0]I_SRAM_data;
    wire                         DRAM_data_ready;
    reg                   [274:0]I_SRAM[1023:0];
    integer                      i;

    //1024 blocks, 256 bit per block, 10 bit index [14:5], 8 bit for offset [4:2], 17 bit tag [31:14], 1 bit valid.
    //cache size 2^10 * (256 + 14 + 1) = 2^10 * 271

    // instruction sram block
    always@(posedge CLK or negedge RESET)begin
        if(!RESET)begin
            for(i=0;i<1024;i=i+1)begin
                I_SRAM[i]    <=    0;
            end
        end
        else if(DRAM_data_ready)begin
            I_SRAM[IF_address_dly[14:5]]    <=    {1'b1,IF_address_dly[31:14],
                                                        DRAM_data_shift[7],DRAM_data_shift[6],
                                                        DRAM_data_shift[5],DRAM_data_shift[4],
                                                        DRAM_data_shift[3],DRAM_data_shift[2],
                                                        DRAM_data_shift[1],DRAM_data_shift[0]};
        end
    end

    always@(posedge CLK or negedge RESET)begin
        if(!RESET)begin
            I_SRAM_data    <=    0;
        end else if( Instr_req | ({DRAM_req_dly,DRAM_req}==2'b10) )begin
            I_SRAM_data    <=    I_SRAM[IF_address[14:5]];
        end
    end

    // output signals
    assign    hit = I_SRAM_data[274] & (IF_address_dly[31:14]==I_SRAM_data[273:256]);
    assign    rom_abort = (~hit & Instr_req_dly) | DRAM_req | DRAM_req_dly;
    assign    DRAM_data_ready = (8==counter);    
        
    always@(*)begin
        case(IF_address_dly[4:2])
            0:    instuction = I_SRAM_data[31:0];
            1:    instuction = I_SRAM_data[63:32];
            2:    instuction = I_SRAM_data[95:64];
            3:    instuction = I_SRAM_data[127:96];
            4:    instuction = I_SRAM_data[159:128];
            5:    instuction = I_SRAM_data[191:160];
            6:    instuction = I_SRAM_data[223:192];
            7:    instuction = I_SRAM_data[255:224];
            default:instuction = I_SRAM_data[31:0];
        endcase
    end
    
    always@(posedge CLK or negedge RESET)begin
        if( !RESET )begin
            DRAM_req    <=    0;
        end else if( ~hit & Instr_req_dly )begin
            DRAM_req    <=    1;
        end else if( DRAM_data_ready )begin
            DRAM_req    <=    0;
        end
    end
    
    always@(posedge CLK)begin
        DRAM_req_dly    <=    DRAM_req;
    end
    
    // physical address of instructions
    always@(posedge CLK) begin
        if( RESET )begin
            DRAM_req_addr    <=    0;
        end else begin
            DRAM_req_addr    <=    {2'b0,IF_address_dly[31:5],3'b0};
        end
    end
    
    // input signal buffer
    always@(posedge CLK) begin
        if( RESET )begin
            Instr_req_dly            <=    0;
        end else begin
            Instr_req_dly            <=    Instr_req;
        end
    end
    
    always@(posedge CLK) begin
        if( RESET )begin
            IF_address_dly    <=    0;
        end else if( (Instr_req_dly & ~hit) || DRAM_req )begin
            IF_address_dly    <=    IF_address_dly;
        end else if(Instr_req)begin
            IF_address_dly    <=    IF_address;
        end
    end
    
    // block counter
    always@(posedge CLK or negedge RESET) begin
        if( !RESET ) begin
            counter    <=    0;
        end else if( DRAM_data_ready ) begin
                counter    <=    0;
        end else if(DRAM_valid)begin
                counter    <=    counter + 1'b1;
        end
    end
    
    // DRAM data buffer
    always@(posedge CLK or negedge RESET) begin
        if( !RESET ) begin
            DRAM_data_shift[0]    <=    0;
            DRAM_data_shift[1]    <=    0;
            DRAM_data_shift[2]    <=    0;
            DRAM_data_shift[3]    <=    0;
            DRAM_data_shift[4]    <=    0;
            DRAM_data_shift[5]    <=    0;
            DRAM_data_shift[6]    <=    0;
            DRAM_data_shift[7]    <=    0;
        end else if( DRAM_data_ready ) begin
            DRAM_data_shift[0]    <=    0;
            DRAM_data_shift[1]    <=    0;
            DRAM_data_shift[2]    <=    0;
            DRAM_data_shift[3]    <=    0;
            DRAM_data_shift[4]    <=    0;
            DRAM_data_shift[5]    <=    0;
            DRAM_data_shift[6]    <=    0;
            DRAM_data_shift[7]    <=    0;
        end else if(DRAM_valid) begin
            DRAM_data_shift[0]    <=    DRAM_data_shift[1];
            DRAM_data_shift[1]    <=    DRAM_data_shift[2];
            DRAM_data_shift[2]    <=    DRAM_data_shift[3];
            DRAM_data_shift[3]    <=    DRAM_data_shift[4];
            DRAM_data_shift[4]    <=    DRAM_data_shift[5];
            DRAM_data_shift[5]    <=    DRAM_data_shift[6];
            DRAM_data_shift[6]    <=    DRAM_data_shift[7];
            DRAM_data_shift[7]    <=    DRAM_data;
        end
    end
    
    
endmodule