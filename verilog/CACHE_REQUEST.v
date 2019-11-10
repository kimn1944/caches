module CACHE_REQUEST
    #(parameter SIZE = 1024,
      parameter BLK_SIZE = 256,
      parameter INDEX = 10,
      parameter TAG = 17,
      parameter OFFSET = 3,
      parameter LEN = 274)
    (input clk,
    input reset,
    input stall,

    ref [LEN - 1:0] cache [SIZE - 1:0],
    input [31:0] data_addr,
    // requesting
    input request,
    output reg [31:0] request_addr,

    // receiving
    input [31:0] data);
    // end arguments

    integer i;
    integer blks = BLK_SIZE / 32;
    reg [OFFSET - 1:0] switch;
    wire [31:0] req_addr;
    wire [31:0] wrt_addr;
    wire [INDEX - 1:0]  wrt_idx;
    wire [TAG - 1:0]    wrt_tag;
    wire [OFFSET - 1:0] ofst;

    assign req_addr = {data_addr[31:OFFSET + 2], switch, 2'b00};
    assign wrt_addr = {data_addr[31:OFFSET + 2], swtich - 1, 2'b00};
    assign wrt_idx  = wrt_addr[INDEX + OFFSET + 1:OFFSET + 2];
    assign wrt_tag  = wrt_addr[TAG + INDEX + OFFSET + 1:INDEX + OFFSET + 2];
    assign ofst     = wrt_addr[OFFSET + 1:2];
    integer wrt_ofst = ofst * 32;


    always @ (posedge clk or negedge reset) begin
        if(!reset) begin
            switch <= 0;
        end
        else if(!stall & request) begin
            // requesting
            request_addr <= req_addr;
            switch <= switch + 1;

            // writing
            cache[wrt_idx][LEN - 2:LEN - 1 - TAG]   <= wrt_tag;
            cache[wrt_idx][wrt_ofst + 31:wrt_ofst]  <= data;
            cache[LEN - 1] <= 1;
        end
        else begin
            request_addr <= 0;
            switch <= 0;
        end
    end

endmodule
