/*
File: CACHE.v
Author: Nikita Kim
Email: kimn1944@gmail.com
Date: 11/8/19
*/

`include "config.v"

module IC
    #(parameter HIT = 1,
      parameter MISS = 9)
    (input clk,
    input reset,
    input stall,

    // reading or writing
    input [31:0] data_addr,
    input is_read,

    input [31:0] requested_data,
    output reg [31:0] request_addr,

    // general outputs
    output reg stop,
    output reg hit,
    output reg [31:0] data);
    // end arguments

    integer i;
    reg [273:0]  cache [1023:0];

    // accessing info in the cache
    wire [9:0]  idx;
    wire [16:0] tag;
    wire [2:0] ofst;
    assign idx  = data_addr[14:5];
    assign tag  = data_addr[31:15];
    assign ofst = data_addr[4:2];

    wire [16:0] cache_tag;
    wire [31:0] cache_data;
    wire cache_valid;
    integer cache_ofst  = 32 * ofst;
    assign cache_tag      = cache[idx][272:256];
    assign cache_data     = cache[idx][cache_ofst +:32];
    assign cache_valid    = cache[idx][273] & (tag == cache_tag);
    // end accessing info in the cache

    // dealing with miss
    wire del_stop;
    assign stop = del_stop | ~cache_valid;
    DELAYER del(
        .clk(clk),
        .reset(reset),
        .stall(stall),
        .delay(is_read & ~cache_valid),
        .stop(del_stop));

    reg [2:0] switch;
    reg invalid;
    integer wrt_ofst  = (switch - 1) * 32;

    always @ (posedge clk or negedge reset) begin
        if(!reset) begin
            switch <= 0;
        end
        else if(!stall & is_read & (invalid | ~cache_valid)) begin
            // requesting
            request_addr  <= {tag, idx, switch, 2'b00};
            switch        <= switch + 1;
            // writing
            cache[idx][272:256]   <= tag;
            cache[idx][wrt_ofst +:32]  <= requested_data;
            cache[idx][273] <= 1;
        end
        else begin
            request_addr  <= 0;
            switch        <= 0;
        end
    end
    // end dealing with miss

    always @(posedge clk or negedge reset) begin
        if(!reset) begin
            for(i = 0; i < 1024; i = i + 1) begin
                cache[273] = 0;
            end
            hit   <= 0;
            data  <= 0;
        end
        else if(is_read & !stop & !stall) begin
            invalid <= ~cache_valid;
            // stop  <= del_stop;
            hit   <= cache_valid;
            data  <= cache_data;
        end
        `ifdef IC_PRINT
            $display("\t\t\t\tIC Output");
            $display("Data Addr: %x, Data: %x", data_addr, data);
            $display("Req Addr: %x, Req Data: %x", request_addr, requested_data);
            $display("Stop: %x, Hit: %x", stop, hit);
            $display("Idx: %x, Tag: %x, Ofst: %x", idx, tag, ofst);
            $display("Cache Val: %x, Cache Data: %x, Cache Tag: %x, Cache Ofst: %d", cache_valid, cache_data, cache_tag, cache_ofst);
            $display("\t\t\t\tIC End");
        `endif

    end

endmodule
