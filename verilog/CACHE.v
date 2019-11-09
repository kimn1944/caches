/*
File: CACHE.v
Author: Nikita Kim
Email: kimn1944@gmail.com
Date: 11/8/19
*/

`include "config.v"

module CACHE
    #(parameter SIZE = 1024,
      parameter BLK_SIZE = 256,
      parameter INDEX = 10,
      parameter TAG = 17,
      parameter OFFSET = 3,
      parameter HIT = 1,
      parameter MISS = 9)
    (input clk,
    input reset,
    input stall,

    // reading or writing
    input [31:0] data_addr,
    input [31:0] data_write,
    input is_write,
    input is_read,

    // dealing with misses
    input is_return,
    input [31:0] _addr_return,
    output [31:0] _return,
    output ready_return,

    output request,
    output [31:0] _addr_request,
    input [31:0] _request,
    input ready_request,

    // general outputs
    output reg stop,
    output reg hit,
    output reg [31:0] data);
    // end arguments

    integer i;
    integer length = BLK_SIZE + 1 + TAG;

    // accessing info in the cache
    reg [length - 1:0]  cache [SIZE - 1:0];
    wire [INDEX - 1:0]  idx;
    wire [TAG - 1:0]    tag;
    wire [OFFSET - 1:0] ofst;
    assign idx  = data_addr[INDEX + OFFSET + 1:OFFSET + 2];
    assign tag  = data_addr[TAG + INDEX + OFFSET + 1:INDEX + OFFSET + 2];
    assign ofst = data_addr[OFFSET + 1:2];

    wire [TAG - 1:0] cache_tag;
    wire [31:0] cache_data;
    wire cache_valid;
    integer cache_offset = 32 * ofst;
    assign cache_tag    = cache[idx][length - 2:length - 1 - TAG];
    assign cache_data   = cache[idx][cache_offset + 31:cache_offset];
    assign cache_valid  = cache[idx][length - 1] & (tag == cache_tag);
    // end accessing info in the cache

    // dealing with miss
    wire del_stop;

    DELAYER del(
        .clk(clk),
        .reset(reset),
        .stall(stall),
        .delay(is_read),
        .stop(del_stop));
    // end dealing with miss

    always @(posedge clk or negedge reset) begin
        if(!reset) begin
            for(i = 0; i < SIZE, i = i + 1) begin
                cache[length - 1] <= 0;
            end
            stall <= 0;
            hit   <= 0;
            data  <= 0;
        end
        else if(is_read & ring[0]) begin
            stall <= ~cache_valid;
            hit   <= cache_valid;
            data  <= cache_data;
        end
        else if(is_read) begin

        end
    end

endmodule
