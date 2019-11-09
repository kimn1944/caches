/*
File: CACHE.v
Author: Nikita Kim
Email: kimn1944@gmail.com
Date: 11/8/19
*/

`include "config.v"

module CACHE
    #(parameter INDEX = 10,
      parameter TAG = 17,
      parameter OFFSET = 3,
      parameter WAY = 1)
    (input clk,
    input reset,

    input [31:0] data_addr,
    input [31:0] data_write,
    input is_write,

    output reg stall,
    output reg [31:0] data,
    
    );

endmodule
