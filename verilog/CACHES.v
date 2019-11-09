/*
File: CACHES.v
Author: Nikita Kim
Email: kimn1944@gmail.com
Date: 11/8/19
*/

`include "config.v"

module CACHES
    #()
    (input clk,
    input reset,

    input [31:0] data_addr,
    input [31:0] data_write,
    input is_write,
    input [31:0] instr_addr,

    output reg stall,
    output reg [31:0] data,
    output reg [31:0] instr);


endmodule
