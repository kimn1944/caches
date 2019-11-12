/*
File: CACHE.v
Author: Nikita Kim
Email: kimn1944@gmail.com
Date: 11/8/19
*/

`include "config.v"

module DC
    #()
    (input clk,
    input reset,
    input stall,
    input flush,

    // general requests
    input [31:0] data_addr,
    input is_read,
    input is_write,
    input [31:0] data_to_write,

    // dealing with miss
    output reg is_request,
    output reg [31:0] request_addr,
    input [31:0] requested_data,

    // dealing with WB
    output reg is_wb,
    output reg [31:0] wb_addr,
    output reg [255:0] wb_data,

    // general outputs
    output reg stop,
    output reg [31:0] data);
    // end arguments

    integer i;
    reg [552:0] cache [511:0];

    // accessing the cache
    wire [8:0]  idx;
    wire [17:0] tag;
    wire [2:0]  ofst;
    assign idx  = data_addr[13:5];
    assign tag  = data_addr[31:14];
    assign ofst = data_addr[4:2];

    wire lru;
    wire c1_valid;
    wire c2_valid;
    wire hit;
    wire [31:0] cdata;
    assign lru      = cache[idx][552];            // 0 - first block lru, 1 - second block lru
    assign c1_vaild = cache[idx][551] & (cache[idx][549:532] == tag);
    assign c2_valid = cache[idx][275] & (cache[idx][273:256] == tag);
    assign hit      = c1_valid | c2_valid;
    assign cdata    = c1_valid ? cache[idx][ofst * 32 + 276 +:32] : cache[idx][ofst * 32 +:32] ;
    // end accessing the cache

    // dealing with miss
    reg [8:0] ring;
    reg [2:0] switch;

        // resetting our counter stuff
    always @(negedge hit) begin
        if(!stall) begin
            if(is_read | is_write) begin
                ring    <= 9'b100000000;
                switch  <= 0;
            end
        end
    end

        // moving the missed data into the cache
    always @(posedge clk) begin
        if(!stall & ~ring[0]) begin
            switch  <= switch + 1;
            ring    <= {1'b1, ring[8:1]};
            cache[idx][549:532] <= lru ? cache[idx][549:532] : tag;
            cache[idx][273:256] <= lru ? tag : cache[idx][273:256];
            cache[idx][switch * 32 + 276 +:32]  <= lru ? cache[idx][switch * 32 + 276 +:32] : requested_data;
            cache[idx][switch * 32 +:32]        <= lru ? requested_data : cache[idx][switch * 32 +:32];
            cache[idx][551:550] <= lru ? cache[idx][551:550] : 2'b10;
            cache[idx][275:274] <= lru ? 2'b10 : cache[idx][275:274];
            // dealing with WB
            if(~lru & cache[idx][550] | lru & cache[idx][274]) begin
                is_wb   <= 1;
                wb_addr <= data_addr;
                wb_data <= cache[idx][550] ? cache[idx][531:276] : cache[idx][255:0];
            end
            else begin
                is_wb   <= 0;
                wb_addr <= 0;
                wb_data <= 0;
            end
            // end dealing with WB
        end
    end
    // end dealing with miss

    always @ * begin
        if(!stall) begin
            is_request   <= ~ring[0];
            request_addr <= {tag, idx, switch, 2'b00};
            stop <= ~ring[0] | (j < 1024);
            // outputting read data
            if(is_read) begin
                data <= cdata;
            end
            // dealing with write, actually writing
            else if(is_write) begin
                cache[idx][ofst * 32 + 276 +:32] <= c1_valid ? data_to_write : cache[idx][ofst * 32 + 276 +:32];
                cache[idx][ofst * 32 +:32]       <= c2_valid ? data_to_write : cache[idx][ofst * 32 +:32];
            end
            // end dealing with write
        end
    end

    // dealing with flush
    // integer j;
    // always @(posedge flush) begin
    //     if(!stall) begin
    //         j <= 0;
    //     end
    // end
    //
    // always @(posedge clk) begin
    //     if(j < 1024 & !stall) begin
    //         is_wb   <= 1;
    //         wb_addr <= data_addr;
    //         wb_data <= cache[idx][550] ? cache[idx][531:276] : cache[idx][255:0];
    //         j       <= j + 1;
    //     end
    // end
    // end dealing with flush

    always @(posedge clk or negedge reset) begin
        if(!reset) begin
            for(i = 0; i < 512; i = i + 1) begin
                cache[i][551:550] = 0;
                cache[i][275:274] = 0;
            end
        end
        else if(!stall & hit) begin
            // update lru
            cache[idx][552] <= c1_valid ? 1 : 0;
            // updating dirty bit
            if(is_write) begin
                cache[idx][550] <= c1_valid ? 1 : 0;
                cache[idx][274] <= c2_valid ? 1 : 0;
            end
        end
    end

endmodule
