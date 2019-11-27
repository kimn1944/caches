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
    input [31:0] instr,
    input [31:0] data_addr,
    input is_read,
    input is_write,
    input [31:0] data_to_write,
    input [1:0] data_to_write_size,

    // dealing with miss
    output reg is_request,
    output reg [31:0] request_addr,
    input [31:0] requested_data,

    // dealing with WB
    output reg is_wb,
    // output reg [31:0] wb_addr,
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
    wire [1:0] alignment;
    wire [31:0] cdata;
    assign lru      = cache[idx][552];            // 0 - first block lru, 1 - second block lru
    assign c1_valid = cache[idx][551] & (cache[idx][549:532] == tag);
    assign c2_valid = cache[idx][275] & (cache[idx][273:256] == tag);
    assign hit      = c1_valid | c2_valid;
    assign alignment = data_addr[1:0];
    assign cdata    = c1_valid ? cache[idx][ofst * 32 + 276 +:32] : cache[idx][ofst * 32 +:32] ;
    // end accessing the cache

    // dealing with miss
    reg [8:0] ring;
    reg [2:0] switch;

        // resetting our counter stuff
    always @(negedge clk) begin
        if(!stall & ~hit & ring[0]) begin
            if(is_read | is_write) begin
                ring    <= 9'b100000000;
                switch  <= 0;
            end
        end
    end

        // moving the missed data into the cache
    always @(posedge clk) begin
        if(!stall & ~ring[0]) begin
            // dealing with WB
            if(~lru & cache[idx][550] | lru & cache[idx][274]) begin
                cache[idx][550] <= lru ? cache[idx][550] : 1'b0;
                cache[idx][274] <= lru ? 1'b0 : cache[idx][274];
            end
            else begin
                switch  <= switch + 1;
                ring    <= {1'b1, ring[8:1]};
                cache[idx][switch * 32 + 276 +:32]  <= lru ? cache[idx][switch * 32 + 276 +:32] : requested_data;
                cache[idx][switch * 32 +:32]        <= lru ? requested_data : cache[idx][switch * 32 +:32];
                if(ring == 9'b111111110) begin  // update the tag, valid and dirty bit only after everything has been written back
                    cache[idx][549:532] <= lru ? cache[idx][549:532] : tag;
                    cache[idx][273:256] <= lru ? tag : cache[idx][273:256];
                    cache[idx][551:550] <= lru ? cache[idx][551:550] : 2'b10;
                    cache[idx][275:274] <= lru ? 2'b10 : cache[idx][275:274];
                end
            end
            // end dealing with WB
        end
    end
    // end dealing with miss

    wire [31:0] modified_data_to_write;

    always @ * begin
        if(!stall) begin
            stop <= ~ring[0] | (j < 1024);
            if(is_read | is_write) begin
                if(~lru & cache[idx][550] | lru & cache[idx][274]) begin
                    is_wb   <= 1;
                    request_addr <= lru ? {cache[idx][273:256], idx, 5'b00000} : {cache[idx][549:532], idx, 5'b00000};
                    wb_data <= lru ? cache[idx][255:0] : cache[idx][531:276];
                end
                else begin
                    is_request   <= ~ring[0];
                    request_addr <= {tag, idx, switch, 2'b00};
                    is_wb        <= 0;
                end
            end
            // assigning write value
            if(is_write) begin
                if(alignment == 2'b00) begin
                    case(data_to_write_size)
                        2'b00: modified_data_to_write <= data_to_write;
                        2'b11: modified_data_to_write <= {data_to_write[23:0], cdata[7:0]};
                        2'b10: modified_data_to_write <= {data_to_write[15:0], cdata[15:0]};
                        2'b01: modified_data_to_write <= {data_to_write[7:0], cdata[23:0]};
                    endcase
                end
                else begin
                    case(data_to_write_size)
                        2'b00: modified_data_to_write <= data_to_write;
                        2'b11: modified_data_to_write <= {cdata[31:24], data_to_write[23:0]};
                        2'b10: modified_data_to_write <= {cdata[31:16], data_to_write[15:0]};
                        2'b01: modified_data_to_write <= {cdata[31:8], data_to_write[7:0]};
                    endcase
                end
            end
            // end dealing with write
        end
    end

    // dealing with flush
    integer j = 1024;
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

    always @(negedge clk or negedge reset) begin
        if(!reset) begin
            for(i = 0; i < 512; i = i + 1) begin
                cache[i][551:550] = 0;
                cache[i][275:274] = 0;
            end
            ring <= 9'b111111111;
        end
        else if(!stall & (ring == 9'b111111111)) begin
            cache[idx][552]     <= c1_valid ? 1 : 0;                  // update lru
            if(is_write) begin
                cache[idx][550] <= c1_valid ? 1 : cache[idx][550];    // update the dirty bit
                cache[idx][274] <= c2_valid ? 1 : cache[idx][274];    // update the dirty bit
                // actually write
                cache[idx][ofst * 32 + 276 +:32] <= c1_valid ? modified_data_to_write : cache[idx][ofst * 32 + 276 +:32];
                cache[idx][ofst * 32 +:32]       <= c2_valid ? modified_data_to_write : cache[idx][ofst * 32 +:32];
            end
            if(is_read) begin
                data <= cdata;
            end
        end
    end

    always @(posedge clk) begin
        `ifdef DC_PRINT
            $display("\t\t\t\tDC Output");
            $display("Data Addr: %x, Data: %x, Stop: %x", data_addr, data, stop);
            $display("Is Read: %x, Is Write: %x, Data to Write: %x, WR Size: %x", is_read, is_write, data_to_write, data_to_write_size);
            $display("Req Addr: %x, Req Data: %x, Is WB: %x", request_addr, requested_data, is_wb);
            $display("Instr: %x, Idx: %x, Tag: %x, Ofst: %x", instr, idx, tag, ofst);
            $display("Cache Val: %x, Cache Data: %x, Cache Tag1: %x, Cache Tag2: %x, Cache Ofst: %d", hit, cdata, cache[idx][549:532], cache[idx][273:256], ofst * 32);
            $display("Switch: %d, Ring: %b, Addr: %b", (switch - 1) * 32, ring, data_addr);
            $display("Cache: %x", cache[idx]);
            $display("\t\t\t\tDC End");
        `endif
    end

endmodule
