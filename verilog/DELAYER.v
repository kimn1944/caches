/*
File: CACHE.v
Author: Nikita Kim
Email: kimn1944@gmail.com
Date: 11/8/19
*/

`include "config.v"

module DELAYER
    #(parameter DELAY = 9)
    (input clk,
    input reset,
    input stall,

    input delay,

    output stop);
    // end arguments

    integer i;
    reg [DELAY - 1:0] ring;

    always @ (posedge clk or negedge reset) begin
        if(!reset) begin
            for(i = 0; i < DELAY; i = i + 1) begin
                ring[i] <= 1;
            end
            stop <= 0;
        end
        else if(!stall & !stop & delay) begin
            ring[DELAY - 1] <= 1'b1;
            stop <= 1;
            for(i = 0; i < DELAY - 1; i = i + 1) begin
                ring[i] <= 1'b0;
            end
        end
        else if(!stall) begin
            ring <= {1'b1, ring[DELAY - 1:1]};
            stop <= ~ring[0];
        end
        `ifdef DELAYER_PRINT
            $display("\t\t\t\tDELAYER Output");
            $display("Delay: %x", delay);
            $display("Stop: %x, Ring: %b", stop, ring);
            $display("\t\t\t\tDELAYER End");
        `endif
    end

endmodule
