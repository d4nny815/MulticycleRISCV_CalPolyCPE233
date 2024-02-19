`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/27/2023 01:15:15 PM
// Design Name: 
// Module Name: test_pipeline
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module test_pipeline();
    logic clk, rst;
    
    // OTTER_MCU DUT (
    //     .CLK        (clk),
    //     .RESET      (rst),
    //     .INTR       (1'b0),
    //     .IOBUS_IN   (32'b0),
    //     .IOBUS_OUT  (),
    //     .IOBUS_ADDR (), 
    //     .IOBUS_WR   () 
    // );

    logic [15:0] leds;
    logic [4:0] btns;
    OTTER_Wrapper DUT (
        .clk        (clk),
        .buttons    (btns),
        .switches   (16'b0),
        .leds       (leds),
        .segs       (),
        .an         ()
    );


    assign btns[4] = rst;

    always begin
    #10 clk = ~clk; 
    end  


    initial begin
        clk = 0; rst = 0;
        // #5;
        // rst = 1'b1;
        // #6;
        // rst = 1'b0;
        #600;
        $finish;
    end

endmodule
