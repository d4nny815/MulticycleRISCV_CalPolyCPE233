`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Ratner Surf Designs
// Engineer:  James Ratner
// 
// Create Date: 02/22/2020 10:57:44 AM
// Design Name: 
// Module Name: DBounce
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: A debouncer; works for both directions. The time that the 
//              module waits for stability is parameterized. The default
//              is low (8) to facilitate testing. 
// 
// Dependencies: 
// 
//  instantiation template:
//
//  DBounce #(.n(5)) my_dbounce(
//    .clk    (),
//    .sig_in (),
//    .DB_out ()   );
//
// Revision:
// Revision 1.00 - (02-22-2020): file created
//          1.01 - (05-15-2020): made _ff assignments to non-blocking
//
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module DBounce(
    input clk,
    input sig_in,
    output DB_out
    );
    
    parameter n = 8;  //- default data-width 
        
    logic  r_dff1; 
    logic  r_dff2; 
    logic  r_buff; 
    logic  [n-1:0] r_count; 
    logic  s_reset; 
    
    //- two input flip-flops
    always_ff @ (posedge clk)
    begin
       r_dff1 <= sig_in; 
       r_dff2 <= r_dff1; 
    end; 
    
    //- counter reset when input flip-flops differ
    assign s_reset = r_dff1 ^ r_dff2; 
    
    // output flip-flop (buffer)    
    always_ff @ (posedge clk)
    begin
       if (r_count[n-1] == 1) 
          r_buff <= r_dff2; 
    end 
    
    //- synthesize count enable 
    assign s_cnt_en = ~r_count[n-1];     

    //- counter register
    always_ff @ (posedge clk)
    begin
       if (s_reset == 1) 
          r_count <= 0; 
       else if (s_cnt_en == 1)
          r_count <= r_count + 1'b1;
    end   
        
    assign DB_out = r_buff;  

endmodule
