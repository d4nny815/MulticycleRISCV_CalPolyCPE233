`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Ratner Surf Designs
// Engineer:  James Ratner
// 
// Create Date: 02/22/2020 10:57:44 AM
// Design Name: 
// Module Name: One-shot
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: A one_shot; has neg and pos outputs. The parameter
//              value determines the width of the output pulse.  
// 
// Dependencies: 
// 
//  instantiation template:
//
//  one_shot_bdir  #(.n(3)) my_oneshot (
//    .clk           (),
//    .sig_in        (),
//    .pos_pulse_out (), 
//    .neg_pulse_out ()  ); 
//
// Revision:
// Revision 1.00 - (02-22-2020) File Created 
//          1.01 - (02-25-2020) parameterized the output pulse widths
//          1.02 - (05-31-2020) fixed typo in instantiation template
//
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module one_shot_bdir (
    input clk,
    input sig_in,
    output pos_pulse_out, 
    output neg_pulse_out
    ); 
   
    parameter n = 8;
    
    logic [n-1:0] pos_pulse_reg;  
    logic [n-1:0] neg_pulse_reg;  
    
    logic  r_dff1; 
    logic  r_dff2;

    logic  s_ret_in; 
    logic  s_fet_in; 
     
    //- two input flip-flops
    always_ff @ (posedge clk)
    begin
       r_dff1 <= sig_in; 
       r_dff2 <= r_dff1; 
    end; 
    
    //- detects difference in input 
    assign sig_change = r_dff1 ^ r_dff2; 
    
    //- ret & fet pulse starts
    assign s_ret_in = r_dff1 & sig_change; 
    assign s_fet_in = r_dff2 & sig_change; 
    
    //- positive pulse output
    always_ff @ (posedge clk)    
       pos_pulse_reg <= {pos_pulse_reg[n-2:0], s_ret_in}; 
        
    //- negative pulse output
    always_ff @ (posedge clk)
       neg_pulse_reg <= {neg_pulse_reg[n-2:0], s_fet_in}; 
       
    //- final output synthesis of one-shots
    assign pos_pulse_out = |pos_pulse_reg; 
    assign neg_pulse_out = ~(|neg_pulse_reg); 
      
endmodule
