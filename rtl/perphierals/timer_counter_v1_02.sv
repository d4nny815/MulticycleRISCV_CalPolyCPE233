`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Ratner Surf Designs
// Engineer: James Ratner
// 
// Create Date: 02/22/2020 04:22:33 PM
// Design Name: 
// Module Name: timer_counter
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description:  The timer & counter (TC) module. When acting as a counter, the 
//        TC counts up until the count is equivalent to the tc_cnt_in value, at
//        which point it clears the internal counter and repeats. The internal
//        counter counts the clock output from the prescaler, which can be
//        divided by a 4-bit value (4 MSBs of the tc_csr input). The counter
//        only operates when the TC is enabled (the LSB of tc_csr). The tc_intr
//        output is a three-cyclc pulse intended to be connected to the 
//        interrupt input of the RISC-V (not one-shot of dbounce required).  
//
//        The output pulse width is parameterized; the default value is 3. 
//
//  instantiation template
//
//  timer_counter   my_tc (
//     .clk        (), 
//     .tc_cnt_in  (),
//     .tc_csr     (),
//     .tc_intr    (),
//     .tc_cnt_out ()  );
// 
// Dependencies: 
// 
// Revision:
// Revision 1.00 - File Created  (02-22-2020)
//          1.01 - parameterized the output pulse width
//          1.02 - removed useless/misleading comments
// 
//////////////////////////////////////////////////////////////////////////////////

module timer_counter(
    input clk,  
    input [31:0] tc_cnt_in,
    input [7:0] tc_csr,
    output tc_intr,
    output [31:0] tc_cnt_out     );
   
    parameter n = 3; 
    logic [n-1:0] pos_pulse_reg;  
   
    logic s_clk_scaled; 
    logic s_pulse; 
    logic [3:0] r_ps_count = 0; 
    logic s_clk_div = 0; 
    logic [31:0] r_counter32b = 0; 
	
	assign tc_cnt_out = r_counter32b;      
    
	//- prescaler (4-bit)
    always_ff @ (posedge clk)
    begin
       if (tc_csr[0]==0) 
         begin
            s_clk_div <= 0;  
            r_ps_count <= 0; 
         end
       else if (r_ps_count == tc_csr[7:4])
         begin
             s_clk_div <= 1'b1; 
             r_ps_count <= 0; 
         end  
       else 
         begin
             s_clk_div <= 0; 
             r_ps_count <= r_ps_count + 1;
         end 
    end 
    
    always_comb
    begin
       if (tc_csr[7:4]==4'b0000) 
          s_clk_scaled = clk; 
       else
          s_clk_scaled = s_clk_div; 
    end
    
    //- the internal 32-bit counter
    always_ff @ (posedge s_clk_scaled)
    begin
       if (tc_csr[0]==0)
       begin
          r_counter32b <= 0;
       end
       else if (r_counter32b == tc_cnt_in) 
       begin
          r_counter32b <= 0;
       end  
       else
       begin
          r_counter32b <= r_counter32b + 1;
       end            
    end
 
    //- send a pulse one clk cycle wide to intr pulse generator
    always_comb
    begin
       if ( (s_clk_div == 1) && (r_counter32b == tc_cnt_in) )
          s_pulse = 1; 
       else 
          s_pulse = 0; 
    end

    //- positive pulse output shift register 
    always_ff @ (posedge clk)    
       pos_pulse_reg <= {pos_pulse_reg[n-2:0], s_pulse}; 
       
    //- final output synthesis of one-shot
    assign tc_intr = |pos_pulse_reg; 
    
endmodule
