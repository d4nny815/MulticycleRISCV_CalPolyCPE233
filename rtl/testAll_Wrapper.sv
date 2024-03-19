`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:   Ratner Surf Designs
// Engineer:  James Ratner 
// 
// Create Date: 03/30/2021 02:46:31 PM
// Design Name: 
// Module Name: OTTER_Wrapper_Testall
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Otter Wrapper: interfaces RISC-V OTTER to basys3 board for 
//              Experiment 6. 
//
// Dependencies: 
// 
// Revision:
// Revision 1.00 - (03-30-2021): created
//          1.01 - (05-14-2021): changed clock divide by 2
//          1.02 - (05-19-2021): changed clock divide by 2 again
//            
//////////////////////////////////////////////////////////////////////////////////

module OTTER_Wrapper(
   input clk,
   input [4:0] buttons,
   input [15:0] switches,
   output logic [15:0] leds,
   output logic [7:0] segs,
   output logic [3:0] an    );
       
   //- INPUT PORT IDS ---------------------------------------------------------
   localparam SWITCHES_PORT_ADDR = 32'h11008000;  // 0x1100_8000
   localparam BUTTONS_PORT_ADDR  = 32'h11008004;  // 0x1100_8004
              
   //- OUTPUT PORT IDS --------------------------------------------------------
   localparam LEDS_PORT_ADDR     = 32'h1100C000;  // 0x1100_C000
   localparam COUNT_ADDR         = 32'h1100C00C;  // 0x1100_C00C
	
   //- Signals for connecting OTTER_MCU to OTTER_wrapper 
   logic s_interrupt;  
   logic s_reset;              
   logic reset;
   logic s_clk;            // divided clock
   logic ss_clk;           // a more divided clock

   logic [31:0] IOBUS_out;
   logic [31:0] IOBUS_in;
   logic [31:0] IOBUS_addr;
   logic IOBUS_wr;
   
   //- register for dev board output devices ---------------------------------
   logic [7:0]  r_segs;   //  register for segments (cathodes)
   logic [15:0] r_leds;   //  register for LEDs
   logic [3:0]  r_an;     //  register for display enables (anodes)
   logic [13:0] r_count;  //  register for the count value

   
   assign s_interrupt = buttons[4];  // for btn(4) connecting to interrupt
   assign s_reset = buttons[3];      // for btn(3) connecting to reset

  //- Instantiate RISC-V OTTER MCU 
  OTTER_MCU  my_otter(
      .RESET       (reset),
      .INTR        (1'b0),
      .CLK         (ss_clk),
//      .CLK         (s_clk),
      .IOBUS_IN    (IOBUS_in),
      .IOBUS_OUT   (IOBUS_out), 
      .IOBUS_ADDR  (IOBUS_addr), 
      .IOBUS_WR    (IOBUS_wr)   );
      
      
  //- Seven-segment display devcide
  univ_sseg my_univ_sseg (
     .cnt1    (r_count), 
     .cnt2    (7'b0000000), 
     .valid   (1'b1), 
     .dp_en   (1'b0), 
     .dp_sel  (2'b0), 
     .mod_sel (2'b10), 
     .sign    (1'b0), 
     .clk     (s_clk), 
     .ssegs   (segs), 
     .disp_en (an)    ); 
   
    // clock divider   
    clk_2n_div_test #(.n(17)) MY_DIV (
      .clockin   (s_clk), 
      .fclk_only (1'b0),          
      .clockout  (ss_clk)   );       
     
   //- Divide clk by 2 
   always @(posedge clk) 
   begin
      if (s_clk == 0)  s_clk <= 1;
      else  s_clk <= 0;
   end
          
   //- Drive dev board output devices with registers 
   always_ff @ (posedge ss_clk)
   begin
      if (IOBUS_wr == 1)
      begin
         case(IOBUS_addr)
            LEDS_PORT_ADDR:   
               r_leds <= IOBUS_out[15:0];    
			   COUNT_ADDR:   
               r_count <= IOBUS_out[13:0];
          endcase
       end
    end
   
    //- MUX to route input devices to I/O Bus
	//-   IOBUS_addr is the select signal to the MUX
   always_comb
   begin
      IOBUS_in=32'b0;
      case(IOBUS_addr)
         SWITCHES_PORT_ADDR: 
		    IOBUS_in[15:0] = switches;
         BUTTONS_PORT_ADDR: 
		    IOBUS_in[4:0] =  buttons;
         default: IOBUS_in=32'b0;
      endcase
   end
	
   //- assign registered outputs to actual outputs 
   assign leds = r_leds; 
   
   typedef enum logic [1:0] {
        INIT,
        STARTUP,
        RUNNING
    } state_t;

    state_t PS, NS;
    
    always_ff @(posedge clk) begin
        if (s_reset) begin
            PS <= INIT;
        end
        else begin
            PS <= NS;
        end
    end


    always_comb begin
        reset = 1'b0;
        case(PS)
            INIT: begin
                reset = 1'b1;
                NS = STARTUP; 
            end
            
            STARTUP: begin
                reset = 1'b1;
                NS = RUNNING;
            end

            RUNNING: begin
                reset = 1'b0;
                NS = RUNNING;
            end
        default: NS = INIT;
        endcase
    end

	
endmodule

