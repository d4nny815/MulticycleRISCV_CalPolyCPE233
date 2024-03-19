`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:   Ratner Surf Designs
// Engineer:  James Ratner, Joseph Callenes-Sloan, Paul Hummel, Celina Lazaro
//    
// Create Date: 11/14/2018 02:46:31 PM 
// Design Name:   
// Module Name: OTTER_Wrapper 
// Project Name:  
// Target Devices:  
// Tool Versions: 
// Description: Otter Wrapper: interfaces RISC-V OTTER to basys3 board. 
//
// Dependencies:     
// 
// Revision:
// Revision 1.02 - (02-02-2020): first released version; not fully tested
//          1.03 - (02-06-2020): removed typos, connected output to regs
//          1.04 - (02-08-2020): removed typo for anodes
//          1.05 - (04-01-2020): cleaned up; added comments
//          1.06 - (11-11-2020): added a few more comments & clean-up
//          1.07 - (02-11-2021): fixed error in always block for registers 
//          1.08 - (05-01-2023): fixed indetation and formatting
//            
//////////////////////////////////////////////////////////////////////////////////
 
  module OTTER_Wrapper(   
    input clk,              // 100 MHz clock 
    input [4:0] buttons,  
    input [15:0] switches,   
    output logic [15:0] leds,
    output logic [7:0] segs,  
    output logic [3:0] an  
    ); 
         
    //- INPUT PORT IDS ---------------------------------------------------------
    localparam SWITCHES_PORT_ADDR     = 32'h11008000;  // 0x11008000  
    localparam BUTTONS_PORT_ADDR      = 32'h11008004;  // 0x11008004
                  
    //- OUTPUT PORT IDS --------------------------------------------------------
    localparam LEDS_PORT_ADDR         = 32'h1100C000;  // 0x1100C000 
    localparam SEGS_PORT_ADDR         = 32'h1100C004;  // 0x1100C004
    localparam ANODES_PORT_ADDR       = 32'h1100C008;  // 0x1100C008
     
    //- Signals for connecting OTTER_MCU to OTTER_wrapper 
    logic s_interrupt;  
    logic s_reset;           
    logic CLK_50MHz = 0;            // 50 MHz clock
 
    logic [31:0] IOBUS_out;
    logic [31:0] IOBUS_in;   
    logic [31:0] IOBUS_addr;  
    logic IOBUS_wr;   
        
    //- register for dev board output devices ---------------------------------
    logic [7:0]  r_segs;   //  register for segments (cathodes)
    logic [15:0] r_leds;   //  register for LEDs
    logic [3:0]  r_an;     //  register for display enables (anodes)

    
    assign s_interrupt = buttons[3];  // for btn(4) connecting to interrupt. Bottom button
    assign s_reset = buttons[4];      // for btn(3) connecting to reset

    // Signals for the DeBouncer
    logic db_out;
    // Signals for the One-Shot
    logic pos_pulse_out; 

    //- timer-counter input support
    localparam TMR_CNTR_CNT_OUT  = 32'h11008008;   // 0x1100_8008

    //- timer-counter output support
    localparam TMR_CNTR_CSR_ADDR     = 32'h1100D000;   // 0x1100_D000
    localparam TMR_CNTR_CNT_IN_ADDR  = 32'h1100D004;   // 0x1100_D004

    logic [7:0]  r_tc_csr;    // timer-counter count input
    logic [31:0] r_tc_cnt_in; // timer-counter count input
    logic [31:0] s_tc_cnt_out; 
    logic s_tc_intr; 

    DBounce #(.n(3)) my_dbounce( 
        .clk (CLK_50MHz),
        .sig_in (s_interrupt),
        .DB_out (db_out)
    );

    one_shot_bdir #(.n(3)) my_oneshot (
        .clk (CLK_50MHz),
        .sig_in (db_out),
        .pos_pulse_out (pos_pulse_out),
        .neg_pulse_out ()  
    );
    
    // instantiate the timer-counter module   
    timer_counter #(.n(3))  my_tc (
        .clk        (CLK_50MHz), 
        .tc_cnt_in  (r_tc_cnt_in), 
        .tc_csr     (r_tc_csr),
        .tc_intr    (s_tc_intr), 
        .tc_cnt_out (s_tc_cnt_out)  
    );
     
    //- Instantiate RISC-V OTTER MCU 
    OTTER_MCU OTTER_CORE(
        .CLK        (CLK_50MHz),   
        .RESET      (s_reset),  
        .INTR       (s_tc_intr | pos_pulse_out),
        .IOBUS_IN   (IOBUS_in),   
        .IOBUS_OUT  (IOBUS_out),  
        .IOBUS_ADDR (IOBUS_addr),
        .IOBUS_WR   (IOBUS_wr)  
    ); 
    
    //- Divide clk by 2 
    always_ff @ (posedge clk)
        CLK_50MHz <= ~CLK_50MHz;
   
    //- Drive dev board output devices with registers 
    always_ff @ (posedge CLK_50MHz) begin
        if (IOBUS_wr == 1) begin 
            case(IOBUS_addr) 
                LEDS_PORT_ADDR:   r_leds <= IOBUS_out[15:0];     
                SEGS_PORT_ADDR:   r_segs <= IOBUS_out[7:0];
                ANODES_PORT_ADDR: r_an   <= IOBUS_out[3:0];
                TMR_CNTR_CSR_ADDR:    r_tc_csr  <= IOBUS_out[7:0];
                TMR_CNTR_CNT_IN_ADDR: r_tc_cnt_in <= IOBUS_out[31:0];          
            endcase
         end  
     end
     
     
    //- MUX to route input devices to I/O Bus
    //-   IOBUS_addr is the select signal to the MUX
    always_comb begin
        IOBUS_in = 32'b0; 
        case(IOBUS_addr)
            BUTTONS_PORT_ADDR:      IOBUS_in[4:0] = buttons;
            TMR_CNTR_CNT_OUT:       IOBUS_in[31:0] = s_tc_cnt_out;
            SWITCHES_PORT_ADDR:     IOBUS_in[15:0] = switches;
            default:                IOBUS_in = 32'b0;
        endcase
    end
    
    //- assign registered outputs to actual outputs 
    assign leds = r_leds;  
    assign segs = r_segs; 
    assign an = r_an;     
endmodule

