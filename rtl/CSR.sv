`timescale 1ns / 1ps
///////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 02/02/2019 03:01:38 PM
// Design Name: 
// Module Name: CSR
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: RISC-V OTTER Control & Status Register model
// 
//  instantiation template
//
//  CSR  my_csr (
//    .CLK        (),
//    .RST        (),
//    .MRET_EXEC  (),
//    .INT_TAKEN  (),
//    .ADDR       (),
//    .PC         (),
//    .WD         (),
//    .WR_EN      (), 
//    .RD         (),
//    .CSR_MEPC   (),  
//    .CSR_MTVEC  (), 
//    .CSR_MSTATUS_MIE ()    ); 
// 
// 
// Dependencies: 
// 
// Revision:
// Revision 1.00 - File Created (02-02/2019) 
//          1.01 - (02-10-2020) cleaned, 
//                     added instantiation template (james) 
//          1.02 - (03-24-2020) fixed typo with MIE 
//          1.03 - (07-12-2022) added support for MSTATUS register 
//                     including MRET_EXEC input and changing 
//                     CSR_MIE input to CSR_MSTATUS_MIE
// Additional Comments:
// 
///////////////////////////////////////////////////////////////////////////
module CSR(
    input CLK,
    input RST,
    input MRET_EXEC,        //- indicates MCU executing mret instruction
    input INT_TAKEN,        //- indicates FSM is in interrupt cycle
    input [11:0] ADDR,      //- CSR register address
    input [31:0] PC,        //- program counter value
    input [31:0] WD,        //- data to be written to CSR
    input WR_EN,            //- write enable signal for CSR writes 
    output logic [31:0] RD,              //- register data read from CSR
    output logic [31:0] CSR_MEPC=0,      //- return from interrupt addr
    output logic [31:0] CSR_MTVEC=0,     //- interrupt vector address  
    output logic CSR_MSTATUS_MIE     //- interrupt enable bit in CSR[mstatus]
    ); 

    
   //- internal register for CSR not in interface 
    logic [31:0] CSR_MSTATUS; 
    
    //- CSR ADDResses
    typedef enum logic [11:0] {       
        MSTATUS   = 12'h300,
        MTVEC     = 12'h305,
        MEPC      = 12'h341
    } csr_t;
    
    localparam MIE  = 3;    //- bit3 in mstatus
    localparam MPIE = 7;    //- bit7 in mstatus


    always_ff @ (posedge CLK)
    begin
        //- clear registers on reset
		if (RST) begin
            CSR_MTVEC   <= 32'd0;
            CSR_MEPC    <= 32'd0;
            CSR_MSTATUS <= 32'd0;           
      end
       
	   //- write to registers 
	   if (WR_EN)
         case(ADDR)
            MSTATUS: CSR_MSTATUS <= WD;  //- interrupt enable
            MTVEC:   CSR_MTVEC   <= WD;  //- vector addr
            MEPC:    CSR_MEPC    <= WD;  //- return addr
         endcase
            
      //- Stuff to do when MCU acts on interrupt 
		if(INT_TAKEN)
         begin
            CSR_MEPC <= PC;                        //- save PC in MEPC
		        CSR_MSTATUS[MPIE] <= CSR_MSTATUS[MIE]; //- copy MIE to MPIE
            CSR_MSTATUS[MIE] <= 1'b0;              //- clear MIE
         end     

      //- stuff to do when MCU executes a ret pseudoinstruction 
		if(MRET_EXEC)
            begin
		        CSR_MSTATUS[MIE] <= CSR_MSTATUS[MPIE];  //- copy MPIE to MIE 
            end
        
    end
    
    //- export MIE bit of CSR[mstatus]
   assign CSR_MSTATUS_MIE = CSR_MSTATUS[MIE]; 
    
    //- read from registers
	always_comb
        case(ADDR)
            MTVEC:   RD = CSR_MTVEC;
            MEPC:    RD = CSR_MEPC;
            MSTATUS: RD = CSR_MSTATUS;            
            default: RD = 32'd0;
        endcase
    
endmodule