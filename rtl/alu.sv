`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/17/2023 03:51:47 AM
// Design Name: 
// Module Name: main
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


module ALU (
    input [3:0] alu_fun,
    input [31:0] srcA, 
    input [31:0] srcB, 
    output reg [31:0] result
    );

    typedef enum logic [3:0] {
        ADD = 4'b0000,
        SLL = 4'b0001,
        SLT = 4'b0010,
        SLTU = 4'b0011,
        XOR = 4'b0100,
        SRL = 4'b0101,
        OR = 4'b0110,
        AND = 4'b0111,
        SUB = 4'b1000,
        LUI = 4'b1001,
        SRA = 4'b1101
    } alu_fun_t;
    alu_fun_t ALU_FUN;
    assign ALU_FUN = alu_fun_t'(alu_fun);
    
    
    always_comb begin
        case(ALU_FUN)
            ADD: result = srcA + srcB;                   
            SLL: result = srcA << srcB[4:0];             
            SLT: result = $signed(srcA) < $signed(srcB); 
            SLTU: result = srcA < srcB;                  
            XOR: result = srcA ^ srcB;                    
            SRL: result = srcA >> srcB[4:0];             
            OR: result = srcA | srcB;                    
            AND: result = srcA & srcB;                   
            SUB: result = srcA - srcB;                   
            LUI: result = srcA;                    
            SRA: result = $signed(srcA) >>> srcB[4:0]; 
            default: result = 32'hDEAD_BEEF;
        endcase
    end
endmodule
