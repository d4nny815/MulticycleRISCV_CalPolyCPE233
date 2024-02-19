`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Ratner Surf Designs
// Engineer: James Ratner
// 
// Create Date: 01/29/2019 04:56:13 PM
// Design Name: 
// Module Name: CU_Decoder
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies:
// 
// CU_DCDR my_cu_dcdr(
//   .br_eq     (), 
//   .br_lt     (), 
//   .br_ltu    (),
//   .opcode    (),    //-  ir[6:0]
//   .func7     (),    //-  ir[30]
//   .func3     (),    //-  ir[14:12] 
//   .alu_fun   (),
//   .pcSource  (),
//   .alu_srcA  (),
//   .alu_srcB  (), 
//   .rf_wr_sel ()   );
//
// 
// Revision:
// Revision 1.00 - File Created (02-01-2020) - from Paul, Joseph, & Celina
//          1.01 - (02-08-2020) - removed unneeded else's; fixed assignments
//          1.02 - (02-25-2020) - made all assignments blocking
//          1.03 - (05-12-2020) - reduced func7 to one bit
//          1.04 - (05-31-2020) - removed misleading code
//          1.05 - (05-01-2023) - reindent and fix formatting
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

// cu_dcdr.v
// Members: Daniel Gutierrez
// Description:
// 
// 

module CU_DCDR(
    input br_eq, 
    input br_lt, 
    input br_ltu,
    input [6:0] opcode,   //-  ir[6:0]
    input func7,          //-  ir[30]
    input [2:0] func3,    //-  ir[14:12]
    input int_taken, 
    output logic [3:0] alu_fun,
    output logic [2:0] pcSource,
    output logic [1:0] alu_srcA,
    output logic [2:0] alu_srcB, 
    output logic [1:0] rf_wr_sel   );
    
    //- datatypes for RISC-V opcode types
    typedef enum logic [6:0] {
        LUI    = 7'b0110111,
        AUIPC  = 7'b0010111,
        JAL    = 7'b1101111,
        JALR   = 7'b1100111,
        BRANCH = 7'b1100011,
        LOAD   = 7'b0000011,
        STORE  = 7'b0100011,
        OP_IMM = 7'b0010011,
        OP_RG3 = 7'b0110011,
        SYS    = 7'b1110011
    } opcode_t;
    opcode_t OPCODE; //- define variable of new opcode type
    assign OPCODE = opcode_t'(opcode); //- Cast input enum 

    //- datatype for func3Symbols tied to values
    typedef enum logic [2:0] {
        //BRANCH labels
        BEQ = 3'b000,
        BNE = 3'b001,
        BLT = 3'b100,
        BGE = 3'b101,
        BLTU = 3'b110,
        BGEU = 3'b111
    } func3_t;    
    func3_t FUNC3; //- define variable of new opcode type
    assign FUNC3 = func3_t'(func3); //- Cast input enum 
       
    always_comb begin 
        //- schedule all values to avoid latch
        pcSource = 3'b000;  alu_srcB = 3'b000;    rf_wr_sel = 2'b11; 
        alu_srcA = 2'b00;   alu_fun  = 4'b0000;
        
        case(OPCODE)
//            pcSource = 3'b000;          // PC + 4
//            alu_srcA = 2'b00;           // rs1
//            rf_wr_sel = 2'b11;      // ALU output
            LUI: begin
                alu_fun = 4'b1001;      // lui instruction
                alu_srcA = 2'b01;        // U-type 
                alu_srcB = 3'b000;       // rs2
            end

            AUIPC: begin
                alu_fun = 4'b0000;      // add
                alu_srcA = 2'b01;        // U-type 
                alu_srcB = 3'b011;       // PC
            end
            
            JAL: begin
                alu_fun = 4'b0000;      // Don't care
                alu_srcB = 3'b000;       // rs2
                pcSource = 3'b011;       // jal
                rf_wr_sel = 2'b00;      // PC + 4 output to register file
            end

            JALR: begin
                alu_fun = 4'b0000;      // add
                alu_srcB = 3'b000;       // I-type
                pcSource = 3'b001;       // jalr
                rf_wr_sel = 2'b00;      // PC + 4 output to register file
            end

            BRANCH: begin
                if (FUNC3 == 3'b000 && br_eq == 1)
                    pcSource = 3'b010;           // beq
                else if (FUNC3 == 3'b001 && br_eq == 0)
                    pcSource = 3'b010;           // bne
                else if (FUNC3 == 3'b100 && br_lt == 1)
                    pcSource = 3'b010;           // blt
                else if (FUNC3 == 3'b101 && br_lt == 0)
                    pcSource = 3'b010;           // bge
                else if (FUNC3 == 3'b110 && br_ltu == 1)
                    pcSource = 3'b010;           // bltu
                else if (FUNC3 == 3'b111 && br_ltu == 0)
                    pcSource = 3'b010;           // bgeu
                else
                    pcSource = 3'b000;           // PC + 4
            end
            
            LOAD: begin
                alu_srcB = 3'b001;       // I-type
                pcSource = 3'b000;       // PC + 4
                rf_wr_sel = 2'b10;      // RAM_DOUT2 output
                alu_fun = 4'b0000;      // Don't care
            end
            
            STORE: begin
                alu_srcB = 3'b010;       // S-type
                rf_wr_sel = 2'b00;      // PC + 4
                case(FUNC3) // TODO: ask about sign extending
                    3'b000: alu_fun = 4'b0000;  // SB
                    3'b001: alu_fun = 4'b0000;  // SH
                    3'b010: alu_fun = 4'b0000;  // SW
                    default: alu_fun = 4'b0000;  // Don't care
                endcase
            end
            
            OP_IMM: begin
                alu_srcB = 3'b001;               // I-type
                alu_fun = func3 == 3'b101 ? {func7, func3} : {1'b0, func3};
            end

            OP_RG3: begin
                alu_srcB = 3'b000;               // rs2
                alu_fun = {func7, func3};
            end

            SYS: begin
                case(func3)
                3'b001: begin                   // csrrw
                    alu_fun = 4'b1001;            // copy rs1 to rd
                    alu_srcA = 2'b00;             // rs1
                    rf_wr_sel = 2'b01;            // csr_RD
                end
                3'b011: begin                   // csrrc
                    alu_fun = 4'b0111;            // and
                    alu_srcA = 2'b01;             // ~rs1
                    alu_srcB = 3'b100;            // csr_RD
                    rf_wr_sel = 2'b01;            // csr_RD to rd
                end
                3'b010: begin                       // csrrs
                    alu_fun = 4'b0110;            // or
                    alu_srcA = 2'b00;             // rs1
                    alu_srcB = 3'b100;            // csr_RD
                    rf_wr_sel = 2'b01;            // csr_RD to rd
                end
                3'b000: begin                       // mret
                    pcSource = 3'b101;            // mepc
                end
                endcase
            end

            default: begin
                pcSource = 3'b000; 
                alu_srcB = 3'b000; 
                rf_wr_sel = 2'b00; 
                alu_srcA = 2'b00; 
                alu_fun = 4'b0000;
            end
        endcase
        if (int_taken == 1)                 // interrupt occurs
            pcSource = 3'b100;              // mtvec into PC
    end
endmodule