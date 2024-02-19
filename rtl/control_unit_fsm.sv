`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  Ratner Surf Designs
// Engineer: James Ratner
//
// Create Date: 01/07/2020 09:12:54 PM
// Design Name:
// Module Name: top_level
// Project Name:
// Target Devices:
// Tool Versions:
// Description:  ate/Starter File for RISC-V OTTER
//
//     //- instantiation template
//     CU_FSM my_fsm(
//        .intr     (xxxx),
//        .clk      (xxxx),
//        .RST      (xxxx),
//        .opcode   (xxxx),   // ir[6:0]
//        .pcWrite  (xxxx),
//        .regWrite (xxxx),
//        .memWE2   (xxxx),
//        .memRDEN1 (xxxx),
//        .memRDEN2 (xxxx),
//        .reset    (xxxx)   );
//
// Dependencies:
//
// Revision:
// Revision 1.00 - File Created - 02-01-2020 (from other people's files)
//          1.01 - (02-08-2020) switched states to enum type
//          1.02 - (02-25-2020) made PS assignment blocking
//                              made rst output asynchronous
//          1.03 - (04-24-2020) added "init" state to FSM
//                              changed rst to reset
//          1.04 - (04-29-2020) removed typos to allow synthesis
//          1.05 - (10-14-2020) fixed instantiation comment (thanks AF)
//          1.06 - (12-10-2020) cleared most outputs, added commentes
//          1.07 - (05-01-2023) fixed indentation and formatting
//
//////////////////////////////////////////////////////////////////////////////////

// cu_fsm.v
// Members: Daniel Gutierrez
// Description:
// 
// 

module CU_FSM(
    input intr,
    input clk,
    input RST,
    input [6:0] opcode,     // ir[6:0]
    input [2:0] func3,      // ir[14:12]
    output logic pcWrite,
    output logic regWrite,
    output logic memWE2,
    output logic memRDEN1,
    output logic memRDEN2,
    output logic reset, 
    output logic csr_WE,
    output logic int_taken,
    output logic mret_exec
);

    typedef  enum logic [2:0] {
       st_INIT,
       st_FET,
       st_EX,
       st_WB,
       st_INTR
    }  state_type;
    state_type  NS,PS;

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
    opcode_t OPCODE;    //- symbolic names for instruction opcodes
    assign OPCODE = opcode_t'(opcode); //- Cast input as enum

    //- state registers (PS)
    always @ (posedge clk) begin
        if (RST == 1)
            PS <= st_INIT;
        else
            PS <= NS;
    end

    always_comb begin
        //- schedule all outputs to avoid latch
        NS = st_INIT;
        pcWrite = 1'b0;    regWrite = 1'b0;    reset = 1'b0;
        memWE2 = 1'b0;     memRDEN1 = 1'b0;    memRDEN2 = 1'b0;
        csr_WE = 1'b0;     int_taken = 1'b0;   mret_exec = 1'b0;
        
        case (PS)
            st_INIT: begin  //waiting state
                reset = 1'b1;
                pcWrite = 1'b0;
                regWrite = 1'b0;
                memWE2 = 1'b0;
                memRDEN1 = 1'b0;
                memRDEN2 = 1'b0;
                csr_WE = 1'b0;
                int_taken = 1'b0;
                mret_exec = 1'b0;
                NS = st_FET;
            end

            st_FET: begin   //waiting state
                pcWrite = 1'b0;         // Don't increment PC
                regWrite = 1'b0;        // No write to register file
                memWE2 = 1'b0;          // No write to memory
                memRDEN1 = 1'b1;        // read for instruction
                memRDEN2 = 1'b0;        // No read from memory
                reset = 1'b0;           // No reset
                csr_WE = 1'b0;          // No write to CSR
                int_taken = 1'b0;       // No interrupt taken
                mret_exec = 1'b0;       // No mret executed
                NS = st_EX;
            end

            st_EX: begin    //decode + execute
                pcWrite = 1'b1;         // PC + 4, when in EX cycle except for LOAD
                reset = 1'b0;           // No reset, when in EX cycle
                memRDEN1 = 1'b0;        // No read for instruction, when in EX cycle
                csr_WE = 1'b0;          // No write to CSR, when in EX cycle
                int_taken = 1'b0;       // No interrupt taken, when in EX cycle
                mret_exec = 1'b0;       // No mret executed, when in EX cycle
                regWrite = 1'b1;        // write to register file
                memWE2 = 1'b0;           // No write to memory
                memRDEN2 = 1'b0;         // No read from memory

                case (OPCODE)
                    LUI: begin 
                        NS = st_FET;
                    end

                    AUIPC: begin 
                        NS = st_FET;
                    end

                    JAL: begin
                        NS = st_FET;
                    end

                    JALR: begin
                        NS = st_FET;
                    end

                    BRANCH: begin
                        regWrite = 1'b0;        // No write to register file
                        NS = st_FET;
                    end

                    LOAD: begin
                        pcWrite = 1'b0;         // Don't increment PC
                        regWrite = 1'b0;        // No write to register file 
                        memRDEN2 = 1'b1;        // read from memory
                        NS = st_WB;
                    end

                    STORE: begin
                        regWrite = 1'b0;        // No write to register file
                        memWE2 = 1'b1;          // write to memory
                        NS = st_FET;
                    end

                    OP_IMM: begin // immediate operations
                        NS = st_FET;
                    end

                    OP_RG3: begin // register operations
                        NS = st_FET;
                    end

                    SYS: begin
                        reset = 1'b0;           // No reset
                        case(func3)
                        3'b000: begin           //mret
                            regWrite = 1'b0;    // No write to register file
                            csr_WE = 1'b0;      // No write to CSR
                            mret_exec = 1'b1;   // mret executed                       
                        end
                        default: begin          // csrrw, csrrs, csrrc
                            regWrite = 1'b1;    // write to register file
                            csr_WE = 1'b1;      // write to CSR
                            mret_exec = 1'b0;   // No mret executed
                        end
                        endcase
                    end

                    default: begin
                        NS = st_FET;
                    end
                endcase
                if (intr == 1 && (OPCODE != LOAD))
                    NS = st_INTR;
            end

            st_WB: begin
                pcWrite = 1'b1;
                regWrite = 1'b1;
                memWE2 = 1'b0;
                memRDEN1 = 1'b0;
                memRDEN2 = 1'b0;
                reset = 1'b0;
                if (intr == 1)
                    NS = st_INTR;
                else 
                    NS = st_FET;
            end

            st_INTR: begin 
                pcWrite = 1'b1;                 // load ISR into PC
                regWrite = 1'b0;                // No write to register file
                memWE2 = 1'b0;                  // No write to memory
                memRDEN1 = 1'b0;                // No read for instruction
                memRDEN2 = 1'b0;                // No read from memory
                reset = 1'b0;                   // No reset
                csr_WE = 1'b0;                  // No write to CSR
                int_taken = 1'b1;               // interrupt taken
                mret_exec = 1'b0;               // No mret executed
                NS = st_FET;
            end

            default: NS = st_INIT;
        endcase //- case statement for FSM states
    end
endmodule
