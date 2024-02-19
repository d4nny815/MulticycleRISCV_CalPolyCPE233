`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  Danny Gutierrez
// 
// Create Date: 05/31/2023 11:17:48 AM
// Design Name: 
// Module Name: riscv_mcu
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
//  This is a RISC-V MCU that implements the RV32I ISA. 
// 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module OTTER_MCU(
    input CLK,    
    input INTR,       
    input RESET,      
    input [31:0] IOBUS_IN,    
    output [31:0] IOBUS_OUT,    
    output [31:0] IOBUS_ADDR,
    output logic IOBUS_WR
);

    // PC wires
    logic [31:0] pc, mux_pc;

    // memory wires
    logic [31:0] ir, dout2;

    // register file wires
    logic [31:0] mux_regfile, rs1, rs2;

    // immediate generation wires
    logic [31:0] u_type, i_type, s_type, b_type, j_type;

    // branch address generation wires
    logic [31:0] jalr, jal, branch;

    // ALU wires
    logic [31:0] alu_out, mux_srcA, mux_srcB;

    // branch condition generation wires
    logic br_eq, br_lt, br_ltu;

    // control unit FSM wires
    logic pcWrite, regWrite, memWE2, memRDEN1, memRDEN2, reset;

    // control unit DCDR wires
    logic [1:0] rf_wr_sel, alu_srcA;
    logic [2:0] alu_srcB, pcSource;
    logic [3:0] alu_fun;

    // CSR wires
    logic [31:0] mepc, mtvec, csr_RD;
    logic mret_exec, int_taken, csr_WE, csr_mstatus_mie;
    
    CSR  my_csr (
        .CLK        (CLK),
        .RST        (reset),
        .MRET_EXEC  (mret_exec),
        .INT_TAKEN  (int_taken),
        .ADDR       (ir[31:20]),
        .PC         (pc),
        .WD         (alu_out),
        .WR_EN      (csr_WE), 
        .RD         (csr_RD),
        .CSR_MEPC   (mepc),  
        .CSR_MTVEC  (mtvec), 
        .CSR_MSTATUS_MIE (csr_mstatus_mie)    
    ); 

    // PC MUX
    always_comb begin
        case(pcSource)
            3'b000: mux_pc = pc + 4;
            3'b001: mux_pc = jalr;
            3'b010: mux_pc = branch;
            3'b011: mux_pc = jal;
            3'b100: mux_pc = mtvec;
            3'b101: mux_pc = mepc;
            default: mux_pc = 0;

        endcase
    end
    
    reg_nb_sclr #(.n(32)) MY_PC (
        .data_in  (mux_pc), 
        .ld       (pcWrite),
        .clk      (CLK), 
        .clr      (reset),    // synchronous reset
        .data_out (pc)        // PC 
    );

    Memory OTTER_MEMORY (
        .MEM_CLK   (CLK),
        .MEM_RDEN1 (memRDEN1), 
        .MEM_RDEN2 (memRDEN2), 
        .MEM_WE2   (memWE2),
        .MEM_ADDR1 (pc[15:2]),  // remove the LSB
        .MEM_ADDR2 (alu_out),
        .MEM_DIN2  (rs2),  
        .MEM_SIZE  (ir[13:12]),
        .MEM_SIGN  (ir[14]),
        .IO_IN     (IOBUS_IN),
        .IO_WR     (IOBUS_WR),
        .MEM_DOUT1 (ir),
        .MEM_DOUT2 (dout2)  
    );
    
    always_comb begin
        case(rf_wr_sel)
            2'b00: mux_regfile = pc + 4;
            2'b01: mux_regfile = csr_RD;
            2'b10: mux_regfile = dout2;
            2'b11: mux_regfile = alu_out;
        endcase
    end
    
    RegFile my_regfile (
        .wd       (mux_regfile),
        .clk      (CLK), 
        .en       (regWrite),
        .adr1     (ir[19:15]),
        .adr2     (ir[24:20]),
        .wa       (ir[11:7]),
        .rs1      (rs1), 
        .rs2      (rs2)  
    );
    assign IOBUS_OUT = rs2; 


    // IMMED GEN
    always_comb begin
        i_type = { {21{ir[31]}}, ir[30:25], ir[24:20]};
        s_type = { {21{ir[31]}}, ir[30:25], ir[11:7]};
        b_type = { {20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0};
        u_type = {ir[31:12], 12'b0};
        j_type = { {12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0};

        jalr   = rs1 + i_type;
        jal    = pc + j_type;
        branch = pc + b_type;
    end

    // ALU MUXs
    always_comb begin
        case(alu_srcA)
            2'b00: mux_srcA = rs1;
            2'b01: mux_srcA = u_type;
            2'b10: mux_srcA = ~rs1;
            default: mux_srcA = 32'hdeadbeef;
        endcase

        case(alu_srcB)
            3'b000: mux_srcB = rs2;
            3'b001: mux_srcB = i_type;
            3'b010: mux_srcB = s_type;
            3'b011: mux_srcB = pc;
            3'b100: mux_srcB = csr_RD;
            default: mux_srcB = 32'hdeadbeef;
        endcase
    end

    ALU my_alu (
        .alu_fun  (alu_fun),
        .srcA     (mux_srcA), 
        .srcB     (mux_srcB), 
        .result   (alu_out)
    );
    assign IOBUS_ADDR = alu_out; 

    always_comb begin
        br_eq  = (rs1 == rs2);
        br_lt  = ($signed(rs1) < $signed(rs2));
        br_ltu = (rs1 < rs2);
    end

    CU_FSM my_fsm (
       .intr      (INTR  && csr_mstatus_mie),
       .clk       (CLK),
       .RST       (RESET),
       .opcode    (ir[6:0]),    // ir[6:0]
       .func3     (ir[14:12]),  // ir[14:12]
       .pcWrite   (pcWrite),
       .regWrite  (regWrite),
       .memWE2    (memWE2),
       .memRDEN1  (memRDEN1),
       .memRDEN2  (memRDEN2),
       .reset     (reset),
       .csr_WE    (csr_WE),
       .int_taken (int_taken),
       .mret_exec (mret_exec)   
    );
    
    CU_DCDR my_cu_dcdr (
        .br_eq     (br_eq), 
        .br_lt     (br_lt), 
        .br_ltu    (br_ltu),
        .opcode    (ir[6:0]),   //-  ir[6:0]
        .func7     (ir[30]),    //-  ir[30]
        .func3     (ir[14:12]), //-  ir[14:12]
        .int_taken (int_taken), 
        .alu_fun   (alu_fun),
        .pcSource  (pcSource),
        .alu_srcA  (alu_srcA),
        .alu_srcB  (alu_srcB), 
        .rf_wr_sel (rf_wr_sel)   
    );
endmodule
