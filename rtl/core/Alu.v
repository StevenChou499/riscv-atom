////////////////////////////////////////////////////////////////////   
//  File        : Alu.v
//  Author      : Saurabh Singh (saurabh.s99100@gmail.com)
//  Description : Arithmetic and logic unit for Atom core, following 
//      Operations are built into the alu:
//          -   Addition
//          -   Subtraction
//          -   Bitwise XOR
//          -   Bitwise OR,
//          -   Bitwise AND,
//          -   Logical Shift Left (Single Cycle)
//          -   Logical Shift Right (Single Cycle)
//          -   Arthmetic Shift Right (Single Cycle)
////////////////////////////////////////////////////////////////////

`include "../Timescale.vh"
`include "Defs.vh"

`default_nettype none

module Alu
(
    input   wire    [31:0]  a_i,
    input   wire    [31:0]  b_i,
    input   wire    [3:0]   sel_i,

    output  reg     [31:0]  result_o
);

    wire sel_add    = (sel_i == `ALU_FUNC_ADD);
    wire sel_sub    = (sel_i == `ALU_FUNC_SUB);
    wire sel_xor    = (sel_i == `ALU_FUNC_XOR);
    wire sel_or     = (sel_i == `ALU_FUNC_OR);
    wire sel_and    = (sel_i == `ALU_FUNC_AND);
    wire sel_sll    = (sel_i == `ALU_FUNC_SLL);
    wire sel_srl    = (sel_i == `ALU_FUNC_SRL);
    wire sel_sra    = (sel_i == `ALU_FUNC_SRA);
    wire sel_mul    = (sel_i == `ALU_FUNC_MUL);
    wire sel_mulh   = (sel_i == `ALU_FUNC_MULH);
    wire sel_mulhu  = (sel_i == `ALU_FUNC_MULHU);
    wire sel_mulhsu = (sel_i == `ALU_FUNC_MULHSU);
    wire sel_div    = (sel_i == `ALU_FUNC_DIV);
    wire sel_divu   = (sel_i == `ALU_FUNC_DIVU);
    wire sel_rem    = (sel_i == `ALU_FUNC_REM);
    wire sel_remu   = (sel_i == `ALU_FUNC_REMU);

    // Result of arithmetic calculations (ADD/SUB)
    wire [31:0] arith_result = a_i + (sel_sub ? ((~b_i)+1) : b_i);

    // Extend a_i and b_i to 64-bits long for multiplication
    reg [63:0] extend_a_i;
    reg [63:0] extend_b_i;
    
    always @(*) begin
        if (sel_mul | sel_mulh) begin
            extend_a_i[63:0]  = {{32{a_i[31]}}, a_i};
            extend_b_i[63:0]  = {{32{b_i[31]}}, b_i};
        end
        else if (sel_mulhu) begin
            extend_a_i[63:0]  = {32'd0, a_i};
            extend_b_i[63:0]  = {32'd0, b_i};
        end
        else begin // if (sel_mulhsu)
            extend_a_i[63:0]  = {{32{a_i[31]}}, a_i};
            extend_b_i[63:0]  = {32'd0, b_i};
        end
    end

    wire [63:0] full_mul_result = extend_a_i * extend_b_i;
    reg [31:0] final_mul_result;

    always @(*) begin
        if (sel_mul) begin
            final_mul_result[31:0] = full_mul_result[31:0];
        end
        else begin // sel_mulh, sel_mulhu, sel_mulhsu
            final_mul_result[31:0] = full_mul_result[63:32];
        end
    end

    wire sign_a = a_i[31];
    wire sign_b = b_i[31];

    reg [31:0] tmp_div_a;
    reg [31:0] tmp_div_b;

    always @(*) begin
        tmp_div_a[31:0] = a_i;
        tmp_div_b[31:0] = b_i;
        if (sign_a & (sel_div | sel_rem)) begin
            tmp_div_a[31:0] = ~{a_i} + 32'd1;
        end
        if (sign_b & (sel_div | sel_rem)) begin
            tmp_div_b[31:0] = ~{b_i} + 32'd1;
        end
    end

    wire [31:0] sign_div_result = tmp_div_a / tmp_div_b;
    wire [31:0] sign_rem_result = tmp_div_a % tmp_div_b;

    reg [31:0] final_div_result;
    reg [31:0] final_rem_result;

    always @(*) begin
        if (b_i == 32'd0) begin
            final_div_result = -32'd1;
            final_rem_result = a_i;
        end
        else if (a_i == (32'd1 << 31) && b_i == -32'd1 && (sel_div || sel_rem)) begin
            final_div_result = (32'd1 << 31);
            final_rem_result = 32'd0;
        end
        else begin
            final_div_result = tmp_div_a / tmp_div_b;
            final_rem_result = tmp_div_a % tmp_div_b;
            if ((sign_a ^ sign_b) & (sel_div)) begin
                final_div_result = ~{sign_div_result} + 32'd1;
            end
            if (sign_a & (sel_rem)) begin
                final_rem_result = ~{sign_rem_result} + 32'd1;
            end
        end
    end

    // Bitreverse
    function [31:0] reverse;
        input [31:0]    ain;
        integer i;
        begin
            for(i=0; i<32; i=i+1) begin
                reverse[32-1-i] = ain[i];
            end
        end
    endfunction
    
    // Input to the universal shifter
    reg signed [32:0] shift_input;
    always @(*) begin
        if (sel_srl)
            shift_input = {1'b0, a_i};
        else if (sel_sra)
            shift_input = {a_i[31], a_i};
        else // if (sel_sll) // this case includes "sel_sll"
            shift_input = {1'b0, reverse(a_i)};
    end

    /* verilator lint_off UNUSED */
    wire [32:0] shift_output = shift_input >>> b_i[4:0];    // Universal shifter
    /* verilator lint_on UNUSED */


    // output of universal shifter
    reg [31:0] final_shift_output;
    always @(*) begin
        if (sel_sll)
            final_shift_output = reverse(shift_output[31:0]);
        else
            final_shift_output = shift_output[31:0];
    end
    
    // Final output mux
    always @(*) begin
        if (sel_add | sel_sub)
            result_o = arith_result;
        else if (sel_sll | sel_srl | sel_sra)
            result_o = final_shift_output;
        else if (sel_xor)
            result_o = a_i ^ b_i;
        else if (sel_or)
            result_o = a_i | b_i;
        else if (sel_and)
            result_o = a_i & b_i;
        else if (sel_mul | sel_mulh | sel_mulhu | sel_mulhsu)
            result_o = final_mul_result;
        else if (sel_div | sel_divu)
            result_o = final_div_result;
        else if (sel_rem | sel_remu)
            result_o = final_rem_result;
        else
            result_o = arith_result;
    end        

endmodule
