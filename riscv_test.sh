#!/bin/bash

test_files=("add" "addi" "and" "andi" "auipc" "beq" "bge" "bgeu" "blt" "bltu" "bne" "jal" "jalr" "lb"  \
              "lbu" "lh" "lhu" "lui" "lw" "or" "ori" "sb" "sh" "simple" "sll" "slli" "slt" "slti" "sltiu"\
              "sltu" "sra" "srai" "srl" "srli" "sub" "sw" "xor" "xori")

test_mul_files=("mul" "mulh" "mulhu" "mulhsu" "div" "divu" "rem" "remu")

for test_file in ${test_files[@]}; do
    echo "testing instruction ${test_file}"
    atomsim ../riscv-tests/isa/rv32ui-p-${test_file}
done

for test_mul_file in ${test_mul_files[@]}; do
    echo "testing instruction ${test_mul_file}"
    atomsim ../riscv-tests/isa/rv32um-p-${test_mul_file}
done