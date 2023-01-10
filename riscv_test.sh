#!/bin/bash

test_files=("add" "addi" "and" "andi" "auipc" "beq" "bge" "bgeu" "blt" "bltu" "bne" "jal" "jalr" "lb"  \
              "lbu" "lh" "lhu" "lui" "lw" "or" "ori" "sb" "sh" "simple" "sll" "slli" "slt" "slti" "sltiu"\
              "sltu" "sra" "srai" "srl" "srli" "sub" "sw" "xor" "xori")

for test_file in ${test_files[@]}; do
    echo "testing instruction ${test_file}"
    atomsim ../riscv-tests/isa/rv32ui-p-${test_file}
done