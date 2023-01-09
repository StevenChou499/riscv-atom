#!/bin/bash

test_files = {add addi and andi auipc beq bge bgeu blt bltu bne jal jalr lb lbu lh lhu lui lw or ori sb sh simple sll slli slt slti sltiu sltu sra srai srl srli sub sw xor xori}

foreach test_file (test_files)
    echo $test_file
end