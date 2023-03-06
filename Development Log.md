# Development Log
# RISCV-Atom and implement RV32M
contributed by <[`鄒崴丞StevenChou`](https://github.com/StevenChou499)>, <[`王漢祺WangHanChi`](https://github.com/WangHanChi)>

## Prerequisites

### Install Dependency
1. Git clone the source code
    > git clone https://github.com/saursin/riscv-atom.git
2. Install git, make, python3, gcc & other tools
    > sudo apt install git python3 build-essential
3. Install Verilator(Version = 5.002)
    > cd $HOME
    git clone https://github.com/verilator/verilator
    cd verilator
    git checkout stable
    export VERILATOR_ROOT=`pwd`
    autoconf
    ./configure
    make
    export VERILATOR_ROOT=\$HOME/verilator
    export PATH=\$VERILATOR_ROOT/bin:\$PATH
4. Install GTK Wave
    > sudo apt install gtkwave
5. Install Screen
    > sudo apt install screen
6. Install RISC-V GNU Toolchain
    :warning:Before doing this, we need to add a file in `riscv-atom`.
    ths file is [here](https://github.com/riscv-collab/riscv-gnu-toolchain/blob/master/configure), or you can do the operation [here](https://hackmd.io/KMhQQ8FZSnmOsBGDSebhlw?view#Fix-a-little-error)
    
    > cd riscv-atom
    > sudo chmod +x install-toolchain.sh
    > sudo  ./install-toolchain.sh -x
    or install from source
    [riscv-gnu-toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain)
7. Install Doxygen
    > sudo apt install doxygen
8. Install Latex Related packages
    > sudo apt -y install texlive-latex-recommended texlive-pictures texlive-latex-extra latexmk
9. Install sphinx & other python dependencies
    > cd docs/ && pip install -r requirements.txt
10. Install socat packages
    > sudo apt install socat

### Building RISC-V Atom
1. RISC-V Atom environment variables
    > cd riscv-atom
    > source sourceme
    > echo "source \<YOUR - PATH>/sourceme" >> ~/.bashrc
2. Building the Simulator
    > make soctarget=atombones
    > atomsim --help
:::    warning
:warning:Because of our `verilator` is installed from source instead of using `sudo apt install verilator`. So we need to change the path in `~/riscv-atom/sim/makefile`.
```makefile=38
####################################################
# Verilog Configs

VC := verilator
VFLAGS := -cc -Wall --relative-includes --trace -D__ATOMSIM_SIMULATION__

# This should point to verilator include directory
#VERILATOR_INCLUDE_PATH := /usr/share/verilator/include
VERILATOR_INCLUDE_PATH := /home/hanchi/verilator/include

ifeq ("$(wildcard $(VERILATOR_INCLUDE_PATH)/verilated_vcd_c.cpp)","")
$(error Verilator include path invalid; Set correct Verilator include path in sim/Makefile)
endif


# Core files
VSRCS =  $(RTL_DIR)/Timescale.vh
VSRCS += $(RTL_DIR)/core/Utils.vh 
VSRCS += $(RTL_DIR)/core/Defs.vh 
VSRCS += $(RTL_DIR)/core/AtomRV.v 
VSRCS += $(RTL_DIR)/core/Alu.v 
VSRCS += $(RTL_DIR)/core/Decode.v 
VSRCS += $(RTL_DIR)/core/RegisterFile.v 
VSRCS += $(RTL_DIR)/core/CSR_Unit.v
```
:::

### Fix a little error
::: warning
:warning:Because there is some bug in his install-toolchain.sh, I install the riscv-gnu-toolchain form the [source](https://github.com/riscv-collab/riscv-gnu-toolchain.git), and we can get the `configure` file in the riscv-gnu-toolchain dictionary. Therefore, we can modify the install-toolchain.sh as following in the **line 60**.
```shell=50
echo -e "${GREEN}Building toolchain... ${NOCOLOR}"
# Create build directory
rm -rf ${BUILD_DIR}
mkdir -p ${BUILD_DIR}

# Build toolchain
cd ${BUILD_DIR}

echo "../../configure --prefix=$(pwd) ${TOOLCHAIN_CONFIG}"
#../../configure --prefix=$(pwd) ${TOOLCHAIN_CONFIG}
~/riscv-gnu-toolchain/configure --prefix=$(pwd) ${TOOLCHAIN_CONFIG}

echo "make -j${BUILD_JOBS}"
make -j${BUILD_JOBS}

# Copy files to the installation directory
echo -e "${GREEN}Copying... ${NOCOLOR}"
rsync -r --progress * ${TOOLCHAIN_INSTALL_PATH}/

cd ${CWDIR}
```
:::
## Running Examples on AtomSim

### Hello World Example
Switch to the examples dictionary
> cd ~/riscv-atom/sw/examples

Compile with RISC-V gcc cross-compiler, generate `hello.elf` in `hello-asm` dictionary
> make soctarget=atombones ex=hello-asm compile

Run the example
> atomsim hello-asm/hello.elf

### Alternatively, use make run to run the example
> make soctarget=atombones ex=hello-asm run

The syntax is asfollowing:
> make soctarget=\<TARGET> ex=\<EXAMPLE> compile
> make soctarget=\<TARGET> ex=\<EXAMPLE> run

### The Runexamples Script
Automatically compile and simulate all examples
> atomsim-runexamples

> make soctarget=atombones run-all

### Using Atomsim Vuart
> atomsim-gen-vports
> screen $RVATOM/userport 9600

In another terminal
> atomsim hello-asm/hello.elf --vuart=$RVATOM/simport

To close the screen command press `ctrl+a`, type `:quit` and press `Enter`.


### Test AtomSim(CPU core)
We can type the `atomsim --help` to get all the instruction about the atom simulation.
```
AtomSim v2.2
Interactive RTL Simulator for Atom based systems [ atombones ]
Usage:
  atomsim [OPTION...] input

 Backend Config options:
      --vuart arg       use provided virtual uart port (default: "")
      --vuart-baud arg  Specify virtual uart port baudrate (default: 9600)
      --imemsize arg    Specify size of instruction memory to simulate (in 
                        KB) (default: 65536)
      --dmemsize arg    Specify size of data memory to simulate (in KB) 
                        (default: 65536)

 Debug options:
  -v, --verbose         Turn on verbose output
  -d, --debug           Start in debug mode
  -t, --trace           Enable VCD tracing 
      --trace-file arg  Specify trace file (default: trace.vcd)
      --dump-file arg   Specify dump file (default: dump.txt)
      --ebreak-dump     Enable processor state dump at hault
      --signature arg   Enable signature dump at hault (Used for riscv 
                        compliance tests) (default: "")

 General options:
  -h, --help       Show this message
      --version    Show version information
      --soctarget  Show current AtomSim SoC target
      --no-color   Don't show colored output
  -i, --input arg  Specify an input file

 Sim Config options:
      --maxitr arg  Specify maximum simulation iterations (default: 
                    1000000)

```
1. Use -t will get the trace.vcd(fst graph) in the `~/riscv-atom/`
2. Use -v will get the detail output

---

## Analyze RISCV-Atom

### 2-Stage Pipeline




This author is talking about his design inspiration come from Arm Cortex m0+ on [RISC-V Atom (Core)](https://riscv-atom.readthedocs.io/en/latest/pages/documentation/riscv_atom/riscv_atom.html) And on [MICROCHIP DEVELOPER HELP](https://microchipdeveloper.com/32arm:m0-pipeline) this website mentions that M0+ can minimize its branch penalty dual to two-stage.Because it can reduce the access to Flash, it can further reduce power consumption, which usually accounts for the power consumption of microcontrollers. Therefore, it can work at ultra-low power consumption.

![](https://i.imgur.com/GGoTjTb.png)
This picture is show the pipeline of Arm Cortex M0+, we can find that the Decode stage is  divided into Pre-decode and Main decode. Such this design can minimize the branch penalty.

![](https://i.imgur.com/yVuFXJU.png)

And Atom also inherits this advantage,  divided into two stages, we can see the above figure, its stage-1 is mainly used for Fetch Instruction, and stage-2 is responsible for Decode, Execute & Write-back, can reduce its branch penalty to 1.

### Design motivation

We can reduce the use of Register file by putting Decode in stage-2, which can reduce the usage of LUT. LUT is a common component of FPGA. It is the same as a ROM. It can pre-store the results of logic functions in it, so that the pre-stored results can be addressed by using the input signal as an address.

![](https://i.imgur.com/dFvUMW7.png)
The reason why the amount of LUT usage can be reduced is because the FPGA is composed of a basic block called CLB (Configurable Logic Block) or LAB (Logic Array Block). The following figure is a typical structure of a CLB block, which includes a full adder (FA), a D-Type flip-flop, three multiplexers (mux) and two three-input Lookup tables (3-LUTs ). Therefore, reducing the CLB will also reduce the LUT usage.


---
## Validate RISC-V Atom by Verilator, including Dhrystone.

### Validate
The version of Verilator in our environment is 5.002
```makefile
hanchi@hanchi:~$ verilator --version
Verilator 5.002 2022-10-29 rev v5.002-29-gdb39d70c7
```
As above using, we can type `make sim` in the ROOTDIR of riscv-atom to build the atomsim. In the makefile, we can find that is 
```makefile=85
# ======== AtomSim ========
.PHONY : sim
sim:                       			## Build atomsim for given target [default: atombones]
	@echo "$(COLOR_GREEN)>> Building Atomsim for soctarget=$(soctarget) $(COLOR_NC)"
	make $(MKFLAGS) -C $(sim_dir) soctarget=$(soctarget) DEBUG=1
```
After get the **atomsim**, we can type `atomsim --help` to get the user guide, that is 
```makefile
hanchi@hanchi:~/riscv-atom$ atomsim --help
AtomSim v2.2
Interactive RTL Simulator for Atom based systems [ atombones ]
Usage:
  atomsim [OPTION...] input

 Backend Config options:
      --vuart arg       use provided virtual uart port (default: "")
      --vuart-baud arg  Specify virtual uart port baudrate (default: 9600)
      --imemsize arg    Specify size of instruction memory to simulate (in 
                        KB) (default: 65536)
      --dmemsize arg    Specify size of data memory to simulate (in KB) 
                        (default: 65536)

 Debug options:
  -v, --verbose         Turn on verbose output
  -d, --debug           Start in debug mode
  -t, --trace           Enable VCD tracing 
      --trace-file arg  Specify trace file (default: trace.vcd)
      --dump-file arg   Specify dump file (default: dump.txt)
      --ebreak-dump     Enable processor state dump at hault
      --signature arg   Enable signature dump at hault (Used for riscv 
                        compliance tests) (default: "")

 General options:
  -h, --help       Show this message
      --version    Show version information
      --soctarget  Show current AtomSim SoC target
      --no-color   Don't show colored output
  -i, --input arg  Specify an input file

 Sim Config options:
      --maxitr arg  Specify maximum simulation iterations (default: 
                    1000000)
```

Now, we can run the simple example to check the atomsim
> atomsim sw/examples/hello-asm/hello.elf

And we will get the 
```makefile=
hanchi@hanchi:~/riscv-atom$ atomsim sw/examples/hello-asm/hello.elf
Loading segment 1 [base=0x00000000, sz=   456 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x04000000, sz=    40 bytes, at=0x04000000] ...	done
Hello World!
      -- from Assembly

```

We also can add some flag likes `-v`, `-d`, `-t`...and so on.
Therefore, we completed the Validate RISC-V Atom by Verilator.

### Dhrystone

The author, [saursin](https://github.com/saursin) have made Dhrystone before, but there are some bugs in the code which made it cannot run. I commented some code in order to fixed it. Now we can type `make dhrystone` in the ROOTDIR to get the dhrystone result.

This code is dhry_1.c which is one fragment of dhrystone.
```c=97
  /* Initializations */
  //#ifdef RISCV
  //serial_init(UART_BAUD_9600);
  //#endif
  
  Next_Ptr_Glob = (Rec_Pointer) malloc (sizeof (Rec_Type));
  Ptr_Glob = (Rec_Pointer) malloc (sizeof (Rec_Type));
```

Here is the result of dhrystone
```makefile
hanchi@hanchi:~/riscv-atom$ make dhrystone 
atomsim --maxitr 100000000 -t sw/examples/dhrystone/dhrystone.elf
Loading segment 1 [base=0x00000000, sz= 11436 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x04000000, sz=  1068 bytes, at=0x04000000] ...	done

Dhrystone Benchmark, Version 2.1 (Language: C)

Program compiled without 'register' attribute

Execution starts, 2000 runs through Dhrystone
Execution ends

Final values of the variables used in the benchmark:

Int_Glob:            5
        should be:   5
Bool_Glob:           1
        should be:   1
Ch_1_Glob:           A
        should be:   A
Ch_2_Glob:           B
        should be:   B
Arr_1_Glob[8]:       7
        should be:   7
Arr_2_Glob[8][7]:    2010
        should be:   Number_Of_Runs + 10
Ptr_Glob->
  Ptr_Comp:          67120132
        should be:   (implementation-dependent)
  Discr:             0
        should be:   0
  Enum_Comp:         2
        should be:   2
  Int_Comp:          17
        should be:   17
  Str_Comp:          DHRYSTONE PROGRAM, SOME STRING
        should be:   DHRYSTONE PROGRAM, SOME STRING
Next_Ptr_Glob->
  Ptr_Comp:          67120132
        should be:   (implementation-dependent), same as above
  Discr:             0
        should be:   0
  Enum_Comp:         1
        should be:   1
  Int_Comp:          18
        should be:   18
  Str_Comp:          DHRYSTONE PROGRAM, SOME STRING
        should be:   DHRYSTONE PROGRAM, SOME STRING
Int_1_Loc:           5
        should be:   5
Int_2_Loc:           13
        should be:   13
Int_3_Loc:           7
        should be:   7
Enum_Loc:            1
        should be:   1
Str_1_Loc:           DHRYSTONE PROGRAM, 1'ST STRING
        should be:   DHRYSTONE PROGRAM, 1'ST STRING
Str_2_Loc:           DHRYSTONE PROGRAM, 2'ND STRING
        should be:   DHRYSTONE PROGRAM, 2'ND STRING

Number Of Runs: 2000
cycles Elapsed: 1718085
Dhrystones_Per_Second_Per_MHz: 1164
DMIPS_Per_MHz: 0.662
```

:::warning
TODO : Add [coremark](https://github.com/riscv-boom/riscv-coremark) and so on.
:::

---

## Study Atomsim

AtomSim is the main part of the riscv-atom. There are mainly two parts, which is core and uncore. core is the "core" part of the processor, "uncore" is the remain part of the processor.

### Core

First we will take a look at the core part of the Atomsim. The CPU is a 2-stage pipeline, The first stage is the Fetch stage, the second stage is the decode, execute, memory and write back stage.

* `Defs.vh` : 

`Defs.vh` defines all the macros we are going to use in the other files, mainly defining all the instruction type, ALU options and comparator option.

* `Decode.vh` : 

`Decode.vh` decodes the fetched instruction, takes out the `opcode` , `func3` and `func7` part. Figure out the destination register `rd` and source register `rs1` and `rs2` .

```verilog=35
   // Decode fields
    wire    [6:0]   opcode  = instr_i[6:0];
    wire    [2:0]   func3   = instr_i[14:12];
    wire    [6:0]   func7   = instr_i[31:25];

    assign mem_access_width_o = func3;
    assign csru_op_sel_o = func3;

    assign  rd_sel_o    = instr_i[11:7];
    assign  rs1_sel_o   = instr_i[19:15];
    assign  rs2_sel_o   = instr_i[24:20];

    reg    [2:0] imm_format;
```

After we decompose the fetched instruction, we can analyze the function we need to prepare and enable or disable the register file, figure out the comparision type.

```verilog=71
    always @(*) begin
        // DEFAULT VALUES
        jump_en_o = 1'b0;
        comparison_type_o = `CMP_FUNC_UN;
        rf_we_o = 1'b0;
        rf_din_sel_o = 3'd0;
        a_op_sel_o = 1'b0;
        b_op_sel_o = 1'b0;
        cmp_b_op_sel_o = 1'b0;
        alu_op_sel_o = `ALU_FUNC_ADD;
        mem_we_o = 1'b0;
        d_mem_load_store = 1'b0;
        imm_format = `RV_IMM_TYPE_U;
        csru_we_o = 0;


        casez({func7, func3, opcode})
            
            /* LUI   */ 
            17'b???????_???_0110111: 
            begin
                rf_we_o = 1'b1;
                rf_din_sel_o = 3'd0;
                imm_format = `RV_IMM_TYPE_U;
            end
            
            ...

                `ifdef verilator
                    if(opcode != 7'b1110011) // EBREAK
                        $display("!Warning: Unimplemented Opcode: %b", opcode);
                `endif
            end            

        endcase
    end
```

Now we figure out the register file we need and the comparison type, the last thing the decoder need to is generate a 32-bit immediate for future 
calculation.

```verilog=49
    /*
        Decode Immediate
    */
    reg [31:0] getExtImm;

    always @(*) /*COMBINATORIAL*/ 
    begin
        case(imm_format)
                `RV_IMM_TYPE_I    :   getExtImm = {{21{instr_i[31]}}, instr_i[30:25], instr_i[24:21], instr_i[20]};
                `RV_IMM_TYPE_S    :   getExtImm = {{21{instr_i[31]}}, instr_i[30:25], instr_i[11:8], instr_i[7]};
                `RV_IMM_TYPE_B    :   getExtImm = {{20{instr_i[31]}}, instr_i[7], instr_i[30:25], instr_i[11:8], 1'b0};
                `RV_IMM_TYPE_U    :   getExtImm = {instr_i[31], instr_i[30:20], instr_i[19:12], {12{1'b0}}};
                `RV_IMM_TYPE_J    :   getExtImm = {{12{instr_i[31]}}, instr_i[19:12], instr_i[20], instr_i[30:25], instr_i[24:21], 1'b0};

                default:
                    getExtImm = 32'd0;
        endcase
    end

    assign imm_o = getExtImm;
```

* `RegisterFile.v` : 

`RegisterFile.v` stores all the RISC-V CPU register, providing a reset function. With in a cycle we can write in one register and read out two register at the same time.


Because RISC-V CPU have 32 registers, so we need 5 bits to locate each of them, also the first register which is `x0` is a hard-wired zero, means it cannot be changed into any value instead of zero.


```verilog=38
    localparam REG_COUNT = 2**REG_ADDR_WIDTH;

    `ifdef RF_R0_IS_ZERO
        reg	[REG_WIDTH-1:0]	regs	[1:REG_COUNT-1] /*verilator public*/;    // register array
    `else
        reg [REG_WIDTH-1:0]	regs	[0:REG_COUNT-1] /*verilator public*/;    // register array
    `endif
```

We can write into the register file every time the clock is trigger (1 clock cycle) and also the write enable signal `Data_We_i` is high, the written register is choosen by the `Rd_Sel_i` signal. If the reset signal `Rst_i` is high, all the register will be formatted into zero.

```verilog=16
    integer i;

    /* === WRITE PORT ===
        Synchronous write
    */
    always @ (posedge Clk_i) begin
        if (Rst_i) begin
            for(i=1; i<REG_COUNT; i=i+1)
                regs[i] <= {REG_WIDTH{1'b0}};
        end
        else if(Data_We_i) begin
            `ifdef RF_R0_IS_ZERO
                if(Rd_Sel_i != 0)
                    regs[Rd_Sel_i] <= Data_i;
            `else
                regs[Rd_Sel_i] <= Data_i;
            `endif
        end
    end
```

* `ALU.v` : 

`ALU.v` is responsible for all the main calculation of the RISC-V CPU, it takes the output of the decoder, which is the register value or an immediate. The main supported calculation are add, subtract, arithmetic shift left and right, bitwise and, or and exclusive or.

ALU uses the select signal `sel_i` to select the correct function of the calculation.

```verilog=30
    wire sel_add = (sel_i == `ALU_FUNC_ADD);
    wire sel_sub = (sel_i == `ALU_FUNC_SUB);
    wire sel_xor = (sel_i == `ALU_FUNC_XOR);
    wire sel_or  = (sel_i == `ALU_FUNC_OR);
    wire sel_and = (sel_i == `ALU_FUNC_AND);
    wire sel_sll = (sel_i == `ALU_FUNC_SLL);
    wire sel_srl = (sel_i == `ALU_FUNC_SRL);
    wire sel_sra = (sel_i == `ALU_FUNC_SRA);
```

Because addition and subtraction is very similar under bare metal, so we can use the same function at the same time.

```verilog=39
    // Result of arithmetic calculations (ADD/SUB)
    wire [31:0] arith_result = a_i + (sel_sub ? ((~b_i)+1) : b_i);
```

Arithmetic right shift needs to pad the most significand bit and logic shift only need to pad zeros, we can obtain the shift amount by bottom 5 bits of the signal `b_i`.

```verilog=53
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
```

After doing all the calculations, we can use the select signal to output the correct answer.

```verilog=78
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
        else
            result_o = arith_result;
    end 
```

---

## Implement the rv32M-Extension

In order to implement RV32M, we have to first understand what RV32M stands for. RV32M is an optional extended instruction set besides RV32I. It is mainly used for multiplication, division and remainder of (non-negative) integers.

### Read the risc-v spec
![](https://i.imgur.com/wvcgpXb.png)

We can get the machine code in M-Extension in the picture.

![](https://i.imgur.com/per8rlU.png)

And we can also know that M-instructions is all R-type.

According to the description on page 44 of the RISC-V specification : 
> REM and REMU provide the remainder of the corresponding division operation. For REM, the sign of the result equals the sign of the dividend.
> ...
> For both signed and unsigned division, it holds that dividend = divisor × quotient + remainder.

Therefore, when doing REM and REMU operations, the sign of the dividend must be the same as the remainder. And all division operations must comply with dividend = divisor x quotient + remainder .

* In addition, we need to pay attention to two exceptions when doing division operations. The first is the case where the divisor is 0, and the second is the division overflow of signed numbers. The following are the operation results in special cases : 

|      Condition       |  Dividend  | Divisor |     | `DIVU`  | `REMU` |  `DIV`   | `REMU` |
|:--------------------:|:----------:|:-------:|:---:|:-------:|:------:|:--------:|:------:|
|     divided by 0     |    $x$     |   $0$   |     | $2^L-1$ |  $x$   |   $-1$   |  $x$   |
| overflow (on signed) | $-2^{L-1}$ |  $-1$   |     |   --    |   --   | $-2^L-1$ |  $0$   |

### Modify the [Defs.vh](https://github.com/WangHanChi/riscv-atom/blob/main/rtl/core/Defs.vh)
We need to expand the ALU for our M instructions.
Remove the old instructions, and we will extend ont bit for **I + M instructions**. Therefore it changed from `3'd0` to `4'd0` in ALU_FUNC_ADD.
```diff=48
- `define ALU_FUNC_ADD 3'd0
- `define ALU_FUNC_SUB 3'd1
-`define ALU_FUNC_XOR 3'd2
-`define ALU_FUNC_OR  3'd3
-`define ALU_FUNC_AND 3'd4
-`define ALU_FUNC_SLL 3'd5
-`define ALU_FUNC_SRL 3'd6
-`define ALU_FUNC_SRA 3'd7
```

```diff=48
+ `define ALU_FUNC_ADD 4'd0
+ `define ALU_FUNC_SUB 4'd1
+ `define ALU_FUNC_XOR 4'd2
+ `define ALU_FUNC_OR  4'd3
+ `define ALU_FUNC_AND 4'd4
+ `define ALU_FUNC_SLL 4'd5
+ `define ALU_FUNC_SRL 4'd6
+ `define ALU_FUNC_SRA 4'd7

// ALU_M_EXTENSION
+ `define ALU_FUNC_MUL    4'd8
+ `define ALU_FUNC_MULH   4'd9
+ `define ALU_FUNC_MULHSU 4'd10
+ `define ALU_FUNC_MULHU  4'd11
+ `define ALU_FUNC_DIV    4'd12
+ `define ALU_FUNC_DIVU   4'd13
+ `define ALU_FUNC_REM    4'd14
+ `define ALU_FUNC_REMU   4'd15
```

### Modfiy the [AtomRV.v](https://github.com/WangHanChi/riscv-atom/blob/main/rtl/core/AtomRV.v)
We will change a little in this verilog file because the file is not related to **Decode**. We will widen the wire for decode alu opcode.
```diff=242
wire            d_a_op_sel;
wire            d_b_op_sel;
wire            d_cmp_b_op_sel;
- wire    [2:0]   d_alu_op_sel;
+ wire    [3:0]   d_alu_op_sel;
wire    [2:0]   d_mem_access_width;
wire            d_mem_load_store;
wire            d_mem_we;
```





### Modfiy the [Decode.v](https://github.com/WangHanChi/riscv-atom/blob/main/rtl/core/Decode.v)


```diff=13
module Decode
(
-    input   wire    [31:0]  instr_i,
+    input   wire    [31:0]  instr_i,    // A full instruction

-    output  wire    [4:0]   rd_sel_o,
-    output  wire    [4:0]   rs1_sel_o,
-    output  wire    [4:0]   rs2_sel_o,
+    output  wire    [4:0]   rd_sel_o,   // rd
+    output  wire    [4:0]   rs1_sel_o,  // rs1
+    output  wire    [4:0]   rs2_sel_o,  // rs2

    output  wire    [31:0]  imm_o,

-    output  reg             jump_en_o,
-    output  reg     [2:0]   comparison_type_o,
+    output  reg             jump_en_o,  // check jump or not
+    output  reg     [2:0]   comparison_type_o,  // check compariosn type
    output  reg             rf_we_o,
    output  reg     [2:0]   rf_din_sel_o,
    output  reg             a_op_sel_o,
    output  reg             b_op_sel_o,
    output  reg             cmp_b_op_sel_o,
-    output  reg     [2:0]   alu_op_sel_o,
+    output  reg     [3:0]   alu_op_sel_o,
    output  wire    [2:0]   mem_access_width_o,
    output  reg             d_mem_load_store,
    output  reg             mem_we_o,
```

```diff=43
    assign mem_access_width_o = func3;
    assign csru_op_sel_o = func3;

-    assign  rd_sel_o    = instr_i[11:7];
-    assign  rs1_sel_o   = instr_i[19:15];
-    assign  rs2_sel_o   = instr_i[24:20];
+    assign  rd_sel_o    = instr_i[11:7];    // rd
+    assign  rs1_sel_o   = instr_i[19:15];   // rs1
+    assign  rs2_sel_o   = instr_i[24:20];   // rs2

    reg    [2:0] imm_format;

```


```diff=427
+ /////////////////////////////////////////////////////////////////////////
+/* M-Extension Instructions */

+            /* MUL      */
+            17'b0000001_000_0110011:
+            begin
+                rf_we_o = 1'b1;
+                rf_din_sel_o = 3'd2;
+                a_op_sel_o = 1'b0;
+                b_op_sel_o = 1'b0;
+                alu_op_sel_o = `ALU_FUNC_MUL;
+            end

+            /* MULH     */
+            17'b0000001_001_0110011:
+            begin
+                rf_we_o = 1'b1;
+                rf_din_sel_o = 3'd2;
+                a_op_sel_o = 1'b0;
+                b_op_sel_o = 1'b0;
+                alu_op_sel_o = `ALU_FUNC_MULH;
+            end

+            /* MULHSU    */
+            17'b0000001_010_0110011:
+            begin
+                rf_we_o = 1'b1;
+                rf_din_sel_o = 3'd2;
+                a_op_sel_o = 1'b0;
+                b_op_sel_o = 1'b0;
+                alu_op_sel_o = `ALU_FUNC_MULHSU;
+            end
+            /* MULHU     */
+            17'b0000001_011_0110011:
+            begin
+                rf_we_o = 1'b1;
+                rf_din_sel_o = 3'd2;
+                a_op_sel_o = 1'b0;
+                b_op_sel_o = 1'b0;
+                alu_op_sel_o = `ALU_FUNC_MULHU;
+            end

+            /* DIV      */
+            17'b0000001_100_0110011:
+            begin
+                rf_we_o = 1'b1;
+                rf_din_sel_o = 3'd2;
+                a_op_sel_o = 1'b0;
+                b_op_sel_o = 1'b0;
+                alu_op_sel_o = `ALU_FUNC_DIV;
+            end

+            /* DIVU     */
+            17'b0000001_101_0110011:
+            begin
+                rf_we_o = 1'b1;
+                rf_din_sel_o = 3'd2;
+                a_op_sel_o = 1'b0;
+                b_op_sel_o = 1'b0;
+                alu_op_sel_o = `ALU_FUNC_DIVU;
+            end

+            /* REM      */
+            17'b0000001_110_0110011:
+            begin
+                rf_we_o = 1'b1;
+                rf_din_sel_o = 3'd2;
+                a_op_sel_o = 1'b0;
+                b_op_sel_o = 1'b0;
+                alu_op_sel_o = `ALU_FUNC_REM;
+            end

+            /* REMU     */
+            17'b0000001_111_0110011:
+            begin
+                rf_we_o = 1'b1;
+                rf_din_sel_o = 3'd2;
+                a_op_sel_o = 1'b0;
+                b_op_sel_o = 1'b0;
+                alu_op_sel_o = `ALU_FUNC_REMU;
+            end+=           
```

#### [Alu.v](https://github.com/WangHanChi/riscv-atom/blob/main/rtl/core/Alu.v)

```diff=22
(
    input   wire    [31:0]  a_i,
    input   wire    [31:0]  b_i,
-    input   wire    [2:0]   sel_i,
+    input   wire    [3:0]   sel_i,

    output  reg     [31:0]  result_o
);


    wire sel_add = (sel_i == `ALU_FUNC_ADD);
    wire sel_sub = (sel_i == `ALU_FUNC_SUB);
    wire sel_xor = (sel_i == `ALU_FUNC_XOR);
    wire sel_or  = (sel_i == `ALU_FUNC_OR);
    wire sel_and = (sel_i == `ALU_FUNC_AND);
    wire sel_sll = (sel_i == `ALU_FUNC_SLL);
    wire sel_srl = (sel_i == `ALU_FUNC_SRL);
    wire sel_sra = (sel_i == `ALU_FUNC_SRA);

+    // Add M-Extension instructions

+    wire sel_mul    =   (sel_i == `ALU_FUNC_MUL);
+    wire sel_mulh   =   (sel_i == `ALU_FUNC_MULH);
+    wire sel_mulhsu =   (sel_i == `ALU_FUNC_MULHSU);
+    wire sel_mulhu  =   (sel_i == `ALU_FUNC_MULHU);
+    wire sel_div    =   (sel_i == `ALU_FUNC_DIV);
+    wire sel_divu   =   (sel_i == `ALU_FUNC_DIVU);
+    wire sel_rem    =   (sel_i == `ALU_FUNC_REM);
+    wire sel_remu   =   (sel_i == `ALU_FUNC_REMU);

    // Result of arithmetic calculations (ADD/SUB)
    wire [31:0] arith_result = a_i + (sel_sub ? ((~b_i)+1) : b_i);

```


```diff=86
        else
            final_shift_output = shift_output[31:0];
    end

+    // M-Extension mux
+    wire [63:0] result_mul;
+    /* verilator lint_off UNUSEDSIGNAL */
+    wire [63:0] result_mulsu;
+    /* verilator lint_off UNUSEDSIGNAL */
+    wire [63:0] result_mulu;
+    /* verilator lint_off UNUSEDSIGNAL */
+    wire [31:0] result_div;
+    wire [31:0] result_divu;
+    wire [31:0] result_rem;
+    wire [31:0] result_remu;

+    assign result_mul[63:0]     =   $signed  ({{32{a_i[31]}}, a_i[31: 0]}) *
+                                    $signed  ({{32{b_i[31]}}, b_i[31: 0]});

+    assign result_mulu[63:0]    =   $unsigned  ({{32{1'b0}}, a_i[31: 0]}) *
+                                    $unsigned  ({{32{1'b0}}, b_i[31: 0]});

+    assign result_mulsu[63:0]   =   $signed  ({{32{a_i[31]}}, a_i[31: 0]}) *
+                                    $unsigned  ({{32{1'b0}}, b_i[31: 0]});

+    assign result_div[31:0]     =   (b_i == 32'h00000000) ? 32'hffffffff :
+                                    ((a_i == 32'h80000000) && (b_i == 32'hffffffff)) ? 32'h80000000 :
+                                    $signed  ($signed  (a_i) / $signed  (b_i));

+    assign result_divu[31:0]    =   (b_i == 32'h00000000) ? 32'hffffffff :
+                                    $unsigned($unsigned(a_i) / $unsigned(b_i));

+    assign result_rem[31:0]     =   (b_i == 32'h00000000) ? a_i :
+                                    ((a_i == 32'h80000000) && (b_i == 32'hffffffff)) ? 32'h00000000 :
+                                    $signed  ($signed  (a_i) % $signed  (b_i));

+    assign result_remu[31: 0]   =   (b_i == 32'h00000000) ? a_i :
+                                    $unsigned($unsigned(a_i) % $unsigned(b_i));

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
+        else if(sel_mul)                             // M start
+            result_o = result_mul[31:0];
+        else if(sel_mulh)
+            result_o = result_mul[63:32];
+        else if(sel_mulhsu)
+            result_o = result_mulsu[63:32];
+        else if(sel_mulhu)
+            result_o = result_mulu[63:32];
+        else if(sel_div)
+            result_o = result_div[31:0];
+        else if(sel_divu)
+            result_o = result_divu[31:0];
+        else if(sel_rem)
+            result_o = result_rem[31:0];
+        else if(sel_remu)
+           result_o = result_remu[31:0];           // M end
        else
            result_o = arith_result;
    end   
```

### riscv-tests

We decide to use [riscv-test](https://github.com/riscv-software-src/riscv-tests) to test **atomsim** finally. There are many places need to modify to meet the requires.

1. Beacuse the author of riscv-atom did not implement `fence` this instruction, we modify the **[decode.v](https://github.com/WangHanChi/riscv-atom/blob/main/rtl/core/Decode.v)** to avoid error.

```diff=519
default: begin
                jump_en_o = 0;
                comparison_type_o = `CMP_FUNC_UN;
                rf_we_o = 0;
                rf_din_sel_o = 0;
                a_op_sel_o = 0;
                b_op_sel_o = 0;
                cmp_b_op_sel_o = 0;
                alu_op_sel_o = 0;
                mem_we_o = 1'b0;
                imm_format = 0;
                csru_we_o = 0;

                `ifdef verilator
-                    if(opcode != 7'b1110011) // EBREAK
+                    if(opcode != 7'b1110011 && opcode != 7'b0001111) // EBREAK
                        $display("!Warning: Unimplemented Opcode: %b", opcode);
                `endif
            end     
```

2. Modify the linker script for atomsim, we need to compare the linker script of riscv-test. So we make the .text section begin at 0x00000000.
 ```diff=18
 SECTIONS
{
    /* ==== ROM ==== */
    .text : 
    { 
        *(.boot*)
-        . = ORIGIN(ROM) + 0x100;
+        . = ORIGIN(ROM) + 0x000;
        
        _svector = .;
        KEEP(*(.vector*))   /* Keep all interrupt vector tables at very start of text section */

        *(.text)            /* Load all text sections (from all files) */
        *(.rodata)

        . = ALIGN(4);
        _etext = .;
    } > ROM
```
 3. git clone the riscv-test project and put it in **riscv-atom/test**. And follow the step form [README.md](https://github.com/riscv-software-src/riscv-tests/blob/master/README.md)
 
 ```shell
$ git clone https://github.com/riscv/riscv-tests
$ cd riscv-tests
$ git submodule update --init --recursive
$ autoconf
$ ./configure --prefix=$RISCV/target
$ make
$ sudo make install
```
 4. Modify the [link script](https://github.com/riscv/riscv-test-env/blob/0666378f353599d01fc48562b431b1dd049faab5/p/link.ld) in `riscv-tests/env/p`. The reason is we want to align the start section between atomsim and riscv-tests. And we find that we should follow the setting of author. He let the ROM is 64MB, and RAM is 64MB in the `riscv-atom/sw/lib/linklink.ld`.**Therefore we should push the `.data section` to `0x04000000`.**

```linker script=1
/*
    LINKER SCRIPT

    @See : https://sourceware.org/binutils/docs/ld/Basic-Script-Concepts.html
    @See : https://interrupt.memfault.com/blog/how-to-write-linker-scripts-for-firmware
*/

OUTPUT_ARCH( "riscv" )
ENTRY(_start)

/* MEMORY LAYOUT */
MEMORY
{
    ROM (rx) : ORIGIN = 0x00000000, LENGTH = 64M    /* 64 MB @ 0x0*/
    RAM (rwx): ORIGIN = 0x04000000, LENGTH = 64M    /* 64 MB @ 0x10000 (0x04000000)*/
}
```
 
 ```diff=1
OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS
{
-  . = 0x80000000;
+  . = 0x00000000;
  .text.init : { *(.text.init) }
  . = ALIGN(0x1000);
  .tohost : { *(.tohost) }
  . = ALIGN(0x1000);
  .text : { *(.text) }
  . = ALIGN(0x1000);
+  . = 0x04000000;
  .data : { *(.data) }
  .bss : { *(.bss) }
  _end = .;
}
```
 
 5. Modify the testing header in order to print 0/1 to check it pass or not. we will change the end part, which **will not change the testing code**.
 First, we will modify the riscv_test.h in `riscv-tests/env/p`. one is `unimp` because atomsim will stop when it decode the word `ebreak`. Therecore, we will change it into `ebreak`.
 ```diff=229
 #define RVTEST_CODE_END\                                               
-       unimp;   
+       ebreak;
```
 And because it will not print any symbol to let us know pass or not, we create the similar `pass result` and `fail result` to print 0/1to let us know which code is passed. And this step is also good for us to write shell script to check. 
 We will modify the same file, which is riscv_test.h in `riscv-tests/env/p`.
 ```diff
 #define RVTEST_PASS                                                     \
        fence;                                                          \
        li TESTNUM, 1;                                                  \
        li a7, 93;                                                      \
        li a0, 0;                                                       \
-        ecall
+        ebreak;
        
+#define MY_RVTEST_PASS                                                 	\
+        fence;                                                          \
+        li TESTNUM, 1;													\
+        addi gp, gp, 48;												\
+        li t1, 0x08000000;                                              \
+        sb gp, 0(t1);													\
+        li gp, 10;                                              \
+        sb gp, 0(t1);													\
+        li a7, 93;                                                      \
+        li a0, 0;                                                       \
+        ebreak;

#define TESTNUM gp
#define RVTEST_FAIL                                                     \
        fence;                                                          \
1:      beqz TESTNUM, 1b;                                               \
        sll TESTNUM, TESTNUM, 1;                                        \
        or TESTNUM, TESTNUM, 1;                                         \
        li a7, 93;                                                      \
        addi a0, TESTNUM, 0;                                            \
-        ecall
+        ebreak;

+#define MY_RVTEST_FAIL                                                  \
+        fence;     														\
+        li TESTNUM, 0;													\
+        addi gp, gp, 48;												\
+        li t1, 0x08000000;                                              \
+        sb gp, 0(t1);													\
+        li gp, 10;                                              \
+        sb gp, 0(t1);													\
+       li a7, 93;                                                      \
+        li a0, 0;                                                       \
+        ebreak;                                                           
```
 
 6. Modify the testing macros to let it print 0/1, we change the end of macro to let it use our similar code which can print 0/1.

```diff=737
#-----------------------------------------------------------------------
# Pass and fail code (assumes test num is in TESTNUM)
#-----------------------------------------------------------------------

#define TEST_PASSFAIL \
        bne x0, TESTNUM, pass; \
fail: \
-        RVTEST_FAIL; \
+        MY_RVTEST_FAIL; \
pass: \
-        RVTEST_PASS \
+        MY_RVTEST_PASS \
```
 There is a file need to modify beacuse it directly call the macro to check the simple function, so we replace it to our pass macro. The file is simple.S in `riscv-test/isa/rv64ui`. The reason which we modify the rv64 file is that the rv32ui testing code is called the same code in rv64ui, so we should modify in rv64ui instead of rv32ui.
```diff=12
#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN
-RVTEST_PASS
+MY_RVTEST_PASS

RVTEST_CODE_END
```

7. We write a shell script to auto confirm the test pass or not in `riscv-atom/scripts`. We can type `make riscv-test` to get the result!

```shell
#!/bin/bash

# riscv-test dir

RED="\e[31m"
GREEN="\e[32m"
ORANGE="\e[33m"
CYAN="\e[36m"
NOCOLOR="\e[0m"

cd /home/$USER/riscv-atom/test/riscv-tests/isa

declare -a isa_RV32UI_p=( "add" "addi" "and" "andi" "auipc" "beq" "bge" "bgeu" "blt" "bltu" "bne" "jal" "jalr" "lb" "lbu" "lh" "lhu" "lui" "lw" "or" 
 "ori" "sb" "sh" "sll" "simple" "slli" "slt" "slti" "sltiu" "sltu" "sra" "srai" "srl" "srli" "sub" "sw" "xor" "xori" )

declare -a isa_RV32UM_p=( "div" "divu" "mul" "mulh" "mulhu" "mulhsu" "rem" "remu")


suss_rv32ui=0
fail_rv32ui=0
total_rv32ui=0

suss_rv32um=0
fail_rv32um=0
total_rv32um=0

echo "Now test rv32ui for riscv-atom!!"
for isa_RV32UI in ${isa_RV32UI_p[@]}
do
    echo testing instruction ${isa_RV32UI}
    atomsim rv32ui-p-${isa_RV32UI}
    output=$(atomsim rv32ui-p-${isa_RV32UI})
    length=${#output}
    substring=${output:length-1:1}
    if [ $substring == "1" ]
    then 
    	suss_rv32ui=$(($suss_rv32ui+1))
    else 
    	fail_rv32ui=$(($fail_rv32ui+1))
    fi
    total_rv32ui=$(($total_rv32ui+1))
    echo
done

echo



echo "Now test rv32um for riscv-atom!!"
for isa_RV32UM in ${isa_RV32UM_p[@]}
do
    echo testing instruction ${isa_RV32UM}
    atomsim rv32um-p-${isa_RV32UM}
    output=$(atomsim rv32um-p-${isa_RV32UM})
    length=${#output}
    substring=${output:length-1:1}
    if [ $substring == "1" ]
    then 
    	suss_rv32um=$(($suss_rv32um+1))
    else 
    	fail_rv32um=$(($fail_rv32um+1))
    fi
    total_rv32um=$(($total_rv32um+1))
    echo
done

echo "=============================="
echo "rv32ui-p instruction set :"
echo -e "${NOCOLOR}The pass rate is ${GREEN}$suss_rv32ui/$total_rv32ui"
echo -e "${NOCOLOR}The fail rate is ${RED}$fail_rv32ui/$total_rv32ui${NOCOLOR}"
if [ $fail_rv32ui == "0" ]
then 
	echo -e "${CYAN}Pass rv32ui-p testing! ${NOCOLOR}"
else
	echo -e "${CYAN}Fail rv32ui-p testing! ${NOCOLOR}"
fi	
echo "=============================="
echo "rv32um-p instruction set :"
echo -e "${NOCOLOR}The pass rate is ${GREEN}$suss_rv32um/$total_rv32um"
echo -e "${NOCOLOR}The fail rate is ${RED}$fail_rv32um/$total_rv32um${NOCOLOR}"
if [ $fail_rv32um == "0" ]
then 
	echo -e "${CYAN}Pass rv32um-p testing! ${NOCOLOR}"
else
	echo -e "${CYAN}Fail rv32um-p testing! ${NOCOLOR}"
fi	
echo "=============================="
```
:::spoiler Result of `make riscv-test`
```makefile
hanchi@hanchi:~/riscv-atom$ make riscv-test 
./scripts/riscv-test.sh	
Now test rv32ui for riscv-atom!!
testing instruction add
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction addi
Loading segment 1 [base=0x00000000, sz=  1148 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction and
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction andi
Loading segment 1 [base=0x00000000, sz=   956 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction auipc
Loading segment 1 [base=0x00000000, sz=   564 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction beq
Loading segment 1 [base=0x00000000, sz=  1212 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction bge
Loading segment 1 [base=0x00000000, sz=  1276 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction bgeu
Loading segment 1 [base=0x00000000, sz=  1340 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction blt
Loading segment 1 [base=0x00000000, sz=  1212 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction bltu
Loading segment 1 [base=0x00000000, sz=  1212 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction bne
Loading segment 1 [base=0x00000000, sz=  1212 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction jal
Loading segment 1 [base=0x00000000, sz=   572 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction jalr
Loading segment 1 [base=0x00000000, sz=   700 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction lb
Loading segment 1 [base=0x00000000, sz=  1084 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
Loading segment 3 [base=0x04000000, sz=    16 bytes, at=0x04000000] ...	done
1

testing instruction lbu
Loading segment 1 [base=0x00000000, sz=  1084 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
Loading segment 3 [base=0x04000000, sz=    16 bytes, at=0x04000000] ...	done
1

testing instruction lh
Loading segment 1 [base=0x00000000, sz=  1148 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
Loading segment 3 [base=0x04000000, sz=    16 bytes, at=0x04000000] ...	done
1

testing instruction lhu
Loading segment 1 [base=0x00000000, sz=  1212 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
Loading segment 3 [base=0x04000000, sz=    16 bytes, at=0x04000000] ...	done
1

testing instruction lui
Loading segment 1 [base=0x00000000, sz=   572 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction lw
Loading segment 1 [base=0x00000000, sz=  1212 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
Loading segment 3 [base=0x04000000, sz=    16 bytes, at=0x04000000] ...	done
1

testing instruction or
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction ori
Loading segment 1 [base=0x00000000, sz=   956 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction sb
Loading segment 1 [base=0x00000000, sz=  1596 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
Loading segment 3 [base=0x04000000, sz=    16 bytes, at=0x04000000] ...	done
1

testing instruction sh
Loading segment 1 [base=0x00000000, sz=  1788 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
Loading segment 3 [base=0x04000000, sz=    32 bytes, at=0x04000000] ...	done
1

testing instruction sll
Loading segment 1 [base=0x00000000, sz=  1852 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction simple
Loading segment 1 [base=0x00000000, sz=   444 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction slli
Loading segment 1 [base=0x00000000, sz=  1148 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction slt
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction slti
Loading segment 1 [base=0x00000000, sz=  1084 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction sltiu
Loading segment 1 [base=0x00000000, sz=  1084 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction sltu
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction sra
Loading segment 1 [base=0x00000000, sz=  1916 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction srai
Loading segment 1 [base=0x00000000, sz=  1212 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction srl
Loading segment 1 [base=0x00000000, sz=  1916 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction srli
Loading segment 1 [base=0x00000000, sz=  1148 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction sub
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction sw
Loading segment 1 [base=0x00000000, sz=  1788 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
Loading segment 3 [base=0x04000000, sz=    48 bytes, at=0x04000000] ...	done
1

testing instruction xor
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction xori
Loading segment 1 [base=0x00000000, sz=   956 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1


Now test rv32um for riscv-atom!!
testing instruction div
Loading segment 1 [base=0x00000000, sz=   700 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction divu
Loading segment 1 [base=0x00000000, sz=   700 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction mul
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction mulh
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction mulhu
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction mulhsu
Loading segment 1 [base=0x00000000, sz=  1724 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction rem
Loading segment 1 [base=0x00000000, sz=   700 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

testing instruction remu
Loading segment 1 [base=0x00000000, sz=   700 bytes, at=0x00000000] ...	done
Loading segment 2 [base=0x00001000, sz=    72 bytes, at=0x00001000] ...	done
1

==============================
rv32ui-p instruction set :
The pass rate is 38/38
The fail rate is 0/38
Pass rv32ui-p testing! 
==============================
rv32um-p instruction set :
The pass rate is 8/8
The fail rate is 0/8
Pass rv32um-p testing! 
==============================
```
:::


## Issue and Pull Request
**We plan to hold a issue to the [autohr](https://github.com/saursin)**
We encounter some problem need to solve:
1. verilator PATH ERROR
    - In `~/riscv-atom/sim/makefile`, because our verilator is install by source code, therefore the PATH is not same with author.Hence, we need to change the path before push to github and CICD.
    ```makefile=44
    # This should point to verilator include directory
    VERILATOR_INCLUDE_PATH := /usr/share/verilator/include
    #VERILATOR_INCLUDE_PATH := /home/hanchi/verilator/include
    ```
2. verilator command
    - We find that the author's verilator version is not the same with us. Hence,we need to  change the /*verilator lint_off UNUSEDSIGNAL*/  to /* verilator lint_off UNUSED */
    - This version can pass github CICD
    ```verilog=92
    /* verilator lint_off UNUSED */
    wire [63:0] result_mulsu;
    /* verilator lint_off UNUSED */
    wire [63:0] result_mulu;
    /* verilator lint_off UNUSED */
    wire [31:0] result_div;
    ```
    - This version can run in our environment
    ```verilog=92
    /*verilator lint_off UNUSEDSIGNAL*/
    wire [63:0] result_mulsu;
    /*verilator lint_off UNUSEDSIGNAL*/
    wire [63:0] result_mulu;
    /*verilator lint_off UNUSEDSIGNAL*/
    wire [31:0] result_div;
    ```
    


## Reference Links
[Computer Archiecture 2022: Term Project](https://hackmd.io/@sysprog/arch2022-projects)
[saursin / riscv-atom](https://github.com/saursin/riscv-atom)
[RISC-V Atom Documentation & User Manual](https://riscv-atom.readthedocs.io/en/latest/index.html)
[srv32](https://github.com/sysprog21/srv32/blob/devel/rtl/riscv.v)
[riscv-spec](https://riscv.org/wp-content/uploads/2017/05/riscv-spec-v2.2.pdf)
[dhrystone](https://github.com/riscv-software-src/riscv-tests/tree/master/benchmarks/dhrystone)
[Cortex m0+](https://www.2cm.com.tw/2cm/zh-tw/tech/76DD94733BF744AE81137AAFDCD070A3)
