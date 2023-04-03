`include "PF3.v"

module ControlUnitModuloPrueba;

    //inputs and outputs for MuxControlSignal
    reg S; //MUXControlSignal Signal
    reg Clk; //Clock Signal 
    reg LE; //Load Enable Signal
    reg R; //Reset Signal

    //Input and Output for Control Unit and MUXControlSignal
    wire [5:0] ID_ALU_op3; //ALU_op3 input sigal
    wire [1:0] ID_op;
    wire [1:0] ID_size_dm;
    wire ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a; //output ControlUnit => input for MuxControlUNit

    wire ID_jmpl_instr_out, ID_Read_Write_out, ID_SE_dm_out, ID_load_instr_out, ID_RF_enable_out, ID_modifyCC_out, ID_Call_instr_out, ID_B_instr_out, ID_29_a_out;
    wire [1:0] ID_size_dm_out; 
    wire [5:0] ID_ALU_op3_out;

    //Output Control Signals for PipelineRegister_ID_EX
    wire EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_modifyCC;
    wire [1:0] EX_size_dm; 
    wire [5:0] EX_ALU_op3;

    //Output Control Signals for PipelineRegister_EX_MEM
    wire MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable;
    wire [1:0] MEM_size_dm;
    
    //Output Control Signals for PipelineRegister_MEM_WB
    wire WB_RF_enable;

    wire [31:0] PipelineRegister_IF_ID_Out; //Output_Pipeline_Register_IF_ID => Input_ControlUnit and Input_PipelineRegister_ID_EX
    wire [31:0] PipelineRegister_ID_EX_Out; //Output_Pipeline_Register_ID_EX => Input_PipelineRegister_EX_MEM
    wire [31:0] PipelineRegister_EX_MEM_Out; //Output_Pipeline_Register_EX_MEM => Input_PipelineRegister_MEM_WB
    wire [31:0] PipelineRegister_MEM_WB_Out; //Output_Pipeline_Register_MEM_WB => Input_PW_RegisterFile

    integer fr, fw, code; //variables para leer archivo
    reg [7:0] FileData; //variablele to store the data from file
    reg [7:0] Address; //variable to indicate where to store in the instruction memory

    reg [31:0]Instr; //INstruction to test Control Unit

    wire [7:0] PC_In; //Output_nPC => Input_PC
    wire [7:0] PC_Out; //Ouput_PC => Input_Sumador4 and Input_InstructionMemory
    wire [7:0] Sumador4_Out; //Output Sumador4 => Input_nPC
    wire [31:0] InstructionMemory_Out; //Output_InsturctionMemory => Input_PipeplineRegister_IF_ID

    PC PC(Clk, PC_In, PC_Out, LE, R); //instancia de PC
    Sumador4 Sumador4(PC_Out, Sumador4_Out); // instancia de Sumador de PC
    nPC nPC(Clk, Sumador4_Out, PC_In, LE, R); // instancia de nPC
    
    InstructionMemory InstructionMemory(InstructionMemory_Out, PC_Out); //instancia de instruction memory

    PipelineRegister_IF_ID PipelineRegister_IF_ID(Clk, InstructionMemory_Out, PipelineRegister_IF_ID_Out, LE, R);

    ControlUnit ControlUnit(PipelineRegister_IF_ID_Out, ID_op, ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, ID_ALU_op3);
    MuxControlSignal MuxControlSignal(S, ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_ALU_op3, ID_jmpl_instr_out, ID_Read_Write_out, ID_SE_dm_out, ID_load_instr_out, ID_RF_enable_out, ID_size_dm_out, ID_modifyCC_out, ID_Call_instr_out, ID_B_instr_out, ID_ALU_op3_out);

    PipelineRegister_ID_EX PipelineRegister_ID_EX(Clk, PipelineRegister_IF_ID_Out, PipelineRegister_ID_EX_Out, ID_jmpl_instr_out, ID_Read_Write_out, ID_ALU_op3_out, ID_SE_dm_out, ID_load_instr_out, ID_RF_enable_out, ID_size_dm_out, ID_modifyCC_out, EX_jmpl_instr, EX_Read_Write, EX_ALU_op3, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_modifyCC);

    PipelineRegister_EX_MEM PipelineRegister_EX_MEM(Clk, PipelineRegister_ID_EX_Out, PipelineRegister_EX_MEM_Out, EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_size_dm);

    PipelineRegister_MEM_WB PipelineRegister_MEM_WB(Clk, PipelineRegister_EX_MEM_Out, PipelineRegister_MEM_WB_Out, MEM_RF_enable, WB_RF_enable);

    initial #52 $finish;

    //Precargar file a Intruction Memory
    initial begin
        fr = $fopen("Fase3Memory.txt", "r");
        Address = 8'b00000000;
        while(!$feof(fr)) //fin del file
            begin
                code = $fscanf(fr, "%b", FileData); //leer del file un dato
                InstructionMemory.Mem[Address] = FileData;
                Address = Address + 1;
            end
            $fclose(fr); //cerrar file de lectura
            Address = 8'b00000000; //make sure adress starts back in 0 after precharge
    end

    initial begin
        Clk = 0; //La simulación debe comenzar inicializando Clk en cero a tiempo cero. Entonces, debe cambiar de estado cada dos unidades de tiempo de manera perpetua.
        LE = 1'b1;
        forever #2 Clk = ~Clk;
    end

    initial begin
        R = 1; // La señal Reset debe tener un valor de 1 a tiempo cero y cambiar a 0 en tiempo 1. 
        #1 R = 0;

    end

    initial begin
        S = 0;  //La señal S del multiplexer debe tener un valor de cero a tiempo cero y debe cambiar a 1 a tiempo 40.
        #40 S = ~S; 
    end

    initial begin
        $monitor("Instruccion: %b PC: %d nPC: %d Clk: %b  Reset: %b  LE %b  S %b  Time: %d \n ID_op %b ID_ALU_op3 %b ID_jmpl_instr: %b , ID_Read_Write: %b , ID_SE_dm: %b , ID_load_instr: %b , ID_RF_enable: %b , ID_size_dm: %b , ID_modifyCC: %b , ID_Call_instr: %b , ID_B_instr: %b , ID_29_a: %b \n EX_jmpl_instr: %b, EX_ALU_op: %b , EX_Read_Write: %b, EX_SE_dm: %b , EX_load_instr: %b , EX_RF_enable: %b, EX_size_dm: %b , EX_modifyCC: %b \n MEM_jmpl_instr: %b , MEM_Read_Write: %b , MEM_SE_dm: %b , MEM_load_instr: %b , MEM_RF_enable: %b , MEM_size_dm: %b \n WB_RF_enable: %b\n", PipelineRegister_IF_ID_Out, PC_Out, PC_In, Clk, R, LE, S, $time, ID_op, ID_ALU_op3, ID_jmpl_instr_out, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, EX_jmpl_instr, EX_ALU_op3, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_modifyCC, MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_size_dm, WB_RF_enable);

    end
     
endmodule