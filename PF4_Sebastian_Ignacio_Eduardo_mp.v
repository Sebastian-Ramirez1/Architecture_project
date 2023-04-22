`include "PF4_Sebastian_Ignacio_Eduardo.v"

module PF4ModuloPrueba;

    //inputs and outputs for MuxControlSignal
    reg S; //MUXControlSignal Signal
    reg Clk; //Clock Signal 
    reg LE; //Load Enable Signal
    reg R; //Reset Signal

    //Input and Output for Control Unit and MUXControlSignal
    wire [15:0] Control_Unit_Out;

    wire ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, ID_DataMem_enable;
    wire [1:0] ID_size_dm; 
    wire [3:0] ID_ALU_op3;
    
    wire [15:0] Mux_out;

    wire [13:0] ID_EX_in = {Mux_out[15:3], Mux_out[0]};
    assign {ID_jmpl_instr, ID_Read_Write, ID_ALU_op3, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_DataMem_enable} = ID_EX_in;
    assign ID_B_instr = Mux_out[2];
    assign ID_29_a = Mux_out[1];
    //Output Control Signals for PipelineRegister_ID_EX
    wire EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_modifyCC, EX_Call_instr, EX_DataMem_enable;
    wire [1:0] EX_size_dm; 
    wire [3:0] EX_ALU_op3;

    wire [13:0] ID_EX_out;
    assign {EX_jmpl_instr, EX_Read_Write, EX_ALU_op3, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_modifyCC, EX_Call_instr, EX_DataMem_enable} = ID_EX_out;
    wire [8:0] EX_MEM_in = {EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_Call_instr, EX_DataMem_enable};
    //Output Control Signals for PipelineRegister_EX_MEM
    wire MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_Call_instr, MEM_DataMem_enable;
    wire [1:0] MEM_size_dm;

    wire [8:0] EX_MEM_out;
    assign {MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_size_dm, MEM_Call_instr, MEM_DataMem_enable} = EX_MEM_out;
    wire MEM_WB_in = MEM_RF_enable;
    //Output Control Signals for PipelineRegister_MEM_WB
    wire WB_RF_enable;

    wire [31:0] Instruction_ControlUnit; //Output_Pipeline_Register_IF_ID => Input_ControlUnit and Input_PipelineRegister_ID_EX

    integer fr, fw, code; //variables para leer archivo
    reg [7:0] FileData; //variablele to store the data from file
    reg [7:0] Address; //variable to indicate where to store in the instruction memory

    wire [31:0] PC_In; //Output_nPC => Input_PC
    wire [31:0] PC_Out; //Ouput_PC => Input_Sumador4 and Input_InstructionMemory
    wire [31:0] Sumador4_Out; //Output Sumador4 => Input_nPC

    wire [31:0] InstructionMemory_Out; //Output_InsturctionMemory => Input_PipeplineRegister_IF_ID

    PC PC(PC_Out, Clk, PC_In, LE, R); //instancia de PC
    Sumador4 Sumador4(Sumador4_Out, PC_In); // instancia de Sumador de PC
    nPC nPC(PC_In, Clk, Sumador4_Out, LE, R); // instancia de nPC
    
    InstructionMemory InstructionMemory(InstructionMemory_Out, PC_Out); //instancia de instruction memory

    PipelineRegister_IF_ID PipelineRegister_IF_ID(Instruction_ControlUnit, Clk, InstructionMemory_Out, LE, R);

    ControlUnit ControlUnit(Control_Unit_Out, Instruction_ControlUnit);

    MuxControlSignal MuxControlSignal(Mux_out, S, Control_Unit_Out);

    PipelineRegister_ID_EX PipelineRegister_ID_EX(ID_EX_out, Clk, ID_EX_in, R);
    //Todas senales en una variable
    //Q, D, Clk, R ||

    PipelineRegister_EX_MEM PipelineRegister_EX_MEM(EX_MEM_out, Clk, EX_MEM_in, R);

    PipelineRegister_MEM_WB PipelineRegister_MEM_WB(WB_RF_enable, Clk, MEM_WB_in, R);

    initial #56 $finish;

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
        Clk = 1'b0; //La simulación debe comenzar inicializando Clk en cero a tiempo cero. Entonces, debe cambiar de estado cada dos unidades de tiempo de manera perpetua.
        LE = 1'b1;
        forever #2 Clk = ~Clk;
    end

    initial begin
        R = 1'b1; // La señal Reset debe tener un valor de 1 a tiempo cero y cambiar a 0 en tiempo 1. 
        #3 R = ~R;
    end

    initial begin
        S = 1'b0;  //La señal S del multiplexer debe tener un valor de cero a tiempo cero y debe cambiar a 1 a tiempo 40.
        #40 S = ~S; 
    end

    initial begin
        $monitor("Instruccion: %b PC: %d nPC: %d Clk: %b  Reset: %b  LE: %b  S: %b  Time: %d \n ID_ALU_op3 %b ID_jmpl_instr: %b , ID_Read_Write: %b , ID_SE_dm: %b , ID_load_instr: %b , ID_RF_enable: %b , ID_size_dm: %b , ID_modifyCC: %b , ID_Call_instr: %b , ID_B_instr: %b , ID_29_a: %b , ID_DataMem_Enable: %b \n EX_jmpl_instr: %b, EX_ALU_op: %b , EX_Read_Write: %b, EX_SE_dm: %b , EX_load_instr: %b , EX_RF_enable: %b, EX_size_dm: %b , EX_modifyCC: %b , EX_call_instr: %b , EX_DataMem_enable: %b \n MEM_jmpl_instr: %b , MEM_Read_Write: %b , MEM_SE_dm: %b , MEM_load_instr: %b , MEM_RF_enable: %b , MEM_size_dm: %b , MEM_call_instr: %b , MEM_DataMem_enable: %b \n WB_RF_enable: %b\n", Instruction_ControlUnit, PC_Out, PC_In, Clk, R, LE, S, $time, ID_ALU_op3, ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, ID_DataMem_enable, EX_jmpl_instr, EX_ALU_op3, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_modifyCC, EX_Call_instr, EX_DataMem_enable, MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_size_dm, MEM_Call_instr, MEM_DataMem_enable, WB_RF_enable);

    end
    
endmodule