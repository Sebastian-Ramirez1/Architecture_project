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

    //Input Signals for Pipeline_IF_ID
    wire [63:0] IF_ID_in = {InstructionMemory_Out ,PC_Out};
    wire [63:0] IF_ID_out = {Instruction_ControlUnit, ID_PC};
    wire IFID_Reset_Signal;

    //Input Signals for Pipeline_ID_EX
    wire [172:0] ID_EX_in = {Mux_out[15:3], Mux_out[0]};
    assign {ID_PC, MUX_DataIn_Out, MUX_PA_Out, MUX_PB_Out, ID_RD_MUX, ID_31_30_24_13, ID_Imm, ID_jmpl_instr, ID_Read_Write, ID_ALU_op3, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_DataMem_enable} = ID_EX_in;
    assign ID_B_instr = Mux_out[2];
    assign ID_29_a = Mux_out[1];
    wire [31:0] ID_PC; 
    wire [31:0] ID_DataIn; 
    wire [31:0] ID_PA; 
    wire [31:0] ID_PB; 
    wire [21:0] ID_Imm = {Instruction_ControlUnit[21:0]};
    wire [3:0] ID_31_30_24_13 = {Instruction_ControlUnit[31], Instruction_ControlUnit[30], Instruction_ControlUnit[24], Instruction_ControlUnit[13]};
    wire [4:0] ID_RD = {Instruction_ControlUnit[29:25]};
    wire [4:0] ID_RD_MUX;
    wire [4:0] ID_RA = {Instruction_ControlUnit[18:14]};
    wire [4:0] ID_RB = {Instruction_ControlUnit[4:0]};
    wire [4:0] ID_RDataIn = {Instruction_ControlUnit[29:25]};
    wire [31:0] MUXPC_Out;
    wire [31:0] Target_Address;
    wire [31:0] nPC_Out;
    wire [1:0] MUX_IF_Signal;
    wire [1:0] Sig_DataHazard_EX;
    wire [1:0] Sig_DataHazard_MEM;
    wire [1:0] Sig_DataHazard_WB;
    wire [31:0] MUX_PA_Out;
    wire [31:0] MUX_PB_Out;
    wire [31:0] MUX_DataIn_Out;


    //Output Signals for PipelineRegister_ID_EX
    wire EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_modifyCC, EX_Call_instr, EX_DataMem_enable;
    wire [1:0] EX_size_dm; 
    wire [3:0] EX_ALU_op3;

    wire [172:0] ID_EX_out;
    assign {EX_PC, EX_DataIn, EX_PA, EX_PB, EX_RD, EX_31_30_24_13, EX_Imm, EX_jmpl_instr, EX_Read_Write, EX_ALU_op3, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_modifyCC, EX_Call_instr, EX_DataMem_enable} = ID_EX_out;
    wire [31:0] EX_PC; 
    wire [31:0] EX_DataIn; 
    wire [31:0] EX_PA; 
    wire [31:0] EX_PB; 
    wire [4:0] EX_RD; 
    wire [3:0] EX_31_30_24_13;
    wire [21:0] EX_Imm;
    wire [3:0] EX_ALU_flags;
    wire [31:0] SourceOperandHanlder_Out;
    wire EX_Cin;
    wire [3:0] PSR_Out;
    wire [3:0] MUXCC_Out;
    wire BranchCondition_Out;
    wire [29:0] ID_CallDisp30 = {Instruction_ControlUnit[29:0]};
    wire [21:0] ID_BranchDisp22 = {Instruction_ControlUnit[21:0]};
    wire [31:0] Mux_Call_Branch_Out;
    wire [29:0] ID_BranchDisp22_SE;
    wire [29:0] Multiplicador4_Out;

    //Input signals EX_MEM
    wire [109:0] EX_MEM_in = {EX_PC, EX_DataIn, EX_ALU_Out, EX_RD, EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_Call_instr, EX_DataMem_enable};
    wire [31:0] EX_ALU_Out;

    //Output Signals for PipelineRegister_EX_MEM
    wire MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_Call_instr, MEM_DataMem_enable;
    wire [1:0] MEM_size_dm;
    wire [109:0] EX_MEM_out;
    assign {MEM_PC, MEM_DataIn, MEM_ALU_Out, MEM_RD, MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_size_dm, MEM_Call_instr, MEM_DataMem_enable} = EX_MEM_out;
    wire [31:0] Load_Data_Out;
    wire [31:0] MEM_PC;
    wire [31:0] MEM_DataIn;
    wire [31:0] MEM_ALU_Out;
    wire [4:0] MEM_RD;
    wire [31:0] MEM_Load_Data;

    //Input Signals MEM_WB
    wire [37:0] MEM_WB_in;
    assign {MEM_PW, MEM_RD, MEM_RF_enable} = MEM_WB_in;
    wire [31:0] MEM_PW;
    
    //Output Signals for PipelineRegister_MEM_WB
    wire [37:0] MEM_WB_out;
    assign {WB_PW, WB_RD, WB_RF_enable} = MEM_WB_out;
    wire WB_RF_enable;
    wire [31:0] WB_PW;
    wire [4:0] WB_RD;

    wire [31:0] Instruction_ControlUnit; //Output_Pipeline_Register_IF_ID => Input_ControlUnit and Input_PipelineRegister_ID_EX

    integer fr, fw, code; //variables para leer archivo
    reg [7:0] FileData; //variablele to store the data from file
    reg [7:0] Address; //variable to indicate where to store in the instruction memory

    wire [31:0] PC_In; //Output_nPC => Input_PC
    wire [31:0] PC_Out; //Ouput_PC => Input_Sumador4 and Input_InstructionMemory
    wire [31:0] Sumador4_Out; //Output Sumador4 => Input_nPC

    wire [31:0] InstructionMemory_Out; //Output_InsturctionMemory => Input_PipeplineRegister_IF_ID

    //IF
    PC PC(PC_Out, Clk, MUXPC_Out, LE, R); //instancia de PC
    Sumador4 Sumador4(Sumador4_Out, PC_In); // instancia de Sumador de PC
    nPC nPC(nPC_Out, Clk, Sumador4_Out, LE, R); // instancia de nPC
    MUXPC MUXPC (MUXPC_Out, EX_ALU_Out, Target_Address, nPC_Out, MUX_IF_Signal);
    InstructionMemory InstructionMemory(InstructionMemory_Out, PC_Out); //instancia de instruction memory
    MUXPCIFID_Reset_Handler  MUXPCIFID_Reset_Handler(IFID_Reset_Signal, MUX_IF_Signal, BranchCondition_Out, EX_jmpl_instr, ID_Call_instr, ID_29_a, ID_B_instr);
   
    //IF

    PipelineRegister_IF_ID PipelineRegister_IF_ID(IF_ID_out, Clk, IF_ID_in, LE, R);

    ControlUnit ControlUnit(Control_Unit_Out, Instruction_ControlUnit);

    MuxControlSignal MuxControlSignal(Mux_out, S, Control_Unit_Out);

    //ID
    DISP22SE DISP22SE (ID_BranchDisp22_SE, ID_BranchDisp22);
    MUXCALLORBRANCH MUXCALLORBRANCH (Mux_Call_Branch_Out, ID_BranchDisp22_SE, ID_CallDisp30, ID_Call_instr);
    Multiplicador4 Multiplicador4(Multiplicador4_Out, Mux_Call_Branch_Out);
    Sumador_TA Sumador_TA(Target_Address, ID_PC, Multiplicador4_Out);

    RegisterFile RegisterFile(ID_PA, ID_PB, ID_DataIn, ID_RA, ID_RB, ID_RDataIn, WB_RD, WB_PW, Clk, WB_RF_enable);
    MUX_DataFowarding MUX_PA(MUX_PA_Out, ID_PA, EX_ALU_Out, MEM_PW, WB_PW, Sig_DataHazard_EX);
    MUX_DataFowarding MUX_PB(MUX_PB_Out, ID_PB, EX_ALU_Out, MEM_PW, WB_PW, Sig_DataHazard_MEM);
    MUX_DataFowarding MUX_DataIn(MUX_DataIn_Out, ID_DataIn, EX_ALU_Out, MEM_PW, WB_PW, Sig_DataHazard_WB);
    ID_MUX_RD ID_MUX_RD(ID_RD_MUX, ID_RD, ID_Call_instr);
    //ID

    PipelineRegister_ID_EX PipelineRegister_ID_EX(ID_EX_out, Clk, ID_EX_in, R);

    //EX
    ALU ALU(EX_ALU_Out, EX_ALU_flags, EX_ALU_op3, EX_PA, SourceOperandHanlder_Out, EX_Cin);
    Source_Operand2_Handler Source_Operand2_Handler(SourceOperandHanlder_Out, EX_31_30_24_13, EX_PB, EX_Imm);

    ProgramStatusRegister PSR(PSR_Out, EX_ALU_flags, EX_modifyCC, EX_Cin);
    MUX_CC MUX_CC(MUXCC_Out, EX_ALU_flags, PSR_Out, EX_modifyCC);
    //EX

    PipelineRegister_EX_MEM PipelineRegister_EX_MEM(EX_MEM_out, Clk, EX_MEM_in, R);

    //MEM
    DataMemory DataMemory(MEM_Load_Data, MEM_Read_Write, MEM_ALU_Out, MEM_DataIn, MEM_size_dm, MEM_SE_dm, MEM_DataMem_enable); //DataMemory
    MEM_MUX_RF MEM_MUX_RF (MEM_PW, MEM_PC, MEM_ALU_Out, MEM_Load_Data, MEM_load_instr, MEM_jmpl_instr, MEM_call_instr); //MUX MEM
    //MEM

    PipelineRegister_MEM_WB PipelineRegister_MEM_WB(MEM_WB_out, Clk, MEM_WB_in, R);

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
        //$monitor("PC: %d, MemoryAdress: %d, R1: %d, R3: %d, R5: %d, R8: %d, R10: %d, R11: %d, R12: %d, Time: %d", PC_Out, MEM_ALU_Out, RegisterFile.Q1, RegisterFile.Q3, RegisterFile.Q5, RegisterFile.Q8, RegisterFile.Q10, RegisterFile.Q11, RegisterFile.Q12, $time); //R3: %d, R5: %d, R8: %d, R10: %d, R11: %d, R12: %d

    end
    
endmodule