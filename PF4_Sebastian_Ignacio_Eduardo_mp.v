`include "PF4_Sebastian_Ignacio_Eduardo.v"

module PF4ModuloPrueba;

    //inputs and outputs for MuxControlSignal
    //reg S; //MUXControlSignal Signal
    reg Clk; //Clock Signal 
    reg LE; //Load Enable Signal
    reg R; //Reset Signal

    wire S; //MUX_NOP
    wire IF_ID_enable;
    wire PC_nPC_enable;

    //Input and Output for Control Unit and MUXControlSignal
    wire [31:0] Instruction_ControlUnit; //Output_Pipeline_Register_IF_ID => Input_ControlUnit and Input_PipelineRegister_ID_EX
    wire [15:0] Control_Unit_Out;
    wire ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, ID_DataMem_enable;
    wire [1:0] ID_size_dm; 
    wire [3:0] ID_ALU_op3;


    wire [13:0] Mux_in = {Control_Unit_Out[15:2]};
    assign ID_29_a = Control_Unit_Out[0];
    assign ID_B_instr = Control_Unit_Out[1];
    assign ID_Call_instr = Control_Unit_Out[3];
    wire [13:0] Mux_out;

    //Input Signals for Pipeline_IF_ID
    wire [31:0] InstructionMemory_Out; //Output_InsturctionMemory => Input_PipeplineRegister_IF_ID
    wire [31:0] MUXPC_Out;
    wire [1:0] MUX_IF_Signal;
    wire [31:0] PC_Out; //Ouput_PC => Input_Sumador4 and Input_InstructionMemory
    wire [31:0] Sumador4_Out; //Output Sumador4 => Input_nPC
    wire [31:0] nPC_Out;
    
    wire [63:0] IF_ID_in = {InstructionMemory_Out, PC_Out};
    wire [63:0] IF_ID_out;

    //wire IFID_Reset_Signal;

    //Input Signals for Pipeline_ID_EX
    wire [31:0] ID_PC;
    assign {Instruction_ControlUnit, ID_PC} = IF_ID_out;

    wire [29:0] ID_CallDisp30 = {Instruction_ControlUnit[29:0]};
    wire [21:0] ID_BranchDisp22 = {Instruction_ControlUnit[21:0]};
    wire [31:0] ID_BranchDisp22_SE;
    wire [31:0] Mux_Call_Branch_Out;
    wire [31:0] Multiplicador4_Out;
    wire [31:0] Target_Address;

    wire [4:0] ID_RA = {Instruction_ControlUnit[18:14]};
    wire [4:0] ID_RD = {Instruction_ControlUnit[29:25]};
    wire [4:0] ID_RD_MUX;
    wire [4:0] ID_RB = {Instruction_ControlUnit[4:0]};
    wire [31:0] ID_PA; 
    wire [31:0] ID_DataIn; 
    wire [31:0] ID_PB; 
    wire [1:0] Sig_MUX_PA;
    wire [1:0] Sig_MUX_DataIn;
    wire [1:0] Sig_MUX_PB;
    wire [31:0] MUX_PA_Out;
    wire [31:0] MUX_DataIn_Out;
    wire [31:0] MUX_PB_Out;
    
    wire [3:0] ID_31_30_24_13 = {Instruction_ControlUnit[31], Instruction_ControlUnit[30], Instruction_ControlUnit[24], Instruction_ControlUnit[13]};
    wire I23 = Instruction_ControlUnit[23];
    wire [21:0] ID_Imm = {Instruction_ControlUnit[21:0]};
    wire [3:0] InstrCondIF = {Instruction_ControlUnit[28:25]};

    wire [173:0] ID_EX_in = {ID_PC, MUX_DataIn_Out, MUX_PA_Out, MUX_PB_Out, ID_RD_MUX, ID_31_30_24_13, I23 ,ID_Imm, Mux_out};
    
    // ID Control Signals for Monitor Function
    assign ID_jmpl_instr = Mux_out[13];
    assign ID_Read_Write = Mux_out[12];
    assign ID_ALU_op3 = Mux_out[11:8];
    assign ID_SE_dm = Mux_out[7]; 
    assign ID_load_instr = Mux_out[6];
    assign ID_RF_enable = Mux_out[5]; 
    assign ID_size_dm = Mux_out[4:3];
    assign ID_modifyCC = Mux_out[2];
    assign ID_Call_instr = Mux_out[1];
    assign ID_DataMem_enable = Mux_out[0];
    
    // wire [4:0] ID_RDataIn = {Instruction_ControlUnit[29:25]};
    
    //Output Signals for PipelineRegister_ID_EX
    wire [173:0] ID_EX_out;
    
    wire [31:0] EX_PC; 
    wire [31:0] EX_DataIn; 
    wire [4:0] EX_RD; 

    wire [31:0] EX_PA; 
    wire [31:0] EX_PB;
    wire [21:0] EX_Imm;
    wire [3:0] EX_31_30_24_13;
    wire [31:0] SourceOperandHanlder_Out;
    wire EX_Cin;
    wire [31:0] EX_ALU_Out;
    wire [3:0] EX_ALU_flags;

    wire [3:0] PSR_Out;
    wire [3:0] MUXCC_Out;
    wire BranchCondition_Out;

    wire EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_modifyCC, EX_Call_instr, EX_DataMem_enable;
    wire [1:0] EX_size_dm; 
    wire [3:0] EX_ALU_op3;

    assign {EX_PC, EX_DataIn, EX_PA, EX_PB, EX_RD, EX_31_30_24_13, EX_I23 ,EX_Imm, EX_jmpl_instr, EX_Read_Write, EX_ALU_op3, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_modifyCC, EX_Call_instr, EX_DataMem_enable} = ID_EX_out;

    //Input signals EX_MEM
    wire [109:0] EX_MEM_in = {EX_PC, EX_DataIn, EX_ALU_Out, EX_RD, EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_Call_instr, EX_DataMem_enable};

    //Output Signals for PipelineRegister_EX_MEM
    wire [109:0] EX_MEM_out;

    wire [31:0] MEM_PC;
    wire [4:0] MEM_RD;
    wire [31:0] MEM_DataIn;
    wire [31:0] MEM_ALU_Out;
    wire [31:0] MEM_Load_Data;
    wire [31:0] MEM_PW;

    wire MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_Call_instr, MEM_DataMem_enable;
    wire [1:0] MEM_size_dm;

    assign {MEM_PC, MEM_DataIn, MEM_ALU_Out, MEM_RD, MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_size_dm, MEM_Call_instr, MEM_DataMem_enable} = EX_MEM_out;

    // wire [31:0] Load_Data_Out;
    
    //Input Signals MEM_WB
    wire [37:0] MEM_WB_in;
    assign MEM_WB_in = {MEM_PW, MEM_RD, MEM_RF_enable};
    
    //Output Signals for PipelineRegister_MEM_WB
    wire [37:0] MEM_WB_out;

    wire [31:0] WB_PW;
    wire [4:0] WB_RD;

    wire WB_RF_enable;

    assign {WB_PW, WB_RD, WB_RF_enable} = MEM_WB_out;
    
    // Code for Loading Instructions
    integer fr, fw, code, fo; //variables para leer archivo
    reg [7:0] FileData; //variablele to store the data from file
    reg [7:0] Address; //variable to indicate where to store in the instruction memory

    // wire [31:0] PC_In; //Output_nPC => Input_PC
    

// ETAPA IF
    PC PC(PC_Out, Clk, MUXPC_Out, PC_nPC_enable, R); //instancia de PC
    Sumador4 Sumador4(Sumador4_Out, MUXPC_Out); // instancia de Sumador de PC
    nPC nPC(nPC_Out, Clk, Sumador4_Out, PC_nPC_enable, R); // instancia de nPC
    MUXPC MUXPC (MUXPC_Out, EX_ALU_Out, Target_Address, nPC_Out, MUX_IF_Signal);
    InstructionMemory InstructionMemory(InstructionMemory_Out, PC_Out); //instancia de instruction memory
    MUXPCIFID_Reset_Handler  MUXPCIFID_Reset_Handler(IFID_Reset_Signal, MUX_IF_Signal, BranchCondition_Out, EX_jmpl_instr, ID_Call_instr, ID_29_a, ID_B_instr);
// ETAPAIF

    //PipelineRegister_IF_ID PipelineRegister_IF_ID(IF_ID_out, Clk, IF_ID_in, LE, R);
    PipelineRegister_IF_ID PipelineRegister_IF_ID(IF_ID_out, Clk, IF_ID_in, IF_ID_enable, R, IFID_Reset_Signal);

    ControlUnit ControlUnit(Control_Unit_Out, Instruction_ControlUnit);

    MuxControlSignal MuxControlSignal(Mux_out, S, Mux_in);

//ETAPA ID
    DISP22SE DISP22SE (ID_BranchDisp22_SE, ID_BranchDisp22);
    MUXCALLORBRANCH MUXCALLORBRANCH (Mux_Call_Branch_Out, ID_BranchDisp22_SE, ID_CallDisp30, ID_Call_instr);
    Multiplicador4 Multiplicador4(Multiplicador4_Out, Mux_Call_Branch_Out);
    Sumador_TA Sumador_TA(Target_Address, ID_PC, Multiplicador4_Out);

    RegisterFile RegisterFile(ID_PA, ID_PB, ID_DataIn, ID_RA, ID_RB, ID_RD, WB_RD, WB_PW, Clk, WB_RF_enable);
    MUX_DataFowarding MUX_PA(MUX_PA_Out, ID_PA, EX_ALU_Out, MEM_PW, WB_PW, Sig_MUX_PA);
    MUX_DataFowarding MUX_PB(MUX_PB_Out, ID_PB, EX_ALU_Out, MEM_PW, WB_PW, Sig_MUX_PB);
    MUX_DataFowarding MUX_DataIn(MUX_DataIn_Out, ID_DataIn, EX_ALU_Out, MEM_PW, WB_PW, Sig_MUX_DataIn);
    ID_MUX_RD ID_MUX_RD(ID_RD_MUX, ID_RD, ID_Call_instr);

    Hazard_Unit Hazard_Unit(Sig_MUX_PA, Sig_MUX_PB, Sig_MUX_DataIn, IF_ID_enable, PC_nPC_enable, S, ID_RA, ID_RB, ID_RD, EX_RD, MEM_RD, WB_RD, EX_RF_enable, MEM_RF_enable, WB_RF_enable, EX_load_instr, ID_Read_Write);
//ETAPA ID

    PipelineRegister_ID_EX PipelineRegister_ID_EX(ID_EX_out, Clk, ID_EX_in, R);

// ETAPA EX
    ALU ALU(EX_ALU_Out, EX_ALU_flags, EX_ALU_op3, EX_PA, SourceOperandHanlder_Out, EX_Cin);
    Source_Operand2_Handler Source_Operand2_Handler(SourceOperandHanlder_Out, EX_31_30_24_13, EX_PB, EX_Imm, EX_I23);

    ProgramStatusRegister PSR(EX_Cin, PSR_Out, EX_ALU_flags, EX_modifyCC);
    MUX_CC MUX_CC(MUXCC_Out, EX_ALU_flags, PSR_Out, EX_modifyCC);
    ConditionHandlerBranch ConditionHandlerBranch(BranchCondition_Out, MUXCC_Out, InstrCondIF, ID_B_instr);
//EX

    PipelineRegister_EX_MEM PipelineRegister_EX_MEM(EX_MEM_out, Clk, EX_MEM_in, R);

//ETAPA MEM
    DataMemory DataMemory(MEM_Load_Data, MEM_Read_Write, MEM_ALU_Out, MEM_DataIn, MEM_size_dm, MEM_SE_dm, MEM_DataMem_enable); //DataMemory
    MEM_MUX_RF MEM_MUX_RF (MEM_PW, MEM_PC, MEM_ALU_Out, MEM_Load_Data, MEM_load_instr, MEM_jmpl_instr, MEM_Call_instr); //MUX MEM
//MEM

//ETAPA WB
    // PipelineRegister_MEM_WB PipelineRegister_MEM_WB({WB_PW, WB_RD, WB_RF_enable}, Clk, {MEM_PW, MEM_RD, MEM_RF_enable}, R);
    PipelineRegister_MEM_WB PipelineRegister_MEM_WB(MEM_WB_out, Clk, MEM_WB_in, R);

    initial #250 $finish;

    //Precargar file a Intruction Memory
    initial begin
        fr = $fopen("testcode_sparc2.txt", "r");
        Address = 32'b00000000000000000000000000000000;
        while(!$feof(fr)) //fin del file
            begin
                code = $fscanf(fr, "%b", FileData); //leer del file un dato
                InstructionMemory.Mem[Address] = FileData;
                Address = Address + 1;
            end
            $fclose(fr); //cerrar file de lectura
            Address = 32'b00000000000000000000000000000000; //make sure adress starts back in 0 after precharge
    end

    initial begin
        fr = $fopen("testcode_sparc2.txt","r");
        Address = 32'b00000000000000000000000000000000;
        while (!$feof(fr)) 
            begin
                code = $fscanf(fr, "%b", FileData);
                DataMemory.Mem[Address] = FileData;
                Address = Address + 1;
            end
    
        $fclose(fr);
        Address = 32'b00000000000000000000000000000000; //make sure adress starts back in 0 after precharge
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
        //S = 1'b0;  //La señal S del multiplexer debe tener un valor de cero a tiempo cero y debe cambiar a 1 a tiempo 40.
        //#40 S = ~S; 
    end

    initial begin
        //General Signals
        // $monitor("Instruccion_IF: %b PC: %d nPC: %d Reset: %b  pC_nPC_enable: %b, IF_ID_Enable: %b  S: %b  Time: %d \n Intruction_ID: %b, ID_ALU_op3 %b ID_jmpl_instr: %b , ID_Read_Write: %b , ID_SE_dm: %b , ID_load_instr: %b , ID_RF_enable: %b , ID_size_dm: %b , ID_modifyCC: %b , ID_Call_instr: %b , ID_B_instr: %b , ID_29_a: %b , ID_DataMem_Enable: %b, ID_RA: %d, ID_RB: %d, ID_RD: %d, ID_RDataIN: %d, ID_PC: %d, sig_MUX_PA: %b, MUX_PA_Out: %d, sig_MUX_PB: %b, MUX_PB_Out: %d, Sig_MUX_DataIn: %b, MUX_DataIn_Out: %d, BranchCOndition: %b, MUX_IF_signal: %b, Target_Address: %d, ID_BranchDisp22: %d, ID_BranchDisp22_SE: %d, Multiplicador4_Out: %d, Z: %b \n EX_jmpl_instr: %b, EX_ALU_op: %b , EX_Read_Write: %b, EX_SE_dm: %b , EX_load_instr: %b , EX_RF_enable: %b, EX_size_dm: %b , EX_modifyCC: %b , EX_call_instr: %b , EX_DataMem_enable: %b, EX_RD: %d, EX_PC: %d, EX_ALU_Out: %d, EX_PA: %d, EX_PB: %d, EX_Imm: %d, SourceOperandHanlder_Out: %d, EX_ALU_Flags: %b, PSR_Out: %b, MUXCC_Out: %b \n MEM_jmpl_instr: %b , MEM_Read_Write: %b , MEM_SE_dm: %b , MEM_load_instr: %b , MEM_RF_enable: %b , MEM_size_dm: %b , MEM_call_instr: %b , MEM_DataMem_enable: %b, MEM_RD: %d, MEM_PC: %d, MEM_PW: %d, MEM_Data_Load: %d, MEM_ALU_OUT: %d \n WB_RF_enable: %b, WB_RD: %d, WB_PW: %d\n", 
        // InstructionMemory_Out, PC_Out, nPC_Out, IFID_Reset_Signal, PC_nPC_enable, IF_ID_enable , S, $time, Instruction_ControlUnit ,ID_ALU_op3, ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, ID_DataMem_enable, ID_RA, ID_RB, ID_RD, /*ID_RDataIn*/ ID_RD, ID_PC, Sig_MUX_PA, MUX_PA_Out, Sig_MUX_PB, MUX_PB_Out, Sig_MUX_DataIn, MUX_DataIn_Out, BranchCondition_Out, MUX_IF_Signal, Target_Address, ID_BranchDisp22, ID_BranchDisp22_SE, Multiplicador4_Out, ConditionHandlerBranch.Z,
        // EX_jmpl_instr, EX_ALU_op3, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_modifyCC, EX_Call_instr, EX_DataMem_enable, EX_RD, EX_PC, EX_ALU_Out, EX_PA, EX_PB, EX_Imm, SourceOperandHanlder_Out, EX_ALU_flags, PSR_Out, MUXCC_Out,
        // MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_size_dm, MEM_Call_instr, MEM_DataMem_enable, MEM_RD, MEM_PC, MEM_PW, MEM_Load_Data, MEM_ALU_Out,
        // WB_RF_enable, WB_RD, WB_PW);

        //Programa Prueba
        // $monitor("PC: %d, R5: %d, R6: %d, R16: %d, R17: %d, R18: %d, Time: %d", 
        // PC_Out, RegisterFile.Q5, RegisterFile.Q6, RegisterFile.Q16, RegisterFile.Q17, RegisterFile.Q18, $time);

        //Programa #1
        // $monitor("PC: %d, R1: %d, R2: %d, R3: %d, R5: %d, R12: %d, Time: %d", 
        // PC_Out, RegisterFile.Q1, RegisterFile.Q2, RegisterFile.Q3, RegisterFile.Q5, RegisterFile.Q12, $time);

        $monitor("PC: %d, R1: %d, R2: %d, R3: %d, R4: %d, R5: %d, R8: %d ,R10: %d, R11: %d, R12: %d, R15: %d, Intruccion_ID: %b, IF_ID_enable: %d, ControlUnit_ModifyCC: %b, EX_Modify: %b, MUX_DataIn: %b, MEM_ALU: %d, MEM_DataIn: %b, MEM_DataEnable: %b, MEM_ReadWrite: %b ,Time: %d", 
        PC_Out, RegisterFile.Q1, RegisterFile.Q2, RegisterFile.Q3, RegisterFile.Q4 ,RegisterFile.Q5, RegisterFile.Q8, RegisterFile.Q10, RegisterFile.Q11, RegisterFile.Q12, RegisterFile.Q15, Instruction_ControlUnit, IF_ID_enable ,Control_Unit_Out[4] ,EX_modifyCC, MUX_DataIn_Out,MEM_ALU_Out, MEM_DataIn, MEM_DataMem_enable, MEM_Read_Write, $time);
        #240
        $display("Address: %d, Word Content: %b", 224, {DataMemory.Mem[224], DataMemory.Mem[225], DataMemory.Mem[226], DataMemory.Mem[227]});
        $display("Address: %d, Word Content: %b", 228, {DataMemory.Mem[228], DataMemory.Mem[229], DataMemory.Mem[230], DataMemory.Mem[231]});
        $display("Address: %d, Word Content: %b", 232, {DataMemory.Mem[232], DataMemory.Mem[233], DataMemory.Mem[234], DataMemory.Mem[235]});
        $display("Address: %d, Word Content: %b", 236, {DataMemory.Mem[236], DataMemory.Mem[237], DataMemory.Mem[238], DataMemory.Mem[239]});
        $display("Address: %d, Word Content: %b", 240, {DataMemory.Mem[240], DataMemory.Mem[241], DataMemory.Mem[242], DataMemory.Mem[243]});
        $display("Address: %d, Word Content: %b", 244, {DataMemory.Mem[244], DataMemory.Mem[245], DataMemory.Mem[246], DataMemory.Mem[247]});
        $display("Address: %d, Word Content: %b", 248, {DataMemory.Mem[248], DataMemory.Mem[249], DataMemory.Mem[250], DataMemory.Mem[251]});
        $display("Address: %d, Word Content: %b", 252, {DataMemory.Mem[252], DataMemory.Mem[253], DataMemory.Mem[254], DataMemory.Mem[255]});
        $display("Address: %d, Word Content: %b", 256, {DataMemory.Mem[256], DataMemory.Mem[257], DataMemory.Mem[258], DataMemory.Mem[259]});
        $display("Address: %d, Word Content: %b", 260, {DataMemory.Mem[260], DataMemory.Mem[261], DataMemory.Mem[262], DataMemory.Mem[263]});
        $finish;
    end
    
endmodule