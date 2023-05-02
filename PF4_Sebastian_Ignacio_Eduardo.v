`include "PF1_Ramirez_Renta_Sebastian_alu.v"

module ControlUnit(output_signals, Instr);
    input [31:0] Instr;
    output reg [15:0] output_signals;
    
    reg ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, ID_DataMem_enable;
    reg [1:0] ID_size_dm;
    reg [3:0] ID_ALU_op3; //should be 4bits for our ALU

    wire [1:0] op = Instr[31:30];
    wire [3:0] op3_last_4bits = Instr[22:19];

    always @(*) 
        begin

            case (op)
            2'b01:
            begin
                //op = CALL
                ID_jmpl_instr = 0;
                ID_Read_Write = 1'b0; //eliminate dont care //leer //no modificar registros
                ID_ALU_op3 = 4'b0000;
                ID_SE_dm = 1'b0;
                ID_load_instr = 0;
                ID_RF_enable = 1; //Write in R15 PC
                ID_size_dm = 2'b00;
                ID_modifyCC = 0;
                ID_Call_instr = 1;
                ID_B_instr = 0;
                ID_29_a = 1'b0;
                ID_DataMem_enable = 0;
            end
    
            2'b00:
            begin
                if (Instr == 32'b00000000000000000000000000000000) begin //check nop
                    ID_jmpl_instr = 0;
                    ID_Read_Write = 0;
                    ID_ALU_op3 = 0000;
                    ID_SE_dm = 0;
                    ID_load_instr = 0;
                    ID_RF_enable = 0;
                    ID_size_dm = 00;
                    ID_modifyCC = 0;
                    ID_Call_instr = 0;
                    ID_B_instr = 0;
                    ID_29_a = 0;
                    ID_DataMem_enable = 0;
                end else if(Instr[24:22] != 3'b100) begin //op = Branch
                    ID_jmpl_instr = 0;
                    ID_Read_Write = 0;
                    ID_ALU_op3 = 0000;
                    ID_SE_dm = 0;
                    ID_load_instr = 0;
                    ID_RF_enable = 0;
                    ID_size_dm = 00;
                    ID_modifyCC = 0;
                    ID_Call_instr = 0;
                    ID_B_instr = 1;
                    ID_29_a = Instr[29];
                    ID_DataMem_enable = 0;
                end else begin //SETHI
                    ID_jmpl_instr = 0;
                    ID_Read_Write = 0;
                    ID_ALU_op3 = 4'b1110; // B
                    ID_SE_dm = 0;
                    ID_load_instr = 0;
                    ID_RF_enable = 1;
                    ID_size_dm = 00;
                    ID_modifyCC = 0;
                    ID_Call_instr = 0;
                    ID_B_instr = 0;
                    ID_29_a = 0;
                    ID_DataMem_enable = 0;
                end
            end

            2'b10:
            begin
                //op = Arithmetic or Jmpl
                //bit 23 = 1 modifyCC = 1 else modifyCC = 0

                if(Instr[24:19] == 6'b111000) begin //JMPL
                    ID_jmpl_instr = 1;
                    ID_modifyCC = 0;
                    ID_ALU_op3 = 4'b0;
                end else if (Instr[23] == 1) begin //can modify CC
                    ID_modifyCC = 1;
                    ID_jmpl_instr = 0;
                end else begin
                    ID_modifyCC = 0;
                    ID_jmpl_instr = 0;
                end
                
                if (Instr[24:19] != 6'b111000)
                begin
                    case (op3_last_4bits)
                        4'd0: ID_ALU_op3 = 4'd0; // Add

                        4'd8: ID_ALU_op3 = 4'd1; // Add + carry

                        4'd4: ID_ALU_op3 = 4'd2; // Sub

                        4'd12: ID_ALU_op3 = 4'd3; //Sub - Carry

                        4'd1: ID_ALU_op3 = 4'd4; // And

                        4'd5: ID_ALU_op3 = (Instr[24] == 1)? 4'd10 : 4'd8; // Logical Shift Left : AndNot

                        4'd2: ID_ALU_op3 = 4'd5; // Or

                        4'd6: ID_ALU_op3 = (Instr[24] == 1)? 4'd11 : 4'd9; // Logical Shift Right : OrNot

                        4'd3: ID_ALU_op3 = 4'd6; // XOR

                        4'd7: ID_ALU_op3 = (Instr[24] == 1)? 4'd12 : 4'd7; // Arithmetic Shift Right : XORNot

                        // 4'd6: ID_ALU_op3 = 4'd13; // A

                        // 4'd7: ID_ALU_op3 = 4'd15; // not B
                    endcase
                end

                ID_Read_Write = 1'b0;
                ID_SE_dm = 1'b0;
                ID_load_instr = 0;
                ID_RF_enable = 1;
                ID_size_dm = 2'b00;
                ID_Call_instr = 0;
                ID_B_instr = 0;
                ID_29_a = 1'b0;
                ID_DataMem_enable = 0;
                //ID_ALU_op3 = Instr[24:19]; //change to 4bits
                //identify op3(6bits) and decodify op3(4bits) for our ALU
            end

            2'b11:
            begin
                //op = Load/Store
                case (Instr[24:19])
                6'b001001: //load sign byte
                begin
                    ID_Read_Write = 0; //Load = 0
                    ID_SE_dm = 1; //Signed Extension
                    ID_load_instr = 1; //Enable
                    ID_RF_enable = 1; //Ubicar en Rd un valor de memoria 
                    ID_size_dm = 00; //byte
                end
                6'b001010: //load sign halfword
                begin
                    ID_Read_Write = 0; //Load = 0
                    ID_SE_dm = 1; //Signed Extension
                    ID_load_instr = 1; //Enable
                    ID_RF_enable = 1; //Ubicar en Rd un valor de memoria 
                    ID_size_dm = 01; //halfword
                end
                6'b000000: //load word
                begin
                    ID_Read_Write = 0; //Load = 0
                    ID_SE_dm = 1'b0; //Signed Extension
                    ID_load_instr = 1; //Enable
                    ID_RF_enable = 1; //Ubicar en Rd un valor de memoria 
                    ID_size_dm = 10; //word
                end
                6'b000001: //load unsigned byte
                begin
                    ID_Read_Write = 0; //Load = 0
                    ID_SE_dm = 0; //Signed Extension
                    ID_load_instr = 1; //Enable
                    ID_RF_enable = 1; //Ubicar en Rd un valor de memoria 
                    ID_size_dm = 00; //byte
                end
                6'b000010: //load unsigned halfword
                begin
                    ID_Read_Write = 0; //Load = 0
                    ID_SE_dm = 0; //Signed Extension
                    ID_load_instr = 1; //Enable
                    ID_RF_enable = 1; //Ubicar en Rd un valor de memoria 
                    ID_size_dm = 01; //halfword
                end
                6'b000101: //store byte
                begin
                    ID_Read_Write = 1; //Store = 1
                    ID_SE_dm = 1'b0; //Signed Extension
                    ID_load_instr = 0; //Enable
                    ID_RF_enable = 0; //Ubicar en Memoria solamente 
                    ID_size_dm = 00; //byte
                end
                6'b000110: //store halfword
                begin
                    ID_Read_Write = 1; //Store = 1
                    ID_SE_dm = 1'b0; //Signed Extension
                    ID_load_instr = 0; //Enable
                    ID_RF_enable = 0; //Ubicar en Memoria solamente 
                    ID_size_dm = 01; //halfword
                end
                6'b000100: //store word
                begin
                    ID_Read_Write = 1; //Store = 1
                    ID_SE_dm = 1'b0; //Signed Extension
                    ID_load_instr = 0; //Enable
                    ID_RF_enable = 0; //Ubicar en Memoria solamente 
                    ID_size_dm = 10; //Word
                end
                endcase
                ID_jmpl_instr = 0;
                ID_ALU_op3 = 0000;
                ID_modifyCC = 0;
                ID_Call_instr = 0;
                ID_B_instr = 0;
                ID_29_a = 1'b0;
                ID_DataMem_enable = 1;
            end

            endcase
        
            output_signals = {ID_jmpl_instr, ID_Read_Write, ID_ALU_op3, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, ID_DataMem_enable};
        end

endmodule

module MuxControlSignal(ControlSignals_Out, S, ControlSignals_In);
    input S;
    input [15:0] ControlSignals_In;
    output reg [15:0] ControlSignals_Out;

    always @(*) 
        begin
        case (S)
            1'b0: // Buffer
            begin
                ControlSignals_Out = ControlSignals_In;
            end
                
            1'b1: //No Operation 
            begin
                ControlSignals_Out = 16'b0;
            end
                
        endcase

        end

endmodule
module MUXPC (output reg [31:0] MUXPC_Out, input [31:0] ALU_out, input [31:0] Branch_Target_Address, input [31:0]NPC_Out, input [1:0] IF_Signal_In);
  always @(ALU_out, Branch_Target_Address, NPC_Out, IF_Signal_In)
    	begin
      	case (IF_Signal_In)
        	2'b00: // call
        	begin
        		MUXPC_Out = Branch_Target_Address;
        	end
        	2'b01: // branch
        	begin
            	MUXPC_Out = Branch_Target_Address;
        	end
       	 	2'b10: // Normal Instruction
        	begin 
            	MUXPC_Out = NPC_Out;
        	end
        	2'b11: // jmpl
        	begin
            	MUXPC_Out = ALU_out;
        	end
        	endcase
    	end
endmodule

module ProgramStatusRegister (output reg C, output reg [3:0] PSR_Out, input[3:0] Flags, input EX_modifyCC );
  reg [31:0] PSR_data;
  always @(Flags, EX_modifyCC)
    begin
      if (EX_modifyCC == 1)
        begin
          PSR_data[23:20] = {Flags[3:2], Flags[0], Flags[1]}; // Almacena los Flags/CC from Alu en bits 23-20
      PSR_Out = PSR_data[23:20];// La salida son los flags/cc del Alu
      C = PSR_data[20]; // La salida es el bit 20 del PSR
        end
     end
    
endmodule

module MUX_CC(CC_Out, EX_ALU_flags, PSR_Out, EX_ModifyCC);
    input EX_ModifyCC;
    input [3:0] PSR_Out;
    input [3:0] EX_ALU_flags;
    output reg [3:0] CC_Out;

    always @(EX_ModifyCC) begin
        if (EX_ModifyCC) CC_Out = EX_ALU_flags;
        else CC_Out = PSR_Out;
    end
endmodule

module MUXPCIFID_Reset_Handler (output reg IFID_Reset_Signal, output reg [1:0] PCMUX_Signal, input BCH_Out, input EX_jmpl_instr, input ID_Call_instr, input ID_29_a, input ID_B_instr);
   
  always @(EX_jmpl_instr,  ID_Call_instr, ID_B_instr, ID_29_a, BCH_Out)
    begin
      PCMUX_Signal = 2'b10; // la instrucion es normal
       IFID_Reset_Signal = 0;
      if (BCH_Out == 1) 
        
        begin
        PCMUX_Signal = 2'b01; // Si se da el branch
      end
      
      if (ID_Call_instr == 1)
        
        begin 
          PCMUX_Signal = 2'b00; // la instruction es un call
        end
      
      if (ID_B_instr == 1)
        
        begin
          PCMUX_Signal = 2'b01; // hay un branch en el instruction
        end
      
       if (EX_jmpl_instr == 1) 
        begin
          PCMUX_Signal = 2'b11; // hay un jmpl en el instruction
        end
          
       if (ID_29_a == 1) begin
        IFID_Reset_Signal = 1; // Se resetea el pipeline register en IF/ID
      end     
    end
endmodule

module ConditionHandlerBranch (output reg BCH_Out, input[3:0] MUXCC_Out, input[3:0] InstrCondIF, input ID_B_instr);
  reg[3:0] mux;
  reg N, Z, V, C;
   
  always @(MUXCC_Out, InstrCondIF, ID_B_instr)
   
    begin
      if (ID_B_instr == 1) begin
        mux = MUXCC_Out;
    N = mux[3];
   	Z = mux[2];
  	V = mux[1];
  	C = mux[0];
        case (InstrCondIF)
          4'b1000:  // ba
             BCH_Out = 1;
          4'b0000: // bn
             BCH_Out = 0;
          4'b1001: // bne
            if(Z != 1)
             BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b0001: // be
            if(Z == 1)
              BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b1010: // bg
            if(Z == 0 ||(( V == 0 && N == 0))|| ((V == 1 && N == 1)))
            BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b0010: //ble
            if(Z == 1 ||(( V == 1 && N == 0))|| (V == 0 && N == 1))
              BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b1011: //bge
            if((V == 0 && N == 0)|| (V == 1 && N == 1))
              BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b0011: //bl
            if((V == 1 && N == 0)|| (V == 0 && N == 1))
            BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b1100: //bgu
               if(C != 1 || Z != 1)
              BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b0100: //bleu
               if(C == 1 || Z == 1)
              BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b1101: //bcc
               if(C !=1)
              BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b0101: //bcs
               if(C == 1)
            BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b1110: //bpos
               if(N != 1)
              BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b0110: //bneg
               if(N == 1)
              BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b1111: //bvc
               if(V != 1)
            BCH_Out = 1;
          else 
            BCH_Out = 0;
          4'b0111: //bvs
            if(V == 1)
       		BCH_Out = 1;
          else 
            BCH_Out = 0;
        endcase
      end
        else 
          BCH_Out = 0;
     end
    
endmodule

module DISP22SE (output reg [21:0] Disp22SE_Out, input[21:0] Disp22);
   
  always @(Disp22)
    begin
      if (Disp22[21] == 1) // si el bit mas significativo es 1 todo lo demas es 1
        Disp22SE_Out = {22'b1111111111111111111111};
        else 
          Disp22SE_Out = {22'b0000000000000000000000}; // sino extiende 0
     end
    
endmodule

module MUXCALLORBRANCH (output reg [31:0] MUXCOB_Out, input[21:0] Disp22SE_Out, input[29:0] Disp30, input ID_Call_instr);
   
  always @(Disp22SE_Out, Disp30, ID_Call_instr)
    begin
      if (ID_Call_instr == 1)
        MUXCOB_Out = Disp30;
        else 
          MUXCOB_Out = Disp22SE_Out;
     end
    
endmodule
	
module Multiplicador4(Multiplicador4_Out, Multiplicador4_In);
    input [29:0] Multiplicador4_In;
    output reg [29:0] Multiplicador4_Out;
    always @(Multiplicador4_In)
        begin
            Multiplicador4_Out = Multiplicador4_In * 4; //nPC = nPC +4
        end
endmodule
module Sumador_TA(Target_Address, PC, Multiplicador4_Out);
    input [31:0] PC;
    input [29:0] Multiplicador4_Out;
    output reg [31:0] Target_Address;
    
    always @(PC, Multiplicador4_Out) begin
        Target_Address = PC + Multiplicador4_Out;
    end
endmodule
module ID_MUX_RD(RegistroDestino_Out, RegistroDestino_In, ID_Call_instr);
    input ID_Call_instr;
    input [4:0] RegistroDestino_In;
    output reg [4:0]RegistroDestino_Out;
    always @(*) begin
        if (ID_Call_instr) RegistroDestino_Out = 5'b01111;
        else RegistroDestino_Out = RegistroDestino_In;
    end
endmodule

module Sumador4(nPC, PC); 
    input [31:0] PC;
    output reg [31:0] nPC;
    always @(PC) 
        begin
            nPC = PC + 4; //nPC = nPC +4
        end
    
endmodule

module nPC (Q, Clk, D, LE, R);
    input [31:0] D;
    input LE;
    input Clk;
    input R;
    output reg[31:0] Q;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
        if (R) Q <= 32'b00000000000000000000000000000100; //En el caso de nPC un reset produce un número binario correspondiente a un 4.
        else if (LE) Q <= D; // LE = 1  D --> Q

endmodule

module PC (Q, Clk, D, LE, R);
    input [31:0] D;
    input LE;
    input Clk;
    input R;
    output reg[31:0] Q;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
        if (R) Q <= 32'b00000000000000000000000000000000; //un reset tienen el efecto de hacer cero todos los bits de salida del registro. 
        else if (LE) Q <= D; // LE = 1  D --> Q
    
endmodule

module InstructionMemory (output reg [31:0] DataOut, input [31:0] Address);

    reg [7:0] Mem [0:511];
    always @(Address) begin
    // Reading the Data
        DataOut = {Mem[Address], Mem[Address+1], Mem[Address+2], Mem[Address+3]};
    
    end

endmodule

module DataMemory(output reg[31:0] DataOut, input RW, input[31:0] Address, input[31:0] DataIn, input [1:0] Size, input SE, input E);

  reg[7:0] Mem[0:511];
  reg[7:0] address;
  reg[31:0] data;
 
    always @ (Size, Address, E, SE, DataIn, RW) begin
    
  if (E==1) begin
    address = Address;
    data = DataIn;
    case (Size)
      //Cases for R/W for bytes
  2'b00:
    //Reading bytes
      if(RW == 0) begin
        if (SE == 0) data = {24'b000000000000000000000000, Mem[address]};
        else  data = {24'b111111111111111111111111, Mem[address]};
        if (data[7] == 0)
         data = {24'b000000000000000000000000, Mem[address]};
      end
    else  
      //Writing Bytes
      if(RW == 1) 
        Mem[address] = data[7:0];
   
  

 //R/W cases for Halfwords
  2'b01:
    //Reading Halfwords
      if(RW == 0) begin
        if (SE == 0) data = {16'b0000000000000000, Mem[address], Mem[address + 1]};
        else data = {16'b1111111111111111, Mem[address], Mem[address + 1]};
        if (data[15] == 0)
        data = {16'b0000000000000000, Mem[address], Mem[address + 1]};
  
      end
    // Writing Halfwords
      else  if(RW == 1)begin
        Mem[address] = data[15:8];
        Mem[address+1] = data[7:0];
        
      end
    
     

  //R/W cases for words
   2'b10:
    //Reading words
    if(RW == 0) begin
  
        data = {Mem[address], Mem[address + 1], Mem[address + 2], Mem[address +3]};
  
      end
  
      //Writing words
      else if (RW == 1 )begin
      Mem[address] = data[31:24];
      Mem[address + 1] = data[23:16];
      Mem[address + 2] = data[15:8]; 
        Mem[address + 3] = data[7:0];

      end
    endcase
     DataOut = data;
    
  end
end
endmodule

module PipelineRegister_IF_ID(Q, Clk, D, LE, R); //eliminar LE 
    //PC = 32 bits
    //Instr = 32 bits
    input [63:0] D;
    input LE;
    input Clk;
    input R;
    output reg [63:0] Q;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
    begin
        if (R) Q <= 64'b00000000000000000000000000000000; //un reset tienen el efecto de hacer cero todos los bits de salida del registro. 
        else if (LE) Q <= D; // LE = 1  D --> Q //else
    end
endmodule

module PipelineRegister_ID_EX(Q, Clk, D, R);
    //jpmpl(1) + read_write(1) + ALU_op3(4) + SE(1) + load_instr(1) + RF_enable(1) + size_dm(2) + modifyCC(1) + call(1) + DataMem_enable(1) = 14 bits
    //ID_PC = 32 bits
    //ID_PA = 32 bits
    //ID_DataIn = 32 bits
    //ID_PB = 32bits
    //ID_Imm = 22bits
    //ID_RD = 5 bits
    //ID_31_30_24_13 = 4bits = 173 bits
    input [172:0] D;
    input Clk;
    input R;
    output reg [172:0] Q;

    always @(posedge Clk) begin
        if (R) Q <= 173'b00000000000000;
        else Q <= D;
    end
endmodule

module PipelineRegister_EX_MEM(Q, Clk, D, R);
    //jpmpl(1) + read_write(1) + SE(1) + load_instr(1) + RF_enable(1) + size_dm(2) + call(1) + DataMem_enable(1) = 9 bits
    //EX_PC = 32 bits
    //EX_DataIn = 32 bits
    //EX_ALU_Out = 32 bits
    //EX_RD = 5bits
    input [109:0] D;
    input Clk;
    input R;
    output reg [109:0] Q;

    always @(posedge Clk) begin
        if (R) Q <= 110'b0;
        else Q <= D;
    end
endmodule

module PipelineRegister_MEM_WB(Q, Clk, D, R);
    //RF_enable(1) =  1 bit
    //PW = 32 bits
    //RW = 5 bits
    input [37:0] D;
    input Clk;
    input R;
    output reg [37:0] Q;

    always @(posedge Clk) begin
        if (R) Q <= 37'b0;
        else Q <= D;
    end
endmodule

module MEM_MUX_RF (Data_Out, PC, ALU_Out, Load_Data, MEM_load_instr, MEM_jmpl_instr, MEM_call_instr);
    input MEM_call_instr, MEM_jmpl_instr, MEM_load_instr;
    input [31:0] PC;
    input [31:0] ALU_Out;
    input [31:0] Load_Data;
    output reg [31:0] Data_Out;

    always @(*) begin
        if (MEM_call_instr) Data_Out = PC;
        else if (MEM_jmpl_instr) Data_Out = ALU_Out;
        else if (MEM_load_instr) Data_Out = Load_Data;
    end
    
endmodule

module MUX_DataFowarding(Data_Out, Content_RegisterFile, Content_EX_RD, Content_MEM_RD, Content_WB_RD, Sig_DataHazard);
    input [1:0] Sig_DataHazard;
    input [31:0] Content_RegisterFile;
    input [31:0] Content_MEM_RD;
    input [31:0] Content_EX_RD;
    input [31:0] Content_WB_RD;
    output reg [31:0] Data_Out;
    always @(*) begin
        case (Sig_DataHazard)
            2'b00: Data_Out = Content_RegisterFile;
            2'b01: Data_Out = Content_EX_RD;
            2'b10: Data_Out = Content_MEM_RD;
            2'b11: Data_Out = Content_WB_RD;
        endcase
    end
endmodule

module RegisterFile(PA, PB, PC, RA, RB, RC, RW, PW, Clk, LE);
    input [4:0] RA, RB, RC, RW;
    input [31:0] PW;
    input Clk;
    input LE;
    output [31:0] PA, PB, PC;

    //salidas de registros
    wire [31:0] Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18, Q19, Q20, Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28, Q29, Q30, Q31;
    wire [31:0] O; //salida binary decoder

    assign Q0 = 32'b00000000000000000000000000000000; //R0

    //5-to-32 Binary Decoder
    binaryDecoder binaryDecoder(O, RW, LE);

    //32 32-Bit Registers
    //register32 R0(Clk, 0, Q0, O[0]); //R0 siempre tendra valor de cero. Is R0 
    register32 R1(Clk, PW, Q1, O[1]);
    register32 R2(Clk, PW, Q2, O[2]);
    register32 R3(Clk, PW, Q3, O[3]);
    register32 R4(Clk, PW, Q4, O[4]);
    register32 R5(Clk, PW, Q5, O[5]);
    register32 R6(Clk, PW, Q6, O[6]);
    register32 R7(Clk, PW, Q7, O[7]);
    register32 R8(Clk, PW, Q8, O[8]);
    register32 R9(Clk, PW, Q9, O[9]);
    register32 R10(Clk, PW, Q10, O[10]);
    register32 R11(Clk, PW, Q11, O[11]);
    register32 R12(Clk, PW, Q12, O[12]);
    register32 R13(Clk, PW, Q13, O[13]);
    register32 R14(Clk, PW, Q14, O[14]);
    register32 R15(Clk, PW, Q15, O[15]);
    register32 R16(Clk, PW, Q16, O[16]);
    register32 R17(Clk, PW, Q17, O[17]);
    register32 R18(Clk, PW, Q18, O[18]);
    register32 R19(Clk, PW, Q19, O[19]);
    register32 R20(Clk, PW, Q20, O[20]);
    register32 R21(Clk, PW, Q21, O[21]);
    register32 R22(Clk, PW, Q22, O[22]);
    register32 R23(Clk, PW, Q23, O[23]);
    register32 R24(Clk, PW, Q24, O[24]);
    register32 R25(Clk, PW, Q25, O[25]);
    register32 R26(Clk, PW, Q26, O[26]);
    register32 R27(Clk, PW, Q27, O[27]);
    register32 R28(Clk, PW, Q28, O[28]);
    register32 R29(Clk, PW, Q29, O[29]);
    register32 R30(Clk, PW, Q30, O[30]);
    register32 R31(Clk, PW, Q31, O[31]);
    
    //3 32-to-1 Multiplexers
    //output primero
    Mux32x1 MuxA(RA, Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18, Q19, Q20, Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28, Q29, Q30, Q31, PA);
    Mux32x1 MuxB(RB, Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18, Q19, Q20, Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28, Q29, Q30, Q31, PB);
    Mux32x1 MuxC(RC, Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18, Q19, Q20, Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28, Q29, Q30, Q31, PC);


endmodule

module register32(Clk, D, Q, LE); //Reset?
    input [31:0] D;
    input LE;
    input Clk;
    input Clr;
    output reg [31:0] Q;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
        begin
            if (LE) Q <= D; // LE = 1  D --> Q
        end
        

endmodule

module Mux32x1(S, I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, I16, I17, I18, I19, I20, I21, I22, I23, I24, I25, I26, I27, I28, I29, I30, I31, P);
    input [31:0] I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, I16, I17, I18, I19, I20, I21, I22, I23, I24, I25, I26, I27, I28, I29, I30, I31;
    input [4:0] S;
    output reg [31:0] P;

    always @(S, I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, I16, I17, I18, I19, I20, I21, I22, I23, I24, I25, I26, I27, I28, I29, I30, I31) 
        begin
        case (S)
        5'b00000: P = I0;
        5'b00001: P = I1;
        5'b00010: P = I2;
        5'b00011: P = I3;
        5'b00100: P = I4;
        5'b00101: P = I5;
        5'b00110: P = I6;
        5'b00111: P = I7;
        5'b01000: P = I8;
        5'b01001: P = I9;
        5'b01010: P = I10;
        5'b01011: P = I11;
        5'b01100: P = I12;
        5'b01101: P = I13;
        5'b01110: P = I14;
        5'b01111: P = I15;
        5'b10000: P = I16;
        5'b10001: P = I17;
        5'b10010: P = I18;
        5'b10011: P = I19;
        5'b10100: P = I20;
        5'b10101: P = I21;
        5'b10110: P = I22;
        5'b10111: P = I23;
        5'b11000: P = I24;
        5'b11001: P = I25;
        5'b11010: P = I26;
        5'b11011: P = I27;
        5'b11100: P = I28;
        5'b11101: P = I29; 
        5'b11110: P = I30;
        5'b11111: P = I31;
        endcase

        end
endmodule

module binaryDecoder(O, RW, LE);
    output reg [31:0] O;
    input [4:0] RW;
    input LE;

    always @(RW, LE)  

        if(LE)
        begin
            case(RW)
            5'b00000: O = 32'b00000000000000000000000000000001;
            5'b00001: O = 32'b00000000000000000000000000000010;
            5'b00010: O = 32'b00000000000000000000000000000100;
            5'b00011: O = 32'b00000000000000000000000000001000;
            5'b00100: O = 32'b00000000000000000000000000010000;
            5'b00101: O = 32'b00000000000000000000000000100000;
            5'b00110: O = 32'b00000000000000000000000001000000;
            5'b00111: O = 32'b00000000000000000000000010000000;
            5'b01000: O = 32'b00000000000000000000000100000000;
            5'b01001: O = 32'b00000000000000000000001000000000;
            5'b01010: O = 32'b00000000000000000000010000000000;
            5'b01011: O = 32'b00000000000000000000100000000000;
            5'b01100: O = 32'b00000000000000000001000000000000;
            5'b01101: O = 32'b00000000000000000010000000000000;
            5'b01110: O = 32'b00000000000000000100000000000000;
            5'b01111: O = 32'b00000000000000001000000000000000;
            5'b10000: O = 32'b00000000000000010000000000000000;
            5'b10001: O = 32'b00000000000000100000000000000000;
            5'b10010: O = 32'b00000000000001000000000000000000;
            5'b10011: O = 32'b00000000000010000000000000000000;
            5'b10100: O = 32'b00000000000100000000000000000000;
            5'b10101: O = 32'b00000000001000000000000000000000;
            5'b10110: O = 32'b00000000010000000000000000000000;
            5'b10111: O = 32'b00000000100000000000000000000000;
            5'b11000: O = 32'b00000001000000000000000000000000;
            5'b11001: O = 32'b00000010000000000000000000000000;
            5'b11010: O = 32'b00000100000000000000000000000000;
            5'b11011: O = 32'b00001000000000000000000000000000;
            5'b11100: O = 32'b00010000000000000000000000000000;
            5'b11101: O = 32'b00100000000000000000000000000000;
            5'b11110: O = 32'b01000000000000000000000000000000;
            5'b11111: O = 32'b10000000000000000000000000000000;
            endcase
        end
        else  O = 32'b00000000000000000000000000000000;

endmodule

module Hazard_Unit(Sig_MUX_PA, Sig_MUX_PB, Sig_MUX_DataIn, IF_ID_enable, PC_nPC_enable, MUX_nop_signal, ID_RA, ID_RB, ID_RDataIn, EX_RD, MEM_RD, WB_RD, EX_RF_enable, MEM_RF_enable, WB_RF_enable, EX_load_instr);
    input EX_RF_enable, MEM_RF_enable, WB_RF_enable, EX_load_instr;
    input [4:0] ID_RA, ID_RB, ID_RDataIn, EX_RD, MEM_RD, WB_RD;
    output reg IF_ID_enable, PC_nPC_enable, MUX_nop_signal;
    output reg [1:0] Sig_MUX_PA, Sig_MUX_PB, Sig_MUX_DataIn;

    always @(*) begin
        IF_ID_enable = 1'b1; //Disable pipeline Load
        PC_nPC_enable = 1'b1; //Disable PC load
        MUX_nop_signal = 1'b0; //NOP; its suppose to 
        Sig_MUX_PA = 2'b00;
        Sig_MUX_PB = 2'b00;
        Sig_MUX_DataIn = 2'b00;

        //Data Hazard by LOAD
        if(EX_load_instr && ( (ID_RA == EX_RD) || (ID_RB == EX_RD) )) 
            begin
                $display("DATA HAZARD BY LOAD");
                IF_ID_enable = 0; //Disable pipelineregsiter IF_ID
                PC_nPC_enable = 0; //Disable PC and nPC
                MUX_nop_signal = 1;// NOP to mux
            end

        //Data Fowarding

        //Fowarding for Rm (First Source Operand)
        if( (EX_RF_enable) && (ID_RA == EX_RD) ) 
            begin
                Sig_MUX_PA = 2'b01; //EX_ALU_Out
            end
        else if ( (MEM_RF_enable) && (ID_RA == MEM_RD) )
            begin
                Sig_MUX_PA = 2'b10; //MEM_PW
            end
        else if ( (WB_RF_enable) && (ID_RA == WB_RD) )
            begin
                Sig_MUX_PA = 2'b11; //WB_PW
            end

        //Fowarding for Rn (Second Source Operand)
        if ( (EX_RF_enable) && (ID_RB == EX_RD) ) 
            begin
                Sig_MUX_PB = 2'b01; //EX_ALU_Out
            end
        else if ( (MEM_RF_enable) && (ID_RB == MEM_RD) )
            begin
                Sig_MUX_PB = 2'b10; //MEM_PW
            end
        else if ( (WB_RF_enable) && (ID_RB == WB_RD) )
            begin
                Sig_MUX_PB = 2'b11; //WB_PW
            end
            
        //Fowarding RDataIn (Store Source Operand)
        if ( (EX_RF_enable) && (ID_RDataIn == EX_RD) ) 
            begin
                Sig_MUX_PB = 2'b01; //EX_ALU_Out
            end
        else if ( (MEM_RF_enable) && (ID_RDataIn == MEM_RD) )
            begin
                Sig_MUX_PB = 2'b10; //MEM_PW
            end
        else if ( (WB_RF_enable) && (ID_RDataIn == WB_RD) )
            begin
                Sig_MUX_PB = 2'b11; //WB_PW
            end

    end
endmodule
