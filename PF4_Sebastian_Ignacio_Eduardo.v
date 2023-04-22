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
module MUXPC (input [31:0] ALU_out, input [31:0] Branch_Target_Address, input [31:0]NPC_Out, input [1:0] IF_Signal_In, output reg [31:0] MUXPC_Out);
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

module ProgramStatusRegister (input[3:0] Flags, input EX_modifyCC, output reg C, output reg [3:0] PSR_Out);
  reg [31:0] PSR_data;
  always @(Flags, Ex_modifyCC)
    begin
      PSR_data[23:20] = {Flags[3:2], Flags[0], Flags[1]}; // Almacena los Flags/CC from Alu en bits 23-20
      PSR_Out = PSR_data[23:20];// La salida son los flags/cc del Alu
      C = PSR_data[20]; // La salida es el bit 20 del PSR
     end
endmodule

module MUXPCIFID_Reset_Handler (input BCH_Out, input EX_jmpl_instr, input ID_Call_instr, input ID_29_a, input ID_B_instr, output reg IFID_Reset_Signal, output reg [1:0] PCMUX_Signal);
   
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

module DISP22SE (input[21:0] Disp22, output reg [21:0] Disp22SE_Out);
   
  always @(Disp22)
    begin
      if (Disp22[21] == 1) // si el bit mas significativo es 1 todo lo demas es 1
        Disp22_Out = {22'b1111111111111111111111};
        else 
          Disp22_Out = {22'b0000000000000000000000}; // sino extiende 0
     end
    
endmodule

module MUXCALLORBRANCH (input[21:0] Disp22SE_Out, input[29:0] Disp30, input ID_Call_instr, output reg [31:0] MUXCOB_Out);
   
  always @(Disp22SE_Out, Disp30, ID_Call_instr)
    begin
      if (ID_Call_instr == 1)
        MUXCOB_Out = Disp30;
        else 
          MUXCOB_Out = Disp22SE_Out;
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

module DataMemory(output reg[31:0] DataOut, input[1:0] RW, input[31:0] Address, input[31:0] DataIn, input [1:0] Size, input [1:0] SE, input [1:0] E);

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
    input [31:0] D;
    input LE;
    input Clk;
    input R;
    output reg [31:0] Q;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
    begin
        if (R) Q <= 32'b00000000000000000000000000000000; //un reset tienen el efecto de hacer cero todos los bits de salida del registro. 
        else if (LE) Q <= D; // LE = 1  D --> Q //else
    end
    
endmodule

module PipelineRegister_ID_EX(Q, Clk, D, R);
    //jpmpl(1) + read_write(1) + ALU_op3(4) + SE(1) + load_instr(1) + RF_enable(1) + size_dm(2) + modifyCC(1) + call(1) + DataMem_enable(1) = 14 bits
    input [13:0] D;
    input Clk;
    input R;
    output reg [13:0] Q;

    always @(posedge Clk) begin
        if (R) Q <= 14'b00000000000000;
        else Q <= D;
    end
endmodule

module PipelineRegister_EX_MEM(Q, Clk, D, R);
    //jpmpl(1) + read_write(1) + SE(1) + load_instr(1) + RF_enable(1) + size_dm(2) + call(1) + DataMem_enable(1) = 9 bits
    input [8:0] D;
    input Clk;
    input R;
    output reg [8:0] Q;

    always @(posedge Clk) begin
        if (R) Q <= 9'b000000000;
        else Q <= D;
    end
endmodule

module PipelineRegister_MEM_WB(Q, Clk, D, R);
    //RF_enable(1) =  1 bit
    input D;
    input Clk;
    input R;
    output reg Q;

    always @(posedge Clk) begin
        if (R) Q <= 0;
        else Q <= D;
    end
endmodule
