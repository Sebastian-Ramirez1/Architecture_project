module ControlUnit(ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, ID_ALU_op3, ID_DataMem_enable, Instr);
    input [31:0] Instr;
    output reg ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, ID_DataMem_enable;
    output reg [1:0] ID_size_dm;
    output reg [3:0] ID_ALU_op3; //should be 4bits for our ALU

    wire [1:0] op = Instr[31:30];

    always @(*) 
        begin

            case (op)
            2'b01:
            begin
                //op = CALL
                ID_jmpl_instr = 0;
                ID_Read_Write = 1'bX; //eliminate dont care //leer //no modificar registros
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
                end else begin //op = Branch
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
                end
            end

            2'b10:
            begin
                //op = Arithmetic or Jmpl
                //bit 23 = 1 modifyCC = 1 else modifyCC = 0

                if(Instr[24:19] == 6'b111000) begin //JMPL
                    ID_jmpl_instr = 1;
                    ID_modifyCC = 0;
                end else if ( (Instr[24:19] == 6'b010000) || (Instr[24:19] == 6'b011000) || (Instr[24:19] == 6'b010100) || (Instr[24:19] == 6'b011100)) begin //can modify CC
                    ID_modifyCC = 1;
                    ID_jmpl_instr = 0;
                end

                ID_Read_Write = 1'b0;
                ID_SE_dm = 1'b0;
                ID_ALU_op3 = 0000; //change to 4bits
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
                    ID_SE_dm = 1'bX; //Signed Extension
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
                    ID_SE_dm = 1'bX; //Signed Extension
                    ID_load_instr = 1; //Enable
                    ID_RF_enable = 0; //Ubicar en Memoria solamente 
                    ID_size_dm = 00; //byte
                end
                6'b000110: //store halfword
                begin
                    ID_Read_Write = 1; //Store = 1
                    ID_SE_dm = 1'bX; //Signed Extension
                    ID_load_instr = 1; //Enable
                    ID_RF_enable = 0; //Ubicar en Memoria solamente 
                    ID_size_dm = 01; //halfword
                end
                6'b000100: //store word
                begin
                    ID_Read_Write = 1; //Store = 1
                    ID_SE_dm = 1'bX; //Signed Extension
                    ID_load_instr = 1; //Enable
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
        
        end



endmodule

module MuxControlSignal(ControlSignals_Out, S, ControlSignals_In);
    input S;
    input [13:0] ControlSignals_In;
    output reg [13:0] ControlSignals_Out;

    always @(*) 
        begin
        case (S)
            1'b0: // Buffer
            begin
                ControlSignals_Out = ControlSignals_In;
            end
                
            1'b1: //No Operation
            begin
                ControlSignals_Out = 14'b00000000000000;
            end
                
        endcase

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