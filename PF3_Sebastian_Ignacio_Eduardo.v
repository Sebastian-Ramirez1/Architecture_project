module ControlUnit(ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a, ID_ALU_op3, Instr);
    input [31:0] Instr;
    output reg ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_modifyCC, ID_Call_instr, ID_B_instr, ID_29_a;
    output reg [1:0] ID_size_dm;
    output reg [5:0] ID_ALU_op3; //should be 4bits for our ALU

    wire [1:0] op = Instr[31:30];

    always @(*) 
        begin

            case (op)
            2'b01:
            begin
                //op = CALL
                ID_jmpl_instr = 0;
                ID_Read_Write = 1'bX; //eliminate dont care //leer //no modificar registros
                ID_SE_dm = 1'bX;
                ID_load_instr = 0;
                ID_RF_enable = 1; //Write in R15 PC
                ID_size_dm = 2'bXX;
                ID_modifyCC = 0;
                ID_Call_instr = 1;
                ID_B_instr = 0;
                ID_29_a = 1'bX;
                ID_ALU_op3 = 6'bXXXXXX;
            end
    
            2'b00:
            begin
                if (Instr == 32'b00000000000000000000000000000000) begin //check nop
                    ID_jmpl_instr = 0;
                    ID_Read_Write = 0;
                    ID_SE_dm = 0;
                    ID_load_instr = 0;
                    ID_RF_enable = 0;
                    ID_size_dm = 00;
                    ID_modifyCC = 0;
                    ID_Call_instr = 0;
                    ID_B_instr = 0;
                    ID_29_a = Instr[29];
                    ID_ALU_op3 = 000000;
                end else begin //op = Branch
                    ID_jmpl_instr = 0;
                    ID_Read_Write = 0;
                    ID_SE_dm = 0;
                    ID_load_instr = 0;
                    ID_RF_enable = 0;
                    ID_size_dm = 00;
                    ID_modifyCC = 0;
                    ID_Call_instr = 0;
                    ID_B_instr = 1;
                    ID_29_a = Instr[29];
                    ID_ALU_op3 = 000000;
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

                ID_Read_Write = 1'bX;
                ID_SE_dm = 1'bX;
                ID_load_instr = 0;
                ID_RF_enable = 1;
                ID_size_dm = 2'bXX;
                ID_Call_instr = 0;
                ID_B_instr = 0;
                ID_29_a = 1'bX;
                ID_ALU_op3 = Instr[24:19]; //change to 4bits
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
                ID_modifyCC = 0;
                ID_Call_instr = 0;
                ID_B_instr = 0;
                ID_29_a = 1'bX;
                ID_ALU_op3 =Instr[24:19];
            end

            endcase
        
        end



endmodule

module MuxControlSignal(ID_jmpl_instr_out, ID_Read_Write_out, ID_SE_dm_out, ID_load_instr_out, ID_RF_enable_out, ID_size_dm_out, ID_modifyCC_out, ID_Call_instr_out, ID_ALU_op3_out, S, ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_Call_instr, ID_ALU_op3);
    input S;
    input ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_modifyCC, ID_Call_instr;
    input [1:0] ID_size_dm; 
    input [5:0] ID_ALU_op3;
    output reg ID_jmpl_instr_out, ID_Read_Write_out, ID_SE_dm_out, ID_load_instr_out, ID_RF_enable_out, ID_modifyCC_out, ID_Call_instr_out, ID_29_a_out;
    output reg [1:0] ID_size_dm_out;
    output reg [5:0] ID_ALU_op3_out;

    always @(*) 
        begin
        case (S)
            1'b0: // Buffer
            begin
                ID_jmpl_instr_out = ID_jmpl_instr;
                ID_Read_Write_out = ID_Read_Write;
                ID_SE_dm_out = ID_SE_dm;
                ID_load_instr_out = ID_load_instr;
                ID_RF_enable_out = ID_RF_enable;
                ID_size_dm_out = ID_size_dm;
                ID_modifyCC_out = ID_modifyCC;
                ID_Call_instr_out = ID_Call_instr;
                ID_ALU_op3_out = ID_ALU_op3;
            end
                
            1'b1: //No Operation
            begin
                ID_jmpl_instr_out = 1'b0;
                ID_Read_Write_out = 1'b0;
                ID_SE_dm_out = 1'b0;
                ID_load_instr_out = 1'b0;
                ID_RF_enable_out = 1'b0;
                ID_size_dm_out = 2'b00;
                ID_modifyCC_out = 1'b0;
                ID_Call_instr_out = 1'b0;
                ID_ALU_op3_out = 6'b000000;    
            end
                
        endcase

        end

    
endmodule

module Sumador4(nPC, PC); 
    input [7:0] PC;
    output reg [7:0] nPC;
    always @(PC) 
        begin
            nPC = PC + 4; //nPC = nPC +4
        end
    
endmodule

module nPC (Q, Clk, D, LE, R);
    input [7:0] D;
    input LE;
    input Clk;
    input R;
    output reg [7:0] Q;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
        if (R) Q <= 8'b0000100; //En el caso de nPC un reset produce un número binario correspondiente a un 4.
        else if (LE) Q <= D; // LE = 1  D --> Q

endmodule

module PC (Q, Clk, D, LE, R);
    input [7:0] D; //cambiar a 9bits
    input LE;
    input Clk;
    input R;
    output reg [7:0] Q;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
        if (R) Q <= 8'b00000000; //un reset tienen el efecto de hacer cero todos los bits de salida del registro. 
        else if (LE) Q <= D; // LE = 1  D --> Q
    
endmodule

module InstructionMemory (output reg [31:0] DataOut, input [7:0] Address);

    reg [7:0] Mem [0:511];
    always @(Address) begin
    // Reading the Data
        DataOut = {Mem[Address], Mem[Address+1], Mem[Address+2], Mem[Address+3]};
    
    end

endmodule

module PipelineRegister_IF_ID(Q, Clk, Instr, LE, R); //eliminar LE 
    input [31:0] Instr;
    input LE;
    input Clk;
    input R;
    output reg [31:0] Q;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
    begin
        if (R) Q <= 32'b00000000000000000000000000000000; //un reset tienen el efecto de hacer cero todos los bits de salida del registro. 
        else if (LE) Q <= Instr; // LE = 1  D --> Q //else
    end
    
endmodule

module PipelineRegister_ID_EX(Q, EX_jmpl_instr, EX_Read_Write, EX_ALU_op3, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_modifyCC, EX_call_instr, Clk, Instr, ID_jmpl_instr, ID_Read_Write, ID_ALU_op3, ID_SE_dm, ID_load_instr, ID_RF_enable, ID_size_dm, ID_modifyCC, ID_call_instr);
    //add Reset to all PipelineRegisters
    //NO enviar Instruccion, solamente control_signals
    input [31:0] Instr;
    input Clk;
    input ID_jmpl_instr, ID_Read_Write, ID_SE_dm, ID_load_instr, ID_RF_enable , ID_modifyCC, ID_call_instr;
    input [1:0] ID_size_dm;
    input [5:0] ID_ALU_op3;
    output reg EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable , EX_modifyCC, EX_call_instr;
    output reg [1:0] EX_size_dm;
    output reg [5:0] EX_ALU_op3;
    output reg [31:0] Q; //concatenacion de todas senales;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
    begin
        Q <= Instr; //Output <= Input
        EX_jmpl_instr <= ID_jmpl_instr;
        EX_Read_Write <= ID_Read_Write;
        EX_ALU_op3 <= ID_ALU_op3; 
        EX_SE_dm <= ID_SE_dm; 
        EX_load_instr <= ID_load_instr; 
        EX_RF_enable <= ID_RF_enable; 
        EX_size_dm <= ID_size_dm; 
        EX_modifyCC <= ID_modifyCC;
        EX_call_instr <= ID_call_instr;
    end
    
endmodule

module PipelineRegister_EX_MEM(Q, MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_size_dm, MEM_call_instr, Clk, Instr, EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_size_dm, EX_call_instr);
    input [31:0] Instr;
    input Clk;
    input EX_jmpl_instr, EX_Read_Write, EX_SE_dm, EX_load_instr, EX_RF_enable, EX_call_instr;
    input [1:0] EX_size_dm;
    output reg MEM_jmpl_instr, MEM_Read_Write, MEM_SE_dm, MEM_load_instr, MEM_RF_enable, MEM_call_instr;
    output reg [1:0] MEM_size_dm;
    output reg [31:0] Q;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
    begin
        Q <= Instr; //Output <= Input
        MEM_jmpl_instr <= EX_jmpl_instr;
        MEM_Read_Write <= EX_Read_Write;
        MEM_SE_dm <= EX_SE_dm; 
        MEM_load_instr <= EX_load_instr; 
        MEM_RF_enable <= EX_RF_enable; 
        MEM_size_dm <= EX_size_dm;
        MEM_call_instr <= EX_call_instr;
    end
    
endmodule

module PipelineRegister_MEM_WB(Q, WB_RF_enable, Clk, Instr, MEM_RF_enable);
    input [31:0] Instr;
    input Clk;
    input MEM_RF_enable;
    output reg WB_RF_enable;
    output reg [31:0] Q;

    always @(posedge Clk) //0 --> 1 en Clk: entra al if
    begin
        Q <= Instr; //Output <= Input
        WB_RF_enable <= MEM_RF_enable; 
    end
    
endmodule