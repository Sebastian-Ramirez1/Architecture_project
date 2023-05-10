`timescale 1ns/1ns

// ALU Module
module ALU (output reg signed[31:0] Out, output reg[3:0] Flags, input[3:0] Op, input signed[31:0]A, B, input Cin);

    reg Carry;

    always @(Op, A, B, Cin)
    begin
        case(Op)

            4'd0:
            begin
                {Carry, Out} = A + B; 
                Flags[3] = (Out == 32'd0)? 1 : 0;
                Flags[2] = Out[31];
                Flags[1] = Carry;
                Flags[0] = 0;

                if (A[31] == 0 && B[31] == 0) 
                begin
                    if (Flags[2]) Flags[0] = 1;
                end

                else if (A[31] == 1 && B[31] == 1) 
                begin
                    if (!Flags[2]) Flags[0] = 1;
                end
            end

            4'd1:
            begin
                {Carry, Out} = A + B + Cin;
                Flags[3] = (Out == 32'd0)? 1 : 0;
                Flags[2] = Out[31];
                Flags[1] = Carry;
                Flags[0] = 0;

                if (A[31] == 0 && B[31] == 0) 
                begin
                    if (Flags[2]) Flags[0] = 1;
                end

                else if (A[31] == 1 && B[31] == 1) 
                begin
                    if (!Flags[2]) Flags[0] = 1;
                end
            end

            4'd2:
            begin
                {Carry, Out} = A - B; 
                Flags[3] = (Out == 32'd0)? 1 : 0;
                Flags[2] = Out[31];
                Flags[1] = Carry;
                Flags[0] = 0;

                if (A[31] == 0 && B[31] == 1)
                begin
                    if (Flags[2]) Flags[0] = 1;
                end

                if (A[31] == 1 && B[31] == 0)
                begin
                    if (!Flags[2]) Flags[0] = 1;
                end
            end
        
            4'd3:
            begin
                Out = A - B - Cin;
                Flags[3] = (Out == 32'd0)? 1 : 0;
                Flags[2] = Out[31];
                {Flags[1], Flags[0]} = 2'b00;

                if (A[31] == 0 && B[31] == 1)
                begin
                    if (Flags[2]) Flags[0] = 1;
                end

                if (A[31] == 1 && B[31] == 0)
                begin
                    if (!Flags[2]) Flags[0] = 1;
                end
            end

            4'd4: 
            begin
                Out = A & B;
                Flags[3] = (Out == 32'd0)? 1 : 0;
                Flags[2] = Out[31];
                {Flags[1], Flags[0]} = 2'd0;
            end

            4'd5:
            begin
                Out = A | B;
                Flags[3] = (Out == 32'd0)? 1 : 0;
                Flags[2] = Out[31];
                {Flags[1], Flags[0]} = 2'd0;
            end

            4'd6:
            begin
                Out = A ^ B;
                Flags[3] = (Out == 32'd0)? 1 : 0;
                Flags[2] = Out[31];
                {Flags[1], Flags[0]} = 2'd0;
            end

            4'd7:
            begin
                Out = ~(A ^ B);
                Flags[3] = (Out == 32'd0)? 1 : 0;
                Flags[2] = Out[31];
                {Flags[1], Flags[0]} = 2'd0;
            end

            4'd8:
            begin
                Out = A & ~B;
                Flags[3] = (Out == 32'd0)? 1 : 0;
                Flags[2] = Out[31];
                {Flags[1], Flags[0]} = 2'd0;
            end

            4'd9:
            begin
                Out = A | ~B;
                Flags[3] = (Out == 32'd0)? 1 : 0;
                Flags[2] = Out[31];
                {Flags[1], Flags[0]} = 2'd0;
            end

            4'd10: 
            begin
                Out = A << B;
                Flags = 4'd0;
            end

            4'd11: 
            begin
                Out = A >> B;
                Flags = 4'd0;
            end

            4'd12: 
            begin
                Out = $signed(A) >>> B;
                Flags = 4'd0;
            end

            4'd13: 
            begin
                Out = A;
                Flags = 4'd0;
            end

            4'd14: 
            begin
                Out = B;
                Flags = 4'd0;
            end

            4'd15:
            begin
                Out = ~B;
                Flags = 4'd0;
            end

        endcase
    end

endmodule

// ALU Test Bench
module ALU_tb;

    reg signed[31:0] A, B;
    reg[3:0] Op;
    reg Cin;

    wire signed[31:0] Out;
    wire[3:0] Flags;

    ALU uut(Out, Flags, Op, A, B, Cin);

    initial begin
        //$display("\n Arithmetic Logic Unit (ALU):\n\n Op\tA\t\t\t\t  (Decimal)    B\t\t\t\t  (Decimal)  Cin  Out\t\t\t\t    (Decimal)    Z N C V");
        $monitor(" %b | %b %d | %b %d | %b | %b %d | %b %b %b %b ", Op, A, A, B, B, Cin, Out, Out, Flags[3], Flags[2], Flags[1], Flags[0]);

        //A + B
        Op = 4'd0; A = 32'hF0000000; B = 32'h08765254; Cin = 0; #20;
        Op = 4'd0; A = 32'h77315843; B = 32'h31539734; Cin = 0; #20;
        $display();

        //A + B + Cin
        Op = 4'd1; A = 32'h70000001; B = 32'h74543585; Cin = 0; #20;
        Op = 4'd1; A = -32'h70000001; B = -32'h0FFFFFFF; Cin = 1; #20;
        $display();

        //A - B
        Op = 4'd2; A = -32'h70000001; B = 32'h0FFFFFFF; Cin = 0; #20;
        Op = 4'd2; A = -32'h70000001; B = 32'h76843217; Cin = 0; #20;
        $display();

        //A - B - Cin
        Op = 4'd3; A = 32'h0ABC8673; B = -32'h7FFFFFFF; Cin = 1; #20;
        Op = 4'd3; A = -32'h7FFFFFFF; B = -32'h7FFFFFFF; Cin = 0; #20;
        $display();

        //A & B
        Op = 4'd4; A = -32'h0FFFFFFF; B = 32'h70000000; Cin = 0; #20;
        $display();

        //A | B
        Op = 4'd5; A = -32'h0FFFFFFF; B = 32'h0FFF0FFF; Cin = 0; #20;
        $display();

        //A ^ B
        Op = 4'd6; A = 32'h0FFFFFFF; B = 32'h40005001; Cin = 0; #20;
        $display();

        //~(A ^ B)
        Op = 4'd7; A = 32'h0FFFFFFF; B = 32'h40005001; Cin = 0; #20;
        $display();

        //A & ~B
        Op = 4'd8; A = 32'h08321FFF; B = 32'h08321FFF; Cin = 0; #20;
        $display();

        //A | ~B
        Op = 4'd9; A = -32'h0FFFFFFF; B = 32'h700A0001; Cin = 0; #20;
        $display();

        //A << B
        Op = 4'd10; A = -32'h0FFFFFFF; B = 32'h00000003; Cin = 0; #20;
        $display();

        //A >> B
        Op = 4'd11; A = -32'h7000000F; B = 32'h00000003; Cin = 0; #20;
        $display();

        //A >>> B
        Op = 4'd12; A = -32'h7000000F; B = 32'h00000003; Cin = 0; #20;
        $display();

        //A
        Op = 4'd13; A = -32'h7000000F; B = 32'h00000003; Cin = 0; #20;
        $display();

        //B
        Op = 4'd14; A = -32'h7000000F; B = 32'h02180003; Cin = 0; #20;
        $display();

        //~B
        Op = 4'd15; A = -32'h7000000F; B = 32'h02180003; Cin = 0; #20;
        $display();
    end

endmodule

// Source Operand2 Handler Module
module Source_Operand2_Handler(output reg[31:0] N, input[3:0] I, input[31:0] R, input[21:0] Imm, input bit23);

    reg[31:0] intermediate;

    always @(R, Imm, I)
    begin
        if (I < 4'd4)
        begin
            N = {Imm, 10'h0};
        end

        if (I >= 4'd4 && I < 4'd8)
        begin
            N = (Imm[21])? {10'h3FF, Imm} : {10'h0, Imm};
        end

        if (I == 4'd8 || I == 4'd12 || I == 4'd14) N = R;

        if (I == 4'd9 || I == 4'd13 || I == 4'd15)
        begin
            intermediate = {10'h0, Imm};
            N = intermediate | 32'hFFFFE000;
            if (!Imm[12]) N = N + 32'h00002000;
        end
        
        case (I)
            4'd10:
            begin
                if(bit23 == 1'b1)begin
                    if(I[0] == 1'b0) N = R; //Is = Bit13
                    else N = {{19{Imm[12]}}, Imm[12:0]};
                end
                else N = {{27{1'b0}}, R[4:0]};
            end
            //N = (R | 32'hFFFFFFE0) + 32'h20;

            4'd11: 
            begin
                intermediate = {10'h0, Imm};
                N = (intermediate | 32'hFFFFFFE0) + 32'h20;
            end
        endcase
    end

endmodule

// Source Operand2 Handler Test Bench
// module Source_Operand_Handler_tb;

//     reg[31:0] R;
//     reg[21:0] Imm;
//     reg[3:0] I;

//     wire[31:0] N;

//     Source_Operand2_Handler uut(N, I, R, Imm);

//     initial
//     begin
//         #500;

//         $display("\n Source Operand2 Handler:\n\n Part 1: Imm = 1000110001000100010011\n I\tR\t\t\t\t   Imm\t\t\t    N ");
//         $monitor(" %b | %b | %b | %b ", I, R, Imm, N);
        
//         // Part 1: Imm = 1000110001000100010011
//         // 0000 <= I <= 0011: Imm || 0b0000000000
//         I = 4'd0; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         I = 4'd1; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         I = 4'd2; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         I = 4'd3; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         // 0100 <= I <= 0111: Imm (Sign Extended)
//         I = 4'd4; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         I = 4'd5; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         I = 4'd6; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         I = 4'd7; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         // R
//         I = 4'd8; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         // Imm(12-0) (sign extended)
//         I = 4'd9; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         // 0b0000…000 ||R(4-0)
//         I = 4'd10; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         // 0b0000…000 ||Imm(4-0)
//         I = 4'd11; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         // R
//         I = 4'd12; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         // Imm(12-0) (sign extended)
//         I = 4'd13; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         // R
//         I = 4'd14; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         // Imm(12-0) (sign extended)
//         I = 4'd15; R = 32'hE0000003; Imm = 22'h231113; #20;
//         $display();

//         // Part 2: Imm = 1000110000000100010011 
//         $display("\n Part 2: Imm = 1000110000000100010011\n I\tR\t\t\t\t   Imm\t\t\t    N ");
//         // R
//         I = 4'd8; R = 32'hE0000003; Imm = 22'h230113; #20;
//         $display();

//         // Imm(12-0) (sign extended)
//         I = 4'd9; R = 32'hE0000003; Imm = 22'h230113; #20;
//         $display();

//         // 0b0000…000 ||R(4-0)
//         I = 4'd10; R = 32'hE0000003; Imm = 22'h230113; #20;
//         $display();

//         // 0b0000…000 ||Imm(4-0)
//         I = 4'd11; R = 32'hE0000003; Imm = 22'h230113; #20;
//         $display();

//         // R
//         I = 4'd12; R = 32'hE0000003; Imm = 22'h230113; #20;
//         $display();

//         // Imm(12-0) (sign extended)
//         I = 4'd13; R = 32'hE0000003; Imm = 22'h230113; #20;
//         $display();

//         // R
//         I = 4'd14; R = 32'hE0000003; Imm = 22'h230113; #20;
//         $display();

//         // Imm(12-0) (sign extended)
//         I = 4'd15; R = 32'hE0000003; Imm = 22'h230113; #20;

//     end

// endmodule