`timescale 1ns / 100ps

module ALU(A,B,OP,C,Cout);

	input [15:0]A;
	input [15:0]B;
	input [3:0]OP;
	output [15:0]C;
	output Cout;

	//TODO
	reg [15:0] C;
	reg Cout=0;
	reg [16:0] temp1;
	reg [15:0]	temp2;

	always @* begin
		case (OP)
		// Arithmetic
		4'b0000 : begin
			C = A + B;
			temp1 = {1'b0,A} + {1'b0,B};
			temp2 = {1'b0,A[14:0]} + {1'b0,B[14:0]};
			Cout = temp1[16] ^ temp2[15];
		end
		4'b0001 : begin
			C = A - B;
			temp1 = {1'b0,A} - {1'b0,B};
			temp2 = {1'b0,A[14:0]} - {1'b0,B[14:0]};
			Cout = temp1[16] ^ temp2[15];			
		end
		// Bitwise Boolean operation
		4'b0010 : C = A & B;
		4'b0011 : C = A | B;
		4'b0100 : C = ~(A & B);
		4'b0101 : C = ~(A | B);
		4'b0110 : C = A ^ B;
		4'b0111 : C = ~(A ^ B);

		// Logic
		4'b1000 : C = A;
		4'b1001 : C = ~A;

		// Shift
		4'b1010 : C = A >> 1;
		4'b1011 : C = {A[15],A[15:1]};
		4'b1100 : C = {A[0],A[15:1]};
		4'b1101 : C = A << 1;
		4'b1110 : C = A <<< 1;
		4'b1111 : C = {A[14:0],A[15]};
		default : C = 0;

		endcase
	end
endmodule
