module ALU(A,B,OP,C,Bcond);

	input [31:0]A;
	input [31:0]B;
	input [4:0]OP;
	output [31:0]C;
	output Bcond;

	//TODO
	reg [31:0] C;
	reg Bcond;

	reg [63:0] temp;

	integer i,j;

	always @* begin
		Bcond = (A == B);

		case (OP)
		// Arithmetic
		// ADD
		5'b00000 : C = A + B; //done
		// SUB
		5'b00001 : C = A - B; //done
		// SLL
		5'b00010 : C = A << B[4:0]; //done
		// SLT
		5'b00011 : begin
			if (A[31] == B[31]) begin
				C = (A < B);
				Bcond = (A < B);
			end
			else begin
				if (A[31] == 0) begin
					C = 0;
					Bcond = 0;
				end
				else begin
					C = 1;
					Bcond = 1;
				end
			end
		end // done
		// SLTU
		5'b00100 : begin
			C = (A < B);
			Bcond = (A < B);
		end // done
		// XOR
		5'b00101 : C = A ^ B; // done
		// SRL
		5'b00110 : C = A >> B[4:0]; //done
		// SRA
		5'b00111 : begin
			i = 16*B[4]+8*B[3]+4*B[2]+2*B[1]+B[0];
			for(j=0;j<i;j=j+1) begin
				C[31-j] = A[31];
			end
			for(;j<32;j=j+1) begin
				C[31-j] = A[31-j+i];
			end
		end // done
		// OR
		5'b01000 : C = A | B; // done
		// AND
		5'b01001 : C = A & B; // done
		// MULT
		5'b01010 : begin
			temp = A * B;
			C = temp[31:0]; 
		end // done
		// MODULO
		5'b01011 : C = A % B;
		// IDENTITY
		5'b01100 : begin 
			C = B;
			Bcond = (A == B);
		end // done
		// NOT equal
		5'b01101 : begin
			C = ~A;
			Bcond = (A != B);
		end // done
		// GE
		5'b01110 : begin
			if (A[31] == B[31]) begin
				C = (A >= B);
				Bcond = (A >= B);
			end
			else begin
				if (A[31] == 0) begin
					C = 1;
					Bcond = 1;
				end
				else begin
					C = 0;
					Bcond = 0;
				end
			end
		end // done
		// GEU
		5'b01111 : begin
			C = (A >= B);
			Bcond = (A >= B);	
		end // done
		// IS_EVEN
		5'b10000 : begin
			if(A[0] == 0) begin
				C = 1;
			end
			else begin
				C = 0;
			end
		end
		default : begin
			C = 0;
			Bcond = 0;
		end

		endcase
	end
endmodule
