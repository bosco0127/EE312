module ADDALU(A,B,C);

	input [31:0]A;
	input [31:0]B;
	output [31:0]C;

	//TODO
	reg [31:0] C;

	always @* begin
		C = A + B; //done
	end
endmodule
