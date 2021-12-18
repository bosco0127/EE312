module MUX(I0,I1,E,F);

	input [31:0]I0;
	input [31:0]I1;
	input E;
	output [31:0]F;

	//TODO
	assign F = (E == 0) ? I0 : I1; 
endmodule