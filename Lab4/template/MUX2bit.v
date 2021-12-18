module MUX(I0,I1,I2,I3,E,F);

	input [31:0]I0;
	input [31:0]I1;
	input [31:0]I2;
	input [31:0]I3;
	input [1:0]E;
	output [31:0]F;

	//TODO
	assign F = (E == 0) ? I0 
			 : (E == 1) ? I1
			 : (E == 2) ? I2
			 : I3; 
endmodule