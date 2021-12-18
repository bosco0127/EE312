module MicroSequencer ();
	// state: IF000, ID001, EX010, MEM011, WB100
	
	reg state;
	always @(posedge clk) begin
	case(opcode)
		7'b1101111: begin //JAL
			case(state)
				3'b000: begin
					state <= 3'b001;
					PVSWriteEn <= 0;
				end
				3'b001: begin
					state <= 3'b010;
					PVSWriteEn <= 0;
				end
				3'b010: begin
					state <= 3'b100;
					PVSWriteEn <= 0;
				end
				3'b100: begin
					state <= 3'b000;
					PVSWriteEn <= 1;
				end
			endcase
		end
		7'b1100111: begin //JALR
			case(state)
				3'b000: begin
					state <= 3'b001;
					PVSWriteEn <= 0;
				end
				3'b001: begin
					state <= 3'b010;
					PVSWriteEn <= 0;
				end
				3'b010: begin
					state <= 3'b100;
					PVSWriteEn <= 0;
				end
				3'b100: begin
					state <= 3'b000;
					PVSWriteEn <= 1;
				end
			endcase
		end
		7'b1100011: begin //BEQ~BGEU
			case(state)
				3'b000: begin
					state <= 3'b001;
					PVSWriteEn <= 0;
				end
				3'b001: begin
					state <= 3'b010;
					PVSWriteEn <= 0;
				end
				3'b010: begin
					state <= 3'b000;
					PVSWriteEn <= 1;
				end
			endcase
		end
		7'b0000011: begin //LW, I-type Load
			case(state)
				3'b000: begin
					state <= 3'b001;
					PVSWriteEn <= 0;
				end
				3'b001: begin
					state <= 3'b010;
					PVSWriteEn <= 0;
				end
				3'b010: begin
					state <= 3'b011;
					PVSWriteEn <= 0;
				end
				3'b011: begin
					state <= 3'b100;
					PVSWriteEn <= 0;
				end
				3'b100: begin
					state <= 3'b000;
					PVSWriteEn <= 1;
				end
			endcase
		end
		7'b0100011: begin //SW, I-type Load
			case(state)
				3'b000: begin
					state <= 3'b001;
					PVSWriteEn <= 0;
				end
				3'b001: begin
					state <= 3'b010;
					PVSWriteEn <= 0;
				end
				3'b010: begin
					state <= 3'b011;
					PVSWriteEn <= 0;
				end
				3'b011: begin
					state <= 3'b000;
					PVSWriteEn <= 1;
				end
			endcase
		end
		7'b0010011: begin //ADDI~SRAI, I-type computational
			case(state)
				3'b000: begin
					state <= 3'b001;
					PVSWriteEn <= 0;
				end
				3'b001: begin
					state <= 3'b010;
					PVSWriteEn <= 0;
				end
				3'b010: begin
					state <= 3'b100;
					PVSWriteEn <= 0;
				end
				3'b100: begin
					state <= 3'b000;
					PVSWriteEn <= 1;
				end
			endcase
		end
		7'b0110011: begin //ADD~AND, R-type
			case(state)
				3'b000: begin
					state <= 3'b001;
					PVSWriteEn <= 0;
				end
				3'b001: begin
					state <= 3'b010;
					PVSWriteEn <= 0;
				end
				3'b010: begin
					state <= 3'b100;
					PVSWriteEn <= 0;
				end
				3'b100: begin
					state <= 3'b000;
					PVSWriteEn <= 1;
				end
			endcase
		end
		7'b0001011: begin //MULT~IS_EVEN, R or I type
			case(state)
				3'b000: begin
					state <= 3'b001;
					PVSWriteEn <= 0;
				end
				3'b001: begin
					state <= 3'b010;
					PVSWriteEn <= 0;
				end
				3'b010: begin
					state <= 3'b100;
					PVSWriteEn <= 0;
				end
				3'b100: begin
					state <= 3'b000;
					PVSWriteEn <= 1;
				end
			endcase
		end
		endcase
	end


endmodule