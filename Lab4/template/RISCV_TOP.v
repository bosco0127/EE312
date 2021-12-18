module RISCV_TOP (
	//General Signals
	input wire CLK,
	input wire RSTn,

	//I-Memory Signals
	output wire I_MEM_CSN,
	input wire [31:0] I_MEM_DI,//input from IM
	output reg [11:0] I_MEM_ADDR,//in byte address

	//D-Memory Signals
	output wire D_MEM_CSN,
	input wire [31:0] D_MEM_DI,
	output wire [31:0] D_MEM_DOUT,
	output wire [11:0] D_MEM_ADDR,//in word address
	output wire D_MEM_WEN,
	output wire [3:0] D_MEM_BE,

	//RegFile Signals
	output wire RF_WE,
	output wire [4:0] RF_RA1,
	output wire [4:0] RF_RA2,
	output wire [4:0] RF_WA1,
	input wire [31:0] RF_RD1,
	input wire [31:0] RF_RD2,
	output wire [31:0] RF_WD,
	output wire HALT,                   // if set, terminate program
	output reg [31:0] NUM_INST,         // number of instruction completed
	output wire [31:0] OUTPUT_PORT      // equal RF_WD this port is used for test
	);

	// TODO: implement multi-cycle CPU
	assign OUTPUT_PORT = RF_WD;

	initial begin
		NUM_INST <= 0;
	end
	
	// Only allow for NUM_INST
	always @ (negedge CLK) begin
		if (RSTn) NUM_INST <= NUM_INST + 1;
	end

	// TODO: implement

	// register for CSN wire
	reg CSN;
	// CSN assignments
	assign I_MEM_CSN = CSN;
	assign D_MEM_CSN = CSN;

	// Control Signal registers & wires
	// state: IF000, ID001, EX010, MEM011, WB100
	reg state;
	reg PVSWriteEn;

	// Control signal registers
	reg isStore;
	reg isLoad;
	reg isJAL;
	reg isJALR;
	reg isBranchTaken;
	reg ALUSrc1;
	reg ALUSrc2;
	// Control signal wires
	// RegDest
	// RegDest = Maybe not necessary
	reg [4:0] ALUOp;
	wire [4:0] ALUOp_wire = ALUOp;
	// MemtoReg = isLoad
	wire MemtoReg = isLoad;
	// RegWrite is equal to RF_WE
	// RegWrite =  !isStore
	assign RF_WE = (~isStore) & (~isBranchTaken);
	// MemRead = isLoad
	// MemRead = Maybe not necessary
	// MemWrite = isStore
	assign D_MEM_WEN = ~isStore;
	// PCSrc = isJAL | isBranchTaken
	wire PCSrc = isJAL | isBranchTaken;

	// ALU input1/2, output of MUX1
	wire [31:0] ALUinput1;
	wire [31:0] ALUinput2;

	// Sign extended immediate, input of MUX1
	reg [31:0] imm_extended;
	wire [31:0] imm_extended_wire = imm_extended;

	// output of MUX2
	wire [31:0] WriteData_wire;
	// sign extend register
	reg [23:0] extend_reg; // for sign extend
	// I0 input of MUX5
	wire [31:0] WriteData_extend_wire = (isLoad && (I_MEM_DI[14:12] == 3'b000)) ? {extend_reg,WriteData_wire[7:0]} : // LB
										(isLoad && (I_MEM_DI[14:12] == 3'b001)) ? {extend_reg[15:0],WriteData_wire[15:0]} : // LH
										(isLoad && (I_MEM_DI[14:12] == 3'b100)) ? {24'h000000,WriteData_wire[7:0]} : // LBU
										(isLoad && (I_MEM_DI[14:12] == 3'b101)) ? {16'h0000,WriteData_wire[15:0]} : WriteData_wire; // LHU or else

	// ALU outputs
	wire [31:0] ALUresult;
	wire Bcond;

	// PC
	reg [31:0] PC;
	// PC wire
	wire [31:0] PC_wire = PC;
	// PC + 4 register
	reg [31:0] PC_4;
	// PC + 4 wire
	wire [31:0] PC_4_wire = PC_4;
	// Jump or Branch instruction target wire
	wire [31:0] JBtarget;
	// Jump or Branch nextPC
	reg [31:0] nextPC;
	// nextPC wire
	wire [31:0] nextPC_wire = nextPC;
	// Branch MUX3 result
	wire [31:0] I_MEM_target;

	// Connect to ALU
	ALU ALU1 (
		.A (ALUinput1),
		.B (ALUinput2),
		.OP (ALUOp_wire),
		.C (ALUresult),
		.Bcond (Bcond)
	);

	// Connect to MUX
	// MUX between REG_FILE & ALU
	MUX2bit MUX1 (
		.I0 (RF_RD2),
		.I1 (32'h00000004),
		.I2 (imm_extended_wire),
		.I3 (32'bz),
		.E (ALUSrc2),
		.F (ALUinput2)
	);

	// MUX between D_MEM & REG_FILE
	MUX MUX2 (
		.I0 (ALUresult),
		.I1 (D_MEM_DI),
		.E (MemtoReg),
		.F (WriteData_wire)
	);

	// MUX for Branch or Jump
	MUX MUX3 (
		.I0 (PC_4_wire),
		.I1 (nextPC_wire),
		.E (PCSrc),
		.F (I_MEM_target)
	);

	// MUX for ALUSrc1
	MUX MUX4 (
		.I0 (RF_RD1),
		.I1 (PC_wire),
		.E (ALUSrc1),
		.F (ALUinput1)
	);

	// MUX for determine JAL_B vs JALR
	MUX MUX6 (
		.I0 (ALUresult),
		.I1 ({ALUresult[31:1],1'b0}),
		.E (isJALR),
		.F (JBtarget)
	);

	// **Continuous Assignments**
	// I-MEM
	assign I_MEM_ADDR = PC[11:0];
	// D-MEM
	// BE register
	reg [3:0] BE;
	assign D_MEM_DOUT = RF_RD2;
	assign D_MEM_ADDR = ALUresult[11:0];
	assign D_MEM_BE = BE;
	// Reg-FILE
	assign RF_RA1 = I_MEM_DI[19:15];
	assign RF_RA2 = I_MEM_DI[24:20];
	assign RF_WA1 = I_MEM_DI[11:7];
	assign RF_WD = WriteData_extend_wire;
	// HALT
	reg HALT_reg1;
	reg HALT_reg2;
	assign HALT = HALT_reg2;

	// repeat concatenation register
	reg [11:0] con_12;
	reg [19:0] con_20;

	// **Sequential Program Counter**
	always @(posedge CLK) begin
		if(~RSTn) begin
			PC <= 0;
		end
		else begin
			if(PVSWriteEn == 0) begin
				PC <= PC;
			end
			else begin
				PC <= I_MEM_target; 
			end
		end
	end

	// **MicroSequencer**

	// *Sequential State Updater*
	// state: IF000, ID001, EX010, MEM011, WB100
	always @(posedge CLK) begin
		if(~RSTn) begin
			state <= 0;
			PVSWriteEn <= 0;
		end
		else begin
			case(I_MEM_DI[6:0])
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
	end
	
	// *Combinational Logic Controller*
	always @ (*) begin
		if (~RSTn) begin
			CSN = 1;
			imm_extended = 0;
			ALUSrc1 = 0;
			ALUSrc2 = 2;
			ALUOp = 5'b00000;
			isStore = 0;
			isLoad = 0;
			isJAL = 0;
			isJALR = 0;
			isBranchTaken = 1;
			PC_4 = ALUresult;
			nextPC = JBtarget;
			HALT_reg1 = 0;
			HALT_reg2 = 0;
			extend_reg = 0;
		end
		else begin
			CSN = 0;
			// Control:
			// 1. imm_extend
			// 2. ALUOp
			// 3. control register
			// 4. PC_4 & nextPC
			case(state)
				3'b000: begin // IF state
					ALUSrc1 = 0;
					ALUSrc2 = 1;
					ALUOp = 5'b00000;
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranchTaken = 0;
					PC_4 = ALUresult;
					nextPC = nextPC;
				end
				3'b001: begin // ID state
					case(I_MEM_DI[6:0])
						7'b1101111: imm_extended = {12{I_MEM_DI[31]}, I_MEM_DI[19:12], I_MEM_DI[20], I_MEM_DI[30:21], 1'b0}; //JAL
						7'b1100111: imm_extended = {20{I_MEM_DI[31]}, I_MEM_DI[30:21]}; //JALR
						7'b1100011: imm_extended = {20{I_MEM_DI[31]}, I_MEM_DI[7], I_MEM_DI[30:25], I_MEM_DI[11:8], 1'b0}; //BEQ~BGEU
						7'b0000011: imm_extended = {20{I_MEM_DI[31]}, I_MEM_DI[31:20]}; //LW
						7'b0100011: imm_extended = {20{I_MEM_DI[31]}, I_MEM_DI[31:25], I_MEM_DI[11:7]}; //SW
						7'b0010011: imm_extended = {20{I_MEM_DI[31]}, I_MEM_DI[31:20]}; //ADDI~SRAI, I-type computational
						7'b0001011: imm_extended = 2; //custom
						default: imm_extended = 4;
					endcase
					// Reg-FILE
					RF_RA1 = I_MEM_DI[19:15];
					RF_RA2 = I_MEM_DI[24:20];
					regA = RF_RD1;
					regB = RF_RD2;
					ALUSrc1 = 0;
					ALUSrc2 = 2;
					ALUOp = 5'b00000;      
				end
				3'b010: begin // EX state
					case(I_MEM_DI[6:0])
						7'b1101111: begin //JAL
						nextPC = ALUout
						ALUSrc1 = 0;
						ALUSrc2 = 1;
						end
						7'b1100111:   begin //JALR
						nextPC = ALUout
						ALUSrc1 = 0;
						ALUSrc2 = 1;
						end
						7'b1100011: begin //BEQ~BGEU
						case(I_MEM_DI[14:12])
							3'b000: ALUOp = 5'b01100; //BEQ   
							3'b001: ALUOp = 5'b01101; //BNE
							3'b100: ALUOp = 5'b00011; //BLT
							3'b101: ALUOp = 5'b01110; //BGE
							3'b110: ALUOp = 5'b00100; //BLTU
							3'b111: ALUOp = 5'b01111; //BGEU
						endcase
						isBranchTaken = Bcond;
						end
						7'b0000011: begin //LW
							ALUSrc1 = 1;
							ALUSrc2 = 2;
						end
						7'b0100011: begin //SW
							ALUSrc1 = 1;
							ALUSrc2 = 2;
						end
						7'b0010011: begin //ADDI~SRAI, I-type computational
							case(I_MEM_DI[14:12])
								3'b000: ALUOp = 5'b00000; //ADDI
								3'b010: ALUOp = 5'b00011; //SLTI
								3'b011: ALUOp = 5'b00100; //SLTIU
								3'b100: ALUOp = 5'b00101; //XORI
								3'b110: ALUOp = 5'b01000; //ORI
								3'b111: ALUOp = 5'b01001; //ANDI 
								3'b001: ALUOp = 5'b00010; //SLLI
								3'b101: begin //SRLI, SRAI
									if (I_MEM_DI[31:25] == 7'b0000000) //SRLI
										ALUOp = 5'b00110;
									else if (I_MEM_DI[31:25] == 7'b0100000) //SRAI
										ALUOp = 5'b00111;
								end
							endcase
							ALUSrc1 = 1;
							ALUSrc2 = 2;
						end
						7'b0110011: begin //ADD~AND
						case(I_MEM_DI[14:12])
							3'b000: begin //ADD, SUB
								if (I_MEM_DI[31:25] == 7'b0000000) //ADD
									ALUOp = 5'b00000;
								if (I_MEM_DI[31:25] == 7'b0100000) //SUB
									ALUOp = 5'b00001;
							end
							3'b001: ALUOp = 5'b00010; //SLL
							3'b010: ALUOp = 5'b00011; //SLT
							3'b011: ALUOp = 5'b00100; //SLTU
							3'b100: ALUOp = 5'b00101; //XOR
							3'b101: begin //SRL, SRA
								if (I_MEM_DI[31:25] == 7'b0000000) //SRL
									ALUOp = 5'b00110;
								if (I_MEM_DI[31:25] == 7'b0100000) //SRA
									ALUOp = 5'b00111;
							end
							3'b110: ALUOp = 5'b01000; //OR
							3'b111: ALUOp = 5'b01001; //AND
						endcase
						ALUSrc1 = 1;
						ALUSrc2 = 0;
						end
						7'b0001011: begin //custom
						case(I_MEM_DI[14:12])
							3'b111: begin //MULT, MODULO
								ALUSrc1 = 1;
								ALUSrc2 = 0;
								if (I_MEM_DI[31:25] == 7'b0000000) begin // MULT
									ALUOp = 5'b01010; // rd = rs1*rs2
								end
								else if (I_MEM_DI[31:25] == 7'b0000001) begin // MODULO
									ALUOp = 5'b01011; // rd = rs1&rs2
								end
							end
							3'b110: begin //IS_EVEN
								ALUSrc1 = 1;
								ALUSrc2 = 2;
								ALUOp = 5'b10000;
							end
						endcase
						end
					endcase
				end
				3'b011: begin // MEM state
					D_MEM_ADDR = ALUresult[11:0]
				end
				3'b100: begin // WB state
					RF_WA1 = I_MEM_DI[11:7];
					// PC =  PC_4 or nextPC
					if ((I_MEM_DI[6:0] == 7'b1101111) || (I_MEM_DI[6:0] == 7'b1100111))
						PCSrc = 1; // nextPC
					else 
						PCSrc = 0; // PC_4
				end
			endcase
			// HALT control
			if (I_MEM_DI == 32'h00c00093) begin
				HALT_reg1 = 1;
			end
			else if ((HALT_reg1 == 1) && (I_MEM_DI == 32'h00008067)) begin
				HALT_reg2 = 1;
			end
			else begin
				HALT_reg1 = 0;
				HALT_reg2 = 0;
			end
		end
		
	end
	
endmodule //
