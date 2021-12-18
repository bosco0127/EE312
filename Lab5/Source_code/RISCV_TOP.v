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

	// TODO: implement

	// register for CSN wire
	reg CSN;
	// CSN assignments
	assign I_MEM_CSN = CSN;
	assign D_MEM_CSN = CSN;

	// pipeline registers
	// IF
	reg stall_IF;
	reg flush_IF;
	// ID
	/*------------------------------------------------------------*/
	reg [31:0] IR;
	reg [31:0] PC_ID;
	reg stall_ID;
	reg flush_ID;
	reg PVSWriteEn_ID;
	reg [31:0] imm_extended;
	reg isStore;
	reg isLoad;
	reg isJAL;
	reg isJALR;
	reg isBranch;
	reg PVSWriteEn;
	reg [1:0] ALUSrc1;
	reg [1:0] ALUSrc2;
	reg [1:0] JBSrc;
	reg [4:0] ALUOp;
	// EX
	/*------------------------------------------------------------*/
	reg [31:0] PC_EX;
	wire [31:0] PC_EX_wire = PC_EX;
	reg [31:0] regA_EX;
	wire [31:0] regA_EX_wire = regA_EX;
	reg [31:0] regB_EX;
	wire [31:0] regB_EX_wire = regB_EX;
	reg [31:0] imm_extended_EX;
	reg [4:0] RA1_EX;
	reg [4:0] RA2_EX;
	reg [4:0] WA1_EX;
	// Control signal registers
	reg isStore_EX;
	reg isLoad_EX;
	reg isJAL_EX;
	reg isJALR_EX;
	reg isBranch_EX;
	reg PVSWriteEn_EX;
	reg [1:0] ALUSrc1_EX;
	reg [1:0] ALUSrc2_EX;
	reg [1:0] JBSrc_EX;
	reg [4:0] ALUOp_EX;
	reg flush_EX_JB;
	reg flush_EX_LW;
	reg HALT_EX;
	// MEM
	/*------------------------------------------------------------*/
	reg [31:0] ALUOut_MEM;
	reg [31:0] regB_MEM;
	reg [4:0] WA1_MEM;
	// Control signal registers
	reg isStore_MEM;
	reg isLoad_MEM;
	reg isBranch_MEM;
	reg isBranchTaken_MEM;
	reg PVSWriteEn_MEM;
	reg HALT_MEM;
	// WB
	/*------------------------------------------------------------*/
	reg [31:0] MDR_WB;
	reg [31:0] ALUOut_WB;
	reg [4:0] WA1_WB;
	// Control signal registers
	reg isStore_WB;
	reg isLoad_WB;
	reg isBranch_WB;
	reg isBranchTaken_WB;
	reg PVSWriteEn_WB;
	reg HALT_WB;
	/*------------------------------------------------------------*/
	// Data Forwarding
	reg isAforward;
	reg [31:0] A_forward;
	wire [31:0] A_forward_wire = A_forward;
	reg isBforward;
	reg [31:0] B_forward;
	wire [31:0] B_forward_wire = B_forward;
	/*------------------------------------------------------------*/

	// ALU outputs
	wire [31:0] ALUresult;
	wire Bcond;

	// Control signal wires
	// RegDest
	// RegDest = Maybe not necessary
	wire [4:0] ALUOp_wire = ALUOp_EX;
	// MemtoReg = isLoad_WB
	wire MemtoReg = isLoad_WB;
	// RegWrite is equal to RF_WE
	// RegWrite =  !isStore
	assign RF_WE = (~isStore_WB) & (~isBranch_WB);
	// MemRead = isLoad
	// MemRead = Maybe not necessary
	// MemWrite = isStore
	assign D_MEM_WEN = ~isStore_MEM;
	// PCSrc = isJAL | isBranchTaken
	wire PCSrc = isJAL_EX | isJALR_EX | (isBranch_EX & Bcond);

	// ALU input1/2, output of MUX1
	wire [31:0] ALUinput1;
	wire [31:0] ALUinput2;

	// Sign extended immediate, input of MUX1
	wire [31:0] imm_extended_wire = imm_extended_EX;

	// PC
	reg [31:0] PC;
	// PC wire
	wire [31:0] PC_wire = PC;
	// PC + 4 register
	//reg [31:0] PC_4;
	// PC + 4 wire
	wire [31:0] PC_4_wire;
	// Jump or Branch instruction target wire
	wire [31:0] JBtarget_wire;
	// Jump or Branch nextPC
	// reg [31:0] nextPC;
	// nextPC wire
	wire [31:0] nextPC_wire;
	// Branch MUX5 result
	wire [31:0] MUX5_result_wire;
	// MUX 2 result
	wire [31:0] MUX2_result_wire;

	// Connect to ALU
	ALU ALU1 (
		.A (ALUinput1),
		.B (ALUinput2),
		.OP (ALUOp_wire),
		.C (ALUresult),
		.Bcond (Bcond)
	);

	ADDALU ADDALU1 (
		.A (PC_wire),
		.B (32'h00000004),
		.C (PC_4_wire)
	);

	ADDALU ADDALU2 (
		.A (MUX5_result_wire),
		.B (imm_extended_wire),
		.C (JBtarget_wire)
	);

	// Connect to MUX
	// MUX between REG_FILE & ALU
	MUX2bit MUX1 (
		.I0 (regB_EX_wire),
		.I1 (32'h00000004),
		.I2 (imm_extended_wire), // forwarding
		.I3 (B_forward_wire),
		.E ({(ALUSrc2_EX[1]|((~ALUSrc2_EX[0]) & isBforward)),(ALUSrc2_EX[0]|((~ALUSrc2_EX[1]) & isBforward))}),
		.F (ALUinput2)
	);

	// MUX between D_MEM & REG_FILE
	MUX MUX2 (
		.I0 (ALUOut_WB),
		.I1 (MDR_WB),
		.E (MemtoReg),
		.F (MUX2_result_wire)
	);

	// MUX for Branch or Jump
	MUX MUX3 (
		.I0 (PC_4_wire),
		.I1 ({JBtarget_wire[31:1],(~isJALR_EX)&(JBtarget_wire[0])}),
		.E (PCSrc),
		.F (nextPC_wire)
	);

	// MUX for ALUSrc1
	MUX2bit MUX4 (
		.I0 (PC_EX_wire),
		.I1 (regA_EX_wire),
		.I2 (A_forward_wire), // forwarding
		.I3 (32'bz),
		.E ({(ALUSrc1_EX[1]|(ALUSrc1_EX[0] & isAforward)),(ALUSrc1_EX[0] & (~isAforward | ALUSrc1_EX[1]))}),
		.F (ALUinput1)
	);

	// MUX for (Un)Conditional Jump
	MUX2bit MUX5 (
		.I0 (PC_EX_wire),
		.I1 (regA_EX_wire),
		.I2 (A_forward_wire), // forwarding
		.I3 (32'bz),
		.E ({(JBSrc_EX[1] | (JBSrc_EX[0] & isAforward)),(JBSrc_EX[0] & (~isAforward | JBSrc_EX[1]))}),
		.F (MUX5_result_wire)
	);

	// **Continuous Assignments**
	// I-MEM
	assign I_MEM_ADDR = PC[11:0];
	// D-MEM
	// BE register
	reg [3:0] BE = 4'b1111;
	assign D_MEM_DOUT = regB_MEM;
	assign D_MEM_ADDR = ALUOut_MEM[11:0];
	assign D_MEM_BE = BE;
	// Reg-FILE
	assign RF_RA1 = IR[19:15];
	assign RF_RA2 = IR[24:20];
	assign RF_WA1 = WA1_WB;
	assign RF_WD = MUX2_result_wire;
	// HALT
	reg HALT_reg1;
	reg HALT_reg2;
	assign HALT = HALT_WB;
	// OUTPUT_PORT
	assign OUTPUT_PORT = ( ~isBranch_WB ) ? RF_WD : {28'h0000000,3'b000,isBranchTaken_WB};

	// repeat concatenation register
	reg [11:0] con_12;
	reg [19:0] con_20;

	initial begin
		NUM_INST <= 0;
	end

	// Only allow for NUM_INST
	always @ (negedge CLK) begin
		if (RSTn) begin
			if(PVSWriteEn_WB == 1) begin
				NUM_INST <= NUM_INST + 1;
			end
			else begin
				NUM_INST <= NUM_INST;
			end
		end
	end

	// **Sequential Program Counter**
	always @(posedge CLK) begin
		if(~RSTn) begin
			PC <= -4;
		end
		else begin
			if( ~stall_IF ) begin
				PC <= nextPC_wire;
			end
			else begin 
				PC <= PC;
			end
		end
	end

	// *Sequential Logic
	always @(posedge CLK) begin
		if(~RSTn) begin
			CSN <= 1;
			/* pipeline register */
			// IF
			/*------------------------------------------------------------*/
			flush_IF <= 0;
			stall_IF <= 0;
			// ID
			/*------------------------------------------------------------*/
			IR <= 0;
			PC_ID <= 0;
			PVSWriteEn_ID <= 0;
			flush_ID <= 1;
			stall_ID <= 0;
			// EX
			/*------------------------------------------------------------*/
			PC_EX <= 0;
			regA_EX <= 0;
			regB_EX <= 0;
			imm_extended_EX <= 0;
			RA1_EX <= 0;
			RA2_EX <= 0;
			WA1_EX <= 0;
			// Control signal registers
			isStore_EX <= 0;
			isLoad_EX <= 0;
			isJAL_EX <= 0; // for PC = 0 at the first inst.
			isJALR_EX <= 0;
			isBranch_EX <= 0;
			PVSWriteEn_EX <= 0;
			flush_EX_JB <= 0;
			flush_EX_LW <= 0;
			HALT_EX <= 0;
			// MEM
			/*------------------------------------------------------------*/
			ALUOut_MEM <= 0;
			regB_MEM <= 0;
			WA1_MEM <= 0;
			// Control signal registers
			isStore_MEM <= 0;
			isLoad_MEM <= 0;	
			isBranch_MEM <= 0;
			isBranchTaken_MEM <= 0;
			PVSWriteEn_MEM <= 0;
			HALT_MEM <= 0;
			// WB
			/*------------------------------------------------------------*/
			MDR_WB <= 0;
			ALUOut_WB <= 0;
			WA1_WB <= 0;
			HALT_WB <= 0;
			// Control signal registers
			isStore_WB <= 0;
			isBranch_WB <= 0;
			isBranchTaken_WB <= 0;
			PVSWriteEn_WB <= 0;
			// forwarding
			/*------------------------------------------------------------*/
			A_forward <= 0;
			B_forward <= 0;
		end
		else begin			
			CSN <= 0;
			// ID
			/*------------------------------------------------------------*/
			if ( ~stall_ID & ~flush_ID ) begin
				IR <= I_MEM_DI;
				PC_ID <= PC;
				PVSWriteEn_ID <= 1;
			end
			if ( flush_ID ) begin
				IR <= 0;
				PC_ID <= 0;
				PVSWriteEn_ID <= 0;
			end
			// EX
			/*------------------------------------------------------------*/
			if ( ((~flush_EX_JB) & (~flush_EX_LW)) ) begin
				PC_EX <= PC_ID;
				regA_EX <= ((IR[19:15] != 0) && (IR[19:15] == WA1_WB) && (~isStore_WB & ~isBranch_WB)) ? MUX2_result_wire:RF_RD1;
				regB_EX <= ((IR[24:20] != 0) && (IR[24:20] == WA1_WB) && (~isStore_WB & ~isBranch_WB)) ? MUX2_result_wire:RF_RD2;
				imm_extended_EX <= imm_extended;
				RA1_EX <= IR[19:15];
				RA2_EX <= IR[24:20];
				WA1_EX <= IR[11:7];
				// Control signal registers
				isStore_EX <= isStore;
				isLoad_EX <= isLoad;
				isJAL_EX <= isJAL; // for PC = 0 at the first inst.
				isJALR_EX <= isJALR;
				isBranch_EX <= isBranch;
				PVSWriteEn_EX <= PVSWriteEn_ID;
				ALUSrc1_EX <= ALUSrc1;
				ALUSrc2_EX <= ALUSrc2;
				JBSrc_EX <= JBSrc;
				ALUOp_EX <= ALUOp;
				HALT_EX <= HALT_reg2;
			end
			else begin
				PC_EX <= 0;
				regA_EX <= 0;
				regB_EX <= 0;
				imm_extended_EX <= 0;
				RA1_EX <= 0;
				RA2_EX <= 0;
				WA1_EX <= 0;
				// Control signal registers
				isStore_EX <= 0;
				isLoad_EX <= 0;
				isJAL_EX <= 0; // for PC = 0 at the first inst.
				isJALR_EX <= 0;
				isBranch_EX <= 0;
				PVSWriteEn_EX <= 0;
				ALUSrc1_EX <= 0;
				ALUSrc2_EX <= 0;
				JBSrc_EX <= 0;
				ALUOp_EX <= 0;
				HALT_EX <= HALT_reg2;
			end
			// MEM
			/*------------------------------------------------------------*/
			ALUOut_MEM <= ALUresult;
			regB_MEM <= ( ~isBforward ) ? regB_EX : B_forward_wire;
			WA1_MEM <= WA1_EX;
			// Control signal registers
			isStore_MEM <= isStore_EX;
			isLoad_MEM <= isLoad_EX;	
			isBranch_MEM <= isBranch_EX;
			isBranchTaken_MEM <= Bcond;
			PVSWriteEn_MEM <= PVSWriteEn_EX;
			HALT_MEM <= HALT_EX;
			// WB
			/*------------------------------------------------------------*/
			MDR_WB <= D_MEM_DI;
			ALUOut_WB <= ALUOut_MEM;
			WA1_WB <= WA1_MEM;
			// Control signal registers
			isStore_WB <= isStore_MEM;
			isLoad_WB <= isLoad_MEM;
			isBranch_WB <= isBranch_MEM;
			isBranchTaken_WB <= isBranchTaken_MEM;
			PVSWriteEn_WB <= PVSWriteEn_MEM;
			HALT_WB <= HALT_MEM;
		end
	end
	
	// *Combinational Logic Controller*
	always @ (*) begin
		if (~RSTn) begin
			imm_extended = 0;
			ALUOp = 5'b0000;
			isStore = 0;
			isLoad = 0;
			isJAL = 0;
			isStore = 0;
			isLoad = 0;
			isJAL = 0;
			isJALR = 0;
			isBranch = 0;
			ALUSrc1 = 2'b01;
			ALUSrc2 = 2'b00;
			JBSrc = 2'b00;
		end
		else begin
			// Control:
			// 1. imm_extend
			// 2. ALUOp
			// 3. control register
			case(IR[6:0])
				7'b0000000: begin // flush
					imm_extended = 0;
					ALUOp = 5'b0000;
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranch = 0;
					ALUSrc1 = 2'b01;
					ALUSrc2 = 2'b00;
					JBSrc = 2'b00;
				end
      			7'b1101111: begin //JAL
					con_12 = {12{IR[31]}};
					imm_extended = {con_12, IR[19:12], IR[20], IR[30:21], 1'b0}; // rd 로 이동
					ALUOp = 5'b00000; // default
					isStore = 0;
					isLoad = 0;
					isJAL = 1;
					isJALR = 0;
					isBranch = 0;
					ALUSrc1 = 2'b00;
					ALUSrc2 = 2'b01;
					JBSrc = 2'b00;
				end
      			7'b1100111: begin //JALR
					con_20 = {20{IR[31]}};
					imm_extended = {con_20, IR[30:21]}; // rd 로 이동
					ALUOp = 5'b00000; // default
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 1;
					isBranch = 0;
					ALUSrc1 = 2'b00;
					ALUSrc2 = 2'b01;
					JBSrc = 2'b01;
				end
      			7'b1100011: begin //BEQ~BGEU
				  	con_20 = {20{IR[31]}};
					imm_extended = {con_20, IR[7], IR[30:25], IR[11:8], 1'b0};
					case(IR[14:12])
						3'b000: begin //BEQ							
							ALUOp = 5'b01100;
						end
						3'b001: begin //BNE
							ALUOp = 5'b01101;
						end
						3'b100: begin //BLT
							ALUOp = 5'b00011;
						end
						3'b101: begin //BGE
							ALUOp = 5'b01110;
						end
						3'b110: begin //BLTU
							ALUOp = 5'b00100;
						end
						3'b111: begin //BGEU
							ALUOp = 5'b01111;
						end
					endcase
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranch = 1;
					ALUSrc1 = 2'b01;
					ALUSrc2 = 2'b00;
					JBSrc = 2'b00;
				end
      			7'b0000011: begin //LW
				  	con_20 = {20{IR[31]}};
					imm_extended = {con_20, IR[31:20]};
					ALUOp = 5'b00000;
					isStore = 0;
					isLoad = 1;
					isJAL = 0;
					isJALR = 0;
					isBranch = 0;
					ALUSrc1 = 2'b01;
					ALUSrc2 = 2'b10;
					JBSrc = 2'b00;
				end
      			7'b0100011: begin //SW
				    con_20 = {20{IR[31]}};
				  	imm_extended = {con_20, IR[31:25], IR[11:7]};
					ALUOp = 5'b00000;					
					isStore = 1;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranch = 0;
					ALUSrc1 = 2'b01;
					ALUSrc2 = 2'b10;
					JBSrc = 2'b00;
				end
      			7'b0010011: begin //ADDI~SRAI, I-type computational
				    con_20 = {20{IR[31]}};
				  	imm_extended = {con_20, IR[31:20]};
					case(IR[14:12])
						3'b000: begin //ADDI
							ALUOp = 5'b00000;
						end
						3'b010: begin //SLTI
							ALUOp = 5'b00011;
						end
						3'b011: begin //SLTIU
							ALUOp = 5'b00100;
						end
						3'b100: begin //XORI
							ALUOp = 5'b00101;
						end
						3'b110: begin //ORI
							ALUOp = 5'b01000;					
						end 
						3'b111: begin //ANDI
							ALUOp = 5'b01001;
						end 
						3'b001: begin //SLLI
							ALUOp = 5'b00010;
						end
						3'b101: begin //SRLI, SRAI
							if (IR[31:25] == 7'b0000000) //SRLI
								ALUOp = 5'b00110;
							else if (IR[31:25] == 7'b0100000) //SRAI
								ALUOp = 5'b00111;
						end
					endcase
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranch = 0;
					ALUSrc1 = 2'b01;
					ALUSrc2 = 2'b10;
					JBSrc = 2'b00;
				end
				7'b0110011: begin //ADD~AND
					case(IR[14:12])
						3'b000: begin //ADD, SUB
							if (IR[31:25] == 7'b0000000) //ADD
								ALUOp = 5'b00000;
							if (IR[31:25] == 7'b0100000) //SUB
								ALUOp = 5'b00001;
						end
						3'b001: ALUOp = 5'b00010; //SLL
						3'b010: ALUOp = 5'b00011; //SLT
						3'b011: ALUOp = 5'b00100; //SLTU
						3'b100: ALUOp = 5'b00101; //XOR
						3'b101: begin //SRL, SRA
							if (IR[31:25] == 7'b0000000) //SRL
								ALUOp = 5'b00110;
							if (IR[31:25] == 7'b0100000) //SRA
								ALUOp = 5'b00111;
						end
						3'b110: ALUOp = 5'b01000; //OR
						3'b111: ALUOp = 5'b01001; //AND
					endcase
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranch = 0;
					ALUSrc1 = 2'b01;
					ALUSrc2 = 2'b00;
					JBSrc = 2'b00;
				end
				7'b0001011: begin //MULT~IS_EVEN
					case(IR[14:12])
						3'b111: begin //MULT, MODULO
							if (IR[31:25] == 7'b0000000) begin // MULT
								ALUOp = 5'b01010; // rd = rs1*rs2
							end
								
							else if (IR[31:25] == 7'b0000001) begin // MODULO
								ALUOp = 5'b01011; // rd = rs1&rs2
							end
							isStore = 0;
							isLoad = 0;
							isJAL = 0;
							isJALR = 0;
							isBranch = 0;
							ALUSrc1 = 2'b01;
							ALUSrc2 = 2'b00;
							JBSrc = 2'b00;
						end
						3'b110: begin //IS_EVEN
							imm_extended = 2;
							ALUOp = 5'b10000;
							isStore = 0;
							isLoad = 0;
							isJAL = 0;
							isJALR = 0;
							isBranch = 0;
							ALUSrc1 = 2'b01;
							ALUSrc2 = 2'b10;
							JBSrc = 2'b00;
						end
					endcase
				end
  			endcase
			// HALT control
			if (IR == 32'h00c00093) begin
				HALT_reg1 = 1;
			end
			else if ((HALT_reg1 == 1) && (IR == 32'h00008067)) begin
				HALT_reg2 = 1;
			end
			else begin
				HALT_reg1 = 0;
				HALT_reg2 = 0;
			end
		end
	end

	// Hazard Control Unit
	always @ (*) begin
		
		// (Un)conditional jump miss
		if ( (~isJAL_EX) & (~isJALR_EX) & ~(isBranch_EX & Bcond) ) begin
			flush_ID = 0;
			flush_EX_JB = 0;		
		end
		else begin
			flush_ID = 1;
			flush_EX_JB = 1;
		end
		
	end

	// Forwarding Unit in EX stage
	always @ (*) begin

		// LW 1 cycle Stall
		if ( ~isLoad_EX ) begin
				stall_IF = 0; // stall IF
				stall_ID = 0; // stall ID
				flush_EX_LW = 0; // flush EX
		end
		else begin
			if ( (IR[19:15] != 0) && (IR[19:15] == WA1_EX) && (~isStore_EX & ~isBranch_EX) ) begin
				stall_IF = 1; // stall IF
				stall_ID = 1; // stall ID
				flush_EX_LW = 1; // flush EX
			end
			else if ( (IR[24:20] != 0) && (IR[24:20] == WA1_EX) && (~isStore_EX & ~isBranch_EX) ) begin
				stall_IF = 1; // stall IF
				stall_ID = 1; // stall ID
				flush_EX_LW = 1; // flush EX
			end
			else begin
				stall_IF = 0; // stall IF
				stall_ID = 0; // stall ID
				flush_EX_LW = 0; // flush EX
			end
		end
		
		// register 1 dataforward
		if ( (RA1_EX != 0) && (RA1_EX == WA1_MEM) && (~isStore_MEM & ~isBranch_MEM) ) begin
			isAforward = 1;
			A_forward = ALUOut_MEM;
		end
		else if ( (RA1_EX != 0) && (RA1_EX == WA1_WB) && (~isStore_WB & ~isBranch_WB) ) begin
			isAforward = 1;
			A_forward = MUX2_result_wire;
		end
		else begin
			isAforward = 0;
			A_forward = 0;
		end

		// register 2 dataforward
		if ( (RA2_EX != 0) && (RA2_EX == WA1_MEM) && (~isStore_MEM & ~isBranch_MEM) ) begin
			isBforward = 1;
			B_forward = ALUOut_MEM;
		end
		else if ( (RA2_EX != 0) && (RA2_EX == WA1_WB) && (~isStore_WB & ~isBranch_WB) ) begin
			isBforward = 1;
			B_forward = MUX2_result_wire;
		end
		else begin
			isBforward = 0;
			B_forward = 0;
		end
	end
	
endmodule //
