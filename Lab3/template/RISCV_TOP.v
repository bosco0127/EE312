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
	// Control signal registers
	reg isItype;
	reg isStore;
	reg isLoad;
	reg isJAL;
	reg isJALR;
	reg isBranchTaken;
	reg isAUIPC;
	// Control signal wires
	// RegDest
	// RegDest = Maybe not necessary
	// ALUSrc2 = isItype
	wire ALUSrc2 = isItype;
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
	// PC + 4 wire
	wire [31:0] PC_4;
	// JAL or Branch instruction target wire
	wire [31:0] JBtarget;
	// MUX3 output
	wire [31:0] JAL_B_wire;
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
	MUX MUX1 (
		.I0 (RF_RD2),
		.I1 (imm_extended_wire),
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

	ADDALU ADDALU1 (
		.A (PC_wire),
		.B (32'h00000004),
		.C (PC_4)
	);

	ADDALU ADDALU2 (
		.A (PC_wire),
		.B (imm_extended_wire),
		.C (JBtarget)
	);

	// MUX for Branch or Jump
	MUX MUX3 (
		.I0 (PC_4),
		.I1 (JBtarget),
		.E (PCSrc),
		.F (JAL_B_wire)
	);

	// MUX for AUIPC
	MUX MUX4 (
		.I0 (RF_RD1),
		.I1 (PC_wire),
		.E (isAUIPC),
		.F (ALUinput1)
	);

	// MUX for determine RF_WD
	MUX MUX5 (
		.I0 (WriteData_extend_wire),
		.I1 (PC_4),
		.E (isJAL),
		.F (RF_WD)
	);

	// MUX for determine JAL_B vs JALR
	MUX MUX6 (
		.I0 (JAL_B_wire),
		.I1 ({WriteData_wire[31:1],1'b0}),
		.E (isJALR),
		.F (I_MEM_target)
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
			PC <= I_MEM_target; 
		end
		
	end
	
	// **Combinational Logic Controller**
	always @ (*) begin
		if (~RSTn) begin
			CSN = 1;
			imm_extended = 0;
			ALUOp = 5'b00000;
			isItype = 0;
			isStore = 0;
			isLoad = 0;
			isJAL = 0;
			isJALR = 0;
			isBranchTaken = 1;
			isAUIPC = 0;
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
			case(I_MEM_DI[6:0])
     	 		7'b0110111: begin //LUI
					con_12 = {12{1'b0}};
					imm_extended = {I_MEM_DI[31:12], con_12};
					ALUOp = 5'b01100;
					isItype = 1;
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranchTaken = 0;
					isAUIPC = 0; 
				end 
      			7'b0010111: begin //AUIPC
					con_12 = {12{1'b0}};
					imm_extended = {I_MEM_DI[31:12], con_12};
					ALUOp = 5'b00000;
					isItype = 1;
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranchTaken = 0;
					isAUIPC = 1;
				end
      			7'b1101111: begin //JAL
					con_12 = {12{I_MEM_DI[31]}};
					imm_extended = {con_12, I_MEM_DI[19:12], I_MEM_DI[20], I_MEM_DI[30:21], 1'b0}; // rd 로 이동
					ALUOp = 5'b00000; // default
					isItype = 0;
					isStore = 0;
					isLoad = 0;
					isJAL = 1;
					isJALR = 0;
					isBranchTaken = 0;
					isAUIPC = 0;
				end
      			7'b1100111: begin //JALR
					con_20 = {20{I_MEM_DI[31]}};
					imm_extended = {con_20, I_MEM_DI[30:21]}; // rd 로 이동
					ALUOp = 5'b00000; // default
					BE = 4'b0000;
					isItype = 1;
					isStore = 0;
					isLoad = 0;
					isJAL = 1;
					isJALR = 1;
					isBranchTaken = 0;
					isAUIPC = 0;
				end
      			7'b1100011: begin //BEQ~BGEU
				  	con_20 = {20{I_MEM_DI[31]}};
					imm_extended = {con_20, I_MEM_DI[7], I_MEM_DI[30:25], I_MEM_DI[11:8], 1'b0};
					case(I_MEM_DI[14:12])
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
					isItype = 0;
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranchTaken = Bcond;
					isAUIPC = 0;
				end
      			7'b0000011: begin //LB~LHU, I-type Load
				  	con_20 = {20{I_MEM_DI[31]}};
					imm_extended = {con_20, I_MEM_DI[31:20]};
					case(I_MEM_DI[14:12])
						3'b000: begin //LB
							ALUOp = 5'b00000;
							BE = 4'b0001;
							extend_reg = {24{WriteData_wire[7]}};
						end
						3'b001: begin //LH
							ALUOp = 5'b00000;
							BE = 4'b0011;
							extend_reg = {24{WriteData_wire[15]}};
						end
						3'b010: begin //LW
							ALUOp = 5'b00000;							
							BE = 4'b1111;
						end
						3'b100: begin //LBU
							ALUOp = 5'b00000;							
							BE = 4'b0001;
						end
						3'b101: begin //LHU
							ALUOp = 5'b00000;							
							BE = 4'b0011;
						end
					endcase
					isItype = 1;
					isStore = 0;
					isLoad = 1;
					isJAL = 0;
					isJALR = 0;
					isBranchTaken = 0;
					isAUIPC = 0;
				end
      			7'b0100011: begin //SB~SW, I-type Load
				    con_20 = {20{I_MEM_DI[31]}};
				  	imm_extended = {con_20, I_MEM_DI[31:25], I_MEM_DI[11:7]};
					case(I_MEM_DI[14:12])
						3'b000: begin //SB
							ALUOp = 5'b00000;
							BE = 4'b0001;
						end
						3'b001: begin //SH
							ALUOp = 5'b00000;
							BE = 4'b0011;
						end
						3'b010: begin //SW
							ALUOp = 5'b00000;
							BE = 4'b1111;						
						end
					endcase
					isItype = 1;
					isStore = 1;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranchTaken = 0;
					isAUIPC = 0;
				end
      			7'b0010011: begin //ADDI~SRAI, I-type computational
				    con_20 = {20{I_MEM_DI[31]}};
				  	imm_extended = {con_20, I_MEM_DI[31:20]};
					case(I_MEM_DI[14:12])
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
							if (I_MEM_DI[31:25] == 7'b0000000) //SRLI
								ALUOp = 5'b00110;
							else if (I_MEM_DI[31:25] == 7'b0100000) //SRAI
								ALUOp = 5'b00111;
						end
					endcase
					isItype = 1;
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranchTaken = 0;
					isAUIPC = 0;
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
					isItype = 0;
					isStore = 0;
					isLoad = 0;
					isJAL = 0;
					isJALR = 0;
					isBranchTaken = 0;
					isAUIPC = 0;		
				end
				7'b0001011: begin //MULT~IS_EVEN
					case(I_MEM_DI[14:12])
						3'b111: begin //MULT, MODULO
							if (I_MEM_DI[31:25] == 7'b0000000) begin // MULT
								ALUOp = 5'b01010; // rd = rs1*rs2
							end
								
							else if (I_MEM_DI[31:25] == 7'b0000001) begin // MODULO
								ALUOp = 5'b01011; // rd = rs1&rs2
							end
							isItype = 0;
							isStore = 0;
							isLoad = 0;
							isJAL = 0;
							isJALR = 0;
							isBranchTaken = 0;
							isAUIPC = 0;	
						end
						3'b110: begin //IS_EVEN
							imm_extended = 2;
							ALUOp = 5'b10000;
							isItype = 1;
							isStore = 0;
							isLoad = 0;
							isJAL = 0;
							isJALR = 0;
							isBranchTaken = 0;
							isAUIPC = 0;
						end
					endcase
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
