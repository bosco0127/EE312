`timescale 1ns/10ps
module Cache (
	input	wire			CLK,
	input	wire			CSN,//clk synchronous negative
	input   wire            OPERATE,
	input	wire	[11:0]	ADDR,
	input	wire			WEN,//write enable negative
	input	wire	[31:0]	DI, //data in
	output  wire    [11:0]  D_ADDR,
	output	wire	[31:0]	DOUT, // data out
	output	wire			READY, /* for pipeline stall when it's 0 */
	output	wire            ISREAD, // for cache DI MUX
	output	wire            ISWRITE, // for DRAM WEN
	// counting
	output wire [31:0] READ_HIT,
	output wire [31:0] READ_MISS,
	output wire [31:0] WRITE_HIT,
	output wire [31:0] WRITE_MISS
);

	reg		[31:0]		outline;
	reg		[135:0]		cache[0 : 7]; // tag[135:129], valid[128], word3[127:96], word2[95:64], word1[63:32], word0[31:0].
	reg		[31:0]		temp;
	reg 	[6:0]		tag;
	reg		[2:0]		idx;
	reg		[1:0]		bo;
	reg hit;

	// register for cache FSM
	reg cache_state;
	reg ready;

	// register for mem_access_latency.
	reg [2:0] mem_state; // for counting 8 cycles.
	reg [11:0] mem_ADDR;
	reg mem_access_read;
	reg mem_access_write;
	reg read_done;

	// Assignments
	assign D_ADDR = mem_ADDR;
	assign DOUT = outline;
	assign READY = ready;
	assign ISREAD = mem_access_read;
	assign ISWRITE = mem_access_write;
	
	reg [31:0] read_total;
	reg [31:0] read_miss;
	reg [31:0] write_total;
	reg [31:0] write_miss;

	// count assign
	assign READ_HIT = read_total - read_miss;
	assign READ_MISS = read_miss;
	assign WRITE_HIT = write_total - write_miss;
	assign WRITE_MISS = write_miss;

	initial begin
		read_total = 0;
		read_miss = 0;
		write_total = 0;
		write_miss = 0;
	end

	always @ (*) begin
		if (~CSN)
		begin
			tag = ADDR[11:5];
			idx = ADDR[4:2];
			bo = ADDR[1:0];
			hit = ((cache[idx][135:129] == tag) & cache[idx][128]);
			if (~OPERATE) begin
				cache_state = 0;
			end
		end
		else begin
			cache[0] = 0;
			cache[1] = 0;
			cache[2] = 0;
			cache[3] = 0;
			cache[4] = 0;
			cache[5] = 0;
			cache[6] = 0;
			cache[7] = 0;
			ready = 1; // 평상시 ready는 1, 작동시 0.
			cache_state = 0;
			mem_state = 0;
			mem_access_read = 0;
			mem_access_write = 0;
			read_done = 0;
		end
	end

	// Asynchronous ready controller
	always @ (negedge CLK) begin
		if (~CSN) begin
			// read
			if (WEN & OPERATE) begin
				if (hit) begin
					ready <= 1;
					// counting hit & miss number
					read_total <= read_total + 1;
				end
				else begin
					ready <= 0;
				end
			end
			// write
			else if (~WEN & OPERATE) begin
				if (mem_access_write & (mem_state==3'b111)) begin
					ready <= 1;
				end
				else begin
					ready <= 0;
				end
			end
		end
	end

	// Asynchronous read
	always @ (*) begin
		// Asynchronous read
		if (~CSN)
		begin
			if (WEN & OPERATE) begin
				if (hit) begin
					case (bo)
						2'b00: outline = cache[idx][31:0];
						2'b01: outline = cache[idx][63:32];
						2'b10: outline = cache[idx][95:64];
						2'b11: outline = cache[idx][127:96];
					endcase
					read_done = 1;
				end
			end
		end
	end

	// read
	always @ (posedge CLK) begin
		if (~CSN)
		begin
			if (WEN & OPERATE)
			begin
				if (~hit) begin
					case (cache_state)
						1'b0: begin
							mem_ADDR <= {ADDR[11:2],2'b00}; // set memory address to access
							cache[idx][128] <= 0; // set valid-bit to 0
							cache[idx][135:129] <= tag; // set tag bit
							mem_state <= 0;
							mem_access_read <= 1;
							read_done <= 0;
							cache_state <= cache_state + 1;						
						end
						// cache-update state
						1'b1: begin
							if(read_done) begin
								read_done <= 0;
								cache_state <= 0;
							end
						end
						default: cache_state <= 0;
					endcase
				end
			end			
		end
	end

	// read from mem in 8 cycles
	always @ (posedge CLK) begin
		if (~CSN) begin
			if (mem_access_read) begin
				case (mem_state)
					3'b000: begin
						cache[idx][31:0] <= DI; // read from mem
						mem_state <= mem_state + 1;				
					end
					3'b001: begin
						mem_ADDR <= mem_ADDR + 1;
						mem_state <= mem_state + 1;
					end					
					3'b010: begin
						cache[idx][63:32] <= DI; // read from mem
						mem_state <= mem_state + 1;
					end
					3'b011: begin
						mem_ADDR <= mem_ADDR + 1;
						mem_state <= mem_state + 1;
					end						
					3'b100: begin
						cache[idx][95:64] <= DI; // read from mem
						mem_state <= mem_state + 1;
					end					
					3'b101: begin
						mem_ADDR <= mem_ADDR + 1;
						mem_state <= mem_state + 1;
					end
					3'b110: begin
						cache[idx][127:96] <= DI; // read from mem
						mem_state <= mem_state + 1;
					end
					3'b111: begin
						// set valid bit to 1 for next state(cache update state)
						if (WEN & OPERATE) begin
							cache[idx][128] <= 1;
							// counting hit & miss number
							read_miss <= read_miss + 1;
						end
						mem_state <= 0;
						mem_access_read <= 0;
						read_done <= 1;
					end
				endcase
			end
		end
	end
	
	// write
	always @ (posedge CLK) begin
		// Synchronous write
		if (~CSN)
		begin			
			if (~WEN & OPERATE)
			begin
				// write-through policy
				if (hit /**/& (cache_state==0)/**/) begin
					// write to cache
					case (bo)
						2'b00: cache[idx][31:0] <= DI;
						2'b01: cache[idx][63:32] <= DI;
						2'b10: cache[idx][95:64] <= DI;
						2'b11: cache[idx][127:96] <= DI;
					endcase
					if (mem_access_write == 0) begin
						mem_state <= 0;
						mem_access_write <= 1; // write-through
					end
				end
				// write-allocate policy				
				else begin
					case (cache_state)
						1'b0: begin
							mem_ADDR <= {ADDR[11:2],2'b00}; // set memory address to access
							cache[idx][128] <= 0; // set valid-bit to 0
							cache[idx][135:129] <= tag;
							mem_state <= 0;
							mem_access_read <= 1;
							read_done <= 0;
							cache_state <= 1;					
						end
						1'b1: begin
							if(read_done) begin
								// same procedure to hit case.
								// write to cache
								case (bo)
									2'b00: cache[idx][31:0] <= DI;
									2'b01: cache[idx][63:32] <= DI;
									2'b10: cache[idx][95:64] <= DI;
									2'b11: cache[idx][127:96] <= DI;
								endcase
								if (mem_access_write == 0) begin
									mem_state <= 0;
									mem_access_write <= 1; // write-through
								end
							end
						end
					endcase
				end
			end
		end
	end

	// write-through policy: write to mem in 8 cycles
	always @ (posedge CLK) begin
		if (~CSN) begin
			if (mem_access_write) begin
				case (mem_state)
					3'b000: begin						
						outline <= cache[idx][31:0]; // write into the DRAM
						mem_ADDR <= {ADDR[11:2],2'b00}; // set memory address to access
						mem_state <= mem_state + 1;				
					end
					3'b001: begin
						mem_ADDR <= mem_ADDR + 1;
						mem_state <= mem_state + 1;
					end					
					3'b010: begin
						outline <= cache[idx][63:32]; // write into the DRAM
						mem_state <= mem_state + 1;
					end
					3'b011: begin
						mem_ADDR <= mem_ADDR + 1;
						mem_state <= mem_state + 1;
					end						
					3'b100: begin
						outline <= cache[idx][95:64]; // write into the DRAM
						mem_state <= mem_state + 1;
					end					
					3'b101: begin
						mem_ADDR <= mem_ADDR + 1;
						mem_state <= mem_state + 1;
					end
					3'b110: begin
						outline <= cache[idx][127:96]; // write into the DRAM
						mem_state <= mem_state + 1;
					end
					3'b111: begin
						mem_state <= 0;
						mem_access_write <= 0;
						cache[idx][128] <= 1; // set valid to 1
						// counting hit & miss number
						write_total <= write_total + 1;
						if (~hit & cache_state & read_done) begin
							write_miss <= write_miss + 1;
						end
						cache_state <= 0;
					end
				endcase
			end
		end
	end

endmodule
