module  IF
(
	input CLK,
	input RESET,

	output reg [31:0] Instr1_OUT,//This should contain the fetched instruction
	output reg [31:0] Instr_PC_OUT,//This should contain the address of the fetched instruction [DEBUG purposes]
	output reg [31:0] Instr_PC_Plus4,//This should contain the address of the instruction after the fetched instruction (used by ID)

	input STALL,//Will be set to true if we need to just freeze the fetch stage.

	input Request_Alt_PC, //There was probably a branch -- please load the alternate PC instead of Instr_PC_Plus4.
	input [31:0] Alt_PC,//Alternate PC to load

	output [31:0] Instr_address_2IM,//Address from which we want to fetch an instruction
	input [31:0]   Instr1_fIM//Instruction received from instruction memory
);

wire [31:0] IncrementAmount;

//Since we're a multicycle datapath, we need to account for the MIPS branch delay slot
//in an unconventional way
reg [31:0]	BranchAddress;
reg [7:0] 	BranchRing;
reg [4:0]   InstructionRing;

assign IncrementAmount = 32'd4; //NB: This might get modified for superscalar.
assign Instr_address_2IM = BranchRing[7]?BranchAddress:Instr_PC_Plus4;


always @(posedge CLK or negedge RESET) begin
	if(!RESET) begin
		Instr1_OUT <= 0;
		Instr_PC_OUT <= 0;
		Instr_PC_Plus4 <= 32'hBFC00000;
		InstructionRing <= 5'b00001;
		$display("FETCH [RESET] Fetching @%x", Instr_PC_Plus4);
	end else if(CLK) begin
		if(!STALL) begin
			Instr1_OUT <= InstructionRing[0]?Instr1_fIM:32'b0;
			Instr_PC_OUT <= InstructionRing[0]?Instr_address_2IM:32'b0;
			//This needs to be changed for a non-multicycle datapath
			BranchAddress <= (Request_Alt_PC)?Alt_PC:BranchAddress;
			BranchRing <= (Request_Alt_PC)?(8'b1):({BranchRing[6:0], {1'b0}});
			InstructionRing <= {{InstructionRing[3:0]}, {InstructionRing[4]}};
			Instr_PC_Plus4 <= InstructionRing[0]?(Instr_address_2IM + IncrementAmount):Instr_PC_Plus4;

			$display("FETCH:Instr@%x=%x;Next@%x",Instr_address_2IM,Instr1_fIM,Instr_address_2IM + IncrementAmount);
			$display("FETCH:ReqAlt[%d]=%x",Request_Alt_PC,Alt_PC);
			$display("FETCH:Ring %09b %08x", BranchRing, BranchAddress);
			$display("FETCH:InstrRing %04b", InstructionRing);
		end else begin
			$display("FETCH: Stalling; next request will be %x",Instr_address_2IM);
		end
	end
end

endmodule

