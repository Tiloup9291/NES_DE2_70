module MicroCodeTableInner(input clk, input ce, input reset, input [7:0] IR, input [2:0] State, output reg [8:0] M);
 
 wire [10:0] addr = {IR, State};  // Combined address (8+3 = 11 bits)
 wire [8:0] data_out_M;
 
 microcode_rom rom_inst (
        .address(addr),
        .clock(clk),
        .q(data_out_M)
);
		  
always @(posedge clk)begin
	if (reset) begin
		M <= 0;
	end else if (ce) begin
		M <= data_out_M;
	end
end
endmodule
	
module MicroCodeTable(input clk, input ce, input reset, input [7:0] IR, input [2:0] State, output [37:0] Mout);
wire [8:0] M;
MicroCodeTableInner inner(clk, ce, reset, IR, State, M);
wire [14:0] data_out_A;

 A_Table_rom a_rom_inst (
        .address(M),
        .clock(clk),
        .q(data_out_A)
);

wire [14:0] R = data_out_A;
wire [14:0] data_out_B;

 B_Table_rom b_rom_inst (
        .address(IR),
        .clock(clk),
        .q(data_out_B)
);

reg [18:0] AluFlags;
always @(posedge clk)begin
	if (reset) begin
		AluFlags <= 0;
	end else if (ce) begin  
		AluFlags <= data_out_B;
	end
end
 
 assign Mout = {AluFlags,// 19
                 M[8:7],  // NextState // 2
                 R[14:13],// LoadT     // 2
                 R[12],   // FlagCtrl  // 1
                 R[11:7], // AddrCtrl  // 5
                 R[6:4],  // MemWrite  // 3
                 M[6:5],  // AddrBus   // 2
                 R[3:2],  // LoadPC    // 2
                 R[1:0]   // LoadSP    // 2
                 };
					  
endmodule 
