module lut(in, out);
	input [2:0]in;
	output reg [15:0] out;
	always @(*)
	begin
		case(in)
			3'd0: out = {5'b00000,11'b00101001010}; //330 E
			3'd1: out = {5'b00000,11'b00110111000}; //440 A
			3'd2: out = {5'b00000,11'b01001001011}; //587 D
			3'd3: out = {5'b00000,11'b01100010000}; //784 G
			3'd4: out = {5'b00000,11'b01111011100}; //985 B
			3'd5: out = {5'b00000,11'b10100100111}; //1319 E
			default: out = {5'b00000,11'b00101001010}; //default low E
		endcase
	end

endmodule
