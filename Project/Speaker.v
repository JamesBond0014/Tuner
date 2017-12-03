module playmusic(clk, audio_out,reset,select);
	input clk;
	input reset;
	input [2:0] select;
	output [23:0] audio_out;
	
	
	
	wire [10:0]frequency;
	lut l1(select, frequency);
	
	wire[15:0] clkdivider = 48000/frequency;
	
	reg [23:0] tone;
	always @(posedge clk) tone <= tone+1;
	reg [16:0] count;
	reg [14:0] counter;
	
	always @(posedge clk) 
	begin
		if(counter==0) 
			//counter <= (tone[23]?clkdivider-1 : clkdivider/2-1); 
			counter <= clkdivider;
		else 
			counter <= counter-1;
		
		if (reset)
			begin
			count <= 17'b00000000000000000;
			counter <= 15'd0;
			end
		if (speaker) audio_out <= 24'b000000001011101110000000;
		else audio_out <= 24'b111111110100010010000000;
	end
	
	reg speaker;
	
	
	always @(posedge clk) 
	begin
		if(counter==0) 
			begin
			count <= count +1;
			speaker <= ~speaker;
			end
	end
	//wire [6:0] ramp = (tone[22] ? tone[21:15] : ~tone[21:15]);
	// That means
	// "if tone[22]=1 then ramp=tone[21:15] else
	//ramp=~tone[21:15]"
	
	
endmodule

module lut(in, out);
	input [2:0]in;
	output reg [10:0] out;

	always @(*)
	begin
		case(in)
			3'd0: out = {11'b00101001010}; //330 E
			3'd1: out = {11'b00110111000}; //440 A
			3'd2: out = {11'b01001001011}; //587 D
			3'd3: out = {11'b01100010000}; //784 G
			3'd4: out = {11'b01111011100}; //985 B
			3'd5: out = {11'b10100100111}; //1319 E
			default: out = {11'b00000000000};
		endcase
	end

endmodule


