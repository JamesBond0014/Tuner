module playmusic(clk,clk_aud, audio_out,reset,select,enable);
	input clk;
	input reset;
	input clk_aud;
	input [2:0] select;
	output [23:0] audio_out;
	input enable;


	wire [15:0]frequency;
	lut l1(select, frequency);

	wire[15:0] clkdivider = 48000/frequency/2;

	reg [23:0] tone;
	reg [23:0] out_sound;
	always @(posedge clk) begin
		if (clk_aud) begin
			tone <= tone+1;
			end
	end
	reg [16:0] count;
	reg [14:0] counter;

	always @(posedge clk)
	
	begin
	if (clk_aud) begin
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
		if (enable)
		begin
			if (speaker) out_sound <= 24'b000000001011101110000000;
			else out_sound <= 24'b111111110100010010000000;
		end
		else out_sound <= 24'd0;
		end
	end

	reg speaker;
	assign audio_out =out_sound;

	always @(posedge clk)
	begin
		if (clk_aud) begin

			if(counter==0)
				begin
				count <= count +1;
				speaker <= ~speaker;
				end
		end
	end


endmodule
