module main (CLOCK_50, CLOCK2_50, KEY, FPGA_I2C_SCLK, FPGA_I2C_SDAT, AUD_XCK,
		        AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK, AUD_ADCDAT, AUD_DACDAT,VGA_CLK,//VGA Clock
				  VGA_HS,							//	VGA H_SYNC
				  VGA_VS,							//	VGA V_SYNC
				  VGA_BLANK_N,						//	VGA BLANK
				  VGA_SYNC_N,						//	VGA SYNC
				  VGA_R,   						//	VGA Red[9:0]
				  VGA_G,	 						//	VGA Green[9:0]
				  VGA_B,  						//	VGA Blue[9:0]
				  SW,
				  HEX5,
				  HEX4,
				  HEX3,
				  HEX2,
				  HEX1,
				  HEX0,
				  LEDR
	);

  input CLOCK_50, CLOCK2_50;
	input [2:0] KEY;
	input [9:0] SW;
	output reg [9:0] LEDR;
	// I2C Audio/Video config interface
	output FPGA_I2C_SCLK;
	inout FPGA_I2C_SDAT;
	// Audio CODEC

	output AUD_XCK;
	input AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK;
	input AUD_ADCDAT;
	output AUD_DACDAT;

	// Local wires.
	wire read_ready, write_ready, read, write;
	wire [23:0] readdata_left, readdata_right;
	wire [23:0] writedata_left, writedata_right;
	wire reset = ~KEY[0];

	wire [15:0] errorRange;
	//Display things
	output [6:0] HEX0, HEX1,HEX2,HEX3,HEX4,HEX5;
	vga_adapter VGA(
		.resetn(KEY[0]),
		.clock(CLOCK_50),
		.colour(colour),
		.x(X),
		.y(Y),
		.plot(writeEn),
		/* Signals for the DAC to drive the monitor. */
		.VGA_R(VGA_R),
		.VGA_G(VGA_G),
		.VGA_B(VGA_B),
		.VGA_HS(VGA_HS),
		.VGA_VS(VGA_VS),
		.VGA_BLANK(VGA_BLANK_N),
		.VGA_SYNC(VGA_SYNC_N),
		.VGA_CLK(VGA_CLK));

		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";

		wire note;
		assign note = SW[9:7];
		reg r,w;
		assign read = r;
		assign write = w;
		always@(clock_50) begin

			if (SW[0]) begin
				r <= read_ready;
				w <= 1'b0;
				end
			if (~SW[0]) begin
				r <= 0'b0;
				w <= write_ready;
				end

			if (!KEY[1] && ![SW[0]])
				soundwave <= audio_out;
			if (SW[0])
				soundwave <= readdata_right;

			if (frequency < expectedfrequency)
				greater <= 0;
			else
				greater <= 1;
		end
		wire [23:0] audio_out;
		playmusic playtone (AUD_XCK,audio_out,reset,note,~KEY[1]);
		assign writedata_left = audio_out;
		assign writedata_right = adio_out;
		wire enable;
		reg [23:0]soundwave;
		assign enable = (~KEY[1] || SW[0]);
		draw_graph draw(CLOCK_50,AUD_XCK,enable,reset,soundwave, writeEn,X,Y);
		wire [15:0] frequency;
		frequency_calculator freq(CLOCK_50, AUD_XCK, read, soundwave, reset, frequency);
		wire [15:0] expectedfrequency;
		lut lookup(note, expectedfrequency);

		assign errorRange = 16'b0000000000110010;
		wire match;
		range_comparator16(frequency,expectedfrequency,errorRange,match);
		wire greater;
		note_decoder hexex({1'b0,notes},HEX5);
		note_decoder hexresult({match, 2'b11,greater},HEX4);

	clock_generator my_clock_gen(
		// inputs
		CLOCK2_50,
		reset,

		// outputs
		AUD_XCK
	);

	audio_and_video_config cfg(
		// Inputs
		CLOCK_50,
		reset,

		// Bidirectionals
		FPGA_I2C_SDAT,
		FPGA_I2C_SCLK
	);

	audio_codec codec(
		// Inputs
		CLOCK_50,
		reset,

		read,	write,
		writedata_left, writedata_right,

		AUD_ADCDAT,

		// Bidirectionals
		AUD_BCLK,
		AUD_ADCLRCK,
		AUD_DACLRCK,

		// Outputs
		read_ready, write_ready,
		readdata_left, readdata_right,
		AUD_DACDAT
	);

endmodule



module draw_graph(clk, enable, reset, soundwave, enable_draw, X_Vga, Y_Vga);
	input clk, enable, reset;
	input [23:0] soundwave;
	output enable_draw;
	output [7:0] X_Vga;
	output [6:0] Y_Vga;

	// Rate divider: every 4 clock cycles
	wire [15:0] load = 16'd1000;
	wire [15:0] count;
	countdown ct(enable, load, reset, clk, count);

	// Enable draw
	wire enable_draw;
	assign enable_draw = (count == 0) ? 1 : 0;

	// Handle reset and enable
	reg [7:0] x;
	always @(posedge clk) begin
		if (reset)
			x <= 8'd0;
		else if (enable_draw) begin
			if (x < 8'b11111110)
				x <= x + 8'd1;
		end
	end

	// Set parameters to draw
	wire [2:0] colour;
	wire [7:0] x_in;
	assign x_in = x;
	simple_draw d1(soundwave, x_in, X_Vga, Y_Vga, colour);

endmodule


// Draw pixel
module simple_draw(soundwave,x,X_Vga,Y_Vga,colour);
		input [23:0] soundwave;
		input [7:0] x;
		output[7:0] X_Vga;
		output reg [6:0] Y_Vga;
		output[2:0] colour;

		assign colour[2:0] = 3'b111;

		wire [6:0] y_center = 7'b0111111;

		always @(*) begin
			if (soundwave[23]) // negative
				Y_Vga = y_center + 7'd10;
			else if (!soundwave[23])
				Y_Vga = y_center - 7'd10;
		end

		assign X_Vga = x;
endmodule


module countdown(enable, load, reset, clock, count_q);
	// reset is posedge
	input enable, reset, clock;
	input [15:0] load;
	output reg [15:0] count_q;

	always @(posedge clock) begin
		if (reset) // reset counter
			count_q <= load;
		else if (enable) // counter when enable is on
			begin
			if (count_q == 16'd0) // counter reached 0
				count_q <= load;
			else // decrease counter
				count_q <= count_q - 16'd1;
			end
	end

endmodule


module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;

    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;
            default: segments = 7'h7f;
        endcase
endmodule
