module Tuner (CLOCK_50, CLOCK2_50, KEY, FPGA_I2C_SCLK, FPGA_I2C_SDAT, AUD_XCK,
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
	
	wire X,Y,writeEn;
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

	//display stuff
	output			VGA_CLK;   				//	VGA Clock      
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]


	// Local wires.
	wire read_ready, write_ready, read, write;
	wire [23:0] readdata_left, readdata_right;
	wire [23:0] writedata_left, writedata_right;
	wire reset = ~KEY[0];

	wire [15:0] errorRange;
	
	//Displays
	output [6:0] HEX0, HEX1,HEX2,HEX3,HEX4,HEX5;
	vga_adapter VGA(
		.resetn(KEY[0]),
		.clock(CLOCK_50),
		.colour(3'b110),
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

		wire [2:0]notes;
		assign notes = SW[9:7];
		
		reg r,w,f,fe;
		assign read = r;
		assign write = w;
		reg [23:0]soundwave; // live/recorded soundwave
		// Logic for when to draw graph, record sound, and play sound
		reg [15:0]displaytest;
		always@(CLOCK_50) begin
			
			if (reset) begin
				f <= 16'd0;
				fe <= 16'd0;
				end
			
			// Record
			if (SW[0]) begin
				// record sound
				r <= read_ready;
				w <= 1'b0;
				// draw graph
				soundwave <= readdata_right;
				end
			// Play expected note
			else if (~SW[0]) begin
				// play sound
				r <= 1'b0;
				w <= write_ready;
				// draw graph
				soundwave <= audio_out;
				end
			
			if (frequency < expectedfrequency)
				greater <= 0;
			else
				greater <= 1;
				
			if (SW[1]) begin
				displaytest <= frequency;
				end
			if (~SW[1]) begin
				displaytest <= expectedfrequency;
				end
		end
		hex_decoder h3(displaytest[15:12],HEX3);
		hex_decoder h2(displaytest[11:8],HEX2);
		hex_decoder h1(displaytest[7:4],HEX1);
		hex_decoder h0(displaytest[3:0],HEX0);
		// Generate expected note soundwave
		wire [23:0] audio_out;
		playmusic playtone (CLOCK_50,AUD_XCK,audio_out,reset,notes,~KEY[1]);
		assign writedata_left = audio_out;
		assign writedata_right = audio_out;
		
		// Draw graph
		wire enable_draw;
		assign enable_draw = (~KEY[1] && ~SW[0]) || SW[0];
		//draw_graph draw(CLOCK_50,AUD_XCK,enable_draw,reset,soundwave, writeEn,X,Y);
		draw_increment(AUD_XCK,enable_draw ,reset,soundwave[23],writeEn,X,Y);
		
		// Calculate frequency
		wire [15:0] frequency;
		frequency_calculator freq(CLOCK_50, read, soundwave[23], reset, frequency);
		// Compare to expected frequency
		wire [15:0] expectedfrequency;


		lut lookup(notes, expectedfrequency);
		assign errorRange = 16'b0000000000110010;
		wire match;
		
		range_comparator16(frequency,expectedfrequency,errorRange,match);
		reg greater;
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





// Draw pixel


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
