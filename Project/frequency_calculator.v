module FreqCalc (CLOCK_50, CLOCK2_50, KEY, FPGA_I2C_SCLK, FPGA_I2C_SDAT, AUD_XCK,
		        AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK, AUD_ADCDAT, AUD_DACDAT, SW, HEX0, HEX1, HEX2, HEX3, LEDR);

	input CLOCK_50, CLOCK2_50;
	input [3:0] KEY;
	input [0:0] SW;
	output [6:0] HEX0, HEX1, HEX2, HEX3;
	output [0:0] LEDR;
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

	/////////////////////////////////
	// Your code goes here
	/////////////////////////////////

	// Call frequency calculator module
	wire [15:0] frequency;
	frequency_calculator(CLOCK_50, AUD_XCK, SW[0], readdata_left, reset, frequency, HEX0, HEX1, HEX2, HEX3, LEDR);


/////////////////////////////////////////////////////////////////////////////////
// Audio CODEC interface.
//
// The interface consists of the following wires:
// read_ready, write_ready - CODEC ready for read/write operation
// readdata_left, readdata_right - left and right channel data from the CODEC
// read - send data from the CODEC (both channels)
// writedata_left, writedata_right - left and right channel data to the CODEC
// write - send data to the CODEC (both channels)
// AUD_* - should connect to top-level entity I/O of the same name.
//         These signals go directly to the Audio CODEC
// I2C_* - should connect to top-level entity I/O of the same name.
//         These signals go directly to the Audio/Video Config module
/////////////////////////////////////////////////////////////////////////////////
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


// Calculates the frequency of given sound input when enable is set to 1.
// Runs for 1 sec.
// To calculate new for new frequency (next 1 sec), flick reset on and off
module frequency_calculator(clk, audio_clock, enable, soundwave, reset, frequency) begin
	input [23:0] soundwave;
	output [15:0] frequency;
	//output [6:0] HEX0, HEX1, HEX2, HEX3;
	//output [0:0] LEDR;

	// calculate frequency
	wire [15:0] load = 16'd48000;

	wire [15:0] counter;
	countdown timer(
		.clk(clk),
		.audio_clock(audio_clock),
		.enable(enable),
		.load(load),
		.reset(reset),
		.count_q(counter)
	);

	// Enable and disable count_ticks (which counts frequency)
	reg enable_count_ticks;
	always @(*) begin
		// Reset: perform before calculating frequency
		if (reset)
			enable_count_ticks = 1;

		// Set count_ticks' enable. Disable when counter reaches 0.
		if (counter == 16'd0)
			enable_count_ticks = 0;
	end
	// wire calculate = (counter != 16'd0);

	assign LEDR = enable_count_ticks;

	count_ticks ct(
		.clk(clk),
		.audio_clock(audio_clock),
		.enable(enable_count_ticks),
		.soundwave(soundwave),
		.tick_count(frequency),
		.reset(reset)
	);

	// Display when countdown is 0
	reg [3:0] display0, display1, display2, display3;
	always @(*) begin
		// display frequency
		display0 = frequency[3:0];
		display1 = frequency[7:4];
		display2 = frequency[11:8];
		display3 = frequency[15:12];

//		// COMMENT OUT: Countdown works
//		// Test countdown, Display 2 when it reaches 0
//		if (counter == 16'd0) begin
//			display0 = 4'd2;
//			display1 = 4'd2;
//			display2 = 4'd2;
//			display3 = 4'd2;
//			end
////		else begin
////			display0 = 4'd0;
////			display1 = 4'd0;
////			display2 = 4'd0;
////			display3 = 4'd0;
////			end

	end

	// Display frequency to HEX
	//hex_decoder hex0(display0, HEX0);
	//hex_decoder hex1(display1, HEX1);
	//hex_decoder hex2(display2, HEX2);
	//hex_decoder hex3(display3, HEX3);

endmodule


// Count number of rising zero soundwave values
module count_ticks(clk, audio_clock, enable, soundwave, tick_count, reset);
	input clk, audio_clock, enable;
	input [23:0] soundwave;
	output reg [15:0] tick_count;
	// posedge reset
	input reset;

	// 1 -> negative, 0 -> positive
	reg prev_soundwave_sign;

	always @(posedge clk)
	begin
		if (reset) begin // reset
			prev_soundwave_sign <= 0;
			tick_count = 16'd0; // 4369 equals hex 1111
		end
		else if (enable && audio_clock) begin // check if current soundwave is at/near 0
			// if at rising 0
			if ((soundwave[23] == 0 || soundwave == 24'd0) && prev_soundwave_sign == 1) begin
				tick_count <= tick_count + 16'd1;
				prev_soundwave_sign = 0;
			end
			else
				prev_soundwave_sign = soundwave[23];
		end
	end
endmodule


module countdown(clk, audio_clock, enable, load, reset, count_q);
	// reset is posedge
	input clock, audio_clock, enable, reset;
	input [15:0] load;
	output reg [15:0] count_q;

	always @(posedge clk) begin
		if (reset) // reset counter
			count_q <= load;
		else if (enable & audio_clock) // counter when enable is on
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
