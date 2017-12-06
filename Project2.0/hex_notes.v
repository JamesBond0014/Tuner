module note_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;


    always @(*) begin
        case (hex_digit)
            4'h0: segments = 7'b000_0110; //E
            4'h1: segments = 7'b000_1000; //A
            4'h2: segments = 7'b010_0001; //D
            4'h3: segments = 7'b000_0010; //G
            4'h4: segments = 7'b000_0011; //B
            4'h5: segments = 7'b000_0110; //E
            4'h6: segments = 7'b100_0111; //L
            4'h7: segments = 7'b000_1001; //H
            4'hE: segments = 7'b011_1111; //-
            4'hF: segments = 7'b011_1111; //-
            default: segments = 7'b111_1011;
        endcase
	  end
endmodule
