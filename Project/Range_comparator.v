// checks if a input is within a certain expected range.
// x is +/- z within y, output 1 for true
module range_comparator24(x,y,z,o);
	input signed[23:0]x,y,z;
	output o;
	
	wire signed[23:0]upper,lower;
	assign signed upper = y+z;
	assign signed lower = y-z;
	wire result1,result2;
	
	comparator c1 (x,upper,result1);
	comparator c2 (x,lower,result2);
	
	
	assign o = (~result1 & result2);
	
	




endmodule 



// checks if a input is within a certain expected range.
// x is +/- z within y, output 1 for true
module range_comparator4(x,y,z,o);
	input [3:0]x,y,z;
	output o;
	
	wire [3:0]upper,lower;
	assign signed upper = signed(y)+ signed(z);
	assign signed lower = signed(y)-signed(z);
	wire result1,result2;
	
	comparator c1 (x,upper,result1);
	comparator c2 (x,lower,result2);
	
	
	assign o = (~result1 & result2);
	
	




endmodule 

module comparator (
    input wire signed [23:0] a,
    input wire signed [23:0] b,
    output reg greater
    );

    always @* begin
      if (a<=b) begin
        greater = 0;
      end
      else begin
        greater = 1;
      end
    end
endmodule 