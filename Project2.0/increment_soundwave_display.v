module draw_increment(clk, enable, reset, soundwave, enable_draw,X_out,Y_out);

  input clk, enable, reset, soundwave;
  output reg enable_draw;
  output [7:0] X_out;
  output [6:0] Y_out;
  reg [7:0] X_;
  reg [6:0] Y_;
  wire [6:0] y_increase =7'd4;
  reg [6:0] y_shift;

  always@(posedge clk) begin
      if (reset) begin
        X_ <= 8'd0;
        Y_ <= 7'd6;
        enable_draw <= 1'b0;
        y_shift <= 7'd0;
        end

      if (enable) begin
        X_ <= X_ + 1;

        if (X_ >= 8'd124)
          begin

          if(Y_>= 7'd110)
            begin
            X_ <= 8'd155;
            Y_ <= 7'd112;
            enable_draw <= 1'b0;
            end
          else
			      begin
				    enable_draw <= 1'b1;
            X_ <= 8'd0;
				    Y_ <= Y_ + y_increase;
			      end
			     end

			   else
				    enable_draw <= 1'b1;

        if(soundwave)
          y_shift<=7'd0;
        else
          y_shift<=7'd2;
        end

      else
        enable_draw <= 1'b0;
    end

    assign X_out = X_;
    assign Y_out = Y_ - y_shift;


endmodule
