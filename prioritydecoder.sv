`timescale 1ns / 1ns
module prioritydecoder 
#(
    parameter WIDTH = 4
)(
    input logic [WIDTH-1:0] in,
    output logic [$clog2(WIDTH)-1:0] out,
    output logic valid
);

    always_comb begin
        valid = 1'b0;
        out   = '0;
        for (int i = WIDTH; i-- > 0; ) begin
            if (in[i] && !valid) begin
                valid = 1'b1;
                out   = i[$clog2(WIDTH)-1:0]; // highest index wins
            end
        end
    end
endmodule
