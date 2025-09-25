module LedDriver (
    input clk,
    input [31:0] value,         // 8 chiffres hexadécimaux
    input [7:0] enable,         // active les HEX[0-7]
    output reg [7:0] HEX0,
    output reg [7:0] HEX1,
    output reg [7:0] HEX2,
    output reg [7:0] HEX3,
    output reg [7:0] HEX4,
    output reg [7:0] HEX5,
    output reg [7:0] HEX6,
    output reg [7:0] HEX7
);

  function [7:0] hex_to_7seg;
    input [3:0] digit;
    begin
      case (digit)
        0:  hex_to_7seg = 8'b11000000;
        1:  hex_to_7seg = 8'b11111001;
        2:  hex_to_7seg = 8'b10100100;
        3:  hex_to_7seg = 8'b10110000;
        4:  hex_to_7seg = 8'b10011001;
        5:  hex_to_7seg = 8'b10010010;
        6:  hex_to_7seg = 8'b10000010;
        7:  hex_to_7seg = 8'b11111000;
        8:  hex_to_7seg = 8'b10000000;
        9:  hex_to_7seg = 8'b10010000;
        10: hex_to_7seg = 8'b10001000; // A
        11: hex_to_7seg = 8'b10000011; // b
        12: hex_to_7seg = 8'b10100111; // c
        13: hex_to_7seg = 8'b10100001; // d
        14: hex_to_7seg = 8'b10000110; // E
        15: hex_to_7seg = 8'b10001110; // F
        default: hex_to_7seg = 8'b11111111;
      endcase
    end
  endfunction

  always @(*) begin
    HEX0 = enable[0] ? hex_to_7seg(value[3:0])   : 8'b11111111;
    HEX1 = enable[1] ? hex_to_7seg(value[7:4])   : 8'b11111111;
    HEX2 = enable[2] ? hex_to_7seg(value[11:8])  : 8'b11111111;
    HEX3 = enable[3] ? hex_to_7seg(value[15:12]) : 8'b11111111;
    HEX4 = enable[4] ? hex_to_7seg(value[19:16]) : 8'b11111111;
    HEX5 = enable[5] ? hex_to_7seg(value[23:20]) : 8'b11111111;
    HEX6 = enable[6] ? hex_to_7seg(value[27:24]) : 8'b11111111;
    HEX7 = enable[7] ? hex_to_7seg(value[31:28]) : 8'b11111111;
  end

endmodule 