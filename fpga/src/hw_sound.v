
module WM8731_Config (
    input clk,          // 24 MHz system clock
    input reset,
    output reg done,    // Config done flag

    // I2C interface
    output reg i2c_scl,
    inout wire i2c_sda
);

// ====== Clock generation for audio (not I2C clocks) ======
// MCLK = 12 MHz = clk / 2
reg mclk_reg = 0;
always @(posedge clk) mclk_reg <= ~mclk_reg;
wire mclk = mclk_reg;

// SCLK = LRCK * 48; LRCK = 32 kHz
// 24 MHz / 16 = 1.5 MHz ~ SCLK
// 24 MHz / 48 = 0.5 MHz ~ LRCK
reg [5:0] sclk_div = 0;
reg sclk_reg = 0;
always @(posedge clk) begin
    if (reset) begin
        sclk_div <= 0;
        sclk_reg <= 0;
    end else if (sclk_div == 7) begin  // Divide 24MHz by 8 for SCLK ~3MHz is too fast, let's do by 16
        sclk_reg <= ~sclk_reg;
        sclk_div <= 0;
    end else begin
        sclk_div <= sclk_div + 1;
    end
end
wire sclk = sclk_reg; // This is SCLK at 24MHz/16=1.5MHz

// LRCK = 32kHz = 24MHz / 750
reg [9:0] lrck_div = 0;
reg lrck_reg = 0;
always @(posedge clk) begin
    if (reset) begin
        lrck_div <= 0;
        lrck_reg <= 0;
    end else if (lrck_div == 374) begin // Toggle every 375 clocks → freq = 24MHz/(2*375)=32kHz
        lrck_reg <= ~lrck_reg;
        lrck_div <= 0;
    end else begin
        lrck_div <= lrck_div + 1;
    end
end
wire lrck = lrck_reg;

// ====== I2C protocol signals ======
// I2C timing from 24MHz clock:
// Standard mode 100kHz, approx 240 cycles per clock cycle of I2C
parameter I2C_FREQ = 100_000; // 100kHz I2C clock
parameter CLK_FREQ = 24_000_000; // 24 MHz
localparam I2C_DIVIDER = CLK_FREQ / (I2C_FREQ * 4); 
// Divide by 4 because SCL needs toggling (high and low periods)

// SDA is bidir
reg sda_out;
reg sda_oe; // output enable for sda (0=input, 1=output low)

assign i2c_sda = sda_oe ? 1'b0 : 1'bz; // Drive SDA low if enabled, else high impedance

// ====== I2C controller FSM ======

localparam [3:0]
  IDLE      = 0,
  START     = 1,
  SEND_BYTE = 2,
  WAIT_ACK  = 3,
  STOP      = 4,
  DONE      = 5;

reg [3:0] state = IDLE;

reg [7:0] i2c_byte;
reg [3:0] bit_cnt;
reg ack;
reg [15:0] clk_cnt;

reg [3:0] reg_index = 0;

// WM8731 I2C slave address = 0x1A (7-bit), write address = 0x34
localparam SLAVE_ADDR = 7'h1A;
localparam SLAVE_WR = {SLAVE_ADDR,1'b0};

// ====== Registers to configure (12-bit value) ======
// Format: {7-bit reg addr, 9-bit reg data}
// These are example typical config for WM8731
reg [15:0] config_regs [0:6];
initial begin
  // Power down control: all on (disable power down)
  config_regs[0] = 16'b0000000_000000111; // reg 0 (0x00) : 0000 00111 = power up all blocks

  // Digital audio interface format: I2S, 16-bit word length, slave mode
  config_regs[1] = 16'b0000001_000001010; // reg 1 (0x01) : I2S format, 16-bit

  // Sample rate control: use normal mode, 256*Fs MCLK divider
  config_regs[2] = 16'b0000010_000000000; // reg 2 (0x02) : 0 (normal mode)

  // Digital interface activation: enable digital interface
  config_regs[3] = 16'b0001111_000000001; // reg 15 (0x0F) : digital interface on

  // Additional: Power management registers, headphone out vol, line out vol etc.
  // Adjust as needed for your hardware

  // Left line out volume (reg 8)
  config_regs[4] = 16'b0010000_000111111; // reg 8 (0x08) - 0x7F volume

  // Right line out volume (reg 9)
  config_regs[5] = 16'b0010001_000111111; // reg 9 (0x09) - 0x7F volume

  // Analog audio path control (reg 4) - enable DAC, disable mic boost
  config_regs[6] = 16'b0000100_000100100; // reg 4 (0x04)
end

integer i;

// Clock divider for I2C speed generation
always @(posedge clk) begin
    if (reset) begin
        clk_cnt <= 0;
    end else if (clk_cnt == I2C_DIVIDER - 1) begin
        clk_cnt <= 0;
    end else begin
        clk_cnt <= clk_cnt + 1;
    end
end

wire i2c_clk_en = (clk_cnt == 0);

// I2C FSM
always @(posedge clk) begin
  if (reset) begin
    state <= IDLE;
    i2c_scl <= 1;
    sda_oe <= 0;
    sda_out <= 1;
    bit_cnt <= 7;
    reg_index <= 0;
    done <= 0;
  end else if (i2c_clk_en) begin
    case(state)
      IDLE: begin
        i2c_scl <= 1;
        sda_oe <= 1;  // SDA high -> idle state
        sda_out <= 1;
        bit_cnt <= 7;
        done <= 0;
        if (reg_index < 7)
          state <= START;
        else
          done <= 1;
      end
      START: begin
        // START condition: SDA goes low while SCL high
        sda_oe <= 1;
        sda_out <= 0;
        i2c_scl <= 1;
        state <= SEND_BYTE;
        i2c_byte <= config_regs[reg_index][15:8]; // send first byte: register address + MSB data (7 bits + 1)
        bit_cnt <= 7;
      end
      SEND_BYTE: begin
        i2c_scl <= 0; // bring SCL low to set SDA
        sda_oe <= 1;
        sda_out <= i2c_byte[bit_cnt];
        state <= WAIT_ACK;
      end
      WAIT_ACK: begin
        i2c_scl <= 1; // raise SCL, slave should ack on SDA low
        sda_oe <= 0;  // release SDA
        if (bit_cnt == 0) begin
          ack <= (i2c_sda == 0);
          if (ack) begin
            // Send low byte (data bits 8:0)
            i2c_byte <= config_regs[reg_index][7:0];
            bit_cnt <= 7;
            state <= SEND_BYTE;
          end else begin
            state <= STOP; // no ack, stop
          end
        end else begin
          bit_cnt <= bit_cnt - 1;
          state <= SEND_BYTE;
        end
      end
      STOP: begin
        // STOP condition: SDA goes high while SCL high
        i2c_scl <= 1;
        sda_oe <= 0; // release SDA
        done <= (reg_index >= 6);
        reg_index <= reg_index + 1;
        if (reg_index < 6) state <= START;
        else state <= DONE;
      end
      DONE: begin
        done <= 1;
        i2c_scl <= 1;
        sda_oe <= 0;
      end
    endcase
  end
end

endmodule
// CLK is 24Mhz. MCLK is divided by two (12Mhz). 24Mhz divide by 16 produces SCLK.
// Divide by 48 produces LRCK.
// produce LRCK = 32kHz. We output 16-bit samples, but internally the DAC
// is in 24-bit mode. SCLK ratio is 48 * 32kHz.
module SoundDriver(input CLK, input [15:0] write_data, input write_left, input write_right,
                   output AUD_MCLK, output AUD_LRCK, output AUD_SCK, output AUD_SDIN);
  reg lrck;
  reg [15:0] leftbuf;
  reg [15:0] rightbuf;
  reg [16:0] currbuf;
  reg [3:0] sclk_div;
  reg [4:0] bitcnt_24;   // Counts 0-23
  wire [4:0] bitcnt_24_new = bitcnt_24 + 1;
  always @(posedge CLK) begin
    // Buffer one sample of each channel.
    if (write_left)  leftbuf <= write_data;
    if (write_right) rightbuf <= write_data;
    // Divide 24MHz by 16 to produce the SCLK frequency (48 * 32000) as well
    // as the 12MHz MCLK frequency.
    sclk_div <= sclk_div + 1;
    // Output new bits on the falling edge of MCLK so that values are
    // stable once MCLK rises.
    if (sclk_div == 4'b1111) begin
      // This runs at the SCLK frequency. Output next bit.
      currbuf <= {currbuf[15:0], 1'b0};
      bitcnt_24 <= bitcnt_24_new;
      if (bitcnt_24_new[4:3] == 2'b11) begin
        bitcnt_24[4:3] <= 2'b00; // 24 -> 0
        lrck <= !lrck;
        currbuf[15:0] <= lrck ? leftbuf : rightbuf;
      end
    end
  end
  assign AUD_MCLK = sclk_div[0];
  assign AUD_SCK = 1; // Internal emphasis turned off
  assign AUD_SDIN = currbuf[16];
  assign AUD_LRCK = lrck;
endmodule 