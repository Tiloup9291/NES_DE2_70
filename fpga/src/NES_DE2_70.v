module SRAM_Controller_DE270 (
    input        clk,
    input        reset_n,
    input        read,         // déclenche une lecture
    input        write,        // déclenche une écriture
    input [23:0] addr,         // adresse byte (24 bits, mais seuls 19:2 utiles)
    input [7:0]  din,          // données à écrire (octet)
    input [1:0]  byte_sel,     // quel octet dans le mot 32 bits (0 à 3)
    output reg [7:0] dout,     // données lues
    output reg   busy,         // occupé tant que l'opération est en cours

    output reg [18:0] SRAM_A,
    inout [31:0] SRAM_DQ,

    output reg SRAM_WE_N,
    output reg SRAM_OE_N,
    output reg SRAM_GW_N,
    output reg [3:0] SRAM_BE_N,
    output reg SRAM_CE1_N,
    output reg SRAM_CE2,
    output reg SRAM_CE3_N,

    output wire SRAM_ADSP_N,
    output wire SRAM_ADSC_N,
    output wire SRAM_ADV_N,
    output wire SRAM_CLK
);

    // Asynchronous config: ces signaux doivent rester désactivés
    assign SRAM_ADSP_N = 1'b1;
    assign SRAM_ADSC_N = 1'b1;
    assign SRAM_ADV_N  = 1'b1;
    assign SRAM_CLK    = 1'b0; // pas de clock pour mode async

    reg [31:0] data_out;
    reg        drive_data;

    assign SRAM_DQ = drive_data ? data_out : 32'bz;

    reg [2:0] state;
    localparam IDLE  = 3'd0,
               SETUP = 3'd1,
               ACCESS = 3'd2,
               HOLD = 3'd3,
               DONE = 3'd4;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            busy <= 0;
            SRAM_WE_N <= 1;
            SRAM_OE_N <= 1;
            SRAM_GW_N <= 1;
            SRAM_BE_N <= 4'b1111;
            SRAM_CE1_N <= 1;
            SRAM_CE2   <= 0; // actif haut
            SRAM_CE3_N <= 1;
            SRAM_A <= 0;
            data_out <= 32'b0;
            drive_data <= 0;
            dout <= 8'b0;
        end else begin
            case (state)
                IDLE: begin
                    busy <= 0;
                    drive_data <= 0;
                    if (read || write) begin
                        busy <= 1;
                        SRAM_A <= addr[20:2]; // aligné sur 4 octets
                        SRAM_CE1_N <= 0;
                        SRAM_CE2   <= 1;
                        SRAM_CE3_N <= 0;

                        // Gestion byte enable
                        SRAM_BE_N <= 4'b1111;
                        SRAM_BE_N[byte_sel] <= 0;

                        // Préparer écriture
                        if (write) begin
                            data_out <= 32'b0;
                            data_out[byte_sel*8 +: 8] <= din;
                            drive_data <= 1;
                            SRAM_WE_N <= 0;
                            SRAM_GW_N <= 0;
                            SRAM_OE_N <= 1;
                        end else begin
                            // Préparer lecture
                            SRAM_WE_N <= 1;
                            SRAM_GW_N <= 1;
                            SRAM_OE_N <= 0;
                        end

                        state <= ACCESS;
                    end
                end

                ACCESS: begin
                    // Maintenir signaux actifs pendant au moins 2 cycles (approx. 20-30ns à 50 MHz)
                    state <= HOLD;
                end

                HOLD: begin
                    SRAM_WE_N <= 1;
                    SRAM_OE_N <= 1;
                    SRAM_GW_N <= 1;
                    SRAM_CE1_N <= 1;
                    SRAM_CE2   <= 0;
                    SRAM_CE3_N <= 1;
                    SRAM_BE_N <= 4'b1111;
                    drive_data <= 0;

                    if (read)
                        dout <= SRAM_DQ[byte_sel*8 +: 8];

                    state <= DONE;
                end

                DONE: begin
                    busy <= 0;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule 

module GameLoader(input clk, input reset,
                  input [7:0] indata, input indata_clk,
                  output reg [21:0] mem_addr, output [7:0] mem_data, output mem_write,
                  output [31:0] mapper_flags,
                  output reg done,
                  output error);
  reg [1:0] state = 0;
  reg [7:0] prgsize;
  reg [3:0] ctr;
  reg [7:0] ines[0:15]; // 16 bytes of iNES header
  reg [21:0] bytes_left;
  
  assign error = (state == 3);
  wire [7:0] prgrom = ines[4];
  wire [7:0] chrrom = ines[5];
  assign mem_data = indata;
  assign mem_write = (bytes_left != 0) && (state == 1 || state == 2) && indata_clk;
  
  wire [2:0] prg_size = prgrom <= 1  ? 0 :
                        prgrom <= 2  ? 1 : 
                        prgrom <= 4  ? 2 : 
                        prgrom <= 8  ? 3 : 
                        prgrom <= 16 ? 4 : 
                        prgrom <= 32 ? 5 : 
                        prgrom <= 64 ? 6 : 7;
                        
  wire [2:0] chr_size = chrrom <= 1  ? 0 : 
                        chrrom <= 2  ? 1 : 
                        chrrom <= 4  ? 2 : 
                        chrrom <= 8  ? 3 : 
                        chrrom <= 16 ? 4 : 
                        chrrom <= 32 ? 5 : 
                        chrrom <= 64 ? 6 : 7;
  
  wire [7:0] mapper = {ines[7][7:4], ines[6][7:4]};
  wire has_chr_ram = (chrrom == 0);
  assign mapper_flags = {16'b0, has_chr_ram, ines[6][0], chr_size, prg_size, mapper};
  always @(posedge clk) begin
    if (reset) begin
      state <= 0;
      done <= 0;
      ctr <= 0;
      mem_addr <= 0;  // Address for PRG
    end else begin
      case(state)
      // Read 16 bytes of ines header
      0: if (indata_clk) begin
           ctr <= ctr + 1;
           ines[ctr] <= indata;
           bytes_left <= {prgrom, 14'b0};
           if (ctr == 4'b1111)
             state <= (ines[0] == 8'h4E) && (ines[1] == 8'h45) && (ines[2] == 8'h53) && (ines[3] == 8'h1A) && !ines[6][2] && !ines[6][3] ? 1 : 3;
         end
      1, 2: begin // Read the next |bytes_left| bytes into |mem_addr|
          if (bytes_left != 0) begin
            if (indata_clk) begin
              bytes_left <= bytes_left - 1;
              mem_addr <= mem_addr + 1;
            end
          end else if (state == 1) begin
            state <= 2;
            mem_addr <= 22'b10_0000_0000_0000_0000_0000; // Address for CHR
            bytes_left <= {1'b0, chrrom, 13'b0};
          end else if (state == 2) begin
            done <= 1;
          end
        end
      endcase
    end
  end
endmodule 

module NES_DE2_70(input iCLK_50,
                 input [3:0] iKEY,//2 = CPU_RESET, 3 = NES_RESET
                 input [15:0] iSW,
                 output [15:0] oLEDR,
                 output [6:0] oHEX0_D,
					  output oHEX0_DP,
					  output [6:0] oHEX1_D,
					  output oHEX1_DP,
					  output [6:0] oHEX2_D,
					  output oHEX2_DP,
					  output [6:0] oHEX3_D,
					  output oHEX3_DP,
					  output [6:0] oHEX4_D,
					  output oHEX4_DP,
					  output [6:0] oHEX5_D,
					  output oHEX5_DP,
					  output [6:0] oHEX6_D,
					  output oHEX6_DP,
					  output [6:0] oHEX7_D,
					  output oHEX7_DP,
                 // UART
                 input iUART_RXD,
                 output oUART_TXD,
                 // VGA
                 output oVGA_VS, output oVGA_HS, output [3:0] oVGA_R, output [3:0] oVGA_G, output [3:0] oVGA_B,
                 // Memory
                 output       oSRAM_OE_N,
					  output       oSRAM_WE_N,
					  output       oSRAM_ADV_N,
					  output       oSRAM_CLK,
					  output       oSRAM_CE1_N,
					  output			oSRAM_CE2,
					  output       oSRAM_CE3_N,
					  output [3:0] oSRAM_BE_N,
					  output [18:0] oSRAM_A,
					  inout  [31:0] SRAM_DQ,
					  output			oSRAM_GW_N,
					  output			oSRAM_ADSC_N,
					  output			oSRAM_ADSP_N,
                 // Sound board
					  output oI2C_SCLK,
					  inout I2C_SDAT,
                 output oAUD_XCK,
                 output AUD_DACLRCK,
                 output AUD_BCLK,
                 output oAUD_DACDAT
                 );
  wire clock_locked;
  wire clk;
  clk_21mhz clk_21mhz_inst(.inclk0(iCLK_50), .c0(clk),  .areset(1'b0), .locked(clock_locked));
 // UART
  wire [7:0] uart_data;
  wire [7:0] uart_addr;
  wire       uart_write;
  wire       uart_error;
  UartDemux uart_demux_inst(clk, 1'b0, iUART_RXD, uart_data, uart_addr, uart_write, uart_error);
  assign     oUART_TXD = 1;
  // Loader
  wire [7:0] loader_input = uart_data;
  wire       loader_clk   = (uart_addr == 8'h37) && uart_write;
  reg  [7:0] loader_conf;
  reg  [7:0] loader_btn, loader_btn_2;
  always @(posedge clk) begin
    if (uart_addr == 8'h35 && uart_write)
      loader_conf <= uart_data;
    if (uart_addr == 8'h40 && uart_write)
      loader_btn <= uart_data;
    if (uart_addr == 8'h41 && uart_write)
      loader_btn_2 <= uart_data;
  end
 // LED Display
	wire [7:0] wHEX0, wHEX1, wHEX2, wHEX3, wHEX4, wHEX5, wHEX6, wHEX7;
  reg [31:0] led_value;
  reg [7:0] led_enable;
  LedDriver led_driver_inst(clk, led_value, led_enable, wHEX0, wHEX1, wHEX2, wHEX3, wHEX4, wHEX5, wHEX6, wHEX7);
  assign oHEX0_D = wHEX0[6:0];
  assign oHEX0_DP = wHEX0[7];
  assign oHEX1_D = wHEX1[6:0];
  assign oHEX1_DP = wHEX1[7];
  assign oHEX2_D = wHEX2[6:0];
  assign oHEX2_DP = wHEX2[7];
  assign oHEX3_D = wHEX2[6:0];
  assign oHEX3_DP = wHEX2[7];
  assign oHEX4_D = wHEX2[6:0];
  assign oHEX4_DP = wHEX2[7];
  assign oHEX5_D = wHEX2[6:0];
  assign oHEX5_DP = wHEX2[7];
  assign oHEX6_D = wHEX2[6:0];
  assign oHEX6_DP = wHEX2[7];
  assign oHEX7_D = wHEX2[6:0];
  assign oHEX7_DP = wHEX2[7];
  
  wire [8:0] cycle;
  wire [8:0] scanline;
  wire [15:0] sample;
  wire [5:0] color;
  wire joypad_strobe;
  wire [1:0] joypad_clock;
  wire [21:0] memory_addr;
  wire memory_read_cpu, memory_read_ppu;
  wire memory_write;
  wire [7:0] memory_din_cpu, memory_din_ppu;
  wire [7:0] memory_dout;
  reg [7:0] joypad_bits, joypad_bits2;
  reg [1:0] last_joypad_clock;
  wire [31:0] dbgadr;
  wire [1:0] dbgctr;

  reg [1:0] nes_ce;

  always @(posedge clk) begin
    if (joypad_strobe) begin
      joypad_bits <= loader_btn;
      joypad_bits2 <= loader_btn_2;
    end
    if (!joypad_clock[0] && last_joypad_clock[0])
      joypad_bits <= {1'b0, joypad_bits[7:1]};
    if (!joypad_clock[1] && last_joypad_clock[1])
      joypad_bits2 <= {1'b0, joypad_bits2[7:1]};
    last_joypad_clock <= joypad_clock;
  end
  
  wire [21:0] loader_addr;
  wire [7:0] loader_write_data;
  wire loader_reset = loader_conf[0];
  wire loader_write;
  wire [31:0] mapper_flags;
  wire loader_done, loader_fail;
  GameLoader loader_inst(clk, loader_reset, loader_input, loader_clk,
                    loader_addr, loader_write_data, loader_write,
                    mapper_flags, loader_done, loader_fail);

  wire reset_nes = (iKEY[3] || !loader_done);
  wire run_mem = (nes_ce == 0) && !reset_nes;
  wire run_nes = (nes_ce == 3) && !reset_nes;
 
//  NES
always @(posedge clk)
    nes_ce <= nes_ce + 1;
    
  NES nes(clk, reset_nes, run_nes,
          mapper_flags,
          sample, color,
          joypad_strobe, joypad_clock, {joypad_bits2[0], joypad_bits[0]},
          iSW[4:0],
          memory_addr,
          memory_read_cpu, memory_din_cpu,
          memory_read_ppu, memory_din_ppu,
          memory_write, memory_dout,
          cycle, scanline,
          dbgadr,
          dbgctr);
 
//  Memory controller
reg        mem_read;
reg        mem_write;
reg [23:0] mem_addr;
reg [7:0]  mem_din;
wire [7:0] mem_dout;
wire       mem_busy;
reg [1:0]  mem_byte_sel;

// Instanciation du contrôleur SRAM
SRAM_Controller_DE270 sram_controller_inst (
    .clk(clk),
    .reset_n(!reset_nes),
    .read(mem_read && run_mem),
    .write(mem_write && run_mem || loader_write),
    .addr(mem_addr),
    .din(mem_din),
    .byte_sel(mem_byte_sel),
    .dout(mem_dout),
    .busy(mem_busy),

    .SRAM_A(oSRAM_A),
    .SRAM_DQ(SRAM_DQ),

    .SRAM_WE_N(oSRAM_WE_N),
    .SRAM_OE_N(oSRAM_OE_N),
    .SRAM_GW_N(oSRAM_GW_N),          // à connecter ou laisser non utilisé
    .SRAM_BE_N(oSRAM_BE_N),
    .SRAM_CE1_N(oSRAM_CE1_N),
    .SRAM_CE2(oSRAM_CE2),
    .SRAM_CE3_N(oSRAM_CE3_N),

    .SRAM_ADSP_N(oSRAM_ADSP_N),
    .SRAM_ADSC_N(oSRAM_ADSC_N),
    .SRAM_ADV_N(oSRAM_ADV_N),
    .SRAM_CLK(oSRAM_CLK)
);

// Interface NES mémoire vers SRAM controller
always @(posedge clk) begin
    mem_read <= memory_read_cpu;
    mem_write <= memory_write;
    mem_addr <= {2'b00, memory_addr}; // Ajuster la largeur si besoin
    mem_din <= memory_din_cpu;
    mem_byte_sel <= 0; // ajuster selon ton besoin d'octet (0..3)
end

assign memory_dout = mem_dout;
reg ramfail;
  always @(posedge clk) begin
    if (loader_reset)
      ramfail <= 0;
    else
      ramfail <= mem_busy && loader_write || ramfail;
  end
//  Video controller
  wire [14:0] doubler_pixel;
  wire doubler_sync;
  wire [9:0] vga_hcounter, doubler_x;
  wire [9:0] vga_vcounter;
  
  VgaDriver vga_inst(clk, oVGA_HS, oVGA_VS, oVGA_R, oVGA_G, oVGA_B, vga_hcounter, vga_vcounter, doubler_x, doubler_pixel, doubler_sync, iSW[0]);
  
  wire [14:0] pixel_in;
  //  NES Palette -> RGB332 conversion
  nes_palette_rom nes_palette_rom_inst(
					  .address(color),
					  .clock(clk),
					  .q(pixel_in)
					  );
  Hq2x hq2x_inst(clk, pixel_in, iSW[5], 
            scanline[8],        // reset_frame
            (cycle[8:3] == 42), // reset_line
            doubler_x,          // 0-511 for line 1, or 512-1023 for line 2.
            doubler_sync,       // new frame has just started
            doubler_pixel
				);     // pixel is outputted
//  Sound controller
wire doneConfig;
WM8731_Config WM8731_Config_inst(
    .clk(clk),          // 24 MHz system clock
    .reset(1'b0),
    .done(doneConfig),    // Config done flag
    .i2c_scl(oI2C_SCLK),
    .i2c_sda(I2C_SDAT)
);
wire [15:0] sound_signal = {sample[15] ^ 1'b1, sample[14:0]};
  always @(posedge clk) begin
    led_enable <= 255;
    led_value <= sound_signal;
  end

  reg [7:0] sound_ctr;
  always @(posedge clk)
    sound_ctr <= sound_ctr + 1;
  wire sound_load = (sound_ctr == 0);
  SoundDriver sound_driver(clk & doneConfig, 
      sound_signal, 
      sound_load,
      sound_load,
      oAUD_XCK, AUD_DACLRCK, AUD_BCLK, oAUD_DACDAT);
 
  assign oLEDR = {12'b0, uart_error, ramfail, loader_fail, loader_done};
endmodule 