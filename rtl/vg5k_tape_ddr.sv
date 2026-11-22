//-----------------------------------------------------------------------------
// Tape image in DDR3 (the HPS memory, from 0x30000000) with a byte port.
// The DDR3 path of Lynx48_MiSTer rtl/tape_ddr.sv (itself from the ZXP3 core)
// without the TZX player: the download is written byte by byte with byte
// enables (ioctl_wait while DDR3 is busy), reads go through a one-word (64-bit)
// cache. rd_ack answers rd_req once the byte at rd_addr is in the cache.
//-----------------------------------------------------------------------------

module vg5k_tape_ddr (
  input             clk,

  input             dl,             // the tape is downloading
  input      [26:0] dl_addr,
  input      [7:0]  dl_data,
  input             dl_wr,
  output reg        dl_wait = 1'b0,
  output reg [24:0] size = 25'd0,   // bytes loaded

  input      [24:0] rd_addr,
  input             rd_req,
  output            rd_ack,
  output     [7:0]  rd_data,

  output            DDRAM_CLK,
  input             DDRAM_BUSY,
  output     [7:0]  DDRAM_BURSTCNT,
  output     [28:0] DDRAM_ADDR,
  input      [63:0] DDRAM_DOUT,
  input             DDRAM_DOUT_READY,
  output            DDRAM_RD,
  output     [63:0] DDRAM_DIN,
  output     [7:0]  DDRAM_BE,
  output            DDRAM_WE
);

  reg [24:3] ram_a;
  reg [63:0] ram_d;
  reg [7:0]  ram_be;
  reg        ram_we = 1'b0;
  reg        ram_rd = 1'b0;

  assign DDRAM_CLK      = clk;
  assign DDRAM_BURSTCNT = 8'd1;
  assign DDRAM_ADDR     = {4'b0011, 3'b000, ram_a};
  assign DDRAM_DIN      = ram_d;
  assign DDRAM_BE       = ram_be;
  assign DDRAM_WE       = ram_we;
  assign DDRAM_RD       = ram_rd;

  reg [24:3] cache_a;
  reg [63:0] cache_d;
  reg        cache_v = 1'b0;
  reg        reading = 1'b0;
  wire       hit = cache_v && cache_a == rd_addr[24:3];

  assign rd_ack  = rd_req && hit && !dl;
  assign rd_data = cache_d[{rd_addr[2:0], 3'b000} +: 8];

  reg dl_q = 1'b0;
  always @(posedge clk) begin
    dl_q <= dl;
    if (!DDRAM_BUSY) begin
      if (ram_we) dl_wait <= 1'b0;
      ram_we <= 1'b0;
      ram_rd <= 1'b0;
    end

    if (dl) begin
      cache_v <= 1'b0;
      if (dl && !dl_q) size <= 25'd0;
      if (dl_wr) begin
        ram_a   <= dl_addr[24:3];
        ram_d   <= {8{dl_data}};
        ram_be  <= 8'd1 << dl_addr[2:0];
        ram_we  <= 1'b1;
        dl_wait <= 1'b1;
        size    <= dl_addr[24:0] + 25'd1;
      end
    end
    else if (!ram_we && !ram_rd && !reading && rd_req && !hit) begin
      ram_a   <= rd_addr[24:3];
      cache_a <= rd_addr[24:3];
      cache_v <= 1'b0;
      ram_rd  <= 1'b1;
      reading <= 1'b1;
    end

    if (reading && DDRAM_DOUT_READY) begin
      cache_d <= DDRAM_DOUT;
      cache_v <= 1'b1;
      reading <= 1'b0;
    end
  end

endmodule
