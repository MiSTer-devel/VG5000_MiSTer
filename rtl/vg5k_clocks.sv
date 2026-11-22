//-----------------------------------------------------------------------------
// VG5000 clock enables from clk_sys = 48 MHz.
//
// The real machine: a 12 MHz crystal drives the EF9345, which outputs 4 MHz
// for the Z80A (Technical Manual p4; service manual p2). Here:
//   ce_cpu_p / ce_cpu_n  Z80 T-state phases, /12 = 4 MHz, or /3 = 16 MHz
//                        (4x) while turbo is requested (tape-standard rule 9:
//                        48 / (4 x 4) = 3, an exact divide; 5x would need
//                        /2.4). The spacing changes only at a T-state boundary,
//                        so p and n always alternate and never share a clock.
//   ce_4m                4 MHz, never sped up (beeper / timers that must not
//                        follow turbo)
//-----------------------------------------------------------------------------

module vg5k_clocks (
  input        clk,
  input        turbo_req,
  output reg   ce_cpu_p = 1'b0,
  output reg   ce_cpu_n = 1'b0,
  output reg   ce_4m = 1'b0,
  output reg   turbo = 1'b0      // spacing in use for the current T-state
);

  reg [3:0] d12 = 4'd0;
  always @(posedge clk) begin
    d12   <= (d12 == 4'd11) ? 4'd0 : d12 + 4'd1;
    ce_4m <= d12 == 4'd0;
  end

  reg [3:0] cpu_div = 4'd0;
  always @(posedge clk) begin
    ce_cpu_p <= 1'b0;
    ce_cpu_n <= 1'b0;
    if (cpu_div == 4'd0) begin
      ce_cpu_p <= 1'b1;
      turbo    <= turbo_req;
      cpu_div  <= turbo_req ? 4'd2 : 4'd11;
    end else begin
      cpu_div <= cpu_div - 4'd1;
      if (cpu_div == (turbo ? 4'd1 : 4'd6)) ce_cpu_n <= 1'b1;
    end
  end

endmodule
