// ----------------------------------------------------------------------------------
// Part of the Verilog translation of the orpigame EF9345 core (VRAM.vhd), 2026.
// Copyright D. Laurens (original design). This source describes Open Hardware
// and is licensed under the CERN-OHL-S v2 (see LICENSE in this directory).
//
// Replacement for the Xilinx UNIMACRO BRAM_TDP_MACRO in the configuration used
// by VRAM.vhd: BRAM_SIZE "36Kb", 32-bit x 1024 on both ports, DOA_REG = DOB_REG
// = 0 (one clock of read latency), WRITE_MODE_A = "WRITE_FIRST", four byte
// write enables on port A, port B read-only (VRAM.vhd ties WEB to "0000" and
// DIB to zero), ENA = ENB = '1', RSTA = RSTB = '0', initial contents 0.
//
// Behaviour:
//  * port A WRITE_FIRST with byte enables: after a write, the written lanes of
//    DOA show the new data, the other lanes the stored data. Built as an
//    old-data read plus a registered bypass, so it does not depend on the
//    inferred RAM's same-port read-during-write mode;
//  * port B reading the address port A writes in the same clock gets the OLD
//    data. (On a Xilinx 7-series BRAM this collision is undefined when the
//    writing port is WRITE_FIRST; old data is what an Intel M10K gives in
//    OLD_DATA mixed-port mode. The equivalence bench models the Xilinx macro
//    the same way.)
//
// Quartus infers one M10K-based RAM per 8-bit lane (1024 x 8 each).
// 2026: explicit generate/genvar and per-lane registers for Quartus 17.
// ----------------------------------------------------------------------------------

module EF9345_BRAM_TDP (
    input  logic        CLK,
    // Port A: read / write (CPU side)
    input  logic [9:0]  ADDRA,
    input  logic [31:0] DIA,
    input  logic [3:0]  WEA,
    output logic [31:0] DOA,
    // Port B: read only (display side)
    input  logic [9:0]  ADDRB,
    output logic [31:0] DOB
);

logic [3:0]  we_d = '0;   // byte enables of the last write
logic [31:0] di_d = '0;   // data of the last write

// 2026: explicit generate / genvar for Quartus 17 (Verilog-2001 style), each
// lane keeps its own old-data registers.
genvar k;
generate
  for (k = 0; k < 4; k = k + 1) begin : lane
    logic [7:0] mem [0:1023];
    logic [7:0] q_a = 8'h00;   // old data, port A
    logic [7:0] q_b = 8'h00;   // old data, port B
    integer i;

    initial begin
      for (i = 0; i < 1024; i = i + 1)
        mem[i] = 8'h00;
    end

    always_ff @(posedge CLK) begin
      q_a <= mem[ADDRA];
      if (WEA[k])
        mem[ADDRA] <= DIA[8*k +: 8];
    end

    always_ff @(posedge CLK)
      q_b <= mem[ADDRB];

    // WRITE_FIRST: a lane written at the last edge shows its new data.
    assign DOA[8*k +: 8] = we_d[k] ? di_d[8*k +: 8] : q_a;
    assign DOB[8*k +: 8] = q_b;
  end
endgenerate

always_ff @(posedge CLK) begin
  we_d <= WEA;
  di_d <= DIA;
end


endmodule
