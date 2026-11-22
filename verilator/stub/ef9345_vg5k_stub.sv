//-----------------------------------------------------------------------------
// Simulation-only behavioural stand-in for rtl/ef9345/ef9345_vg5k.sv (make stub).
// Used by the Verilator build only while the real file is absent.
//
// A behavioural EF9345 after MAME src/devices/video/ef9345.cpp (GPL-2.0+,
// D. Coulom / S. Ronco), just enough for the VG5000 BASIC ROM:
//   - indirect access: address latch (OUT 8F), R0-R7 data (CF), bit 3 = XQR
//     executes R0; R0 read = status (bit 7 busy, bit 2 vsync, S4/S5/S6)
//   - commands KRF / KRG (read, write, +inc), CLF / CLG, OCT (main / aux),
//     KRL (12-bit), IND (TGS MAT PAT DOR ROR, ROM read), INY, NOP / VRM / VSM,
//     MVB / MVD / MVT
//   - 16 KB VRAM (MAME's map; the VG5000 has 8 KB)
//   - display: 40-column 16-bit (BASIC) and 24-bit bichrome modes, ROM
//     charset (verilator/files/charset.rom, MAME layout), user characters in
//     VRAM, attribute latches, insert / flash / conceal / negative /
//     underline, cursor, margins. No double size, no quadrichrome, no
//     80 columns, no accents (drawn with the plain glyph).
//   - raster: PAL 15.625 kHz / 50 Hz progressive, 312 lines of 64 µs,
//     ce_pix = 8 MHz (48 / 6), active 336 x 270 = MAME's visible area
//     (42 x 27 cells of 8 x 10, the outer ring is the margin)
//   - irq_n low for 100 µs at the start of each frame (MAME vg5k.cpp)
//-----------------------------------------------------------------------------

module ef9345_vg5k (
  input  logic       clk,        // clk_sys 48 MHz
  input  logic       reset,
  input  logic       addr_wr,
  input  logic       data_wr,
  input  logic       data_rd,
  input  logic [7:0] din,
  output logic [7:0] dout,
  output logic       ce_pix,
  output logic       r, g, b,
  output logic       hsync, vsync, hblank, vblank,
  output logic       irq_n
);

`ifndef EF9345_CHARSET_HEX
`define EF9345_CHARSET_HEX "ef9345_charset.hex"
`endif

  // ---------------------------------------------------------------- memories
  logic [7:0] charset [0:8191];
  initial $readmemh(`EF9345_CHARSET_HEX, charset);
  logic [7:0] vram [0:16383] /*verilator public*/;

  // ---------------------------------------------------------------- registers
  logic [7:0] R [0:7] /*verilator public*/;
  logic [7:0] tgs /*verilator public*/, mat /*verilator public*/, pat /*verilator public*/,
              dor /*verilator public*/, ror /*verilator public*/;
  logic [7:0] ad;
  logic [6:0] st;          // status bits 6:0
  logic       busy;
  logic [1:0] blink;

  function automatic [13:0] indexram(input [7:0] x, input [7:0] y0);
    logic [7:0] y;
    begin
      y = (y0 < 8) ? (y0 & 8'd1) : y0;
      indexram = {y[5], x[6], x[7], y[4:0], x[5:0]};
    end
  endfunction
  function automatic [7:0] incx(input [7:0] x);
    logic [6:0] i;
    begin
      i = {1'b0, x[5:0]} + 7'd1;
      if (i > 39) i = i - 7'd40;
      incx = {x[7:6], i[5:0]};
    end
  endfunction
  function automatic [7:0] incy(input [7:0] y);
    logic [5:0] i;
    begin
      i = {1'b0, y[4:0]} + 6'd1;
      if (i > 31) i = i - 6'd24;
      incy = {y[7:5], i[4:0]};
    end
  endfunction

  assign dout = (ad[2:0] == 3'd0) ? {busy, st} : R[ad[2:0]];

  // ---------------------------------------------------------------- command execution
  typedef enum logic [3:0] {X_IDLE, X_W1, X_W2, X_W3, X_R1, X_R2, X_R3, X_RD, X_LOOP, X_MV1, X_MV2, X_MV3, X_DONE} xs_t;
  xs_t        xs;
  logic [7:0] cmd;
  logic [10:0] cnt;
  logic [13:0] pa;           // port A address
  logic [7:0]  pa_d;
  logic        pa_we;
  logic [7:0]  pa_q;
  logic [7:0]  mv_q [0:2];
  logic [1:0]  mv_i;
  logic [7:0]  hold;         // minimum busy time

  always_ff @(posedge clk) begin
    if (pa_we) vram[pa] <= pa_d;
    pa_q <= vram[pa];
  end

  wire [13:0] a_main = indexram(R[7], R[6]);
  wire [13:0] a_aux  = indexram(R[5], R[4]);
  wire        mvn_stop = cmd[0];
  wire [1:0]  mvn = cmd[7:4] == 4'hD ? 2'd1 : cmd[7:4] == 4'hE ? 2'd2 : 2'd3;
  wire        mv_mp_to_ap = cmd[3:2] == 2'b01;

  logic [8:0] rd_tag;    // which register a read lands in: {reg[2:0], ...}

  always_ff @(posedge clk) begin
    pa_we <= 1'b0;
    if (hold != 0) hold <= hold - 8'd1;
    if (reset) begin
      for (int i = 0; i < 8; i++) R[i] <= 8'd0;
      tgs <= 0; mat <= 0; pat <= 0; dor <= 0; ror <= 0; ad <= 0; st <= 0; busy <= 0; xs <= X_IDLE; hold <= 0;
    end else begin
      if (addr_wr) ad <= din;
      if (data_wr && !(ad[3] && busy)) begin
        R[ad[2:0]] <= din;
      end
      if (data_rd && ad[2:0] == 3'd0) st <= st;   // (MAME latches the status here)
      if (data_wr && ad[3] && xs == X_IDLE) begin
        cmd  <= (ad[2:0] == 3'd0) ? din : R[0];
        busy <= 1'b1;
        hold <= 8'd48;
        st[6:4] <= {1'b0, R[7][5:0] == 6'd39, R[5][5:0] == 6'd39};
        st[3] <= 1'b0; st[1:0] <= 2'b00;
        cnt  <= 11'd0;
        // decode
        begin
          logic [7:0] c;
          c = (ad[2:0] == 3'd0) ? din : R[0];
          casez (c)
            8'h00, 8'h01, 8'h02, 8'h03, 8'h30, 8'h31, 8'h34, 8'h35, 8'h50, 8'h51: xs <= X_W1;
            8'h05, 8'h07: xs <= X_LOOP;
            8'h08, 8'h09, 8'h0A, 8'h0B, 8'h38, 8'h39, 8'h3C, 8'h3D, 8'h58, 8'h59: xs <= X_R1;
            8'hD5, 8'hD6, 8'hD9, 8'hDA, 8'hE5, 8'hE6, 8'hE9, 8'hEA, 8'hF5, 8'hF6, 8'hF9, 8'hFA: xs <= X_MV1;
            default: xs <= X_DONE;
          endcase
        end
      end
      case (xs)
        X_IDLE: ;
        // ---- writes: KRF / KRG / OCT / KRL
        X_W1: begin
          pa_we <= 1'b1;
          if (cmd[7:4] == 4'h3) begin                    // OCT
            pa <= cmd[2] ? a_aux : a_main; pa_d <= R[1];
            if (cmd[0]) begin
              if (cmd[2]) R[5] <= incx(R[5]);
              else begin
                R[7] <= incx(R[7]);
                if (incx(R[7]) == {R[7][7:6], 6'd0}) R[6] <= incy(R[6]);
                if (R[7][5:0] == 6'd39) st[6] <= 1'b1;
              end
            end
            xs <= X_DONE;
          end else if (cmd[7:4] == 4'h5) begin           // KRL write: R1, then 4 bits of R3
            pa <= a_main; pa_d <= R[1]; xs <= X_W2;
          end else begin
            pa <= a_main; pa_d <= R[1]; xs <= X_W2;
          end
        end
        X_W2: begin
          if (cmd[7:4] == 4'h5) begin
            // read-modify-write the shared attribute byte
            pa <= a_main[11] ? a_main + 14'h0800 : a_main + 14'h1000;
            xs <= X_W3;
          end else begin
            pa_we <= 1'b1; pa <= a_main + 14'h0800; pa_d <= R[2];
            xs <= (cmd[1] == 1'b0) ? X_W3 : X_DONE;
            if (cmd[1] && cmd[0]) begin R[7] <= incx(R[7]); if (R[7][5:0] == 6'd39) st[6] <= 1'b1; end
          end
        end
        X_W3: begin
          if (cmd[7:4] == 4'h5) begin
            pa_we <= 1'b1;
            pa_d <= a_main[11] ? {pa_q[7:4], R[3][3:0]} : {R[3][7:4], pa_q[3:0]};
            if (cmd[0]) begin
              if (!R[7][7]) R[7][7] <= 1'b1;
              else begin R[7] <= incx({1'b0, R[7][6:0]}); end
            end
          end else begin
            pa_we <= 1'b1; pa <= a_main + 14'h1000; pa_d <= R[3];
            if (cmd[0]) begin R[7] <= incx(R[7]); if (R[7][5:0] == 6'd39) st[6] <= 1'b1; end
          end
          xs <= X_DONE;
        end
        // ---- reads: KRF / KRG / OCT / KRL
        X_R1: begin
          pa <= (cmd[7:4] == 4'h3 && cmd[2]) ? a_aux : a_main;
          xs <= X_R2;
        end
        X_R2: begin
          // pa_q valid next clock
          xs <= X_RD;
        end
        X_RD: begin
          if (cnt == 0) begin
            R[1] <= pa_q;
            if (cmd[7:4] == 4'h3) begin
              if (cmd[0]) begin
                if (cmd[2]) R[5] <= incx(R[5]);
                else begin
                  R[7] <= incx(R[7]);
                  if (incx(R[7]) == {R[7][7:6], 6'd0}) R[6] <= incy(R[6]);
                end
              end
              xs <= X_DONE;
            end else begin
              pa  <= (cmd[7:4] == 4'h5) ? (a_main[11] ? a_main + 14'h0800 : a_main + 14'h1000) : a_main + 14'h0800;
              cnt <= 11'd1; xs <= X_R2;
            end
          end else if (cnt == 1) begin
            if (cmd[7:4] == 4'h5) begin
              R[3] <= pa_q;
              if (cmd[0]) begin
                if (!R[7][7]) R[7][7] <= 1'b1;
                else R[7] <= incx({1'b0, R[7][6:0]});
              end
              xs <= X_DONE;
            end else begin
              R[2] <= pa_q;
              if (cmd[1]) begin
                if (cmd[0]) R[7] <= incx(R[7]);
                xs <= X_DONE;
              end else begin
                pa <= a_main + 14'h1000; cnt <= 11'd2; xs <= X_R2;
              end
            end
          end else begin
            R[3] <= pa_q;
            if (cmd[0]) R[7] <= incx(R[7]);
            xs <= X_DONE;
          end
        end
        // ---- CLF / CLG: 1280 cells
        X_LOOP: begin
          pa_we <= 1'b1;
          case (mv_i)
            2'd0: begin pa <= a_main;            pa_d <= R[1]; mv_i <= 2'd1; end
            2'd1: begin pa <= a_main + 14'h0800; pa_d <= R[2]; mv_i <= (cmd == 8'h05) ? 2'd2 : 2'd3; end
            2'd2: begin pa <= a_main + 14'h1000; pa_d <= R[3]; mv_i <= 2'd3; end
            default: begin
              pa_we <= 1'b0;
              mv_i <= 2'd0;
              R[7] <= incx(R[7]);
              if (incx(R[7]) == {R[7][7:6], 6'd0}) R[6] <= incy(R[6]);
              cnt <= cnt + 11'd1;
              if (cnt == 11'd1279) xs <= X_DONE;
            end
          endcase
        end
        // ---- MVB / MVD / MVT
        X_MV1: begin   // read source byte mv_i
          pa <= (mv_mp_to_ap ? a_main : a_aux) + {mv_i, 11'd0};
          xs <= X_MV2;
        end
        X_MV2: xs <= X_MV3;
        X_MV3: begin
          pa_we <= 1'b1;
          pa <= (mv_mp_to_ap ? a_aux : a_main) + {mv_i, 11'd0};
          pa_d <= pa_q;
          if (mv_i + 2'd1 < {1'b0, mvn}) begin mv_i <= mv_i + 2'd1; xs <= X_MV1; end
          else begin
            mv_i <= 2'd0;
            R[5] <= incx(R[5]); R[7] <= incx(R[7]);
            cnt <= cnt + 11'd1;
            xs <= X_MV1;
            if ((incx(R[5]) == {R[5][7:6], 6'd0}) && mvn_stop) xs <= X_DONE;
            if (incx(R[7]) == {R[7][7:6], 6'd0}) begin
              if (mvn_stop) xs <= X_DONE; else R[6] <= incy(R[6]);
            end
            if (cnt == 11'd1279) xs <= X_DONE;
          end
        end
        X_DONE: begin
          // instantaneous commands
          if (cmd[7:4] == 4'h8) begin
            if (!cmd[3]) case (cmd[2:0])
              3'd1: tgs <= R[1]; 3'd2: mat <= R[1]; 3'd3: pat <= R[1]; 3'd4: dor <= R[1]; 3'd7: ror <= R[1];
              default: ;
            endcase
            else case (cmd[2:0])
              3'd0: R[1] <= charset[{R[6][5] ? 1'b1 : 1'b0, R[7][6], R[6][4:0], R[7][5:0]} & 13'h1FFF];
              3'd1: R[1] <= tgs; 3'd2: R[1] <= mat; 3'd3: R[1] <= pat; 3'd4: R[1] <= dor; 3'd7: R[1] <= ror;
              default: ;
            endcase
            st[6:4] <= 3'b000;
          end
          if (cmd == 8'hB0) begin R[6] <= incy(R[6]); st[6:4] <= 3'b000; end
          if (cmd[7:4] >= 4'hD) st[6:4] <= 3'b000;
          mv_i <= 2'd0;
          xs <= X_IDLE;
        end
        default: xs <= X_IDLE;
      endcase
      if (xs == X_IDLE && hold == 0) busy <= 1'b0;
    end
  end

  // ---------------------------------------------------------------- blink (0.5 s)
  logic [24:0] bt;
  always_ff @(posedge clk) begin
    if (bt == 25'd23_999_999) begin bt <= 0; blink <= blink + 2'd3; end
    else bt <= bt + 25'd1;
  end

  // ---------------------------------------------------------------- raster
  logic [2:0] cediv;
  logic [8:0] hc;      // 0..511
  logic [8:0] vc;      // 0..311
  always_ff @(posedge clk) begin
    ce_pix <= 1'b0;
    if (cediv == 3'd5) begin cediv <= 0; ce_pix <= 1'b1; end else cediv <= cediv + 3'd1;
  end
  always_ff @(posedge clk) if (ce_pix) begin
    if (hc == 9'd511) begin
      hc <= 0;
      vc <= (vc == 9'd311) ? 9'd0 : vc + 9'd1;
    end else hc <= hc + 9'd1;
  end
  assign hblank = hc >= 9'd336;
  assign hsync  = hc >= 9'd368 && hc < 9'd406;     // 4.75 µs
  assign vblank = vc >= 9'd270;
  assign vsync  = vc >= 9'd286 && vc < 9'd289;

  // vsync status flag and the interrupt
  logic [12:0] irq_t;
  always_ff @(posedge clk) begin
    if (ce_pix && hc == 0 && vc == 9'd0) irq_t <= 13'd4800;
    else if (irq_t != 0) irq_t <= irq_t - 13'd1;
  end
  assign irq_n = irq_t == 0;

  // ---------------------------------------------------------------- next-line fetch (during hblank)
  // line buffer: pattern, background, foreground per cell
  logic [7:0] lb_pat [0:39];
  logic [2:0] lb_c0  [0:39];
  logic [2:0] lb_c1  [0:39];

  // next line
  wire [8:0] vn = (vc == 9'd311) ? 9'd0 : vc + 9'd1;
  logic [4:0] f_row;   // 0..24 text row of the next line
  logic [3:0] f_sl;    // slice 0..9
  logic       f_text;
  always_comb begin
    logic [8:0] t;
    t = vn - 9'd10;
    f_text = vn >= 9'd10 && vn < 9'd260;
    f_row = 5'(t / 10);
    f_sl  = 4'(t % 10);
  end

  wire        mode16 = pat[7];
  wire [13:0] blk = 14'({ror[7], ror[6] | ror[5], 2'b00} * 14'h0800);
  logic [4:0] jrow;
  always_comb begin
    logic [5:0] j;
    j = (f_row == 0) ? {5'd0, tgs[5]} : ({1'b0, ror[4:0]} + {1'b0, f_row} - 6'd1);
    if (j > 31) j = j - 6'd24;
    jrow = j[4:0];
  end

  typedef enum logic [2:0] {F_IDLE, F_A, F_B, F_C, F_CH, F_PIX} fs_t;
  fs_t        fs;
  logic [5:0] fx;
  logic [7:0] fa, fb, fc;
  logic [13:0] pb;
  logic [7:0]  pb_q;
  logic        pb_cs;   // 1 = char ROM
  logic [12:0] pc;
  logic [7:0]  pc_q;
  always_ff @(posedge clk) begin
    pb_q <= vram[pb];
    pc_q <= charset[pc];
  end
  logic [2:0] l_c0;
  logic       l_m, l_i, l_u;
  wire [13:0] ib = blk + {3'd0, jrow, fx};

  // per-cell decode (combinational on fa, fb, fc)
  logic [3:0]  d_type;
  logic [6:0]  d_ch;
  logic [2:0]  d_c0, d_c1;
  logic        d_ins, d_fl, d_con, d_neg, d_ul;
  logic [2:0]  n_c0; logic n_m, n_i, n_u;
  always_comb begin
    n_c0 = l_c0; n_m = l_m; n_i = l_i; n_u = l_u;
    if (fx == 0) begin n_c0 = 0; n_m = 0; n_i = 0; n_u = 0; end
    if (mode16) begin
      // fa = byte at block + iblock (R1 of KRG), fb = + 0x800 (R2)
      d_type = {fb[7], 1'b0, fa[7], 1'b0};
      d_ch   = fb[6:0];
      if (fb[7:5] == 3'b100) begin
        d_type = 0; d_ch = 7'd127; n_m = fb[0]; n_i = fb[1]; n_u = fb[2];
      end
      if (fa[7]) n_c0 = fa[6:4];
      d_c0 = n_c0; d_c1 = fa[2:0]; d_ins = n_i; d_fl = fa[3]; d_con = n_m;
      d_neg = fa[7] ? 1'b0 : fa[6]; d_ul = n_u;
    end else begin
      // fa = c (char), fb = b, fc = a
      d_type = fb[7:4]; d_ch = fa[6:0];
      d_c0 = fc[2:0]; d_c1 = fc[6:4]; d_ins = fb[0]; d_fl = fc[3]; d_con = fb[2]; d_neg = fc[7];
      d_ul = (((d_type & 4'h6) == 0) || ((d_type & 4'hC) == 4'h4)) ? fb[4] : 1'b0;
      if (fb[7:6] == 2'b11) begin d_type = 4'd12; end   // quadrichrome: not drawn
    end
  end
  wire [10:0] d_addr = {d_ch[6:2], 6'd0} + {9'd0, d_ch[1:0]} + {5'd0, f_sl, 2'b00};
  wire [13:0] ram_base = (d_type[1] == 1'b0) ? {dor[2:0], 11'd0} : ({dor[5:4], 12'd0} + (d_type[0] ? 14'h0800 : 14'h0000));

  // colours (MAME makecolors)
  logic [5:0] cy; logic [5:0] cx_; logic cursor;
  always_comb begin
    logic [4:0] i;
    i = R[6][4:0]; if (i < 8) i = i & 5'd1;
    cursor = {jrow, fx} == {i, R[7][5:0]};
  end
  logic [2:0] o_c0, o_c1; logic o_ul;
  always_comb begin
    logic [2:0] c0, c1, t, m; logic cul;
    c0 = d_c0; c1 = d_c1; m = 0; cul = 0;
    if (d_neg) begin t = c1; c1 = c0; c0 = t; end
    if (cursor) case (mat[6:4])
      3'b100: m = 3'd7;
      3'b101: cul = 1;
      3'b110: if (blink[0]) m = 3'd7;
      3'b111: if (blink[0]) cul = 1;
      default: ;
    endcase
    case (pat[5:4])
      2'b00: if (d_ins) begin c0 = 0; c1 = c1 ^ m; end else begin c0 = 0; c1 = 0; end
      2'b01: if (d_ins) begin c0 = c0 ^ m; c1 = c1 ^ m; end else begin c0 = 0; c1 = 0; end
      default: begin c0 = c0 ^ m; c1 = c1 ^ m; end
    endcase
    if ((d_fl && pat[6] && d_neg == blink[1]) || (d_con && pat[3])) c1 = c0;
    o_c0 = c0; o_c1 = c1;
    o_ul = d_ul | (cul & !((d_type[2:0] == 3'd2) || (d_type[2:0] == 3'd3)));
  end

  always_ff @(posedge clk) begin
    case (fs)
      F_IDLE: if (ce_pix && hc == 9'd340 && f_text) begin fx <= 0; fs <= F_A; pb <= blk + {3'd0, jrow, 6'd0}; end
      F_A: begin pb <= ib + 14'h0800; fs <= F_B; end
      F_B: begin fa <= pb_q; pb <= ib + 14'h1000; fs <= F_C; end
      F_C: begin fb <= pb_q; fs <= F_CH; end
      F_CH: begin
        fc <= pb_q;
        // char slice address
        pc <= {d_type[1:0], d_addr};
        pb <= (d_type[3:2] == 2'b10) ? ram_base + {3'd0, d_addr} : {3'd0, d_addr};
        fs <= F_PIX;
      end
      F_PIX: begin
        // fa/fb/fc are stable here, decode is valid; pick the slice
        lb_pat[fx] <= (f_sl == 4'd9 && o_ul) ? 8'hFF :
                      (d_type[3:2] == 2'b00 || d_type[3:2] == 2'b01) ? pc_q :
                      (d_type[3:2] == 2'b10) ? pb_q : (d_type == 4'd12 ? 8'h00 : pb_q);
        lb_c0[fx] <= o_c0; lb_c1[fx] <= o_c1;
        l_c0 <= n_c0; l_m <= n_m; l_i <= n_i; l_u <= n_u;
        if (fx == 6'd39) fs <= F_IDLE;
        else begin fx <= fx + 6'd1; pb <= blk + {3'd0, jrow, fx + 6'd1}; fs <= F_A; end
      end
      default: fs <= F_IDLE;
    endcase
  end

  // ---------------------------------------------------------------- pixels
  wire [5:0] cel = hc[8:3];               // 0..41 in the active part
  wire [2:0] px   = hc[2:0];
  logic [4:0] row_now;
  logic       text_now;
  always_comb begin
    logic [8:0] t;
    t = vc - 9'd10;
    text_now = vc >= 9'd10 && vc < 9'd260;
    row_now = 5'(t / 10);
  end
  wire row_en = (row_now == 0) ? pat[0] : (row_now < 12) ? pat[1] : pat[2];
  logic [2:0] col;
  always_ff @(posedge clk) if (ce_pix) begin
    if (text_now && row_en && cel >= 1 && cel <= 40)
      col <= lb_pat[cel - 1][px] ? lb_c1[cel - 1] : lb_c0[cel - 1];
    else
      col <= mat[2:0];
  end
  assign r = col[0];
  assign g = col[1];
  assign b = col[2];

endmodule
