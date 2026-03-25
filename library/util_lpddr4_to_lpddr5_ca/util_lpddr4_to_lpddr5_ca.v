// ***************************************************************************
// ***************************************************************************
// Copyright 2014 - 2024 (c) Analog Devices, Inc. All rights reserved.
//
// In this HDL repository, there are many different and unique modules, consisting
// of various HDL (Verilog or VHDL) components. The individual modules are
// developed independently, and may be accompanied by separate and unique license
// terms.
//
// The user should read each of these license terms, and understand the
// freedoms and responsibilities that he or she has by using this source/core.
//
// This core is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE.
//
// Redistribution and use of source or resulting binaries, with or without modification
// of this file, are permitted under one of the following two license terms:
//
//   1. The GNU General Public License version 2 as published by the
//      Free Software Foundation, which can be found in the top level directory
//      of this repository (LICENSE_GPL2), and also online at:
//      <https://www.gnu.org/licenses/old-licenses/gpl-2.0.html>
//
// OR
//
//   2. An ADI specific BSD license, which can be found in the top level directory
//      of this repository (LICENSE_ADIBSD), and also on-line at:
//      https://github.com/analogdevicesinc/hdl/blob/master/LICENSE_ADIBSD
//      This will allow to generate bit files and not release the source code,
//      as long as it attaches to an ADI device.
//
// ***************************************************************************
// ***************************************************************************
//
// LPDDR4 to LPDDR5 Command/Address (CA) Bus Converter
//
// Converts the 2-cycle, 6-bit LPDDR4 CA bus to the 2-cycle, 7-bit LPDDR5 CA
// bus.  Both protocols encode one command across two consecutive clock cycles
// (2N mode).  The converter captures both LPDDR4 cycles, decodes the command
// and address fields, then re-encodes them in the LPDDR5 format and replays
// the result across two LPDDR5 output cycles with a fixed 2-cycle pipeline
// latency relative to the end of the LPDDR4 command.
//
// -----------------------------------------------------------------------
// LPDDR4 Command Encoding (JESD209-4, 2N mode, CA[5:0])
// -----------------------------------------------------------------------
//   Cycle | CA5  CA4  CA3  CA2  CA1  CA0
//  -------+-----------------------------
//   ACT C0|  H    H   BA2  R17  R16  R15
//   ACT C1| R14  R13  R12  R11  R10   R9
//  -------+-----------------------------
//   RD  C0|  H    L    H   C9   AP   BL
//   RD  C1| BA2  BA1  BA0   C8   C7   C6
//  -------+-----------------------------
//   WR  C0|  H    L    L   C9   AP   BL   (BL=0: BL16, BL=1: BL32)
//   WR  C1| BA2  BA1  BA0   C8   C7   C6
//  -------+-----------------------------
//   MWR C0|  H    L    L   C9   AP    H   (CA0=H distinguishes MWR from WR-BL16)
//   MWR C1| BA2  BA1  BA0   C8   C7   C6
//  -------+-----------------------------
//   PRE C0|  L    L    H    L    L   AB
//   PRE C1| BA2  BA1  BA0   L    L    L   (AB=H -> all-bank)
//  -------+-----------------------------
//   REF C0|  L    L    L    L    L    L
//   REF C1|  L    L    L    L    L    L
//  -------+-----------------------------
//   SRE C0|  L    L    L    L    L    H
//   SRE C1|  L    L    L    L    L    L
//  -------+-----------------------------
//   PDE C0|  L    L    L    L    H    L
//   PDE C1|  L    L    L    L    L    L
//  -------+-----------------------------
//   MRS C0|  L    H   MA5  MA4  MA3  MA2
//   MRS C1| OP7  OP6  OP5  OP4  OP3  OP2
//
// Note: In LPDDR4, ACT carries only BA2 on the CA bus; BA[1:0] are assumed
//       to be 0 in this converter (add dedicated BA[1:0] ports if required).
//       MWR is identified by CA0=H when CA[5:3]=3'b100 (overlaps with
//       WR-BL32; for strict WR-BL32 support, signal via a sideband port).
//
// -----------------------------------------------------------------------
// LPDDR5 Command Encoding (JESD209-5, 2N mode, CA[6:0])
// -----------------------------------------------------------------------
//   Cycle | CA6  CA5  CA4  CA3  CA2  CA1  CA0
//  -------+------------------------------------
//   ACT C0|  H    H   R16  R15  R14  R13  R12
//   ACT C1| R11  R10   R9   R8  BG1  BG0  BA0
//  -------+------------------------------------
//   RD  C0|  H    L    H   C9   AP   BL    L
//   RD  C1| BG1  BG0  BA0   C8   C7   C6   C5
//  -------+------------------------------------
//   WR  C0|  H    L    L   C9   AP   BL    L
//   WR  C1| BG1  BG0  BA0   C8   C7   C6   C5
//  -------+------------------------------------
//   MWR C0|  H    L    L   C9   AP    H    L
//   MWR C1| BG1  BG0  BA0   C8   C7   C6   C5
//  -------+------------------------------------
//   PRE C0|  L    L    H    L    L    L   AB
//   PRE C1| BG1  BG0  BA0   L    L    L    L  (AB=H -> all-bank)
//  -------+------------------------------------
//   REF C0|  L    L    L    L    L    L    L
//   REF C1|  L    L    L    L    L    L    L
//  -------+------------------------------------
//   SRE C0|  L    L    L    L    L    L    H
//   SRE C1|  L    L    L    L    L    L    L
//  -------+------------------------------------
//   PDE C0|  L    L    L    L    L    H    L
//   PDE C1|  L    L    L    L    L    L    L
//  -------+------------------------------------
//   MRS C0|  L    H    L   MA5  MA4  MA3  MA2
//   MRS C1| OP7  OP6  OP5  OP4  OP3  OP2   L
//
// -----------------------------------------------------------------------
// Bank Address Mapping  (both devices have 8 banks total)
// -----------------------------------------------------------------------
//   LPDDR4 BA[2:0]  ->  LPDDR5 BG[1:0] + BA[0]
//     BA[2] -> BG[1]
//     BA[1] -> BG[0]
//     BA[0] -> BA[0]
//
// -----------------------------------------------------------------------
// Pipeline Timing (2-cycle command latency after LPDDR4 C1)
// -----------------------------------------------------------------------
//   Cycle N  : LPDDR4 C0 sampled (lpddr4_cs_n=0)
//   Cycle N+1: LPDDR4 C1 sampled; decoded LPDDR5 words registered
//   Cycle N+2: LPDDR5 C0 driven  (lpddr5_cs_n=0)
//   Cycle N+3: LPDDR5 C1 driven
//
// ***************************************************************************

`timescale 1ns/1ps

module util_lpddr4_to_lpddr5_ca (
  input             clk,          // Command clock (LPDDR4 CK, rising-edge active)
  input             reset_n,      // Active-low synchronous reset

  // LPDDR4 input interface
  input             lpddr4_cs_n,  // LPDDR4 chip-select (active-low; H = NOP)
  input  [5:0]      lpddr4_ca,    // LPDDR4 command/address bus

  // LPDDR5 output interface
  output reg        lpddr5_cs_n,  // LPDDR5 chip-select (active-low; H = NOP)
  output reg [6:0]  lpddr5_ca     // LPDDR5 command/address bus
);

  // -----------------------------------------------------------------------
  // FSM state encoding
  // -----------------------------------------------------------------------
  localparam [1:0] S_IDLE   = 2'd0;  // No command, LPDDR5 output = NOP
  localparam [1:0] S_LP4_C1 = 2'd1;  // LPDDR4 C0 captured, waiting for C1
  localparam [1:0] S_LP5_C0 = 2'd2;  // Driving LPDDR5 C0
  localparam [1:0] S_LP5_C1 = 2'd3;  // Driving LPDDR5 C1

  // -----------------------------------------------------------------------
  // Registers
  // -----------------------------------------------------------------------
  reg [1:0] state;
  reg [5:0] lp4_c0_r;   // Captured LPDDR4 C0
  reg [6:0] lp5_c0_r;   // Registered LPDDR5 C0 word
  reg [6:0] lp5_c1_r;   // Registered LPDDR5 C1 word
  reg       lp5_cs_r;   // Registered CS_n for LPDDR5 output phase

  // -----------------------------------------------------------------------
  // Command-type decode (combinatorial from stored C0)
  // -----------------------------------------------------------------------
  // Priority-encoded: ACT > RD > WR/MWR > PRE > REF > SRE > PDE > MRS > NOP
  wire is_act = (lp4_c0_r[5:4] == 2'b11);
  wire is_rd  = (lp4_c0_r[5:3] == 3'b101);
  // WR/MWR share CA[5:3]=3'b100; CA0=L => WR, CA0=H => MWR
  wire is_wrx = (lp4_c0_r[5:3] == 3'b100);
  wire is_mwr = is_wrx & lp4_c0_r[0];
  wire is_wr  = is_wrx & ~lp4_c0_r[0];
  wire is_pre = (lp4_c0_r[5:3] == 3'b001);
  wire is_ref = (lp4_c0_r[5:0] == 6'b000000);
  wire is_sre = (lp4_c0_r[5:0] == 6'b000001);
  wire is_pde = (lp4_c0_r[5:0] == 6'b000010);
  wire is_mrs = (lp4_c0_r[5:4] == 2'b01);

  // -----------------------------------------------------------------------
  // Combinatorial LPDDR5 word computation
  //
  // All inputs are stable in state S_LP4_C1:
  //   lp4_c0_r   : LPDDR4 C0 (registered in S_IDLE -> S_LP4_C1 transition)
  //   lpddr4_ca  : LPDDR4 C1 (driven by controller this cycle)
  //
  // Intermediate address field wires (named for readability):
  // -----------------------------------------------------------------------

  // --- ACT fields ---
  // LPDDR4 C0: {H,H, BA2, R17, R16, R15}
  // LPDDR4 C1: {R14, R13, R12, R11, R10, R9}
  wire       act_ba2  = lp4_c0_r[3];
  wire [8:0] act_row  = {lp4_c0_r[2], lp4_c0_r[1], lp4_c0_r[0],   // R17..R15
                          lpddr4_ca[5], lpddr4_ca[4], lpddr4_ca[3], // R14..R12
                          lpddr4_ca[2], lpddr4_ca[1], lpddr4_ca[0]};// R11..R9
  // LPDDR5 ACT: C0={H,H, R16,R15,R14,R13,R12}  C1={R11,R10,R9,R8, BG1,BG0,BA0}
  // act_row[8]=R17 (not carried to LP5 in this mapping; R[16:8] mapped)
  // Map: act_row[7]=R16, act_row[6]=R15, act_row[5]=R14, act_row[4]=R13,
  //      act_row[3]=R12, act_row[2]=R11, act_row[1]=R10, act_row[0]=R9
  // R8 = 0 (not available from LPDDR4 CA encoding in this command format)
  // BG1=act_ba2, BG0=0, BA0=0  (BA[1:0] not carried in LPDDR4 ACT CA bus)
  wire [6:0] lp5_c0_act = {1'b1, 1'b1,
                            act_row[7], act_row[6], act_row[5],
                            act_row[4], act_row[3]};
  wire [6:0] lp5_c1_act = {act_row[2], act_row[1], act_row[0],
                            1'b0,             // R8
                            act_ba2, 1'b0, 1'b0}; // BG1, BG0=0, BA0=0

  // --- RD/WR fields ---
  // LPDDR4 C0: {H,L,R/W, C9, AP, BL}
  // LPDDR4 C1: {BA2, BA1, BA0, C8, C7, C6}
  wire       rw_c9  = lp4_c0_r[2];
  wire       rw_ap  = lp4_c0_r[1];
  wire       rw_bl  = lp4_c0_r[0];   // BL=0:BL16  BL=1:BL32 (WR only; MWR ignores)
  wire [2:0] rw_ba  = lpddr4_ca[5:3];
  wire       rw_c8  = lpddr4_ca[2];
  wire       rw_c7  = lpddr4_ca[1];
  wire       rw_c6  = lpddr4_ca[0];
  // LPDDR5 bank group: BA[2]->BG1, BA[1]->BG0, BA[0]->BA0
  wire       rw_bg1 = rw_ba[2];
  wire       rw_bg0 = rw_ba[1];
  wire       rw_ba0 = rw_ba[0];
  // LPDDR5 RD: C0={H,L,H, C9,AP,BL,L}  C1={BG1,BG0,BA0, C8,C7,C6,C5=0}
  wire [6:0] lp5_c0_rd  = {1'b1, 1'b0, 1'b1, rw_c9, rw_ap, rw_bl, 1'b0};
  wire [6:0] lp5_c1_rd  = {rw_bg1, rw_bg0, rw_ba0, rw_c8, rw_c7, rw_c6, 1'b0};
  // LPDDR5 WR: C0={H,L,L, C9,AP,BL,L}  C1={BG1,BG0,BA0, C8,C7,C6,C5=0}
  wire [6:0] lp5_c0_wr  = {1'b1, 1'b0, 1'b0, rw_c9, rw_ap, rw_bl, 1'b0};
  wire [6:0] lp5_c1_wr  = {rw_bg1, rw_bg0, rw_ba0, rw_c8, rw_c7, rw_c6, 1'b0};
  // LPDDR5 MWR: C0={H,L,L, C9,AP,H,L}  C1={BG1,BG0,BA0, C8,C7,C6,C5=0}
  wire [6:0] lp5_c0_mwr = {1'b1, 1'b0, 1'b0, rw_c9, rw_ap, 1'b1, 1'b0};
  wire [6:0] lp5_c1_mwr = {rw_bg1, rw_bg0, rw_ba0, rw_c8, rw_c7, rw_c6, 1'b0};

  // --- PRE fields ---
  // LPDDR4 C0: {L,L,H, L,L, AB}   CA0=AB (0:single bank, 1:all banks)
  // LPDDR4 C1: {BA2,BA1,BA0, L,L,L}
  wire       pre_ab  = lp4_c0_r[0];
  wire [2:0] pre_ba  = lpddr4_ca[5:3];
  wire       pre_bg1 = pre_ba[2];
  wire       pre_bg0 = pre_ba[1];
  wire       pre_ba0 = pre_ba[0];
  // LPDDR5 PRE: C0={L,L,H, L,L,L,AB}  C1={BG1,BG0,BA0, L,L,L,L}
  wire [6:0] lp5_c0_pre = {1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, pre_ab};
  wire [6:0] lp5_c1_pre = {pre_bg1, pre_bg0, pre_ba0, 4'b0000};

  // --- MRS fields ---
  // LPDDR4 C0: {L,H, MA5,MA4,MA3,MA2}
  // LPDDR4 C1: {OP7,OP6,OP5,OP4,OP3,OP2}
  wire [5:2] mrs_ma = lp4_c0_r[3:0];   // MA[5:2]; MA[1:0] always 0 in LPDDR4
  wire [7:2] mrs_op = lpddr4_ca[5:0];  // OP[7:2]; OP[1:0] = 0
  // LPDDR5 MRS: C0={L,H,L, MA5,MA4,MA3,MA2}  C1={OP7,OP6,OP5,OP4,OP3,OP2,L}
  wire [6:0] lp5_c0_mrs = {1'b0, 1'b1, 1'b0,
                            mrs_ma[5], mrs_ma[4], mrs_ma[3], mrs_ma[2]};
  wire [6:0] lp5_c1_mrs = {mrs_op[7], mrs_op[6], mrs_op[5],
                            mrs_op[4], mrs_op[3], mrs_op[2], 1'b0};

  // -----------------------------------------------------------------------
  // Main FSM
  // -----------------------------------------------------------------------
  always @(posedge clk) begin
    if (!reset_n) begin
      state       <= S_IDLE;
      lp4_c0_r    <= 6'd0;
      lp5_c0_r    <= 7'd0;
      lp5_c1_r    <= 7'd0;
      lp5_cs_r    <= 1'b1;
      lpddr5_cs_n <= 1'b1;
      lpddr5_ca   <= 7'd0;
    end else begin
      case (state)

        // -----------------------------------------------------------------
        // S_IDLE: no command active; drive NOP on LPDDR5 output
        // -----------------------------------------------------------------
        S_IDLE: begin
          lpddr5_cs_n <= 1'b1;
          lpddr5_ca   <= 7'd0;
          if (!lpddr4_cs_n) begin
            lp4_c0_r <= lpddr4_ca;
            state    <= S_LP4_C1;
          end
        end

        // -----------------------------------------------------------------
        // S_LP4_C1: C0 stored in lp4_c0_r; lpddr4_ca carries C1 this cycle.
        //
        //   Decode both cycles -> compute LPDDR5 C0/C1 words from the
        //   combinatorial wires above (which see the correct C0/C1 values).
        //   Register the result; it will be driven in the next two states.
        // -----------------------------------------------------------------
        S_LP4_C1: begin
          lpddr5_cs_n <= 1'b1;  // Still NOP on output while decoding
          lpddr5_ca   <= 7'd0;
          state       <= S_LP5_C0;

          if (is_act) begin
            lp5_c0_r <= lp5_c0_act;
            lp5_c1_r <= lp5_c1_act;
            lp5_cs_r <= 1'b0;
          end else if (is_rd) begin
            lp5_c0_r <= lp5_c0_rd;
            lp5_c1_r <= lp5_c1_rd;
            lp5_cs_r <= 1'b0;
          end else if (is_mwr) begin
            lp5_c0_r <= lp5_c0_mwr;
            lp5_c1_r <= lp5_c1_mwr;
            lp5_cs_r <= 1'b0;
          end else if (is_wr) begin
            lp5_c0_r <= lp5_c0_wr;
            lp5_c1_r <= lp5_c1_wr;
            lp5_cs_r <= 1'b0;
          end else if (is_pre) begin
            lp5_c0_r <= lp5_c0_pre;
            lp5_c1_r <= lp5_c1_pre;
            lp5_cs_r <= 1'b0;
          end else if (is_ref) begin
            lp5_c0_r <= 7'b0000000;
            lp5_c1_r <= 7'b0000000;
            lp5_cs_r <= 1'b0;
          end else if (is_sre) begin
            lp5_c0_r <= 7'b0000001;
            lp5_c1_r <= 7'b0000000;
            lp5_cs_r <= 1'b0;
          end else if (is_pde) begin
            lp5_c0_r <= 7'b0000010;
            lp5_c1_r <= 7'b0000000;
            lp5_cs_r <= 1'b0;
          end else if (is_mrs) begin
            lp5_c0_r <= lp5_c0_mrs;
            lp5_c1_r <= lp5_c1_mrs;
            lp5_cs_r <= 1'b0;
          end else begin
            // NOP / unrecognised: pass through as NOP
            lp5_c0_r <= 7'd0;
            lp5_c1_r <= 7'd0;
            lp5_cs_r <= 1'b1;
          end
        end

        // -----------------------------------------------------------------
        // S_LP5_C0: drive first LPDDR5 command cycle
        // -----------------------------------------------------------------
        S_LP5_C0: begin
          lpddr5_cs_n <= lp5_cs_r;
          lpddr5_ca   <= lp5_c0_r;
          state       <= S_LP5_C1;
        end

        // -----------------------------------------------------------------
        // S_LP5_C1: drive second LPDDR5 command cycle
        // -----------------------------------------------------------------
        S_LP5_C1: begin
          lpddr5_cs_n <= lp5_cs_r;
          lpddr5_ca   <= lp5_c1_r;
          // Accept a back-to-back LPDDR4 command if one arrives
          if (!lpddr4_cs_n) begin
            lp4_c0_r <= lpddr4_ca;
            state    <= S_LP4_C1;
          end else begin
            state    <= S_IDLE;
          end
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
