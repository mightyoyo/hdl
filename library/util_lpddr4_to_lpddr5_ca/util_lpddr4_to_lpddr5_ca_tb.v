// ***************************************************************************
// ***************************************************************************
// Copyright 2014 - 2024 (c) Analog Devices, Inc. All rights reserved.
//
// Testbench for util_lpddr4_to_lpddr5_ca
//
// Exercises ACT, RD, WR, MWR, PRE (all-bank), REF, SRE, PDE, MRS and
// verifies the LPDDR5 output encoding for each.
//
// Input driving / output sampling convention
// ------------------------------------------
//   Inputs are driven on the negedge BEFORE the target posedge, so that
//   setup time is satisfied.  Outputs are sampled 1 ns after the posedge
//   at which the DUT registers the new value.
//
// Pipeline timing (see RTL header)
//   Posedge N  : DUT latches LPDDR4 C0  (state: S_IDLE -> S_LP4_C1)
//   Posedge N+1: DUT latches LPDDR4 C1  (state: S_LP4_C1 -> S_LP5_C0)
//                LP5 C0/C1 words are registered at this posedge
//   Posedge N+2: LP5 C0 driven           (state: S_LP5_C0 -> S_LP5_C1)
//   Posedge N+3: LP5 C1 driven           (state: S_LP5_C1 -> S_IDLE)
//
// ***************************************************************************

`timescale 1ns/1ps

module util_lpddr4_to_lpddr5_ca_tb;

  // -------------------------------------------------------------------------
  // Clock & reset
  // -------------------------------------------------------------------------
  localparam CLK_HALF = 5;  // 100 MHz -> 10 ns period

  reg clk     = 0;
  reg reset_n = 0;

  always #CLK_HALF clk = ~clk;

  // -------------------------------------------------------------------------
  // DUT
  // -------------------------------------------------------------------------
  reg        lpddr4_cs_n = 1'b1;
  reg  [5:0] lpddr4_ca   = 6'd0;
  wire       lpddr5_cs_n;
  wire [6:0] lpddr5_ca;

  util_lpddr4_to_lpddr5_ca dut (
    .clk         (clk),
    .reset_n     (reset_n),
    .lpddr4_cs_n (lpddr4_cs_n),
    .lpddr4_ca   (lpddr4_ca),
    .lpddr5_cs_n (lpddr5_cs_n),
    .lpddr5_ca   (lpddr5_ca)
  );

  // -------------------------------------------------------------------------
  // Test infrastructure
  // -------------------------------------------------------------------------
  integer pass_cnt = 0;
  integer fail_cnt = 0;

  // Drive an LPDDR4 command (C0 then C1) using negedge-based timing so that
  // the values are stable before the next rising edge.
  //
  //   negedge before posedge N  : assert cs_n=0, present C0
  //   negedge before posedge N+1: present C1
  //   negedge before posedge N+2: deassert cs_n
  //
  task drive_lpddr4_cmd;
    input [5:0] c0;
    input [5:0] c1;
    begin
      @(negedge clk);           // negedge before posedge N
      lpddr4_cs_n = 1'b0;
      lpddr4_ca   = c0;
      @(negedge clk);           // negedge before posedge N+1
      lpddr4_ca   = c1;
      @(negedge clk);           // negedge before posedge N+2
      lpddr4_cs_n = 1'b1;
      lpddr4_ca   = 6'd0;
    end
  endtask

  // After drive_lpddr4_cmd completes, the last negedge is the one before
  // posedge N+2.  At posedge N+2 the state machine moves to S_LP5_C0 and
  // registers the LP5 words.  The LP5 C0 output appears AFTER posedge N+2,
  // i.e. it is first visible at the subsequent negedge / sample point.
  //
  // Calling sequence:
  //   drive_lpddr4_cmd(...)    <- drives C0 at posedge N, C1 at posedge N+1
  //   check_lpddr5_output(...) <- samples after posedge N+2 (LP5 C0)
  //                                and after posedge N+3 (LP5 C1)
  //
  // We advance by two posedges after drive returns to reach the LP5 output.
  task check_lpddr5_output;
    input [6:0] exp_c0;
    input [6:0] exp_c1;
    input [8*16-1:0] label;
    reg [6:0] got_c0;
    reg [6:0] got_c1;
    reg       got_cs0;
    reg       got_cs1;
    begin
      // Advance to posedge N+2 and sample LP5 C0
      @(posedge clk); #1;
      got_cs0 = lpddr5_cs_n;
      got_c0  = lpddr5_ca;

      // Advance to posedge N+3 and sample LP5 C1
      @(posedge clk); #1;
      got_cs1 = lpddr5_cs_n;
      got_c1  = lpddr5_ca;

      // Evaluate
      if (got_cs0 !== 1'b0 || got_cs1 !== 1'b0) begin
        $display("FAIL [%-16s]  CS_n: C0=%b C1=%b (expected both 0)",
                 label, got_cs0, got_cs1);
        fail_cnt = fail_cnt + 1;
      end else if (got_c0 !== exp_c0) begin
        $display("FAIL [%-16s]  LP5 C0: got %07b  exp %07b",
                 label, got_c0, exp_c0);
        fail_cnt = fail_cnt + 1;
      end else if (got_c1 !== exp_c1) begin
        $display("FAIL [%-16s]  LP5 C1: got %07b  exp %07b",
                 label, got_c1, exp_c1);
        fail_cnt = fail_cnt + 1;
      end else begin
        $display("PASS [%-16s]  LP5 C0=%07b  C1=%07b", label, got_c0, got_c1);
        pass_cnt = pass_cnt + 1;
      end

      // Return to idle (1 extra negedge gap between tests)
      @(negedge clk);
    end
  endtask

  // -------------------------------------------------------------------------
  // Stimulus
  // -------------------------------------------------------------------------
  initial begin
    // Release reset
    repeat (4) @(posedge clk);
    @(negedge clk); reset_n = 1'b1;
    repeat (2) @(posedge clk);

    // =======================================================================
    // Test 1 – ACT
    //   LPDDR4:
    //     C0 = {H,H, BA2=0, R17=1, R16=0, R15=1} = 6'b110101
    //     C1 = {R14=1, R13=0, R12=1, R11=0, R10=0, R9=1} = 6'b101001
    //
    //   act_row[8:0] = {R17,R16,R15, R14,R13,R12, R11,R10,R9}
    //                = {1,  0,  1,   1,  0,  1,   0,  0,  1}  = 9'b101101001
    //   act_row[7]=R16=0  [6]=R15=1  [5]=R14=1  [4]=R13=0  [3]=R12=1
    //   act_row[2]=R11=0  [1]=R10=0  [0]=R9=1
    //   act_ba2 = lp4_c0_r[3] = C0[3] = 0
    //
    //   LPDDR5 C0 = {1,1, R16,R15,R14,R13,R12}
    //             = {1,1, 0,  1,  1,  0,  1  } = 7'b1101101
    //   LPDDR5 C1 = {R11,R10,R9, R8=0, BG1=act_ba2=0, BG0=0, BA0=0}
    //             = {0,  0,  1,  0,    0,             0,    0} = 7'b0010000
    // =======================================================================
    drive_lpddr4_cmd(6'b110101, 6'b101001);
    check_lpddr5_output(7'b1101101, 7'b0010000, "ACT");

    // =======================================================================
    // Test 2 – RD
    //   LPDDR4:
    //     C0 = {H,L,H, C9=1, AP=1, BL=0} = 6'b101110
    //     C1 = {BA2=1,BA1=0,BA0=1, C8=0,C7=1,C6=0} = 6'b101010
    //
    //   rw_bg1=BA2=1, rw_bg0=BA1=0, rw_ba0=BA0=1
    //   rw_c9=1, rw_ap=1, rw_bl=0
    //   rw_c8=0, rw_c7=1, rw_c6=0
    //
    //   LPDDR5 C0 = {1,0,1, C9=1, AP=1, BL=0, L}  = 7'b1011100
    //   LPDDR5 C1 = {BG1=1,BG0=0,BA0=1, C8=0,C7=1,C6=0,C5=0} = 7'b1010100
    // =======================================================================
    drive_lpddr4_cmd(6'b101110, 6'b101010);
    check_lpddr5_output(7'b1011100, 7'b1010100, "RD");

    // =======================================================================
    // Test 3 – WR  (BL=0 to avoid CA0=1 which would decode as MWR)
    //   LPDDR4:
    //     C0 = {H,L,L, C9=0, AP=0, BL=0} = 6'b100000
    //     C1 = {BA2=0,BA1=1,BA0=1, C8=1,C7=1,C6=0} = 6'b011110
    //
    //   rw_bg1=0, rw_bg0=1, rw_ba0=1
    //   rw_c9=0, rw_ap=0, rw_bl=0
    //   rw_c8=1, rw_c7=1, rw_c6=0
    //
    //   LPDDR5 C0 = {1,0,0, 0,0,0, L} = 7'b1000000
    //   LPDDR5 C1 = {0,1,1, 1,1,0,0}  = 7'b0111100
    // =======================================================================
    drive_lpddr4_cmd(6'b100000, 6'b011110);
    check_lpddr5_output(7'b1000000, 7'b0111100, "WR");

    // =======================================================================
    // Test 4 – MWR  (CA0=H in C0 marks masked write)
    //   LPDDR4:
    //     C0 = {H,L,L, C9=1, AP=1, H} = 6'b100111   (CA0=H => MWR)
    //     C1 = {BA2=1,BA1=1,BA0=0, C8=1,C7=0,C6=0} = 6'b110100
    //
    //   rw_bg1=1, rw_bg0=1, rw_ba0=0
    //   rw_c9=1, rw_ap=1
    //   rw_c8=1, rw_c7=0, rw_c6=0
    //
    //   LPDDR5 C0 = {1,0,0, C9=1, AP=1, H, L} = 7'b1001110
    //   LPDDR5 C1 = {1,1,0, 1,0,0,0}           = 7'b1101000
    // =======================================================================
    drive_lpddr4_cmd(6'b100111, 6'b110100);
    check_lpddr5_output(7'b1001110, 7'b1101000, "MWR");

    // =======================================================================
    // Test 5 – PRE all-bank  (AB=1)
    //   LPDDR4:
    //     C0 = {L,L,H, L,L,AB=1} = 6'b001001
    //     C1 = {BA2,BA1,BA0, L,L,L} = 6'b000000  (ignored when AB=1)
    //
    //   pre_ab=1
    //   pre_bg1/bg0/ba0 = 0 (from C1[5:3]=0)
    //
    //   LPDDR5 C0 = {L,L,H, L,L,L,AB=1} = 7'b0010001
    //   LPDDR5 C1 = {0,0,0, 0,0,0,0}    = 7'b0000000
    // =======================================================================
    drive_lpddr4_cmd(6'b001001, 6'b000000);
    check_lpddr5_output(7'b0010001, 7'b0000000, "PRE_ALL");

    // =======================================================================
    // Test 6 – REF
    //   LPDDR4:  C0=6'b000000  C1=6'b000000
    //   LPDDR5:  C0=7'b0000000  C1=7'b0000000
    // =======================================================================
    drive_lpddr4_cmd(6'b000000, 6'b000000);
    check_lpddr5_output(7'b0000000, 7'b0000000, "REF");

    // =======================================================================
    // Test 7 – SRE
    //   LPDDR4:  C0=6'b000001  C1=6'b000000
    //   LPDDR5:  C0=7'b0000001  C1=7'b0000000
    // =======================================================================
    drive_lpddr4_cmd(6'b000001, 6'b000000);
    check_lpddr5_output(7'b0000001, 7'b0000000, "SRE");

    // =======================================================================
    // Test 8 – PDE
    //   LPDDR4:  C0=6'b000010  C1=6'b000000
    //   LPDDR5:  C0=7'b0000010  C1=7'b0000000
    // =======================================================================
    drive_lpddr4_cmd(6'b000010, 6'b000000);
    check_lpddr5_output(7'b0000010, 7'b0000000, "PDE");

    // =======================================================================
    // Test 9 – MRS  (MA[5:2]=4'b0011 i.e. address 12; OP[7:2]=6'b100101)
    //   LPDDR4:
    //     C0 = {L,H, MA5=0,MA4=0,MA3=1,MA2=1} = 6'b010011
    //     C1 = {OP7=1,OP6=0,OP5=0,OP4=1,OP3=0,OP2=1} = 6'b100101
    //
    //   mrs_ma[5:2] = C0[3:0] = 4'b0011
    //   mrs_op[7:2] = C1[5:0] = 6'b100101
    //
    //   LPDDR5 C0 = {L,H,L, MA5=0,MA4=0,MA3=1,MA2=1} = 7'b0100011
    //   LPDDR5 C1 = {OP7=1,OP6=0,OP5=0,OP4=1,OP3=0,OP2=1, L} = 7'b1001010
    // =======================================================================
    drive_lpddr4_cmd(6'b010011, 6'b100101);
    check_lpddr5_output(7'b0100011, 7'b1001010, "MRS");

    // =======================================================================
    // Test 10 – PRE single bank  (AB=0, BA=3'b011)
    //   LPDDR4:
    //     C0 = {L,L,H, L,L,AB=0} = 6'b001000
    //     C1 = {BA2=0,BA1=1,BA0=1, L,L,L} = 6'b011000
    //
    //   pre_ab=0, pre_bg1=0, pre_bg0=1, pre_ba0=1
    //
    //   LPDDR5 C0 = {L,L,H, L,L,L,AB=0} = 7'b0010000
    //   LPDDR5 C1 = {BG1=0,BG0=1,BA0=1, L,L,L,L} = 7'b0110000
    // =======================================================================
    drive_lpddr4_cmd(6'b001000, 6'b011000);
    check_lpddr5_output(7'b0010000, 7'b0110000, "PRE_SINGLE");

    // -----------------------------------------------------------------------
    repeat (4) @(posedge clk);
    $display("-------------------------------------------");
    $display("Results:  %0d passed,  %0d failed", pass_cnt, fail_cnt);
    if (fail_cnt == 0)
      $display("ALL TESTS PASSED");
    else
      $display("SOME TESTS FAILED");
    $display("-------------------------------------------");
    $finish;
  end

  // -------------------------------------------------------------------------
  // Timeout watchdog
  // -------------------------------------------------------------------------
  initial begin
    #100000;
    $display("TIMEOUT");
    $finish;
  end

  // -------------------------------------------------------------------------
  // Waveform dump
  // -------------------------------------------------------------------------
  initial begin
    $dumpfile("util_lpddr4_to_lpddr5_ca_tb.vcd");
    $dumpvars(0, util_lpddr4_to_lpddr5_ca_tb);
  end

endmodule
