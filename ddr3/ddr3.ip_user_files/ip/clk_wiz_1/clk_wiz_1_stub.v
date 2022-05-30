// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2019.2 (win64) Build 2708876 Wed Nov  6 21:40:23 MST 2019
// Date        : Mon May 30 17:19:10 2022
// Host        : Aeternitas running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               C:/Users/jarib/OneDrive/work/mag/vivado-proj/art_ip/clk_wiz_1/clk_wiz_1_stub.v
// Design      : clk_wiz_1
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7s50csga324-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
module clk_wiz_1(clk_out1_ddr, clk_out2_ddr_90, clk_out3_ref, 
  clk_out4_div, reset, locked, clk_in1)
/* synthesis syn_black_box black_box_pad_pin="clk_out1_ddr,clk_out2_ddr_90,clk_out3_ref,clk_out4_div,reset,locked,clk_in1" */;
  output clk_out1_ddr;
  output clk_out2_ddr_90;
  output clk_out3_ref;
  output clk_out4_div;
  input reset;
  output locked;
  input clk_in1;
endmodule
