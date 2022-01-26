-- Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2019.2 (win64) Build 2708876 Wed Nov  6 21:40:23 MST 2019
-- Date        : Wed Jan 26 20:33:19 2022
-- Host        : Aeternitas running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub
--               C:/Users/jarib/OneDrive/work/mag/vivado-proj/art_ip/ila_0/ila_0_stub.vhdl
-- Design      : ila_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7s50csga324-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity ila_0 is
  Port ( 
    clk : in STD_LOGIC;
    trig_in : in STD_LOGIC;
    trig_in_ack : out STD_LOGIC;
    probe0 : in STD_LOGIC_VECTOR ( 127 downto 0 );
    probe1 : in STD_LOGIC_VECTOR ( 127 downto 0 );
    probe2 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe3 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe4 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe5 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe6 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe7 : in STD_LOGIC_VECTOR ( 27 downto 0 );
    probe8 : in STD_LOGIC_VECTOR ( 27 downto 0 );
    probe9 : in STD_LOGIC_VECTOR ( 127 downto 0 );
    probe10 : in STD_LOGIC_VECTOR ( 127 downto 0 )
  );

end ila_0;

architecture stub of ila_0 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clk,trig_in,trig_in_ack,probe0[127:0],probe1[127:0],probe2[0:0],probe3[0:0],probe4[0:0],probe5[0:0],probe6[0:0],probe7[27:0],probe8[27:0],probe9[127:0],probe10[127:0]";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "ila,Vivado 2019.2";
begin
end;
