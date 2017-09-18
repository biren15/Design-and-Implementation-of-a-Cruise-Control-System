
module cruisecontrol_DW01_inc_0 ( A, SUM );
  input [7:0] A;
  output [7:0] SUM;

  wire   [7:2] carry;

  HAX1 U1_1_6 ( .A(A[6]), .B(carry[6]), .YC(carry[7]), .YS(SUM[6]) );
  HAX1 U1_1_5 ( .A(A[5]), .B(carry[5]), .YC(carry[6]), .YS(SUM[5]) );
  HAX1 U1_1_4 ( .A(A[4]), .B(carry[4]), .YC(carry[5]), .YS(SUM[4]) );
  HAX1 U1_1_3 ( .A(A[3]), .B(carry[3]), .YC(carry[4]), .YS(SUM[3]) );
  HAX1 U1_1_2 ( .A(A[2]), .B(carry[2]), .YC(carry[3]), .YS(SUM[2]) );
  HAX1 U1_1_1 ( .A(A[1]), .B(A[0]), .YC(carry[2]), .YS(SUM[1]) );
  INVX1 U1 ( .A(A[0]), .Y(SUM[0]) );
  XOR2X1 U2 ( .A(carry[7]), .B(A[7]), .Y(SUM[7]) );
endmodule


module cruisecontrol_DW01_inc_1 ( A, SUM );
  input [7:0] A;
  output [7:0] SUM;

  wire   [7:2] carry;

  HAX1 U1_1_6 ( .A(A[6]), .B(carry[6]), .YC(carry[7]), .YS(SUM[6]) );
  HAX1 U1_1_5 ( .A(A[5]), .B(carry[5]), .YC(carry[6]), .YS(SUM[5]) );
  HAX1 U1_1_4 ( .A(A[4]), .B(carry[4]), .YC(carry[5]), .YS(SUM[4]) );
  HAX1 U1_1_3 ( .A(A[3]), .B(carry[3]), .YC(carry[4]), .YS(SUM[3]) );
  HAX1 U1_1_2 ( .A(A[2]), .B(carry[2]), .YC(carry[3]), .YS(SUM[2]) );
  HAX1 U1_1_1 ( .A(A[1]), .B(A[0]), .YC(carry[2]), .YS(SUM[1]) );
  XOR2X1 U1 ( .A(carry[7]), .B(A[7]), .Y(SUM[7]) );
endmodule


module cruisecontrol ( clk, reset, throttle, set, accel, coast, cancel, resume, 
        brake, speed, cruisespeed, cruisectrl );
  output [7:0] speed;
  output [7:0] cruisespeed;
  input clk, reset, throttle, set, accel, coast, cancel, resume, brake;
  output cruisectrl;
  wire   n311, n312, n313, n314, n315, n316, n317, n318, n319, n320, n321,
         n322, n323, n324, n325, n326, N58, N71, N72, N73, N74, N75, N76, N77,
         N128, N130, N131, N132, N133, N134, N135, N146, N147, N148, N149,
         N150, N151, N152, N153, N163, N164, N165, N166, N167, N168, N169,
         N170, N189, N190, n2, n3, n4, n5, n6, n7, n8, n18, n19, n20, n22, n23,
         n25, n26, n27, n28, n29, n34, n35, n37, n38, n43, n44, n45, n46, n47,
         n49, n50, n51, n52, n54, n55, n57, n58, n60, n62, n63, n65, n67, n68,
         n70, n72, n73, n75, n77, n78, n80, n82, n83, n85, n87, n88, n90, n91,
         n92, n93, n96, n98, n99, n100, n101, n102, n104, n107, n108, n109,
         n110, n111, n113, n116, n118, n119, n120, n121, n123, n125, n126,
         n127, n128, n129, n130, n131, n132, n133, n134, n135, n138, n139,
         n140, n141, n142, n143, n144, n145, n146, n147, n148, n149, n150,
         n151, n152, n153, n154, n155, n156, n157, n158, n159, n160, n162,
         n164, n165, n166, n167, n168, n169, n170, n171, n172, n173, n174,
         n175, n176, n177, n178, n179, n181, n182, n183, n184, \r116/carry[7] ,
         \r116/carry[6] , \r116/carry[5] , \r116/carry[4] , \r116/carry[3] ,
         n187, n192, n194, n196, n198, n200, n202, n204, n206, n208, n210,
         n212, n214, n215, n216, n217, n218, n219, n220, n221, n222, n223,
         n224, n225, n226, n227, n228, n229, n230, n231, n232, n233, n234,
         n235, n236, n237, n238, n239, n240, n241, n242, n243, n244, n245,
         n246, n247, n248, n249, n250, n251, n252, n253, n254, n255, n256,
         n257, n258, n259, n260, n261, n262, n263, n264, n265, n266, n267,
         n268, n269, n270, n271, n272, n273, n274, n275, n276, n277, n278,
         n279, n280, n281, n282, n283, n284, n285, n286, n287, n288, n289,
         n290, n291, n292, n293, n294, n295, n296, n297, n298, n299, n300,
         n301, n302, n303, n304, n305, n306, n307, n308, n309, n310;
  wire   [2:0] state;
  wire   SYNOPSYS_UNCONNECTED__0;

  DFFPOSX1 \cruisespeed_reg[0]  ( .D(n184), .CLK(clk), .Q(n325) );
  DFFPOSX1 \cruisespeed_reg[7]  ( .D(n183), .CLK(clk), .Q(n318) );
  DFFPOSX1 \cruisespeed_reg[6]  ( .D(n182), .CLK(clk), .Q(n319) );
  DFFPOSX1 \speed_reg[7]  ( .D(n181), .CLK(clk), .Q(n311) );
  DFFPOSX1 \state_reg[2]  ( .D(n276), .CLK(clk), .Q(state[2]) );
  DFFPOSX1 \speed_reg[0]  ( .D(n179), .CLK(clk), .Q(N128) );
  DFFPOSX1 \speed_reg[6]  ( .D(n178), .CLK(clk), .Q(n312) );
  DFFPOSX1 \speed_reg[5]  ( .D(n177), .CLK(clk), .Q(n313) );
  DFFPOSX1 \speed_reg[4]  ( .D(n176), .CLK(clk), .Q(n314) );
  DFFPOSX1 \state_reg[0]  ( .D(n175), .CLK(clk), .Q(state[0]) );
  DFFPOSX1 \state_reg[1]  ( .D(n174), .CLK(clk), .Q(state[1]) );
  DFFPOSX1 \speed_reg[3]  ( .D(n173), .CLK(clk), .Q(n315) );
  DFFPOSX1 \speed_reg[2]  ( .D(n172), .CLK(clk), .Q(n316) );
  DFFPOSX1 \speed_reg[1]  ( .D(n171), .CLK(clk), .Q(n317) );
  DFFPOSX1 \cruisespeed_reg[1]  ( .D(n170), .CLK(clk), .Q(n324) );
  DFFPOSX1 \cruisespeed_reg[2]  ( .D(n169), .CLK(clk), .Q(n323) );
  DFFPOSX1 \cruisespeed_reg[3]  ( .D(n168), .CLK(clk), .Q(n322) );
  DFFPOSX1 \cruisespeed_reg[4]  ( .D(n167), .CLK(clk), .Q(n321) );
  DFFPOSX1 \cruisespeed_reg[5]  ( .D(n166), .CLK(clk), .Q(n320) );
  DFFPOSX1 cruisectrl_reg ( .D(n165), .CLK(clk), .Q(n326) );
  OAI21X1 U4 ( .A(n19), .B(n20), .C(n277), .Y(n18) );
  OAI22X1 U6 ( .A(n23), .B(resume), .C(n291), .D(n25), .Y(n19) );
  AOI21X1 U7 ( .A(n26), .B(n27), .C(n22), .Y(n174) );
  NOR2X1 U8 ( .A(n28), .B(n29), .Y(n27) );
  AOI22X1 U9 ( .A(n300), .B(n283), .C(n298), .D(n285), .Y(n26) );
  AOI21X1 U11 ( .A(n35), .B(n290), .C(n22), .Y(n175) );
  OAI21X1 U12 ( .A(n37), .B(n38), .C(n280), .Y(n22) );
  NAND2X1 U13 ( .A(n278), .B(n296), .Y(n38) );
  NAND2X1 U14 ( .A(n23), .B(n310), .Y(n37) );
  NOR2X1 U16 ( .A(n28), .B(n44), .Y(n35) );
  AOI21X1 U17 ( .A(n45), .B(n46), .C(n25), .Y(n28) );
  OAI21X1 U18 ( .A(n47), .B(n212), .C(n49), .Y(n181) );
  NAND2X1 U19 ( .A(n50), .B(n51), .Y(n49) );
  OAI21X1 U20 ( .A(n52), .B(n295), .C(n54), .Y(n51) );
  AOI22X1 U21 ( .A(N135), .B(n20), .C(N77), .D(n55), .Y(n54) );
  OAI21X1 U24 ( .A(n47), .B(n208), .C(n57), .Y(n178) );
  NAND2X1 U28 ( .A(n50), .B(n58), .Y(n57) );
  OAI21X1 U29 ( .A(n52), .B(n303), .C(n60), .Y(n58) );
  AOI22X1 U30 ( .A(N134), .B(n20), .C(N76), .D(n55), .Y(n60) );
  OAI21X1 U33 ( .A(n47), .B(n206), .C(n62), .Y(n177) );
  NAND2X1 U34 ( .A(n50), .B(n63), .Y(n62) );
  OAI21X1 U35 ( .A(n52), .B(n304), .C(n65), .Y(n63) );
  AOI22X1 U36 ( .A(N133), .B(n20), .C(N75), .D(n55), .Y(n65) );
  OAI21X1 U38 ( .A(n47), .B(n204), .C(n67), .Y(n176) );
  NAND2X1 U39 ( .A(n50), .B(n68), .Y(n67) );
  OAI21X1 U40 ( .A(n52), .B(n305), .C(n70), .Y(n68) );
  AOI22X1 U41 ( .A(N132), .B(n20), .C(N74), .D(n55), .Y(n70) );
  OAI21X1 U43 ( .A(n47), .B(n202), .C(n72), .Y(n173) );
  NAND2X1 U44 ( .A(n50), .B(n73), .Y(n72) );
  OAI21X1 U45 ( .A(n52), .B(n306), .C(n75), .Y(n73) );
  AOI22X1 U46 ( .A(N131), .B(n20), .C(N73), .D(n55), .Y(n75) );
  OAI21X1 U49 ( .A(n47), .B(n200), .C(n77), .Y(n172) );
  NAND2X1 U50 ( .A(n50), .B(n78), .Y(n77) );
  OAI21X1 U51 ( .A(n52), .B(n307), .C(n80), .Y(n78) );
  AOI22X1 U52 ( .A(N130), .B(n20), .C(N72), .D(n55), .Y(n80) );
  OAI21X1 U55 ( .A(n47), .B(n198), .C(n82), .Y(n171) );
  NAND2X1 U56 ( .A(n50), .B(n83), .Y(n82) );
  OAI21X1 U57 ( .A(n52), .B(n308), .C(n85), .Y(n83) );
  AOI22X1 U58 ( .A(n198), .B(n20), .C(N71), .D(n55), .Y(n85) );
  OAI21X1 U60 ( .A(n47), .B(n210), .C(n87), .Y(n179) );
  NAND2X1 U61 ( .A(n50), .B(n88), .Y(n87) );
  OAI21X1 U62 ( .A(n52), .B(speed[0]), .C(n90), .Y(n88) );
  AOI22X1 U63 ( .A(speed[0]), .B(n20), .C(n210), .D(n55), .Y(n90) );
  NAND3X1 U64 ( .A(n91), .B(n92), .C(n93), .Y(n55) );
  AOI21X1 U65 ( .A(n288), .B(n299), .C(n43), .Y(n93) );
  OAI21X1 U66 ( .A(n291), .B(n96), .C(n23), .Y(n43) );
  NAND2X1 U67 ( .A(n299), .B(n294), .Y(n96) );
  OAI21X1 U69 ( .A(n298), .B(n300), .C(n285), .Y(n91) );
  OAI22X1 U70 ( .A(resume), .B(n98), .C(n25), .D(n294), .Y(n20) );
  AOI21X1 U73 ( .A(n99), .B(n299), .C(n44), .Y(n52) );
  NAND3X1 U74 ( .A(n100), .B(n101), .C(n102), .Y(n44) );
  AOI22X1 U75 ( .A(n298), .B(throttle), .C(resume), .D(n302), .Y(n102) );
  OAI21X1 U77 ( .A(n34), .B(n287), .C(n300), .Y(n100) );
  NOR2X1 U78 ( .A(n285), .B(n287), .Y(n34) );
  OAI21X1 U79 ( .A(n45), .B(N189), .C(n284), .Y(n99) );
  AND2X1 U81 ( .A(n47), .B(n280), .Y(n50) );
  OAI21X1 U83 ( .A(n46), .B(n25), .C(n278), .Y(n109) );
  NAND3X1 U85 ( .A(n104), .B(n280), .C(n111), .Y(n110) );
  AOI21X1 U86 ( .A(throttle), .B(n297), .C(n113), .Y(n111) );
  NAND3X1 U88 ( .A(n310), .B(n301), .C(n309), .Y(n101) );
  NAND3X1 U89 ( .A(n309), .B(n301), .C(state[1]), .Y(n104) );
  NOR2X1 U90 ( .A(n107), .B(n288), .Y(n46) );
  OAI21X1 U92 ( .A(n292), .B(n118), .C(n119), .Y(n107) );
  OAI21X1 U95 ( .A(resume), .B(n296), .C(n120), .Y(n108) );
  AND2X1 U96 ( .A(n121), .B(n92), .Y(n120) );
  NAND3X1 U97 ( .A(n286), .B(n299), .C(N189), .Y(n92) );
  NAND3X1 U99 ( .A(n123), .B(n289), .C(n125), .Y(n45) );
  NOR2X1 U100 ( .A(throttle), .B(coast), .Y(n125) );
  OAI21X1 U101 ( .A(n299), .B(n302), .C(N190), .Y(n121) );
  OAI21X1 U104 ( .A(n127), .B(n23), .C(n98), .Y(n126) );
  NOR2X1 U105 ( .A(n128), .B(n129), .Y(n127) );
  NAND3X1 U106 ( .A(n210), .B(n198), .C(n130), .Y(n129) );
  NOR2X1 U107 ( .A(speed[3]), .B(speed[2]), .Y(n130) );
  NAND3X1 U110 ( .A(n204), .B(n206), .C(n131), .Y(n128) );
  NOR2X1 U111 ( .A(speed[7]), .B(speed[6]), .Y(n131) );
  NAND2X1 U114 ( .A(n132), .B(n133), .Y(n183) );
  AOI22X1 U115 ( .A(N153), .B(n134), .C(N170), .D(n135), .Y(n133) );
  AOI22X1 U116 ( .A(n279), .B(n311), .C(cruisespeed[7]), .D(n282), .Y(n132) );
  NAND2X1 U117 ( .A(n138), .B(n139), .Y(n182) );
  AOI22X1 U118 ( .A(N152), .B(n134), .C(N169), .D(n135), .Y(n139) );
  AOI22X1 U119 ( .A(n279), .B(n312), .C(n319), .D(n282), .Y(n138) );
  NAND2X1 U120 ( .A(n140), .B(n141), .Y(n166) );
  AOI22X1 U121 ( .A(N151), .B(n134), .C(N168), .D(n135), .Y(n141) );
  AOI22X1 U122 ( .A(n279), .B(n313), .C(n320), .D(n282), .Y(n140) );
  NAND2X1 U123 ( .A(n142), .B(n143), .Y(n167) );
  AOI22X1 U124 ( .A(N150), .B(n134), .C(N167), .D(n135), .Y(n143) );
  AOI22X1 U125 ( .A(n279), .B(n314), .C(cruisespeed[4]), .D(n282), .Y(n142) );
  NAND2X1 U126 ( .A(n144), .B(n145), .Y(n168) );
  AOI22X1 U127 ( .A(N149), .B(n134), .C(N166), .D(n135), .Y(n145) );
  AOI22X1 U128 ( .A(n279), .B(n315), .C(n322), .D(n282), .Y(n144) );
  NAND2X1 U129 ( .A(n146), .B(n147), .Y(n169) );
  AOI22X1 U130 ( .A(N148), .B(n134), .C(N165), .D(n135), .Y(n147) );
  AOI22X1 U131 ( .A(n279), .B(n316), .C(cruisespeed[2]), .D(n282), .Y(n146) );
  NAND2X1 U132 ( .A(n148), .B(n149), .Y(n170) );
  AOI22X1 U133 ( .A(N147), .B(n134), .C(N164), .D(n135), .Y(n149) );
  AOI22X1 U134 ( .A(n279), .B(n317), .C(n324), .D(n282), .Y(n148) );
  NAND2X1 U135 ( .A(n150), .B(n151), .Y(n184) );
  AOI22X1 U136 ( .A(N146), .B(n134), .C(N163), .D(n135), .Y(n151) );
  NOR2X1 U137 ( .A(n152), .B(n116), .Y(n135) );
  NOR2X1 U138 ( .A(n152), .B(n119), .Y(n134) );
  NAND2X1 U139 ( .A(n299), .B(n280), .Y(n152) );
  AOI22X1 U141 ( .A(n279), .B(N128), .C(n325), .D(n282), .Y(n150) );
  NAND3X1 U144 ( .A(n153), .B(n280), .C(n113), .Y(n154) );
  OAI21X1 U145 ( .A(n123), .B(n25), .C(n155), .Y(n113) );
  OAI21X1 U147 ( .A(n156), .B(n25), .C(n157), .Y(n153) );
  NOR2X1 U148 ( .A(n292), .B(n158), .Y(n156) );
  NAND2X1 U149 ( .A(n119), .B(n116), .Y(n158) );
  NAND3X1 U150 ( .A(n123), .B(n289), .C(coast), .Y(n116) );
  NAND2X1 U152 ( .A(accel), .B(n123), .Y(n119) );
  OAI21X1 U154 ( .A(reset), .B(n159), .C(n160), .Y(n165) );
  NAND2X1 U155 ( .A(cruisectrl), .B(n281), .Y(n160) );
  AOI21X1 U157 ( .A(n300), .B(n162), .C(n29), .Y(n159) );
  AOI21X1 U158 ( .A(n23), .B(n98), .C(n293), .Y(n29) );
  NAND3X1 U160 ( .A(n309), .B(n310), .C(state[2]), .Y(n98) );
  NAND3X1 U162 ( .A(state[0]), .B(n310), .C(state[2]), .Y(n23) );
  OAI21X1 U163 ( .A(n123), .B(n25), .C(n157), .Y(n162) );
  AOI21X1 U164 ( .A(n300), .B(n287), .C(reset), .Y(n157) );
  NAND3X1 U166 ( .A(N58), .B(throttle), .C(set), .Y(n164) );
  NAND3X1 U167 ( .A(state[1]), .B(n301), .C(state[0]), .Y(n25) );
  NOR2X1 U168 ( .A(cancel), .B(brake), .Y(n123) );
  NAND3X1 U170 ( .A(n310), .B(n301), .C(state[0]), .Y(n155) );
  OR2X2 U82 ( .A(n108), .B(n109), .Y(n47) );
  OR2X2 U93 ( .A(n285), .B(coast), .Y(n118) );
  cruisecontrol_DW01_inc_0 add_118 ( .A({cruisespeed[7], n319, n320, 
        cruisespeed[4], n322, cruisespeed[2], n324, n325}), .SUM({N153, N152, 
        N151, N150, N149, N148, N147, N146}) );
  cruisecontrol_DW01_inc_1 r113 ( .A({speed[7], n312, speed[5:0]}), .SUM({n2, 
        n3, n4, n5, n6, n7, n8, SYNOPSYS_UNCONNECTED__0}) );
  INVX8 U173 ( .A(n232), .Y(cruisespeed[1]) );
  INVX1 U174 ( .A(n324), .Y(n232) );
  INVX8 U175 ( .A(n269), .Y(cruisespeed[6]) );
  INVX1 U176 ( .A(n319), .Y(n269) );
  INVX2 U177 ( .A(n326), .Y(n187) );
  INVX8 U178 ( .A(n187), .Y(cruisectrl) );
  INVX8 U179 ( .A(n231), .Y(cruisespeed[3]) );
  INVX1 U180 ( .A(n322), .Y(n231) );
  INVX8 U181 ( .A(n230), .Y(cruisespeed[5]) );
  INVX1 U182 ( .A(n320), .Y(n230) );
  INVX8 U183 ( .A(N163), .Y(cruisespeed[0]) );
  INVX1 U184 ( .A(n325), .Y(N163) );
  INVX2 U185 ( .A(n318), .Y(n192) );
  INVX8 U186 ( .A(n192), .Y(cruisespeed[7]) );
  INVX2 U187 ( .A(n323), .Y(n194) );
  INVX8 U188 ( .A(n194), .Y(cruisespeed[2]) );
  OR2X2 U189 ( .A(n200), .B(cruisespeed[2]), .Y(n239) );
  INVX2 U190 ( .A(n321), .Y(n196) );
  INVX8 U191 ( .A(n196), .Y(cruisespeed[4]) );
  OR2X2 U192 ( .A(n204), .B(cruisespeed[4]), .Y(n242) );
  INVX2 U193 ( .A(n317), .Y(n198) );
  INVX2 U194 ( .A(n314), .Y(n204) );
  INVX2 U195 ( .A(n313), .Y(n206) );
  INVX2 U196 ( .A(N128), .Y(n210) );
  INVX2 U197 ( .A(n316), .Y(n200) );
  INVX2 U198 ( .A(n315), .Y(n202) );
  INVX2 U199 ( .A(n312), .Y(n208) );
  INVX2 U200 ( .A(n311), .Y(n212) );
  INVX8 U201 ( .A(n198), .Y(speed[1]) );
  INVX8 U202 ( .A(n200), .Y(speed[2]) );
  INVX8 U203 ( .A(n202), .Y(speed[3]) );
  INVX8 U204 ( .A(n204), .Y(speed[4]) );
  INVX8 U205 ( .A(n206), .Y(speed[5]) );
  INVX8 U206 ( .A(n208), .Y(speed[6]) );
  INVX8 U207 ( .A(n210), .Y(speed[0]) );
  INVX8 U208 ( .A(n212), .Y(speed[7]) );
  XNOR2X1 U209 ( .A(speed[7]), .B(\r116/carry[7] ), .Y(N135) );
  OR2X1 U210 ( .A(\r116/carry[6] ), .B(n312), .Y(\r116/carry[7] ) );
  XNOR2X1 U211 ( .A(n312), .B(\r116/carry[6] ), .Y(N134) );
  OR2X1 U212 ( .A(\r116/carry[5] ), .B(speed[5]), .Y(\r116/carry[6] ) );
  XNOR2X1 U213 ( .A(speed[5]), .B(\r116/carry[5] ), .Y(N133) );
  OR2X1 U214 ( .A(\r116/carry[4] ), .B(speed[4]), .Y(\r116/carry[5] ) );
  XNOR2X1 U215 ( .A(n314), .B(\r116/carry[4] ), .Y(N132) );
  OR2X1 U216 ( .A(\r116/carry[3] ), .B(speed[3]), .Y(\r116/carry[4] ) );
  XNOR2X1 U217 ( .A(speed[3]), .B(\r116/carry[3] ), .Y(N131) );
  OR2X1 U218 ( .A(speed[1]), .B(speed[2]), .Y(\r116/carry[3] ) );
  XNOR2X1 U219 ( .A(n316), .B(speed[1]), .Y(N130) );
  NAND2X1 U220 ( .A(n198), .B(n210), .Y(n214) );
  OAI21X1 U221 ( .A(n210), .B(n198), .C(n214), .Y(N71) );
  NOR2X1 U222 ( .A(n214), .B(speed[2]), .Y(n216) );
  AOI21X1 U223 ( .A(n214), .B(n316), .C(n216), .Y(n215) );
  NAND2X1 U224 ( .A(n216), .B(n202), .Y(n217) );
  OAI21X1 U225 ( .A(n216), .B(n202), .C(n217), .Y(N73) );
  NOR2X1 U226 ( .A(n217), .B(speed[4]), .Y(n219) );
  AOI21X1 U227 ( .A(n217), .B(n314), .C(n219), .Y(n218) );
  NAND2X1 U228 ( .A(n219), .B(n206), .Y(n220) );
  OAI21X1 U229 ( .A(n219), .B(n206), .C(n220), .Y(N75) );
  XNOR2X1 U230 ( .A(n312), .B(n220), .Y(N76) );
  NOR2X1 U231 ( .A(n312), .B(n220), .Y(n221) );
  XOR2X1 U232 ( .A(speed[7]), .B(n221), .Y(N77) );
  INVX2 U233 ( .A(n218), .Y(N74) );
  INVX2 U234 ( .A(n215), .Y(N72) );
  NAND2X1 U235 ( .A(n232), .B(N163), .Y(n222) );
  OAI21X1 U236 ( .A(N163), .B(n232), .C(n222), .Y(N164) );
  NOR2X1 U237 ( .A(n222), .B(cruisespeed[2]), .Y(n224) );
  AOI21X1 U238 ( .A(n222), .B(cruisespeed[2]), .C(n224), .Y(n223) );
  NAND2X1 U239 ( .A(n224), .B(n231), .Y(n225) );
  OAI21X1 U240 ( .A(n224), .B(n231), .C(n225), .Y(N166) );
  NOR2X1 U241 ( .A(n225), .B(cruisespeed[4]), .Y(n227) );
  AOI21X1 U242 ( .A(n225), .B(cruisespeed[4]), .C(n227), .Y(n226) );
  NAND2X1 U243 ( .A(n227), .B(n230), .Y(n228) );
  OAI21X1 U244 ( .A(n227), .B(n230), .C(n228), .Y(N168) );
  XNOR2X1 U245 ( .A(n319), .B(n228), .Y(N169) );
  NOR2X1 U246 ( .A(n319), .B(n228), .Y(n229) );
  XOR2X1 U247 ( .A(cruisespeed[7]), .B(n229), .Y(N170) );
  INVX2 U248 ( .A(n226), .Y(N167) );
  INVX2 U249 ( .A(n223), .Y(N165) );
  NOR2X1 U250 ( .A(speed[7]), .B(speed[6]), .Y(n235) );
  NAND3X1 U251 ( .A(speed[3]), .B(speed[2]), .C(speed[1]), .Y(n233) );
  OAI21X1 U252 ( .A(n236), .B(speed[4]), .C(speed[5]), .Y(n234) );
  NAND2X1 U253 ( .A(n235), .B(n234), .Y(N58) );
  INVX2 U254 ( .A(n233), .Y(n236) );
  NAND2X1 U255 ( .A(cruisespeed[7]), .B(n212), .Y(n265) );
  NAND2X1 U256 ( .A(speed[5]), .B(n230), .Y(n260) );
  AND2X1 U257 ( .A(n242), .B(n260), .Y(n245) );
  NAND2X1 U258 ( .A(cruisespeed[2]), .B(n200), .Y(n251) );
  NAND2X1 U259 ( .A(n239), .B(n251), .Y(n253) );
  NAND2X1 U260 ( .A(speed[0]), .B(N163), .Y(n237) );
  OAI21X1 U261 ( .A(n198), .B(n237), .C(n324), .Y(n238) );
  OAI21X1 U262 ( .A(speed[1]), .B(n275), .C(n238), .Y(n241) );
  NAND2X1 U263 ( .A(speed[3]), .B(n231), .Y(n254) );
  AND2X1 U264 ( .A(n239), .B(n254), .Y(n240) );
  OAI21X1 U265 ( .A(n253), .B(n241), .C(n240), .Y(n243) );
  NOR2X1 U266 ( .A(n231), .B(speed[3]), .Y(n256) );
  NAND2X1 U267 ( .A(cruisespeed[4]), .B(n204), .Y(n257) );
  NAND2X1 U268 ( .A(n242), .B(n257), .Y(n259) );
  NAND3X1 U269 ( .A(n243), .B(n267), .C(n268), .Y(n244) );
  NOR2X1 U270 ( .A(n230), .B(speed[5]), .Y(n262) );
  AOI21X1 U271 ( .A(n245), .B(n244), .C(n262), .Y(n246) );
  XOR2X1 U272 ( .A(speed[6]), .B(n269), .Y(n248) );
  AOI22X1 U273 ( .A(speed[6]), .B(n269), .C(n246), .D(n248), .Y(n247) );
  NOR2X1 U274 ( .A(n212), .B(cruisespeed[7]), .Y(n266) );
  OAI21X1 U275 ( .A(n271), .B(n247), .C(n272), .Y(N189) );
  NOR2X1 U276 ( .A(N163), .B(speed[0]), .Y(n250) );
  AOI21X1 U277 ( .A(n198), .B(n250), .C(n324), .Y(n249) );
  OAI21X1 U278 ( .A(n250), .B(n198), .C(n273), .Y(n252) );
  OAI21X1 U279 ( .A(n253), .B(n252), .C(n251), .Y(n255) );
  OAI21X1 U280 ( .A(n256), .B(n255), .C(n254), .Y(n258) );
  OAI21X1 U281 ( .A(n259), .B(n258), .C(n257), .Y(n261) );
  OAI21X1 U282 ( .A(n262), .B(n261), .C(n260), .Y(n263) );
  OAI22X1 U283 ( .A(n270), .B(n263), .C(speed[6]), .D(n269), .Y(n264) );
  OAI21X1 U284 ( .A(n266), .B(n274), .C(n265), .Y(N190) );
  INVX2 U285 ( .A(n256), .Y(n267) );
  INVX2 U286 ( .A(n259), .Y(n268) );
  INVX2 U287 ( .A(n248), .Y(n270) );
  INVX2 U288 ( .A(n265), .Y(n271) );
  INVX2 U289 ( .A(n266), .Y(n272) );
  INVX2 U290 ( .A(n249), .Y(n273) );
  INVX2 U291 ( .A(n264), .Y(n274) );
  INVX2 U292 ( .A(n237), .Y(n275) );
  INVX2 U293 ( .A(n18), .Y(n276) );
  INVX2 U294 ( .A(n22), .Y(n277) );
  INVX2 U295 ( .A(n110), .Y(n278) );
  INVX2 U296 ( .A(n154), .Y(n279) );
  INVX2 U297 ( .A(reset), .Y(n280) );
  INVX2 U298 ( .A(n162), .Y(n281) );
  INVX2 U299 ( .A(n153), .Y(n282) );
  INVX2 U300 ( .A(n34), .Y(n283) );
  INVX2 U301 ( .A(n107), .Y(n284) );
  INVX2 U302 ( .A(throttle), .Y(n285) );
  INVX2 U303 ( .A(n45), .Y(n286) );
  INVX2 U304 ( .A(n164), .Y(n287) );
  INVX2 U305 ( .A(n116), .Y(n288) );
  INVX2 U306 ( .A(accel), .Y(n289) );
  INVX2 U307 ( .A(n43), .Y(n290) );
  INVX2 U308 ( .A(cancel), .Y(n291) );
  INVX2 U309 ( .A(n123), .Y(n292) );
  INVX2 U310 ( .A(resume), .Y(n293) );
  INVX2 U311 ( .A(brake), .Y(n294) );
  INVX2 U312 ( .A(n2), .Y(n295) );
  INVX2 U313 ( .A(n126), .Y(n296) );
  INVX2 U314 ( .A(n101), .Y(n297) );
  INVX2 U315 ( .A(n104), .Y(n298) );
  INVX2 U316 ( .A(n25), .Y(n299) );
  INVX2 U317 ( .A(n155), .Y(n300) );
  INVX2 U318 ( .A(state[2]), .Y(n301) );
  INVX2 U319 ( .A(n98), .Y(n302) );
  INVX2 U320 ( .A(n3), .Y(n303) );
  INVX2 U321 ( .A(n4), .Y(n304) );
  INVX2 U322 ( .A(n5), .Y(n305) );
  INVX2 U323 ( .A(n6), .Y(n306) );
  INVX2 U324 ( .A(n7), .Y(n307) );
  INVX2 U325 ( .A(n8), .Y(n308) );
  INVX2 U326 ( .A(state[0]), .Y(n309) );
  INVX2 U327 ( .A(state[1]), .Y(n310) );
endmodule

