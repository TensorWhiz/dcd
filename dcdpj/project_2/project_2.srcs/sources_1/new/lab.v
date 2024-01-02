`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/01/02 21:25:52
// Design Name: 
// Module Name: lab
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module NPC(
    input [31:0] pc_in,
    output[31:0] pc_out);
 assign pc_out=pc_in+32'd4;
endmodule

module MUX2x32 (
    input [31:0] a,
    input [31:0] b,
    input M,
    output reg [31:0] data_out
 );
 always@(*)
 begin 
    case(M)
     1'b0:data_out=a;
     1'b1:data_out=b;
    endcase
 end
endmodule

module PC(
   input clk,
   input rst,
   input [31:0] data_in,
   output reg[31:0] data_out
 );

 always@(posedge clk,posedge rst) begin
    if(rst)data_out<=32'b0;
    else data_out=data_in;
 end
endmodule

module IMEM#(parameter OLIMIT=16)(
    input [31:0] address,
    input IM_R,//信号：读指令
    output [31:0] order
 );
 reg [31:0] imem[OLIMIT-1:0];

 initial begin
   $readmemh("./imem.mem",imem);
 end
 assign order=(IM_R)?imem[address[31:2]]:32'bx;
endmodule

module RegFile#(parameter RLIMIT=16)(
   input clk,
   input rst,
   input RF_W,//允许写入
   input [4:0] rsc,//读出数据地址1
   input [4:0] rtc,//读出数据地址2
   input [4:0] rdc,//写入数据地址
   input [31:0] rd,//写入数据
   output[31:0] rs,//读出数据1
   output[31:0] rt//读出数据2 
 );
 reg [31:0] regarray [RLIMIT-1:0] ;
 integer i;
 always @ (posedge clk,posedge rst)begin
   if(rst)begin //复位
      i=0;
      while(i<RLIMIT)begin
      regarray[i]=0;
      i=i+1;
      end
   end
   else if(RF_W) begin //写入
      if(rdc!=0)regarray[rdc]=rd;
   end
 end

 assign rs=regarray[rsc];
 assign rt=regarray[rtc];
endmodule

module EXT16(
   input [15:0] data_in,
   input sign,
   output reg [31:0] data_out

 );
 always @(data_in or sign)begin
   if(sign==1&&data_in[15]==1)data_out={{16{1'b1}},data_in};
   else data_out={16'b0,data_in};
 end
endmodule

module ALU(
   input [31:0] A,//与rs对应
   input [31:0] B,//与rt等对应
   input [2:0] ALUC,//指令选择信号
   output reg[31:0] data_out

 );

 always@(*)begin
   case(ALUC)
    0: data_out=A+B;
    1: data_out=A-B;
    2: data_out=A&B;
    3: data_out=A|B;
    4: data_out= A<B?1:0;
    default:;
   endcase
   
 end
endmodule

module DMEM#(parameter DLIMIT=64)(
   input clk,
   input rst,
   input CS,//数据存储器片选信号
   input DM_R,//
   input DM_W,
   input [31:0] addr,
   input [31:0] wdata,
   output [31:0] rdata
 );
 reg [31:0] dmem [DLIMIT-1:0];
 assign rdata=(CS&DM_R)?dmem[addr[31:2]]:0;
 always @(posedge clk) begin
   if(CS&DM_W) dmem[addr[31:2]]<=wdata;
 end
endmodule

module JOINT(
    input [27:0] imempart,
    input [3:0] pcpart,
    output [31:0] data_out
 );
 assign data_out={pcpart,imempart};
endmodule

module Decoder(
   input [31:0] order,
   input clk,
   output IM_R,
   output M1,
   output M2,
   output M3,
   output [2:0]ALUC ,
   output RF_W,
   output CS,
   output DM_R,
   output DM_W,
   output M4, 
   output sign
 );
 wire [5:0] func=order[5:0];
 wire [5:0] op=order[31:26];
 assign R_type= (op==6'b0);//~op[5] & ~op[4] & ~op[3]& ~op[2]& ~op[1] & ~op[0];
 assign add= R_type && (func==6'b100000);
 assign sub= R_type && (func==6'b100010);
 assign andu=R_type && (func==6'b100100);
 assign oru= R_type && (func==6'b100101);
 assign slt= R_type && (func==6'b101010);
 assign addi= (op==6'b001000);
 assign andi= (op==6'b001100);
 assign ori=  (op==6'b001101);
 assign slti= (op==6'b001010);
 assign lw= (op==6'b100011);
 assign sw= (op==6'b101011);
 assign nop= R_type && (func==6'b000000);
 assign j= (op==6'b000010);

 assign IM_R=1;
 assign rsc= order[25:21];
 assign rtc= order[20:16];
 assign rdc= order[15:11]&(add|sub|andu|oru|slt)|order[20:16]&(addi|andi|ori|slti|lw);
 assign M1=add|sub|andu|oru|slt|addi|andi|ori|slti|lw|sw|nop;
 assign M2=add|sub|andu|oru|slt|j|nop;
 assign M3=add|sub|andu|oru|slt|addi|andi|ori|slti|sw|j|nop;
 assign ALUC[0]=sub|oru|slt|ori|slti;
 assign ALUC[1]=andu|oru|slt|andi|ori|slti;
 assign ALUC[2]=slt|slti;
 assign RF_W=add|sub|andu|oru|slt|addi|andi|ori|slti|lw;
 assign CS=lw|sw;
 assign DM_R=lw;
 assign DM_W=sw;
 assign M4=addi | andi|ori|slti|lw;
 assign sign=lw |sw;
endmodule

module CPU(
   input clk,
   input [31:0]order,
   input rst,
   input [31:0] rdata,

   output [31:0] pc_out,
   output [31:0] maddr,
   output [31:0] wdata,
   output IM_R,
   output CS,
   output DM_R,
   output DM_W,
   output [31:0]alu_r

 );
 wire RF_W,M1,M2,M3,M4,sign ;
 wire [2:0]ALUC;
 wire [31:0]mux1_out,mux2_out,mux3_out,alu_out;
 wire [31:0]rf_A,rf_B;
 wire [31:0]ext16_out;
 wire [31:0]npc_out;
 wire [31:0]joint_out;
wire[27:0] tem1;
wire [4:0]tem2=order[25:21];
wire [4:0]tem3=order[20:16];
 assign alu_r=alu_out;
 assign maddr=alu_out;
 assign wdata=rf_B;
 wire [4:0]mux4_out=M4?order[20:16]:order[15:11];
 assign tem1=order[25:0]<<2;
 Decoder cpu_decoder(order,clk,IM_R,M1,M2,M3,ALUC,RF_W,CS,DM_R,DM_W,M4,sign);

 PC cpu_pc(clk,rst,mux1_out,pc_out);

 NPC cpu_npc(pc_out,npc_out);

 MUX2x32 mux1(joint_out,npc_out,M1,mux1_out);

 JOINT cpu_joint(tem1,pc_out[31:28],joint_out);

 RegFile cpu_rf(clk,rst,RF_W,tem2,tem3,mux4_out,mux3_out,rf_A,rf_B);

 ALU cpu_alu(rf_A,mux2_out,ALUC,alu_out);

 EXT16 cpu_ext16(order[15:0] ,sign,ext16_out);

 MUX2x32 mux2(ext16_out,rf_B,M2,mux2_out);

 MUX2x32 mux3(rdata,alu_out,M3,mux3_out);
endmodule

module TOP(
   input clk_in,
   input rst,
   output clk,
   output [31:0]pc,
   output [31:0]order,
   output [31:0]addr,
   output [31:0]rdata,
   output  [31:0] wdata,
   output IM_R,
   output CS,//数据存储器片选信号
   output DM_R,//
   output DM_W,
   output [31:0]alu_r

 );
 assign clk=clk_in;
 CPU  cpu(clk_in,order,rst,rdata,pc,addr,wdata,IM_R, CS,DM_R,DM_W,alu_r);
 IMEM imem(pc,IM_R,order);
 DMEM dmem(clk_in,rst,CS,DM_R,DM_W,addr,wdata,rdata);
endmodule

module top_nt();
 reg clk_in;
 reg reset;
 wire clk;
wire [31:0]pc;
 wire [31:0] order;
   wire [31:0]addr;
   wire [31:0]rdata;
   wire  [31:0] wdata;
   wire IM_R;
  wire CS;
   wire DM_R;
   wire DM_W;
   wire [31:0]alu_r;
   initial
   begin
   clk_in=0;
   forever #25  clk_in=~clk_in;  
   end
   
   initial
   begin
   reset=0;
   #5 reset=1;
   #5 reset=0;
   
   end
    
   
   TOP uut(clk_in,reset,clk,pc,order,addr,rdata,wdata,IM_R,CS,DM_R,DM_W,alu_r);
   
endmodule