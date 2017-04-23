module Pipelined_TestBench;

 reg Clock;
 integer i;
 initial begin
 Clock = 1;
 end
 //clock controls
 always begin
 Clock = ~Clock;
 #25;
 end

 initial begin

 // Instr Memory intialization
 pipelined.IM.regfile[0] = 32'h8c030000;
 pipelined.IM.regfile[4] = 32'h8c040001;
 pipelined.IM.regfile[8] = 32'h8c050002;
 pipelined.IM.regfile[12] = 32'h8c010002;
 pipelined.IM.regfile[16] = 32'h10600004;
 pipelined.IM.regfile[20] = 32'h00852020;
 pipelined.IM.regfile[24] = 32'h00852822;
 pipelined.IM.regfile[28] = 32'h00611820;
 pipelined.IM.regfile[32] = 32'h1000fffb;
 pipelined.IM.regfile[36] = 32'hac040006;
 // Data Memory intialization
 pipelined.DM.regfile[0] = 32'd8;
 pipelined.DM.regfile[1] = 32'd1;
 pipelined.DM.regfile[2] = -32'd1;
 pipelined.DM.regfile[3] = 0;

 pipelined.piperegs.regfile[0] = 0;
 // Register File initialization
 for (i = 0; i < 32; i = i + 1)
 pipelined.piperegs.regfile[i] = 32'd0;
 end
 //Instantiate cpu
 cpu pipelined(Clock);

endmodule
