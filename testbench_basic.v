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
       // Dump Waves
    $dumpfile("dump.vcd");
    $dumpvars(1);


 	// Instr Memory intialization
	pipelined.IM.regfile[0] = 32'h8C030000; //lw R3,0(R1)
   	pipelined.IM.regfile[4] = 32'h8C040001;//lw R4,1(R0)
   	pipelined.IM.regfile[8] = 32'h00642820;//add R5,R3,R4
   	pipelined.IM.regfile[12] = 32'h00A43022;//sub R6,R5,R4
   	pipelined.IM.regfile[16] = 32'h00643824;//and R7,R3,R4
   	pipelined.IM.regfile[20] = 32'h00644025;//or R8,R3,R4
   	pipelined.IM.regfile[24] = 32'h00644827;//nor R9,R3,R4
   	pipelined.IM.regfile[28] = 32'h00C5502A;//slt R10,R6,R5
 	pipelined.IM.regfile[32] = 32'h80000008;//j startloop
 	pipelined.IM.regfile[36] = 32'h2063FFFF;//loop: addi R3,R3,-1
 	pipelined.IM.regfile[40] = 32'h14E3FFFE;//startloop: bne R3,R7,-2
 	pipelined.IM.regfile[44] = 32'h01295818;//mult R11,R9,R9
 	pipelined.IM.regfile[48] = 32'h0166601A;//div R12,R11,R6
 	pipelined.IM.regfile[52] = 32'h34CE0002;//ori R14,R6,2
 	pipelined.IM.regfile[56] = 32'h11CC0000;//beq R14,R12, next
 	pipelined.IM.regfile[60] = 32'hADCE0006;//sw

 	// Data Memory intialization
 	pipelined.DM.regfile[0] = 32'd8;
 	pipelined.DM.regfile[1] = 32'd1;

 	pipelined.piperegs.regfile[0] = 0;
 	// Register File initialization
 	for (i = 0; i < 32; i = i + 1)
 		pipelined.piperegs.regfile[i] = 32'd0;

 end

  // Instantiate cpu
 cpu pipelined(Clock);

endmodule
