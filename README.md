# 4-bit-Ripple-Carry-Adder-using-Task-and-4-bit-Ripple-Counter-using-Function-with-Testbench
Aim:
To design and simulate a 4-bit Ripple Carry Adder using Verilog HDL with a task to implement the full adder functionality and verify its output using a testbench.
To design and simulate a 4-bit Ripple Counter using Verilog HDL with a function to calculate the next state and verify its functionality using a testbench.

Apparatus Required:
Computer with Vivado or any Verilog simulation software.
Verilog HDL compiler.
module rom_design(clk,rst,address,dataout);
input clk,rst;
input[2:0] address;
output reg[3:0] dataout;
reg [3:0] rom_design[7:0];
initial begin
rom_design[0]=4'd1;
rom_design[1]=4'd2;
rom_design[2]=4'd3;
rom_design[4]=4'd10;
rom_design[5]=4'd11;
rom_design[6]=4'd12;
rom_design[7]=4'd15 ;
end
always@(posedge clk) begin
if(rst)
dataout=4'd0;
else
dataout=rom_design[address];
end
endmodule
module rom_design_tb;
reg clk,rst;
red[2:0]address;
wire[3:0]dataout;
rom_design dut(
.clk(clk),
.rst(rst),
.address(address),
.dataout(dataout)
);
initial begin
clk=1'b0;
forever #5 clk=~clk;
end
intitial begin
rst=1'b1;
address=3'b000;
#10 rst=1'b0;
#10 address=3'b000;
#10 $display("Address: %d, Dataout: %d, address, dataout);
#10 address=3'b001;
#10 $display("Address: %d, Dataout: %d, address, dataout);
#10 address=3'b010;
#10 $display("Address: %d, Dataout: %d, address, dataout);
#10 address=3'b100;
#10 $display("Address: %d, Dataout: %d, address, dataout);
#10 address=3'b101;
#10 $display("Address: %d, Dataout: %d, address, dataout);
#10 address=3'b110;
#10 $display("Address: %d, Dataout: %d, address, dataout);
#10 address=3'b111;
#10 $display("Address: %d, Dataout: %d, address, dataout);
#10 rst=1'b1;
#10 $display("Reset asserted, Dataout: %d, dataout);
end
endmodule
module ram(clk,rst,en,data_in,data_out,address);
input clk,rst,en;
input[11:0] address;
input[7:0] data_in;
output reg[7:0] data_out;
reg[1023:0] mem[7:0];
always@(posedge clk)
begin
if(rst)
data_out<=8'd0;
else if(en)
mem[address]<=data_in;
else
data_out<=mem[address];
end
endmodule
module rom_design_tb;
reg clk,rst;
reg[2:0]address;
wire[3:0]dataout;
rom_design dut(
.clk(clk),
.rst(rst),
.address(address),
.dataout(dataout)
);
initial begin
rst=1'b1;
address=3'b000;
#10 rst=1'b0;
#10 address=3'b000;
Output:
FIRST IN FIRST OUT[FIFO]:
To design and simulate a FIFO memory using Verilog HDL and verify the functionality
through a testbench in the Vivado 2023.1 simulation environment.
Apparatus RequiredVivado 2023.1 or equivalent Verilog simulation tool. Computer
system with a suitable operating system.
ProcedureLaunch Vivado 2023.1:
Open Vivado and create a new project. Design the Verilog Code for FIFO:
#10 $display("Address: %d, Dataout:%d, address, dataout);
#10 address=3'b001;
#10 $display("Address: %d, Dataout:%d, address, dataout);
#10 address=3'b010;
#10 $display("Address: %d, Dataout:%d, address, dataout);
#10 address=3'b100;
#10 $display("Address: %d, Dataout:%d, address, dataout);
#10 address=3'b101;
#10 $display("Address: %d, Dataout:%d, address, dataout);
#10 address=3'b110;
#10 $display("Address: %d, Dataout:%d, address, dataout);
#10 address=3'b111;
#10 $display("Address: %d, Dataout:%d, address, dataout);
#10 arst=1'b1;
#10 $display("Reset assertion, Dataout:%d,dataout);
end
endmodule
module fifo_8 #(parameter depth=8, data_width=8)
(input clk,rst,
input w_en,r_en,
input [data_width-1:0] data_in,
output reg[data_width-1:0] data_out,
output full, empty);
reg[$clog2(depth)-1:0] w_ptr, r_pt;
reg[data_width-1:0] fifo [depth-1:0];
always@(posedge clk) begin
if(lrst) begin
w_ptr<=0;
r_ptr<=0;
data_out<=0;
end
end
always@(posedge clk) begin
if(w_en && !full) begin
fifo[w_ptr]<=data_in;
w_ptr<=(w_ptr+1) % depth;
end
end
always@(posedge clk) begin
if(r_en && !empty) begin
 data_out<=fifo[r_ptr];
 r_ptr<=(r_ptr+1) % depth;
end
end
assign full =(w_ptr+1==r_ptr);
assign empty=(w_ptr==r_ptr);
endmodule
module fifo_8_tb;
reg clk,rst;
reg w_en, r_en;
reg[7:0] data_in;
wire[7:0] data_out;
wire full,empty;
fifo_8 #(depth(8), .data_width(8)) dut(
.clk(clk),
.rst(rst),
.w_en(w_en),
.r_en(r_en),
.data_in(data_in),
.data_out(data_out),
.full(full),
.empty(empty)
);
initial begin
clk=1'b0;
forever #5 clk= ~clk;
end
initial begin
rst=1'b1;
w_en=1'b0;
r_en=1'b0;
data_in=8'd0;
#10 rst=1'bo;
#10 w_en = 1'b1;
data_in=8'd1;
#10;
w_en=1'b0;
$disaply("Write 1,Full: %b, Empty: %b, full, empty);
#10 w_en=1'b1;
data_in=8'd2;
#10;
w_en=1'b0;
$disaply("Write 2,Full: %b, Empty: %b, full, empty);
#10 w_en=1'b1; data_in=8'd3; #10; w_en=1'b0;
#10 w_en=1'b1; data_in=8'd4; #10; w_en=1'b0;
#10 w_en=1'b1; data_in=8'd5; #10; w_en=1'b0;
#10 w_en=1'b1; data_in=8'd6; #10; w_en=1'b0;
#10 w_en=1'b1; data_in=8'd7; #10; w_en=1'b0;
#10 w_en=1'b1; data_in=8'd8; #10; w_en=1'b0;
$display("Write 3-8, Full: %b, Empty: %b, full, empty);
#10 r_en=1'b1;
#10;
r_en=1'b0;
$display("Read 1, Data: %d, Full: %b, Empty: %b, data_out, full,
empty);
#10 r_en=1'b1; #10; r_en=1'b0;
#10 r_en=1'b1; #10; r_en=1'b0;
#10 r_en=1'b1; #10; r_en=1'b0;
#10 r_en=1'b1; #10; r_en=1'b0;
#10 r_en=1'b1; #10; r_en=1'b0;
#10 r_en=1'b1; #10; r_en=1'b0;
#10 r_en=1'b1; #10; r_en=1'b0;
$display("Read 2-8, Full: %b, Empty: %b, full,empty);
#10 w_en=1'b1;
data_in=8'd9;
#10;
$display("Pverflow, Full: %b, Empty: %b, full, empty);
#10 r_en=1'b1;
#10;
$display("Underflow, Full: %b, Empty: %b, full, empty);
end
endmodule

Conclusion:
The 4-bit Ripple Carry Adder was successfully designed and implemented using Verilog HDL with the help of a task for the full adder logic. The testbench verified that the ripple carry adder correctly computes the 4-bit sum and carry-out for various input combinations. The simulation results matched the expected outputs.

The 4-bit Ripple Counter was successfully designed and implemented using Verilog HDL. A function was used to calculate the next state of the counter.

