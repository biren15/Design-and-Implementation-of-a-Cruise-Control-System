//##############################
//###Cruise Control TB##########
//##############################

module cruisecontrol_tb();

reg clk, reset, throttle, set, accel, coast, cancel, resume, brake;
wire [7:0] speed;
wire [7:0] cruisespeed;
wire cruisectrl;
initial
begin
//initialization
clk = 1'b1;
reset = 1'b0;
throttle = 1'b0;
set= 1'b0;
accel= 1'b0;
coast= 1'b0;
cancel= 1'b0;
resume= 1'b0;
brake= 1'b0;

//apply reset 
#10 reset = 1'b1;
#5 reset = 1'b0;

// incr speed upto 30mph
#5 throttle = 1'b1;
//#300 throttle = 0;

//try to set cruise control now
#290 set = 1'b1;
#5 set = 1'b0;

//turn the throttle off upto 20mph
#5 throttle =1'b0;

// increase upto 50mph
#100 throttle = 1'b1; 
#290

//set cruise cntrol set
#10 set = 1'b1;
#10 set = 1'b0;

//increase speed upto 60mph
//remove throttle and let it drop to 50mph
#90 throttle = 0;
#100 
#50 

//apply brake now
#10 brake =1'b1;
#5 brake = 1'b0;

//resume now
#95 resume =1'b1;
#5 resume = 1'b0;
#250

//increase the cruise speed by5 using accel 5 pulses
#10 accel = 1'b1;
#50 accel = 1'b0;
#50


//decrease the cruise speed by5 using coast 5 pulses
#10 coast = 1'b1;
#50 coast = 1'b0;
#50

//apply cancel to reduce speed to 0
#15 cancel = 1'b1;
#5 cancel = 1'b0;
#500

$finish;

//
end


always 
begin
   #5 clk =~clk;
end

cruisecontrol cruisecontrol_inst1(clk, reset, throttle, set, accel, coast, cancel, resume, brake, speed, cruisespeed, cruisectrl);


endmodule
