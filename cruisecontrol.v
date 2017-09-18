//###########################
//###Verilog of CruiseControl 
//###########################

module cruisecontrol(
clk,
reset,
throttle,
set,
accel,
coast,
cancel,
resume,
brake,
speed,
cruisespeed,
cruisectrl
);

//input and outputs
input clk;
input reset;
input throttle;
input set;
input accel;
input coast;
input cancel;
input resume;
input brake;

wire clk;
wire reset;
wire throttle;
wire set;
wire accel;
wire coast;
wire cancel;
wire resume;
wire brake;

output reg [7:0] speed;
output reg [7:0] cruisespeed;
output reg cruisectrl;

reg [2:0] state;

//States of FSM
parameter INIT = 3'b000;
parameter ACC  = 3'b001;
parameter DEC  = 3'b010;
parameter CRCT = 3'b011;
parameter BRK  = 3'b100;
parameter CNCL = 3'b101;

//code starts here
//

always @(posedge clk)
begin
  if(reset == 1'b1) 
     begin
     state <= #1 INIT;
     speed <= 7'b0;
     cruisespeed <= 7'b0;
     cruisectrl <= 0; 
     end 
  else 
      case(state) 
          INIT:if(throttle == 1'b1) 
		  begin
                  state <= #1 ACC;
                  speed <= speed + 1'b1;
                  end
           ACC:if (set == 1'b1 & speed > 7'b0010_1101 & throttle == 1)
                   begin 
                   state <= #1 CRCT;
		   speed <= speed + 1'b1;
		   cruisespeed <= speed;
                   cruisectrl <= 1'b1;
		   end 
	       else if (throttle == 0)
                   begin
          	   state <= #1 DEC;
                   speed <= speed - 1'b1;
		   end
               else
                   begin
                   state <= #1 ACC;
                   speed <= speed + 1'b1;
		   end
           DEC:if (throttle == 1'b1)
		   begin
                   state <= #1 ACC;
                   speed <= speed + 1'b1;
		   end
               else if(throttle == 0)
		   begin
          	   state <= #1 DEC;
                   speed <= speed - 1'b1;
                   end
           CRCT:if(brake == 1'b1)
                   begin 
                   state <= #1 BRK;   
		   speed <= speed - 2'b10;
		   cruisespeed <= speed;
                   cruisectrl <= 1'b0;
  		   end
                else if(cancel == 1'b1)
                   begin
                   state <= #1 CNCL;
		   cruisespeed <= speed;
                   speed <= speed - 1'b1;
                   cruisectrl <= 1'b0;
                   end   
		else if (accel == 1'b1)
                   begin
                   state <= #1 CRCT;
                   cruisespeed <= cruisespeed + 1'b1;
                   speed <= speed + 1'b1;
                   end
                else if (coast == 1'b1)
                   begin
                   state <= #1 CRCT;
                   cruisespeed <= cruisespeed - 1'b1;
                   speed <= speed - 1'b1;
                   end
                else if(throttle == 1'b1)
                   begin
                   state <= #1 CRCT;
                   speed <= speed + 1'b1;
		   end     
                else if(throttle == 1'b0)
                   begin 
                   state <= #1 CRCT;
                        if(speed > cruisespeed)
                          begin
 			  speed = speed - 1'b1;
			  end
                        else if (speed < cruisespeed)
                	begin
                	speed <= speed + 1'b1;
                	end
                   end                 
           BRK:if(resume == 1'b1)
 		begin
		state <= #1 CRCT;
                cruisectrl <= 1'b1;
		if (speed < cruisespeed)
		begin
		speed <= speed + 1'b1;
		end		
		end  
               else  
		 begin
		   state <= #1 BRK;
		   speed <= speed - 2'b10;
		 end
           CNCL:if(resume == 1'b1)
 		begin
		state <= #1 CRCT;
                cruisectrl <= 1'b1;
		end  
               else if (speed != 7'b0) 
		begin
		   state <= #1 CNCL;
		   speed <= speed - 1'b1;
		end
          default: state <= #1 INIT;
endcase
end

endmodule
