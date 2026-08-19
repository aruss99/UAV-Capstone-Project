`timescale  1ns / 1ps 

module PWM_Generator_Verilog_tb();
 
 reg clk; // 12MHz clock input 

 // for Servo, at 50 Hz 
 reg increase_duty0; // input to increase 10% duty cycle 
 reg decrease_duty0; // input to decrease 10% duty cycle 

 // for ESC, at 50 Hz
 reg increase_duty1; // input to increase 10% duty cycle 
 reg decrease_duty1; // input to decrease 10% duty cycle 


 wire PWM_OUT0; // 50Hz PWM output signal  
 wire PWM_OUT1; // 50 Hz PWM output, for ESC


PWM_Generator_Verilog uut(
	.clk(clk),
	.increase_duty0(increase_duty0),
	.decrease_duty0(decrease_duty0),
	.increase_duty1(increase_duty1),
	.decrease_duty1(decrease_duty1),
	.PWM_OUT0(PWM_OUT0), 
	.PWM_OUT1(PWM_OUT1)
); 


initial begin 
	clk = 0; 
forever #5 clk = ~clk; 
end 

initial begin 
	increase_duty0 = 0;
	decrease_duty0 = 0;
	increase_duty1 = 0; 
	decrease_duty1 = 0; 
 #100; 
    increase_duty0 = 1; 
  #100;// increase duty cycle by 10%
    increase_duty0 = 0;
  #100; 
    increase_duty0 = 1;
  #100;// increase duty cycle by 10%
    increase_duty0 = 0;
  #100; 
    increase_duty0 = 1;
  #100;// increase duty cycle by 10%
    increase_duty0 = 0;
  #100;
    decrease_duty0 = 1; 
  #100;//decrease duty cycle by 10%
    decrease_duty0 = 0;
  #100; 
    decrease_duty0 = 1;
  #100;//decrease duty cycle by 10%
    decrease_duty0 = 0;
  #100;
    decrease_duty0 = 1;
  #100;//decrease duty cycle by 10%
    decrease_duty0 = 0;
 #100; 
    increase_duty0 = 1; 
  #100;// increase duty cycle by 10%
    increase_duty0 = 0;
  #100; 
    increase_duty1 = 1;
  #100;// increase duty cycle by 10%
    increase_duty1 = 0;
  #100; 
    increase_duty1 = 1;
  #100;// increase duty cycle by 10%
    increase_duty1 = 0;
  #100;
    decrease_duty1 = 1; 
  #100;//decrease duty cycle by 10%
    decrease_duty1 = 0;
  #100; 
    decrease_duty1 = 1;
  #100;//decrease duty cycle by 10%
    decrease_duty1 = 0;
  #100;
    decrease_duty1 = 1;
  #100;//decrease duty cycle by 10%
    decrease_duty1 = 0;
 end
endmodule

