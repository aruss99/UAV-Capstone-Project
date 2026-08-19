module PWM_Generator_Verilog
 (
 clk, // 12MHz clock input 
 increase_duty, // input to increase 10% duty cycle 
 decrease_duty, // input to decrease 10% duty cycle 
 PWM_OUT // 50Hz PWM output signal 
    );
 input clk;
 input increase_duty;
 input decrease_duty;
 output PWM_OUT;


 wire slow_clk_enable; // slow clock enable signal for debouncing FFs
 reg[27:0] counter_debounce=0;// counter for creating slow clock enable signals 
 wire tmp1,tmp2,duty_inc;// temporary flip-flop signals for debouncing the increasing button
 wire tmp3,tmp4,duty_dec;// temporary flip-flop signals for debouncing the decreasing button
 reg[18:0] counter_PWM=0;// counter for creating 50 Hz PWM signal
// 60000 of a 240000-count frame is 25%: at 12 MHz, a 5 ms pulse in a 20 ms
// frame, outside the standard 1-2 ms servo band. The duty is driven from the
// buttons, so this is a starting point rather than a neutral position.
 reg[18:0] DUTY_CYCLE=60000; // initial duty cycle is 25% (5 ms of a 20 ms frame)
  // Debouncing 2 buttons for inc/dec duty cycle 
  // Firstly generate slow clock enable for debouncing flip-flop (4Hz)
 always @(posedge clk)
 begin
   counter_debounce <= counter_debounce + 1;
   //if(counter_debounce>=25000000) then  
   // for running on FPGA -- comment when running simulation
   if(counter_debounce>=1) 
   // for running simulation -- comment when running on FPGA
    counter_debounce <= 0;
 end
 // assign slow_clk_enable = counter_debounce == 25000000 ?1:0;
 // for running on FPGA -- comment when running simulation 
 assign slow_clk_enable = counter_debounce == 1 ?1:0;
 // for running simulation -- comment when running on FPGA
 // debouncing FFs for increasing button
 DFF_PWM PWM_DFF1(clk,slow_clk_enable,increase_duty,tmp1);
 DFF_PWM PWM_DFF2(clk,slow_clk_enable,tmp1, tmp2); 
 assign duty_inc =  tmp1 & (~ tmp2) & slow_clk_enable;
 // debouncing FFs for decreasing button
 DFF_PWM PWM_DFF3(clk,slow_clk_enable,decrease_duty, tmp3);
 DFF_PWM PWM_DFF4(clk,slow_clk_enable,tmp3, tmp4); 
 assign duty_dec =  tmp3 & (~ tmp4) & slow_clk_enable;
 // vary the duty cycle using the debounced buttons above


 always @(posedge clk)
 begin
   if(duty_inc==1 && DUTY_CYCLE <= 60000) 
    DUTY_CYCLE <= DUTY_CYCLE + 6000;// increase duty cycle by 10%
   else if(duty_dec==1 && DUTY_CYCLE>=1) 
    DUTY_CYCLE <= DUTY_CYCLE - 6000;//decrease duty cycle by 10%
 end 
// Create 50 Hz PWM signal with variable duty cycle controlled by 2 buttons 
 always @(posedge clk)
 begin
   counter_PWM <= counter_PWM + 1;
   if(counter_PWM>=240000) 
    counter_PWM <= 0;
 end
 assign PWM_OUT = counter_PWM < DUTY_CYCLE ? 1:0;
endmodule
// Debouncing DFFs for push buttons on FPGA
module DFF_PWM(clk,en,D,Q);
input clk,en,D;
output reg Q;
always @(posedge clk)
begin 
 if(en==1) // slow clock enable signal 
  Q <= D;
end 
endmodule 