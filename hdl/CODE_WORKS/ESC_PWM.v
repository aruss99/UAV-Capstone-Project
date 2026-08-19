module PWM_Generator_Verilog
 (
 clk, // 12MHz clock input 

 // for Servo, at 50 Hz 
 increase_duty0, // input to increase 10% duty cycle 
 decrease_duty0, // input to decrease 10% duty cycle 

 // for ESC, at 500 Hz
 increase_duty1, // input to increase 10% duty cycle 
 decrease_duty1, // input to decrease 10% duty cycle 


 PWM_OUT0, // 50Hz PWM output signal  
 PWM_OUT1 // 500 Hz PWM output, for ESC


    );
 input clk;
 input increase_duty0;
 input decrease_duty0;
 input increase_duty1;
 input decrease_duty1;
 output PWM_OUT0;
 output PWM_OUT1; 


 wire slow_clk_enable; // slow clock enable signal for debouncing FFs
 reg[27:0] counter_debounce=0;// counter for creating slow clock enable signals 
 wire tmp1,tmp2,duty_inc0;// temporary flip-flop signals for debouncing the increasing button
 wire tmp3,tmp4,duty_dec0;// temporary flip-flop signals for debouncing the decreasing button
 wire tmp5,tmp6, duty_inc1;// temporary flip-flop signals for debouncing the increasing button - ESC
 wire tmp7,tmp8, duty_dec1;// temporary flip-flop signals for debouncing the decreasing button - ESC


 reg[18:0] counter_PWM0=0;// counter for creating 50 Hz PWM signal
 reg[16:0] counter_PWM1=0;// counter for creating 500 Hz PWM signal


 reg[18:0] DUTY_CYCLE0=60000; 
 reg[16:0] DUTY_CYCLE1=6000; 

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


 // debouncing FFs for increasing button - Servo
 DFF_PWM PWM_DFF1(clk,slow_clk_enable,increase_duty0,tmp1);
 DFF_PWM PWM_DFF2(clk,slow_clk_enable,tmp1, tmp2); 

 assign duty_inc0 =  tmp1 & (~ tmp2) & slow_clk_enable;

 // debouncing FFs for decreasing button
 DFF_PWM PWM_DFF3(clk,slow_clk_enable,decrease_duty0, tmp3);
 DFF_PWM PWM_DFF4(clk,slow_clk_enable,tmp3, tmp4); 
 assign duty_dec0 =  tmp3 & (~ tmp4) & slow_clk_enable;
 // vary the duty cycle using the debounced buttons above



 // debouncing FFs for increasing button - ESC
 DFF_PWM PWM_DFF5(clk,slow_clk_enable,increase_duty1,tmp5);
 DFF_PWM PWM_DFF6(clk,slow_clk_enable,tmp5, tmp6); 

 assign duty_inc1 =  tmp5 & (~ tmp6) & slow_clk_enable;

 // debouncing FFs for decreasing button
 DFF_PWM PWM_DFF7(clk,slow_clk_enable,decrease_duty1, tmp7);
 DFF_PWM PWM_DFF8(clk,slow_clk_enable,tmp7, tmp8); 
 assign duty_dec1 =  tmp7 & (~ tmp8) & slow_clk_enable;
 // vary the duty cycle using the debounced buttons above






 always @(posedge clk)
 begin
   if(duty_inc0==1 && DUTY_CYCLE0 <= 60000) 
    DUTY_CYCLE0 <= DUTY_CYCLE0 + 6000;// increase duty cycle by 10%
   else if(duty_dec0==1 && DUTY_CYCLE0>=1) 
    DUTY_CYCLE0 <= DUTY_CYCLE0 - 6000;//decrease duty cycle by 10%
 end 
// Create 50 Hz PWM signal with variable duty cycle controlled by 2 buttons 
 always @(posedge clk)
 begin
   counter_PWM0 <= counter_PWM0 + 1;
   if(counter_PWM0>=240000) 
    counter_PWM0 <= 0;
 end
 assign PWM_OUT0 = counter_PWM0 < DUTY_CYCLE0 ? 1:0;



 always @(posedge clk)
 begin
   if(duty_inc1==1 && DUTY_CYCLE1 <= 6000) 
    DUTY_CYCLE1 <= DUTY_CYCLE1 + 600;// increase duty cycle by 10%
   else if(duty_dec1==1 && DUTY_CYCLE1>=1) 
    DUTY_CYCLE1 <= DUTY_CYCLE1 - 600;//decrease duty cycle by 10%
 end 
// Create 500 Hz PWM signal with variable duty cycle controlled by 2 buttons 
 always @(posedge clk)
 begin
   counter_PWM1 <= counter_PWM1 + 1;
   if(counter_PWM1>=24000) 
    counter_PWM1 <= 0;
 end
 assign PWM_OUT1 = counter_PWM1 < DUTY_CYCLE1 ? 1:0;
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