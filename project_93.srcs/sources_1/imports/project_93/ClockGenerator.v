`timescale 1ns / 1ps

module ClockGenerator (
    input sys_clkn,
    input sys_clkp,     
    input [23:0] ClkDivThreshold,
    input [23:0] ClkDivThreshold_I2C,
    output reg FSM_Clk,    
    output reg ILA_Clk,
    output reg FSM_Clk_I2C,    
    output reg ILA_Clk_I2C,
    output reg slow_clk,
    output reg ILA_Clk_I2C
    );

    //Generate high speed main clock from two differential clock signals        
    wire clk;
    reg [23:0] ClkDiv = 24'd0; 
    reg [23:0] ClkDiv_I2C = 24'd0;
     
    reg [23:0] ClkDivILA = 24'd0;     
    reg [23:0] ClkDivILA_I2C = 24'd0;    

    IBUFGDS osc_clk(
        .O(clk),
        .I(sys_clkp),
        .IB(sys_clkn)
    );    
         
    // Initialize the two registers used in this module  
    initial begin
        FSM_Clk = 1'b0;        
        ILA_Clk = 1'b0;
    end
 
    // We derive a clock signal that will be used for sampling signals for the ILA
    // This clock will be 10 times slower than the system clock.    
    always @(posedge clk) begin        
        if (ClkDivILA == 5) begin
            ILA_Clk <= !ILA_Clk;                       
            ClkDivILA <= 0;
        end else begin                        
            ClkDivILA <= ClkDivILA + 1'b1;
        end
    end      

    // We will derive a clock signal for the finite state machine from the ILA clock
    // This clock signal will be used to run the finite state machine for the I2C protocol
    always @(posedge ILA_Clk) begin        
       if (ClkDiv == ClkDivThreshold) begin
         FSM_Clk <= !FSM_Clk;                   
         ClkDiv <= 0;
       end else begin
         ClkDiv <= ClkDiv + 1'b1;             
       end
    end
    
    
    initial begin
        FSM_Clk_I2C = 1'b0;        
        ILA_Clk_I2C = 1'b0;
    end
 
    // We derive a clock signal that will be used for sampling signals for the ILA
    // This clock will be 10 times slower than the system clock.    
    always @(posedge clk) begin        
        if (ClkDivILA_I2C == 10) begin
            ILA_Clk_I2C <= !ILA_Clk_I2C;                      
            ClkDivILA_I2C <= 0;
        end else begin                        
            ClkDivILA_I2C <= ClkDivILA_I2C + 1'b1;
        end
    end      

    // We will derive a clock signal for the finite state machine from the ILA clock
    // This clock signal will be used to run the finite state machine for the I2C protocol
    always @(posedge ILA_Clk_I2C) begin        
       if (ClkDiv_I2C == ClkDivThreshold_I2C) begin
         FSM_Clk_I2C <= !FSM_Clk_I2C;                  
         ClkDiv_I2C <= 0;
       end else begin
         ClkDiv_I2C <= ClkDiv_I2C + 1'b1;            
       end
    end    
    
    reg [23:0] clkdiv200Hz;
    reg [7:0] counter200;
    reg slow_clk;
   
    initial begin
        clkdiv200Hz = 0;
        counter200 = 8'h00;
    end
   
    always @(posedge clk) begin
        clkdiv200Hz <= clkdiv200Hz + 1'b1;
        if (clkdiv200Hz == 50000) begin                  /////////
            slow_clk <= ~slow_clk;
            clkdiv200Hz <= 0;
        end
    end
    
    
    
endmodule

