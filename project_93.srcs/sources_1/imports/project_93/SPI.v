`timescale 1ns / 1ps

module SPI_transmit(    
   // input  sys_clkn,
  //  input  sys_clkp,
    output CVM300_CLK_IN,
    input CVM300_CLK_OUT,
    output CVM300_SYS_RES_N,
    output CVM300_SPI_CLK,
    output CVM300_SPI_IN,
    output CVM300_SPI_EN,
    input CVM300_SPI_OUT,
    output CVM300_FRAME_REQ,
    
    input wire CLK_IN,
    output reg SPI_CLK,
    inout wire SPI_OUT,
    output reg SPI_IN,
    output reg SPI_EN,
    output reg [7:0] counter,

    
    output reg [7:0] State,
    input wire [31:0] PC_control,
    input wire [31:0] write_in,
    input wire [31:0] addr,
    output reg [31:0] out,
    output reg spi_in
    );
    

    assign CVM300_SPI_CLK = SPI_CLK;
    assign CVM300_SPI_IN = SPI_IN;
    assign CVM300_SPI_EN = SPI_EN;
    assign CVM300_CLK_IN = CLK_IN;
    assign CVM300_SYS_RES_N = res_n;
    assign led = 8'b0; 
//    assign ct = counter;

//    wire [23:0] ClkDivThreshold = 100;   
//    wire FSM_Clk, ILA_Clk; 
//    ClockGenerator ClockGenerator1 (  .sys_clkn(sys_clkn),
//                                      .sys_clkp(sys_clkp),                                      
//                                      .ClkDivThreshold(ClkDivThreshold),
//                                      .FSM_Clk(FSM_Clk),                                      
//                                      .ILA_Clk(ILA_Clk));

    
    initial  begin
          
        State = 8'd0; 
        res_n = 1'b0;
        SPI_CLK = 1'b0;
        SPI_IN = 1'b0;
        SPI_EN = 1'b0;
        counter = 8'b0;

    end
    
    always @(*) begin          
//        FSM_Clk_reg = CLK_IN;
//        ILA_Clk_reg = ILA_Clk;   
    end   

    

    localparam INIT1      = 8'd0;
    localparam INIT2     = 8'd1;
    localparam START     = 8'd2;
    localparam SEND_DATA = 8'd3;
    localparam READ_DATA = 8'd4;
    localparam DONE1      = 8'd5;
    localparam DONE2      = 8'd6;
    localparam DONE3      = 8'd7;



    reg [7:0] counter;
    reg [7:0] State;
    reg pc_control;
    reg res_n;
//    reg [31:0] PC_control;
//    reg [31:0] addr;
//    reg [31:0] write_in;
    reg [31:0] out;
    reg [31:0] sent;
    
    always @(posedge CLK_IN) begin  
        case (State)
            //buffer state for start up -- wait 1 microsec
            INIT1 : begin
                if(PC_control[0] == 1'b1) pc_control <= 1'b1;
                if ((counter >= 8'd20) && pc_control == 1'b1)begin
                    res_n <= 1'b1;
                    counter <= 8'd0;
                    State <= INIT2;
                end                  
                else begin
                    counter <= counter + 1;
                    
                end 
            end

            INIT2 : begin
                
//                SPI_CLK <= ~SPI_CLK;
                if(pc_control == 1'b1 &&(counter >= 8'd20)) begin
                    State <= START;
                    counter <= 8'd6;
                    SPI_EN <= 1'b1;
                    SPI_IN <= addr[7];
//                    SPI_CLK <= ~SPI_CLK;
                        
                end else begin
                    
                    SPI_CLK <= 1'b0;
                    SPI_IN <= 1'b0;
                    counter <= counter + 1;
                end
                
            end



            START : begin
                SPI_CLK <= ~SPI_CLK;
                if(SPI_CLK) begin
                    
                    SPI_IN <= addr[counter];
                    counter <= counter - 1'b1;

                    if(counter >= 8'd8) begin
                        counter <= 8'd6;
                        if(addr[7] == 1'b1) begin State <= SEND_DATA; SPI_IN <= write_in[7]; sent[7] <= write_in[7];end
                        else if (addr[7] == 1'b0)begin  State <= READ_DATA; out[7] <= SPI_OUT; end
                    end
                end
            end

            SEND_DATA : begin
                SPI_CLK <= ~SPI_CLK;
                if(SPI_CLK) begin
                    SPI_IN <= write_in[counter];
                    sent[counter] <= write_in[counter];
                    counter <= counter - 1'b1;
                    if(counter >= 8'd8) begin 
                         
                        State <= DONE1;
                    end
                end
            

            end


            READ_DATA : begin
                SPI_CLK <= ~SPI_CLK;
                if(SPI_CLK) begin
                    out[counter] <= SPI_OUT; 
                    counter <= counter - 1'b1;
                    if(counter >= 8'd8) begin 
                        
                        State <= DONE1;
                    end
                end

            end
            
            DONE1 : begin
                State <= DONE2;
                counter <= 0;
                SPI_CLK <= 0;
                SPI_EN <= 1'b0;
//                SPI_CLK = ~SPI_CLK;
//                if(PC_control[0] == 1'b0) pc_control <= 1'b0;
                
            end

            DONE2 : begin
                SPI_EN <= 1'b0;
                if(PC_control[0] == 1'b0) pc_control <= 1'b0;
                if(pc_control == 1'b0) begin
                    State <= INIT1;
                
                end


            end
            
//            DONE3 : begin
//                State <= INIT1;

//            end
            
            default : begin
                //   error_bit <= 0;
            end                              
        endcase                           
    end      

    
 

endmodule
