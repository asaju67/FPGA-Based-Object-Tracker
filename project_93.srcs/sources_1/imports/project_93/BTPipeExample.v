`timescale 1ns / 1ps

module JTEG_Test_File( 
    input [3:0] button,
    output [7:0] led,
    input sys_clkn,
    input sys_clkp,  

    output CVM300_CLK_IN,
    input CVM300_CLK_OUT,
    output CVM300_SYS_RES_N,
    output CVM300_SPI_CLK,
    output CVM300_SPI_IN,
    output CVM300_SPI_EN,
    input CVM300_SPI_OUT,
    output CVM300_FRAME_REQ,
    input [9:0] CVM300_D,
    input CVM300_Line_valid,
    input CVM300_Data_valid,
    
     output ADT7420_A0,
    output ADT7420_A1,
    output I2C_SCL_1,
    inout I2C_SDA_1,
    
    
    output  PMOD_A1,
    output  PMOD_A2,
    
    input  [4:0] okUH,
    output [2:0] okHU,
    inout  [31:0] okUHU,
    inout  okAA      
);
    assign CVM300_FRAME_REQ = FRAME_REQ;
    
   
    wire TrigerEvent;    
    wire SPI_CLK, SPI_EN, spi_in; 
    wire SPI_OUT = CVM300_SPI_OUT;
    wire [7:0] ct, SPI_State, I2C_State;
    wire [31:0] PC_control, write_in, out, addr, sent, Reset_Counter, FIFO_data_out ;
    wire [31:0] I2C_control, SAD, SUB, write_in_I2C, read1byte, flag_write, pulses;
    wire [31:0] out_x,out_y,out_z;
    wire CLK_OUT = CVM300_CLK_OUT;
    
    localparam STATE_INIT                = 8'd0;
    localparam STATE_RESET               = 8'd1;   
    localparam STATE_DELAY               = 8'd2;
    localparam STATE_RESET_FINISHED      = 8'd3;
    localparam STATE_ENABLE_WRITING      = 8'd4;
    localparam STATE_COUNT               = 8'd5;
    localparam STATE_FINISH              = 8'd6;
   
    reg [31:0] counter = 8'd0;
    reg [15:0] counter_delay = 16'd0;
    reg [7:0] BT_State = STATE_INIT;
    reg [7:0] led_register = 0;
    reg [3:0] button_reg, write_enable_counter;  
    reg write_reset, read_reset, write_enable, FRAME_REQ;
    
    wire FIFO_read_enable, FIFO_BT_BlockSize_Full, FIFO_full, FIFO_empty, BT_Strobe;
    
    assign led[0] = ~FIFO_empty; 
    assign led[1] = ~FIFO_full;
    assign led[2] = ~FIFO_BT_BlockSize_Full;
    assign led[3] = ~FIFO_read_enable;  
    assign led[7] = ~read_reset;
    assign led[6] = ~write_reset;
    initial begin
        write_reset <= 1'b0;
        read_reset <= 1'b0;
        write_enable <= 1'b1;    
    end
    
    
                                         
    always @(posedge FSM_Clk) begin     
        button_reg <= ~button;   // Grab the values from the button, complement and store them in register                
        if (Reset_Counter[0] == 1'b1) BT_State <= STATE_RESET;
        
        case (BT_State)
            STATE_INIT:   begin                              
                write_enable <= 1'b0;
                if (Reset_Counter[0] == 1'b1) BT_State <= STATE_RESET;                
            end
            
            STATE_RESET:   begin
                counter <= 0;
                counter_delay <= 0;
                write_reset <= 1'b1;
                read_reset <= 1'b1;                
                if (Reset_Counter[0] == 1'b0) BT_State <= STATE_RESET_FINISHED;             
            end                                     
 
            STATE_RESET_FINISHED:   begin
                write_reset <= 1'b0;
                read_reset <= 1'b0;                 
                BT_State <= STATE_DELAY;                                   
            end   
                          
            STATE_DELAY:   begin
                if (counter_delay == 16'b0000_1111_1111_1111)  BT_State <= STATE_ENABLE_WRITING;
                else counter_delay <= counter_delay + 1;
            end
            
             STATE_ENABLE_WRITING:   begin
                write_enable <= 1'b1;
//                CVM_FRAM_REQ <= 1;
                FRAME_REQ <= 1'b1;
                BT_State <= STATE_COUNT;
             end
                                  
             STATE_COUNT:   begin
                // write_enable <= 0;    
                counter <= counter+1;
                     
                BT_State <= STATE_INIT;
             end
            
            STATE_FINISH:   begin                         
                write_enable <= 1'b0; 
                FRAME_REQ <= 1'b0;                                                          
           end

        endcase
    end
    
    
    
     I2C_Transmit I2C_Test1 (        

        .ADT7420_A0(ADT7420_A0),
        .ADT7420_A1(ADT7420_A1),
        .I2C_SCL_1(I2C_SCL_1),
        .I2C_SDA_1(I2C_SDA_1),            
        .FSM_Clk(FSM_Clk_I2C),        

        .SCL(SCL),
        .SDA(SDA),
        .State(I2C_State),
        .PC_control(I2C_control),
        .SAD(SAD),
        .write_in(write_in_I2C),
        .SUB(SUB),
        .flag_write(flag_write),
        .read1byte(read1byte),
       
        .out_x(out_x),
        .out_y(out_y),
        .out_z(out_z)
   
        );
        
        
    
    wire [23:0] ClkDivThreshold = 1;   
    wire [23:0] ClkDivThreshold_I2C = 100;
    wire FSM_Clk, ILA_Clk, ILA_Clk_I2C; 
    wire FSM_Clk_I2C;
    
    ClockGenerator ClockGenerator1 (  .sys_clkn(sys_clkn),
                                      .sys_clkp(sys_clkp),                                      
                                      .ClkDivThreshold(ClkDivThreshold),
                                      .ClkDivThreshold_I2C(ClkDivThreshold_I2C),
                                      .FSM_Clk(FSM_Clk),   
                                      .FSM_Clk_I2C(FSM_Clk_I2C),                                    
                                      .ILA_Clk(ILA_Clk),
                                      .slow_clk(slow_clk),
                                      .ILA_Clk_I2C(ILA_Clk_I2C)
                                      );
                                      
    
    assign TrigerEvent = I2C_control[0]; 
    //Instantiate the module that we like to test
    SPI_transmit SPI_Test1 (  
        
        .CVM300_CLK_IN(CVM300_CLK_IN),
//        .CVM300_CLK_OUT(CVM300_CLK_OUT),
        .CVM300_SYS_RES_N(CVM300_SYS_RES_N),
        .CVM300_SPI_CLK(CVM300_SPI_CLK),
        .CVM300_SPI_IN(CVM300_SPI_IN),
        .CVM300_SPI_EN(CVM300_SPI_EN),
        .CVM300_SPI_OUT(CVM300_SPI_OUT),


//        .CLK_OUT(CLK_OUT),
        .SPI_EN(SPI_EN),
        .SPI_CLK(SPI_CLK),
        .SPI_IN(SPI_IN),
        .SPI_OUT(SPI_OUT),
        .CLK_IN(FSM_Clk),

        .State(SPI_State),
        .PC_control(PC_control),
        .counter(ct),
        .out(out),
        .write_in(write_in),
        .addr(addr)

        );
    
    
    wire okClk;
    wire [112:0]    okHE;  //These are FrontPanel wires needed to IO communication    
    wire [64:0]     okEH;  //These are FrontPanel wires needed to IO communication 
   
    
    //This is the OK host that allows data to be sent or recived    
    okHost hostIF (
        .okUH(okUH),
        .okHU(okHU),
        .okUHU(okUHU),
        .okClk(okClk),
        .okAA(okAA),
        .okHE(okHE),
        .okEH(okEH)
    );
    
    //  PC_controll is a wire that contains data sent from the PC to FPGA.
    //  The data is communicated via memeory location 0x00
    okWireIn wire10 (   .okHE(okHE), 
                        .ep_addr(8'h01), 
                        .ep_dataout(Reset_Counter));
    okWireIn wire11 (   .okHE(okHE), 
                        .ep_addr(8'h02), 
                        .ep_dataout(write_in));
    okWireIn wire12 (   .okHE(okHE), 
                        .ep_addr(8'h03), 
                        .ep_dataout(addr));
    okWireIn wire13 (   .okHE(okHE), //unused
                        .ep_addr(8'h04), 
                        .ep_dataout(frame_req));
    okWireIn wire14 (   .okHE(okHE), 
                        .ep_addr(8'h05), 
                        .ep_dataout(PC_control));
    okWireIn wire15 (   .okHE(okHE), 
                        .ep_addr(8'h06), 
                        .ep_dataout(SAD));
    okWireIn wire16 (   .okHE(okHE), 
                        .ep_addr(8'h07), 
                        .ep_dataout(SUB));
    okWireIn wire17 (   .okHE(okHE), 
                        .ep_addr(8'h08), 
                        .ep_dataout(write_in_I2C));
    okWireIn wire18 (   .okHE(okHE), 
                        .ep_addr(8'h09), 
                        .ep_dataout(flag_write));
    okWireIn wire19 (   .okHE(okHE), 
                        .ep_addr(8'h0a), 
                        .ep_dataout(read1byte));
    okWireIn wire20 (   .okHE(okHE), 
                        .ep_addr(8'h0b), 
                        .ep_dataout(direction));                    
    okWireIn wire21 (   .okHE(okHE), 
                        .ep_addr(8'h0c), 
                        .ep_dataout(pulses));

    okWireIn wire22 (   .okHE(okHE), 
                        .ep_addr(8'h0d), 
                        .ep_dataout(I2C_control));
    
    localparam  endPt_count = 7;///
    wire [endPt_count*65-1:0] okEHx;///
    okWireOR # (.N(endPt_count)) wireOR (okEH, okEHx);///

    okWireOut wire31 (  .okHE(okHE), 
            .okEH(okEHx[ 1*65 +: 65 ]),
            .ep_addr(8'h21), 
            .ep_datain(out));
    okWireOut wire32 (  .okHE(okHE), 
            .okEH(okEHx[ 2*65 +: 65 ]),
            .ep_addr(8'h22), 
            .ep_datain(pulse_count));    
    
    okWireOut wire33 (  .okHE(okHE),
                        .okEH(okEHx[ 3*65 +: 65 ]),
                        .ep_addr(8'h23),
                        .ep_datain(out_x));
                       
    
    okWireOut wire34 (  .okHE(okHE),
                        .okEH(okEHx[ 4*65 +: 65 ]),
                        .ep_addr(8'h24),
                        .ep_datain(out_y));
    
    okWireOut wire35 (  .okHE(okHE),
                        .okEH(okEHx[ 5*65 +: 65 ]),
                        .ep_addr(8'h25),
                        .ep_datain(out_z));                   
    
    
    
                        
    reg [31:0] pulse_count = 0;
    reg EN;
    reg DIR;
    assign PMOD_A1 = EN;
    assign PMOD_A2 = DIR;


    always @(posedge slow_clk) begin 
        DIR <= (direction)? 1'b1: 1'b0; 
        if ((pulse_count < pulses)) begin
            pulse_count <= pulse_count + 1'b1;
            EN <= 1'b1;
        end

        else if (pulse_count == pulses) begin
            pulse_count <= pulse_count;
            EN <= 1'b0;
        end

        else if (pulse_count > pulses) begin
            pulse_count <= 0;
            EN <= 1'b0;
        end

    end  
   
   
                                       
          
    
    okBTPipeOut CounterToPC (
        .okHE(okHE), 
        .okEH(okEHx[ 0*65 +: 65 ]),
        .ep_addr(8'ha0), 
        .ep_datain({FIFO_data_out[7:0],FIFO_data_out[15:8],FIFO_data_out[23:16],FIFO_data_out[31:24]}),
        .ep_read(FIFO_read_enable),
        .ep_blockstrobe(BT_Strobe), 
        .ep_ready(FIFO_BT_BlockSize_Full)
    );
    
    fifo_generator_0 FIFO_for_Counter_BTPipe_Interface (
        .wr_clk(~CVM300_CLK_OUT),
        .wr_rst(write_reset),
        .rd_clk(okClk),
        .rd_rst(read_reset),
        .din(CVM300_D[9:2]),
        .wr_en(CVM300_Data_valid),
        .rd_en(FIFO_read_enable),
        .dout(FIFO_data_out),
        .full(FIFO_full),
        .empty(FIFO_empty),       
        .prog_full(FIFO_BT_BlockSize_Full)        
    );
    
    //Instantiate the ILA module
    ila_0 ila_sample12 ( 
        .clk(ILA_Clk),
        .probe0({SPI_State[7:0],FSM_Clk_I2C, I2C_State[7:0], SDA, SCL, SPI_CLK, SPI_IN, SPI_OUT, SPI_EN, BT_State[7:0], FRAME_REQ, Reset_Counter[0],NP_variable_1, slow_clk, DP_variable_2}),                             
        .probe1({FSM_Clk, TrigerEvent})
        );                        
endmodule
