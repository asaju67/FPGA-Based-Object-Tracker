`timescale 1ns / 1ps

module I2C_Transmit(    
    output [7:0] led,
    input  FSM_Clk,
    output ADT7420_A0,
    output ADT7420_A1,
    output I2C_SCL_1,
    inout  I2C_SDA_1,        
    output reg ACK_bit,
    output reg SCL,
    output reg SDA,
    output reg [7:0] State,
    input wire [31:0] PC_control,
    input wire [31:0] SAD,
    input wire [31:0] write_in,
    input wire [31:0] SUB,
    input wire [31:0] flag_write,
    input wire [31:0] read1byte,
//    input  wire    [4:0] okUH,
//    output wire    [2:0] okHU,
//    inout  wire    [31:0] okUHU,   
//    inout wire okAA     
    output reg [31:0] out_x,
    output reg [31:0] out_y,
    output reg [31:0] out_z
    );
    
    //Instantiate the ClockGenerator module, where three signals are generate:
    //High speed CLK signal, Low speed FSM_Clk signal     
    
                                        
    reg [7:0] unwanted;        
    reg error_bit = 1'b1;
//    reg [6:0] SAD; // make okwire in
//    reg [7:0] write_in; // make okwire in
//    reg [7:0] SUB; // make okwire in
    reg [2:0] counter;
//    reg [15:0] out_x;  //make okwire out
//    reg [15:0] out_y; //make okwire out
//    reg [15:0] out_z; //make okwire out
//    reg flag_write; // make okwire in
    reg MSB_flag;
    reg byte_flag;     
       
    localparam STATE_INIT       = 8'd0;    
    
    assign led[0] = PC_control[0];
    assign led[7] = ACK_bit;
    assign led[6] = error_bit;
    assign ADT7420_A0 = 1'b0;
    assign ADT7420_A1 = 1'b0;
    assign I2C_SCL_1 = SCL;
    assign I2C_SDA_1 = SDA; 
   
    

    
    initial  begin
        SCL = 1'b1;
        SDA = 1'b1;
        ACK_bit = 1'b1;  
        State = 8'd0; 
        counter = 8'd7;
        MSB_flag = 1'b1;
        byte_flag = 1'b0;
        
    end
    

    
                               
    always @(posedge FSM_Clk) begin                       
        case (State)
            // Press Button[3] to start the state machine. Otherwise, stay in the STATE_INIT state        
            STATE_INIT : begin
                 if (PC_control[0] == 1'b1) State <= 8'd1;                    
                 else begin                 
                      SCL <= 1'b1;
                      SDA <= 1'b1;
                      State <= 8'd0;
                      counter <= 8'd6;
                      out_x <= 32'b0;
                      out_y <= 32'b0;
                      out_z <= 32'b0;
                  end
            end            
            
            // This is the Start sequence            
            8'd1 : begin SCL <= 1'b1; SDA <= 1'b0; State <= State + 1'b1; end                                  
            8'd2 : begin SCL <= 1'b0; SDA <= 1'b0; State <= State + 1'b1; end
            
            // Default device address is 0011001 + 0 (write)
    
    
            // Transmit SAD + write
            8'd3 : begin SCL <= 1'b0; SDA <= SAD[counter]; State <= State + 1'b1; end   // setting bit 6 of address
            8'd4 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd5 : begin SCL <= 1'b1; State <= State + 1'b1; counter <= counter - 1'b1; end   
            8'd6 : begin SCL <= 1'b0; if(counter == 8'd7) begin State <= State + 1'b1; counter <= 8'd7; end else State <= 8'd3; end   

            8'd7 : begin SCL <= 1'b0; SDA <= 1'b0; State <= State + 1'b1; end   // setting bit 6 of address
            8'd8 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd9 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd10 : begin SCL <= 1'b0; State <= State + 1'b1; end
            
                        
            // read the ACK bit from the sensor and display it on LED[7]
            8'd11 : begin SCL <= 1'b0; SDA <= 1'bz; State <= State + 1'b1; end   
            8'd12 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd13 : begin SCL <= 1'b1;  ACK_bit <= SDA; State <= State + 1'b1; end   
            8'd14 : begin SCL <= 1'b0; SDA <= 1'bz; if(ACK_bit == 1'b0) begin State <= State + 1'b1; end else State <= 8'd255;  end  
            
            
            // Transmit SUB (this will change from different measurements)
            // Set to 1 (hardcoded to 1) + 010 1000 (28) -- only for accelerometer
            
            // transmit SUB
            8'd15 : begin SCL <= 1'b0; SDA <= SUB[counter]; State <= State + 1'b1; end   // setting bit 6 of address
            8'd16 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd17 : begin SCL <= 1'b1; State <= State + 1'b1; counter <= counter - 1'b1; end   
            8'd18 : begin SCL <= 1'b0; if(counter == 8'd7) begin State <= State + 1'b1; end else State <= 8'd15; end
            
            
            
            // read the ACK bit from the sensor and display it on LED[7]
            8'd19 : begin SCL <= 1'b0; SDA <= 1'bz; State <= State + 1'b1; end   
            8'd20 : begin SCL <= 1'b1;  State <= State + 1'b1; end   
            8'd21 : begin SCL <= 1'b1; ACK_bit <= SDA; State <= State + 1'b1; end   
            8'd22 : begin SCL <= 1'b0; SDA <= 1'bz; begin 
                if(ACK_bit == 1'b0) begin
                    if(flag_write[0] == 1'b1) State <= 8'd23;
                    else State <= 8'd31; end
                else State <= 8'd255; end 
            end
            
                

            //write branch (single byte)
            8'd23 : begin SCL <= 1'b0; SDA <= write_in[counter]; State <= State + 1'b1; end   // setting bit 6 of address
            8'd24 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd25 : begin SCL <= 1'b1; State <= State + 1'b1; counter <= counter - 1'b1; end   
            8'd26 : begin SCL <= 1'b0; if(counter == 8'd7) begin State <= State + 1'b1; counter <= 8'd7; end else State <= 8'd23; end   

            // read the ACK bit from the sensor and display it on LED[7]
            8'd27 : begin SCL <= 1'b0; SDA <= 1'bz; State <= State + 1'b1; end   
            8'd28 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd29 : begin SCL <= 1'b1; ACK_bit <= SDA; State <= State + 1'b1; end   
            8'd30 : begin SCL <= 1'b0; SDA <= 1'bz; if(ACK_bit == 1'b0) State <= 8'd95; else State <= 8'd255; end
            
            
            //read branch
            // Repeat Start
            8'd31 : begin SCL <= 1'b0; SDA <= 1'b1; State <= State + 1'b1;  end   
            8'd32 : begin SCL <= 1'b1; SDA <= 1'b1; State <= State + 1'b1;  end
            8'd33 : begin SCL <= 1'b1; SDA <= 1'b0; State <= State + 1'b1;  end   
            8'd34 : begin SCL <= 1'b0; SDA <= 1'b0; State <= State + 1'b1;  counter <= 8'd6; end
            
            // Transmit Slave Address Device + read
            8'd35 : begin SCL <= 1'b0; SDA <= SAD[counter]; State <= State + 1'b1; end  
            8'd36 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd37 : begin SCL <= 1'b1; State <= State + 1'b1; counter <= counter - 1; end   
            8'd38 : begin SCL <= 1'b0; if(counter == 8'd7) begin State <= State + 1'b1; counter <= 8'd7; end else State <= 8'd35; end   

            8'd39 : begin SCL <= 1'b0; SDA <= 1'b1; State <= State + 1'b1; end   // setting bit 6 of address
            8'd40 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd41 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd42 : begin SCL <= 1'b0; SDA <= 1'b1; State <= State + 1'b1; end
            
                        
            // read the ACK bit from the sensor and display it on LED[7]
            8'd43 : begin SCL <= 1'b0; SDA <= 1'bz; State <= State + 1'b1; end   
            8'd44 : begin SCL <= 1'b1;State <= State + 1'b1; end   
            8'd45 : begin SCL <= 1'b1; ACK_bit <= SDA; State <= State + 1'b1; end   
            8'd46 : begin SCL <= 1'b0; SDA <= 1'bz;
                if(SAD[6:0] == 7'b0011110) MSB_flag = 1'b1;
                else MSB_flag <= 1'b0;
                if(ACK_bit == 1'b0) State <= State + 1'b1; else State <= 8'd255; 

                end  //check if reading magnetometer or accelerometer then change LSB to MSB or MSB to LSB
            

            //read x////////////////////////////////////
            8'd47 : begin SCL <= 1'b0; out_x[counter + 8'd8 * MSB_flag] <= SDA; State <= State + 1'b1; end   
            8'd48 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd49 : begin SCL <= 1'b1; State <= State + 1'b1; counter <= counter - 1'b1; end   
            8'd50 : begin SCL <= 1'b0; 
                if(counter == 8'd7) begin 
                   State <= State + 1'b1;
                   MSB_flag <= ~MSB_flag; 
                end else State <= 8'd47; end
            
            
            
            // read the ACK bit from the sensor and display it on LED[7]
            8'd51 : begin SCL <= 1'b0; SDA <= read1byte[0]; State <= State + 1'b1; end   
            8'd52 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd53 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd54 : begin SCL <= 1'b0; SDA <= 1'bz; if(read1byte[0] == 1'b0) State <= State + 1'b1; else State <= 8'd95; end
            
            8'd55 : begin SCL <= 1'b0; out_x[counter + 8'd8 * MSB_flag] <= SDA; State <= State + 1'b1; end   
            8'd56 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd57 : begin SCL <= 1'b1; State <= State + 1'b1; counter <= counter - 1'b1; end   
            8'd58 : begin SCL <= 1'b0; 
                if(counter == 8'd7) begin 
                   State <= State + 1'b1;
                   MSB_flag <= ~MSB_flag; 
                end else State <= 8'd55; end
            
            
            
            // read the ACK bit from the sensor and display it on LED[7]
            8'd59 : begin SCL <= 1'b0; SDA <= 1'b0; State <= State + 1'b1; end   
            8'd60 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd61 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd62 : begin SCL <= 1'b0; SDA <= 1'bz; State <= 8'd63; end
            
            //read y////////////////////////////////
            8'd63 : begin SCL <= 1'b0; out_y[counter + 8'd8 * MSB_flag] <= SDA; State <= State + 1'b1; end   // setting bit 6 of address
            8'd64 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd65 : begin SCL <= 1'b1; State <= State + 1'b1; counter <= counter - 1'b1; end   
            8'd66 : begin SCL <= 1'b0;  
                if(counter == 8'd7) begin 
                   State <= State + 1'b1;
                   MSB_flag <= ~MSB_flag; 
                end else State <= 8'd63; end 
                            
            // read the ACK bit from the sensor and display it on LED[7]
            8'd67 : begin SCL <= 1'b0; SDA <= 1'b0; State <= State + 1'b1; end   
            8'd68 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd69 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd70 : begin SCL <= 1'b0; SDA <= 1'bz; State <= State + 1'b1; end
            
            8'd71 : begin SCL <= 1'b0; out_y[counter + 8'd8 * MSB_flag] <= SDA; State <= State + 1'b1; end   // setting bit 6 of address
            8'd72 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd73 : begin SCL <= 1'b1; State <= State + 1'b1; counter <= counter - 1'b1; end   
            8'd74 : begin SCL <= 1'b0;  
                if(counter == 8'd7) begin 
                   State <= State + 1'b1;
                   MSB_flag <= ~MSB_flag; 
                end else State <= 8'd71; end
            
            // read the ACK bit from the sensor and display it on LED[7]
            8'd75 : begin SCL <= 1'b0; SDA <= 1'b0; State <= State + 1'b1; end   
            8'd76 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd77 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd78 : begin SCL <= 1'b0; SDA <= 1'bz; State <= State + 1'b1; end
            
            //read z///////////////////////////////
            8'd79 : begin SCL <= 1'b0; out_z[counter + 8'd8 * MSB_flag] <= SDA; State <= State + 1'b1; end   // setting bit 6 of address
            8'd80 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd81 : begin SCL <= 1'b1; State <= State + 1'b1; counter <= counter - 1'b1; end   
            8'd82 : begin SCL <= 1'b0; 
                if(counter == 8'd7) begin 
                   State <= State + 1'b1;
                   MSB_flag <= ~MSB_flag; 
                end else State <= 8'd79; end
            
            
            
            // ACK bit from the sensor and display it on LED[7]
            8'd83 : begin SCL <= 1'b0; SDA <= 1'b0; State <= State + 1'b1; end   
            8'd84 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd85 : begin SCL <= 1'b1; State <= State + 1'b1; end    
            8'd86 : begin SCL <= 1'b0; SDA <= 1'bz; State <= State + 1'b1; end


            8'd87 : begin SCL <= 1'b0; out_z[counter + 8'd8 * MSB_flag] <= SDA; State <= State + 1'b1; end   // setting bit 6 of address
            8'd88 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd89 : begin SCL <= 1'b1; State <= State + 1'b1; counter <= counter - 1'b1; end   
            8'd90 : begin SCL <= 1'b0; 
                if(counter == 8'd7) begin 
                   State <= State + 1'b1;
                   MSB_flag <= ~MSB_flag; 
                end else State <= 8'd87; end
   
            // NACK 
            8'd91 : begin SCL <= 1'b0; SDA <= 1'b1; State <= State + 1'b1; end   
            8'd92 : begin SCL <= 1'b1; State <= State + 1'b1; end   
            8'd93 : begin SCL <= 1'b1; State <= State + 1'b1; end    
            8'd94 : begin SCL <= 1'b0; SDA <= 1'bz; State <= State + 1'b1; end
   
            
            
            //stop bit sequence and go back to STATE_INIT           
            8'd95 : begin SCL <= 1'b0; SDA <= 1'b0; State <= State + 1'b1; end   
            8'd96 : begin SCL <= 1'b1; SDA <= 1'b0;State <= State + 1'b1; end                                    
            8'd97 : begin SCL <= 1'b1; SDA <= 1'b1;
                  if(PC_control[0] == 1'b0) State <= STATE_INIT; end              
            
            //If the FSM ends up in this state, there was an error in teh FSM code
            //LED[6] will be turned on (signal is active low) in that case.
            default : begin
                  error_bit <= 0;
            end                              
        endcase                           
    end      
    
    
    // OK Interface
                
endmodule
