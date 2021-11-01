// Code your testbench here
// or browse Examples
`timescale 1ns / 1ps

module tb_reg_mem;

    parameter DATA_WIDTH = 8; //8 bit wide data
    parameter ADDR_BITS = 5; //32 Addresses
    

    reg [ADDR_BITS-1:0] addr;
    reg [DATA_WIDTH-1:0] data_in;
    wire [DATA_WIDTH-1:0] data_out;
    reg wen, clk;

    //Note passing of parameters syntax
    reg_mem #(DATA_WIDTH,ADDR_BITS) RM (addr, data_in, wen, clk, data_out);

    initial begin
       
        
        
        clk = 0;
        wen = 1;
       
        //Write 10-42 to addresses 0-31 
      for(int i=10;i<43;i=i+1) 
        begin
            data_in = i; 
            addr = (i+2);
            $display("Write %d to address %d",data_in,addr);
            repeat (2) #1 clk = ~clk;
        end
        wen =0;
        #1;
        //Read 10-42 to addresses 0-31
      for(int i=10;i<43;i=i+1) 
        begin
            data_in = i; 
            addr = (i+2);
            $display("Read %d from address %d",data_in,addr);
            repeat (2) #1 clk = ~clk;
        end


        
    end
endmodule

`timescale 1ns / 1ps
 
module tb_simple_CPU;
       
    parameter DATA_WIDTH = 8; //8 bit wide data
    parameter ADDR_BITS = 5; //32 Addresses
    parameter INSTR_WIDTH =20; //20b instruction
   
    reg clk, rst;
    reg [INSTR_WIDTH-1:0] instruction;
 	reg [DATA_WIDTH-1:0] output_reg;
  
   
    
    simple_cpu  #(DATA_WIDTH,ADDR_BITS,INSTR_WIDTH) SCPU_DUT(clk, rst, instruction);
  
     
  	 
    
    initial begin
        $dumpfile("dump.vcd");
      	$dumpvars();
      
        clk = 1'b1;
        rst = 1'b1;
        instruction = 20'd0;
        repeat(3) #1 clk=!clk;
        rst = 1'b0;
        repeat(2) #1 clk=!clk;
                
                
        /*Info on the simple CPU:
            * Reset sets regfile = [0,1,2,3]
            * ADD = opcode 0, SUB = opcode 1  
        */
            
                                        //ADD:    reg0  = reg1 + reg3   //1+3=4
        //In the instruction this is:    (instr)  (X1)    (X2)   (X3)  
        instruction = 20'b01000111000000000000;
        repeat(8) #1 clk=!clk; //4 rising edges
        
                                        //ADD:    reg1  = reg0 + reg3   //4+3=7
        //In the instruction this is:    (instr)  (X1)    (X2)   (X3)
        instruction = 20'b01010011000000000000;
        repeat(6) #1 clk=!clk; 
                
                                         //SUB:   reg3  = reg0 - reg2  //4-2=2  
       //In the instruction this is:    (instr)  (X1)    (X2)   (X3) 
        instruction = 20'b01110010000000000001;
        repeat(6) #1 clk=!clk;
        
                                         //STORE_R:   DATA_MEM(reg2 + 15) = reg1  //DATA_MEM(2+15)=7  
        //In the instruction this is:    (instr)               (X2)         (X1)
        instruction = 20'b11011000000011110000;
        repeat(6) #1 clk=!clk;
        
                                           //STORE_R:   DATA_MEM(reg3 + 20) = reg0  //DATA_MEM(2+20)= 4  
        //In the instruction this is:    (instr)                 (X2)         (X1)
        instruction = 20'b11001100000101100000;
        repeat(6) #1 clk=!clk;

                                           //LOAD_R:   DATA_MEM(reg2 + 15) = reg3  //reg3 = DATA_MEM(2+15)  -> reg3 becomes 7  
        //In the instruction this is:    (instr)                (X2)         (X1)
        instruction = 20'b10111000000011110000;
        repeat(7) #1 clk=!clk;
        
      
      //LOAD_R:   DATA_MEM(reg2 + 15) = reg3  //reg3 = DATA_MEM(2+15)  -> reg3 becomes 7  
        //In the instruction this is:    (instr)                (X2)         (X1)
        instruction = 20'b10111000000011110000;
      	repeat(7) #1 clk=!clk;
        
    end
    
    
endmodule