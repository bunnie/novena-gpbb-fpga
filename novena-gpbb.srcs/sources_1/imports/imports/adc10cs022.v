`timescale 1 ns / 1 ps
module adc10cs022(
		  output wire DIG_ADC_CS,
		  output wire DIG_ADC_IN,
		  input wire DIG_ADC_OUT,
		  output wire DIG_ADC_SCLK,

		  output reg [9:0] adc_in,
		  input wire [2:0] adc_chan,
		  output reg adc_valid,
		  input wire adc_go,
		  
		  input wire clk_3p2MHz,
		  input wire reset
		  );
   
   /////////////////////////////////
   //////////////////////// ADC block
   /////////////////////////////////
   parameter ADC_INIT           =   5'b1 << 0;
   parameter ADC_START          =   5'b1 << 1;
   parameter ADC_SHIFT          =   5'b1 << 2;
   parameter ADC_SHIFT_TERM     =   5'b1 << 3;
   parameter ADC_DONE           =   5'b1 << 4;

   parameter ADC_nSTATES = 5;

   reg [(ADC_nSTATES-1):0] ADC_cstate = {{(ADC_nSTATES-1){1'b0}}, 1'b1};
   reg [(ADC_nSTATES-1):0] ADC_nstate;

   wire 		   motor_reset_3p2;
   sync_reset  motor_reset_sync_3p2(
			  .clk(clk_3p2MHz),
			  .glbl_reset(reset),
			  .reset(motor_reset_3p2) );


   reg 			   adc_go_d;
   reg 			   adc_go_edge;
   reg [4:0] 		   adc_shift_count;
   reg [15:0] 		   adc_shift_out;
   reg [15:0] 		   adc_shift_in;
   reg  		   adc_cs;
   
   always @(posedge clk_3p2MHz) begin
      adc_go_d <= adc_go;
      adc_go_edge <= adc_go & !adc_go_d;
   end // always @ (posedge clk)
   
   always @ (posedge clk_3p2MHz) begin
      if (motor_reset_3p2)
	ADC_cstate <= ADC_INIT; 
      else
	ADC_cstate <= ADC_nstate;
   end

   always @ (*) begin
      case (ADC_cstate) //synthesis parallel_case full_case
	ADC_INIT: begin
	   if( adc_go_edge ) begin 
	      ADC_nstate = ADC_START;
	   end else begin
	      ADC_nstate = ADC_INIT;
	   end
	end // case: ADC_INIT

	ADC_START: begin
	   ADC_nstate = ADC_SHIFT;
	end

	ADC_SHIFT: begin
	   ADC_nstate = (adc_shift_count[4:0] == 5'he) ? ADC_SHIFT_TERM : ADC_SHIFT;
	end

	ADC_SHIFT_TERM: begin
	   ADC_nstate = ADC_DONE;
	end
	
	ADC_DONE: begin
	   ADC_nstate = ADC_INIT;
	end

      endcase // case (ADC_cstate)
   end
   

   always @ (posedge clk_3p2MHz) begin
      case (ADC_cstate) //synthesis parallel_case full_case
	ADC_INIT: begin
	   adc_shift_count <= 5'b0;
	   adc_shift_out <= 32'b1111_1111_1111_1111;
	   adc_shift_in <= 16'b0;
	   adc_cs <= 2'b1;
	   
	   adc_valid <= adc_valid;
	   adc_in <= adc_in;
	end
	ADC_START: begin
	   adc_shift_count <= 5'b0;
//	   adc_shift_out <= {11'b0,adc_chan[0],adc_chan[1],adc_chan[2],2'b0};
	   adc_shift_out <= {2'b0,adc_chan[2],adc_chan[1],adc_chan[0],11'b0};
	   adc_shift_in <= adc_shift_in;
	   adc_cs <= 1'b0;
	   
	   adc_valid <= 0;
	   adc_in <= adc_in;
	end
	ADC_SHIFT: begin
	   adc_shift_count <= adc_shift_count + 6'b1;
	   adc_shift_out <= {adc_shift_out[14:0],1'b1};
	   adc_shift_in[15:0] <= {adc_shift_in[14:0], DIG_ADC_OUT};
	   adc_cs <= adc_cs;
	   
	   adc_valid <= 0;
	   adc_in <= adc_in;
	end
	ADC_SHIFT_TERM: begin
	   adc_shift_count <= adc_shift_count + 6'b1;
	   adc_shift_out <= {adc_shift_out[14:0],1'b1};
	   adc_shift_in[15:0] <= {adc_shift_in[14:0], DIG_ADC_OUT};
	   adc_cs <= adc_cs;
	   
	   adc_valid <= 0;
	   adc_in <= adc_in;
	end
	ADC_DONE: begin
	   adc_shift_count <= adc_shift_count;
	   adc_shift_out <= adc_shift_out;
	   adc_shift_in <= adc_shift_in;
	   adc_cs <= 2'b1;
	   
	   adc_valid <= 1;
	   adc_in <= adc_shift_in[11:2];
	end
      endcase // case (ADC_cstate)
   end // always @ (posedge clk)

   ODDR2 adc_clk_mirror (.D0(1'b0), .D1(1'b1),   // note inversion of clk_3p2MHz
			 .C0(clk_3p2MHz), .C1(!clk_3p2MHz), 
			 .Q(DIG_ADC_SCLK), .CE(1'b1), .R(1'b0), .S(1'b0) );
   
   assign DIG_ADC_IN = adc_shift_out[15];
   assign DIG_ADC_CS = adc_cs;

endmodule // adc10cs022
