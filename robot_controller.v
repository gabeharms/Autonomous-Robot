////-----------------------------------------------
//-- Designer 		: Gabe Harms, Neal Malkani, Bryan Dickens, Joe Wolfe, Luke Uliana, Andrew Quintero
//-- Date 			: 12/1/2014
//-- Course 		: CMPEN 417
//-- Affiliation 	: The Pennsylvania State University
//-- Description 	: Final Project hw 6
////------------------------------------------------


module robot_controller(input clk, output TxD, output [2:0] state_machine);

assign state_machine = current_state;
assign TxD = Tx;

reg [7:0] SOH = 8'hAA;
reg [15:0] length = 16'h0000; //change this
reg [7:0] version = 8'h01;
reg [31:0] timestamp = 32'd0;
reg [7:0] flags = 8'h00;
reg [15:0] msg_type = 16'h0000;//change this
reg [7:0] stx = 8'h55;
//payload...
reg [15:0] crc = 16'h1024;

reg [15:0] desired_speed = 16'h0064;
reg [15:0] desired_left_turn_radius = 16'h0001;
reg [15:0] desired_right_turn_radius = 16'hFFFF;

reg [7:0] desired_ppr = 8'h64;
reg [7:0] desired_scale = 8'h01;

reg [15:0] desired_request_frequency = 16'h0064;
reg [15:0] temp_message_type;

parameter [2:0] config_encoder = 3'd0,
					 subscribe_data = 3'd1,
					 straight = 3'd2,
					 left = 3'd3,
					 right = 3'd4,
					 waiting = 3'd5;

reg [7:0] iterations;			 
reg  [2:0] current_state = config_encoder;

reg change_state = 0;
reg direction;
reg readInData;
reg initial_config = 1;


parameter [7:0] straight_msg_length = 8'd143;
parameter [7:0] encoder_msg_length = 8'd127;
parameter [7:0] distance_msg_length = 8'd111;


reg [straight_msg_length:0] msg_straight;
reg [encoder_msg_length:0]msg_encoder;
reg [distance_msg_length:0]msg_distance;

reg [31:0] temp_payload;
reg [31:0] current_distance_travelled;

reg [47:0] payload_straight;
reg [31:0] payload_encoder;
reg [15:0] payload_distance;

reg [500:0] receiverInMessage;
reg [9:0] read_state;

wire Tx;
reg TxStart = 0;
wire TxBusy;
wire BitTick_temp;
reg temp;
reg messageReady = 0;
reg [7:0] output_data;
reg [7:0] temp_output;

//Transmit
async_transmitter TX(.clk(clk), .TxD(Tx), .TxD_start(TxStart), .TxD_data(output_data), .BitTick_temp(BitTick_temp), .TxD_busy(TxBusy));

wire Rx;
wire input_ready;
wire [7:0] temp_input;
reg [7:0] input_data;
reg [7:0] message_length;

//Receive
async_receiver RX(.clk(clk), .RxD(Rx), .RxD_data_ready(input_ready), .RxD_data(temp_input));
always @(posedge clk) if(input_ready) input_data <= temp_input;


always @(posedge clk)
begin
	case(current_state)
		config_encoder: begin
						if (initial_config)
						begin
						msg_type = 16'h0802;
						length = 16'h10EF;
						payload_encoder [31:24] = desired_ppr;
						payload_encoder [23:16] = desired_scale;
						payload_encoder [15:0] = 16'd0;
						iterations = 8'd0;
						msg_encoder = {SOH, length, version, timestamp, flags, 
													msg_type, stx, payload_encoder, crc};
													
						initial_config = 0;
						end
					 
						if ( !TxBusy)
						begin
							output_data = msg_encoder[encoder_msg_length:encoder_msg_length-7];
							msg_encoder = msg_encoder<<8;
							TxStart = 1;
			
							 
							iterations = iterations + 8'd8;
						end
						else
							TxStart = 0;
					 
						if (iterations >= encoder_msg_length)
						begin
							initial_config = 1;
							current_state = subscribe_data;
						end 

					 end
		
		subscribe_data: 	 begin
						if (initial_config)
						begin
						msg_type = 16'h4800;
						length = 16'h0EF1;
						payload_distance [15:0] = desired_request_frequency;
						iterations = 8'd0;
						msg_encoder = {SOH, length, version, timestamp, flags, 
													msg_type, stx, payload_distance, crc};
													
						initial_config = 0;
						end
					 
						if ( !TxBusy)
						begin
						output_data = msg_encoder[distance_msg_length:distance_msg_length-7];
						msg_distance = msg_distance<<8;
						TxStart = 1;
		
						 
						iterations = iterations + 8'd8;
						end
						else
							TxStart = 0;
					 
						if (iterations >= distance_msg_length)
						begin
							initial_config = 1;
							current_state = straight;
						end 

					 end
					 
					 
		straight: begin
					 if (initial_config)
					 begin
					 msg_type = 16'h0205;
					 length = 16'h12ED;
					 payload_straight [47:32] = desired_speed;
					 payload_straight [31:0] = 32'd0;
					 iterations = 8'd0;
					 msg_straight = {SOH, length, version, timestamp, flags, 
												msg_type, stx, payload_straight, crc};
												
					 initial_config = 0;
					 end
					 
					 if ( !TxBusy)
					 begin
					 output_data = msg_straight[straight_msg_length:straight_msg_length-7];
					 msg_straight = msg_straight<<8;
					 TxStart = 1;
	
					 
					 iterations = iterations + 8'd8;
					 end
					 else
							TxStart = 0;
					 
					 if (iterations >= straight_msg_length)
					 begin
						initial_config = 1;
						current_state = waiting;
					 end 
					 
					 end
		
		left:	    begin
					if (initial_config)
					begin
					msg_type = 16'h0205;
					length = 16'h12ED;
					payload_straight [47:32] = desired_speed;
					payload_straight [31:16] = desired_left_turn_radius;
					payload_straight [15:0] = 16'd0;
					iterations = 8'd0;
					msg_straight = {SOH, length, version, timestamp, flags, 
												msg_type, stx, payload_straight, crc};
												
					initial_config = 0;
					end
					 
					if ( !TxBusy)
					begin
					output_data = msg_straight[straight_msg_length:straight_msg_length-7];
					msg_straight = msg_straight<<8;
					TxStart = 1;
						 
					iterations = iterations + 8'd8;
					end
					else
					TxStart = 0;
					 
						if (iterations >= straight_msg_length)
						begin
						initial_config = 1;
						current_state = straight;
					 end 
					 
					 end
					 
		
		right: 	 begin
					 if (initial_config)
					 begin
					 msg_type = 16'h0205;
					 length = 16'h12ED;
					 payload_straight [47:32] = desired_speed;
					 payload_straight [31:16] = desired_right_turn_radius;
					 payload_straight [15:0] = 16'd0;
					 iterations = 8'd0;
					 msg_straight = {SOH, length, version, timestamp, flags, 
												msg_type, stx, payload_straight, crc};
												
					 initial_config = 0;
					 end
					 
					 if ( !TxBusy)
					 begin
					 output_data = msg_straight[straight_msg_length:straight_msg_length-7];
					 msg_straight = msg_straight<<8;
					 TxStart = 1;
	
					 
					 iterations = iterations + 8'd8;
					 end
					 else
							TxStart = 0;
					 
					 if (iterations >= straight_msg_length)
					 begin
						initial_config = 1;
						current_state = straight;
					 end 
					 
					 end
					 
					 
		
		waiting:  begin
					//get read data ready if initial_config
					if (initial_config)
					begin
				    TxStart = 0;
					read_state = 10'd1;
					initial_config = 0;
					end
					
					if(input_ready)
					begin
						case(read_state)
						
							2'd0: //waiting for SOH	 
							begin
								//if receiver data is AA
								if(input_data==8'hAA)read_state = read_state + 10'd1;
							end
							
							2'd1: //Waiting for length 
							begin
								read_state = read_state + 10'd1;
								message_length = input_data;
							end
							
							2'd2: //waiting for ~length 	 
							begin
								read_state = read_state + 10'd1;
								//valid length
							end
							
							default: //waiting for rest of the message	
							begin
								read_state = read_state + 10'd1;
								if(read_state < message_length)
								begin
									//continuously store data from read_state all the way to message length
									//receiverInMessage[((read_state-3)*8)-1:((read_state-3)*8)-8] = input_data;
									receiverInMessage[7:0] = input_data;
									receiverInMessage = receiverInMessage << 8;
									
									if(read_state == 7)
									begin
										temp_message_type[15:8] = input_data;
									end
									else if(read_state == 8)
									begin
										temp_message_type[7:0] = input_data;
									end
									//11-14 is distance travelled
									else if(read_state == 11)temp_payload[31:24] = input_data;
									else if(read_state == 12)temp_payload[23:16] = input_data;
									else if(read_state == 13)temp_payload[15:8] = input_data;
									else if(read_state == 14)temp_payload[7:0] = input_data;
									
								end
								else
								begin
									read_state = 0;
									messageReady = 1;
								end
								//return receiverInMessage
							end
						
						//whenever ready do an always 
						//readInData = !readInData;
						 endcase
						 if(messageReady)
						 begin
							//use receiverInMessage
							
							//parse the message type
							if(temp_message_type == 16'h8800)//this is encoder for distance
							begin
								//get the distance
								if(temp_payload - current_distance_travelled >= 10)
									current_distance_travelled = temp_payload;
									if (direction)
									begin
									current_state = left;
									direction = !direction;
									end
									else
									begin
									current_state = right;
									direction = !direction;
									end
							end
						 end
					 end
				end 
	endcase 
end



endmodule





