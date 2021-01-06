#ifndef OpticalModDemod_H
#define OpticalModDemod_H
#define	CLOCK_SPEED		16000000  // standard Arduino clock 16MHz

int send_flag;				// light send flag

class OpticalTransmitter
{
	unsigned long tx_speed;		// bits/second
	uint8_t tx_buffer[44];
	uint8_t tx_bitnum;			// bit number to be transmitted
	uint8_t LIGHT_SEND_PIN;
	
	public:
		void set_speed(unsigned long custom_speed){
			
			tx_speed = custom_speed;
			
		}
		
		unsigned long get_speed(){
			
			return (tx_speed);
			
		}
		
		void set_txpin(int laser_pin){
			
			LIGHT_SEND_PIN = laser_pin;
			
		}
		
		void begin(){
			tx_bitnum = 0;      // bit number to be transmitted
			send_flag = 0;      // light send flag
			
			cli();//stop interrupts

			//set timer2 interrupt at specified frequency
			TCCR2A = 0;// set entire TCCR2A register to 0
			TCCR2B = 0;// same for TCCR2B
			TCNT2  = 0;//initialize counter value to 0
			// set compare match register for 2khz increments
			OCR2A = (CLOCK_SPEED / (2 * tx_speed * 64)) - 1;// = (16*10^6) / (2000*64) - 1 (must be <256)

			// turn on CTC mode
			TCCR2A |= (1 << WGM21);
			// Set CS22 bit for 64 prescaler
			TCCR2B |= (1 << CS22);   
			// enable timer compare interrupt
			TIMSK2 |= (1 << OCIE2A);

			sei();//allow interrupts
			
			//set light pin as output
			pinMode(LIGHT_SEND_PIN, OUTPUT);
		}
		
		void manchester_modulate(uint16_t light_msg){
			int i;
			uint8_t tmp;
			
			// first start bit
			for(i = 0; i < 4; i++){
				tx_buffer[i] = i%2;
			}
			
			for(i = 15; i >= 8; i--){
				tmp = !!(light_msg & (1 << i));         // i-th bit value of tx_buffer
				tx_buffer[2*(15-i) + 4] = (tmp ^ 1);
				tx_buffer[2*(15-i) + 5] = (tmp ^ 0);
			}
			
			// first stop bit
			tx_buffer[20] = 1;
			tx_buffer[21] = 0;
			
			// second start bit
			for(i = 22; i < 26; i++){
				tx_buffer[i] = i%2;
			}
			
			for(i = 7; i >= 0; i--){
				tmp = !!(light_msg & (1 << i));         // i-th bit value of tx_buffer
				tx_buffer[2*(7-i) + 26] = (tmp ^ 1);
				tx_buffer[2*(7-i) + 27] = (tmp ^ 0);
			}
			
			// second stop bit
			tx_buffer[42] = 1;
			tx_buffer[43] = 0;
			
			send_flag = 1;
		

			//begin debug print
			//Serial.print("Mod: ");
			//for (i=0; i<44; i++){Serial.print(tx_buffer[i]);} Serial.println();
			//end debug print

			}
		
		void transmit()
		{
			//generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)

			// Generate wave based on manchester input
			if(send_flag)
			{
				digitalWrite(LIGHT_SEND_PIN,tx_buffer[tx_bitnum]);

				// shift to next bit in the send buffer
				if(tx_bitnum < 43)
				{
					tx_bitnum++;    // next bit
				} else {
					tx_bitnum = 0;    // reset to beginning of buffer
					send_flag = 0;    // done sending


					// begin debug print
					//Serial.print("Xmt: ");
					//for (int i=0; i<44; i++){Serial.print(tx_buffer[i]);} //Serial.println();
					// end debug print

				}

			}

		}
		
		void dummy_transmit()
		{
			if(send_flag)
			{
				digitalWrite(LIGHT_SEND_PIN, HIGH);
                send_flag = 0;
			
			}
			if (!(send_flag))
			{
				digitalWrite(LIGHT_SEND_PIN, LOW);
			}
		}
	
};


class OpticalReceiver
{
	unsigned long rx_speed;			// bits/second
	uint8_t rx_buffer[44];
	uint8_t rx_bitnum;			// number of bits received
	int recv_flag;				// light receive flag
	uint16_t msg_raw;
	uint8_t msg_done;
	uint8_t char_ready;
	uint8_t LIGHT_RECEIVE_PIN;
	uint8_t LIGHT_SEND_PIN;
	bool inverted;

	public:
		void set_speed(unsigned long custom_speed){
			
			rx_speed = custom_speed;
			
		}
		
		unsigned long get_speed(){
			
			return (rx_speed);
			
		}

		void set_rxpin(int phototransistor_pin) {
			
			LIGHT_RECEIVE_PIN = phototransistor_pin;
		
		}

		void set_inverted(bool rxinverted) {
			
			inverted = rxinverted;
		
		}

		void set_txpin(int laser_pin){
			
			LIGHT_SEND_PIN = laser_pin;
			
		}
		
		void begin(){
			rx_bitnum = 0;      // bit number to be transmitted
			recv_flag = 0;      // light send flag
			char_ready = 0;
			
			cli();//stop interrupts

			//set timer2 to grab bits at the desired speed
			TCCR2A = 0;  // set TCCR2A register to 0
			TCCR2B = 0;  // same for TCCR2B
			TCNT2  = 0;  //initialize counter value to 0
			// set compare match register for specified increments
			OCR2A = (CLOCK_SPEED / (2 * rx_speed * 64)) - 1; // if 2KHz = (16*10^6) / (2000*64) - 1 (must be <256)
			// turn on compare to count (CTC) mode
			TCCR2A |= (1 << WGM21);
			// Set CS22 bit for a 64 prescaler
			TCCR2B |= (1 << CS22);   
			// enable timer compare interrupt
			TIMSK2 |= (1 << OCIE2A);

			sei();//allow interrupts
		//Serial.print("Rx = ");
		//Serial.print(rx_speed);
		//Serial.print("  Clk = ");
		//Serial.println((CLOCK_SPEED / (2 * rx_speed * 64)) - 1);
			//set photodiode pin as input
			pinMode(LIGHT_RECEIVE_PIN, INPUT);
			pinMode(LIGHT_SEND_PIN, OUTPUT);
		}
		
		uint16_t manchester_demodulate(){
			uint8_t i, check1, check2;
			uint16_t out = 0;
			
			/* first frame */
			for(i = 4; i < 20; i+=2){
				check1 = rx_buffer[i];
				check2 = rx_buffer[i+1];
				if(!check1 && check2) out |= (1 << (15-((i-4)/2)));  // if this is a 0-to-1 transition?, set the ith bit to 1 else set to 0
			}
			
			/* second frame */
			for(i = 26; i < 42; i+=2){
				check1 = rx_buffer[i];
				check2 = rx_buffer[i+1];
				if(!check1 && check2) out |= (1 << (7-((i-26)/2))); 
			}

					// begin debug print
					//Serial.print("Dem: ");
					//for (int i=0; i<44; i++){Serial.print(rx_buffer[i]);} Serial.println();
					// end debug print

			return (out);
		}
		
		void receive(){
			boolean tmp;
			
			if(!recv_flag){  // if a valid frame hasn't been found, keep looking
			tmp = PIND & (1 << LIGHT_RECEIVE_PIN); //get a bit by using direct pin access for speed
			tmp ^= inverted;
			//Serial.println(tmp, BIN);
				switch(rx_bitnum){  // here we start a register to check for the sync bits  (0 then 1) and if found, begin building a buffer
					case 0:
						if(tmp == 1) rx_bitnum = 1;
						break;
					case 1:
						if(tmp == 0) rx_bitnum = 2;
						else rx_bitnum = 0;
						break;
					case 2:
						if(tmp == 1) rx_bitnum = 3;
						else rx_bitnum = 0;
						break;
					case 3:
						recv_flag = 1;
						for(rx_bitnum = 0; rx_bitnum < 4; rx_bitnum++){
							rx_buffer[rx_bitnum] = rx_bitnum % 2;
						}
						rx_bitnum = 4;
						rx_buffer[rx_bitnum] = tmp;
						rx_bitnum++;
						break;
				}
			} else if(rx_bitnum < 44){  // keep filling the buffer until 44 bits are captured
				tmp = PIND & (1 << LIGHT_RECEIVE_PIN);
				rx_buffer[rx_bitnum] = inverted ^ tmp;
				rx_bitnum++;
			} else {
				rx_bitnum = 0;
				recv_flag = 0;
					
					// begin debug print
					//Serial.print("Rcv: ");
					//for (int i=0; i<44; i++){Serial.print(rx_buffer[i]);} Serial.println();
					// end debug print
				
				// Read data from the photodiode
				msg_raw = manchester_demodulate(); // get the demodulated bits 
				msg_done = hamming_byte_decoder((msg_raw >> 8), (0xFF & msg_raw)); // send 4 bits to the decoder along with the full message
				char_ready = 1;  // set a flag to show a valid message is ready
				
			}
		}
		
	
		//void printByte(){
		//	if(char_ready){
		//		Serial.print((char)msg_done);
		//		Serial.println();
				//Serial.print(msg_done,BIN);
				//Serial.println(" ");
				//return ((char)msg_done);
		//		char_ready = 0;
		//	} 
		//}
		
		//Restructuring this to suppress some of the noise of 'bad' data reads while waiting for laser signal
		bool GetByte(uint8_t &c)
		{
			if(char_ready)
			{
				char_ready = 0;
				c = msg_done;
				return(1);
			} else {
				return (0);
			}

			
		}

};

#endif
