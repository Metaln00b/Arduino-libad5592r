
/*
		Analog Instruments AD5592R(-1) Arduino Library
		
		Author: Florian (Metaln00b: https://github.com/Metaln00b/Arduino-libad5592r)
		
		Contributor: dzalf (https://github.com/dzalf)
		
		libad5592r Library Example: Setting DAC voltage via the serial ports
		
									Enhanced version of the original example SetDAC which parsed a voltage value
									from the serial port and wrote it to Port 0 of the DAC
									
									This update queries the channel and the voltage to be written in the selected port
									
									It uses the updated .cpp and .h files uploaded through dzalf's fork
		
		Date: 20- May-2020 (COVID-19 Lock-down aftermath)
	

*/


#include <libad5592r.h>

#define VDD 5000
#define CHATTY_CODE 	true

const int syncPin = 10;

volatile bool serialReceived = false;
bool channelSet;
bool voltageSet;

typedef enum{
	
	WAITING_INPUT,
	REQUEST_CHANNEL,
	REQUEST_VOLTAGE,
	RESET_FLAGS
	
} sysStatus;

sysStatus serialStatus = REQUEST_CHANNEL;

byte channel;

String serialIn = "";

ad5592r_t ad5592r;

void setup() {
	
    Serial.begin (115200);
	
    ad5592r_init(&ad5592r, syncPin);
    ad5592r_reset(&ad5592r);
    ad5592r_setup_source_ref_set(&ad5592r, AD5592R_VREF_SOURCE_INTERNAL);
    ad5592r_setup_double_vref_set(&ad5592r, true);
    ad5592r_setup_supply_voltage_mV(&ad5592r, VDD);	// My board uses VDD = 5.0 V

	/* When using this method, the second argument 
	   can be a byte [0,255] as the implementation compares each bit from
	   the number with a mask. For instance, 255 will set all 8 ports as DAC pins while
	   170 (DEC) sets pins: 1010 1010 (BIN) or AA (HEX)*/
	   
    /*  Previous --> ad5592r_setup_dac_all(&ad5592r, AD5592R_IO(0)); 
		This form created some issues
		Instead, the ad5592r_setup_dac method could be used for individual Ports or in a for loop
		for setting all ports
	 */

	ad5592r_setup_dac_all(&ad5592r, 255);		//Setting all pins as DAC
	
    ad5592r_setup_ldac(&ad5592r, AD5592R_LDAC_MODE_IMMEDIATELY);
	
	channelSet = false;
	voltageSet = false;
	
	delay(3000);	// On Teensy there is no reset on serial
}

void loop() {
	
	switch (serialStatus){
		
		case(REQUEST_CHANNEL):
		
			Serial.print("Set channel >> ");
			
			serialStatus = WAITING_INPUT;
		
		break;
		
		case(REQUEST_VOLTAGE):
			
			Serial.print("Set voltage [0, VDD] >> ");
			
			serialStatus = WAITING_INPUT;
		
		break;
		
		case(WAITING_INPUT):
		
		
			if (serialReceived) {
				
				if(CHATTY_CODE) Serial.println("Input string received!");
				
				serialReceived = false;
				
				
				if((channelSet == false) && (voltageSet == false)){
					
					retrieveChannel();
					
				}else if((voltageSet == false) && (channelSet == true)){
					
					retrieveVoltage();
					
				}
				
			}
		
		break;
		
		case(RESET_FLAGS):
		
			channelSet = false;
			voltageSet = false;
			
			if(CHATTY_CODE)  Serial.println("Ready to set channel!");
			
			serialStatus = REQUEST_CHANNEL;
				
		break;
	}
	
}


void retrieveChannel(){
	
	channel = serialIn.toInt();
	
	serialIn = "";
	
	if((channel >=0) && (channel <= 7)){
	
		if(CHATTY_CODE) Serial.print("Writing to channel: ");
		
		Serial.println(channel);
		
		channelSet = true;
		serialStatus = REQUEST_VOLTAGE;
	
	} else {
		
		Serial.println("Channel out of bounds!");
		
		channelSet = false;
		serialStatus = REQUEST_CHANNEL;
		
	}
}

void retrieveVoltage(){
	
	 double voltage_mV_set = serialIn.toInt();
	 
	 serialIn = "";
	 
	 
	 if((voltage_mV_set >=0) && (voltage_mV_set <= VDD)){
		 
 
        if(CHATTY_CODE) Serial.print("Value received: ");
		
        Serial.print(voltage_mV_set);
        Serial.print("\n");
		
		if(CHATTY_CODE) Serial.println("Setting voltage!");
		
        bool DACResponse = ad5592r_dac_voltage_set_mV(&ad5592r, AD5592R_IO(channel), voltage_mV_set);
		
		if (DACResponse){
			
			Serial.println("OK!");
				
			serialStatus = RESET_FLAGS;
			
		} else{
			
			Serial.println("Something went wrong :'( _____ Try again...");
			
			voltageSet = false;
					
			serialStatus = REQUEST_VOLTAGE;
			
		}
			
		
	 } else{
		
		
		Serial.println("Voltage  out of bounds!");
		
		voltageSet = false;
					
		serialStatus = REQUEST_VOLTAGE;
		
	}
	
}


void serialEvent() {
	
// From the serialEvent examples
	
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    serialIn += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      serialReceived = true;
    }
  }
}
