#include "Reciever.h"

Reciever::Reciever(){

	channels[0] = {27, 0, 0};
	channels[1] = {14, 0, 0};
	channels[2] = {25, 0, 0};
	channels[3] = {26, 0, 0};
	channels[4] = {34, 0, 0};
	channels[5] = {35, 0, 0};
}

void Reciever::setup(){
	for(int i = 0; i < 6; i++){
		// pinMode(channels[i].channelPin, INPUT_PULLUP);
		gpio_config_t io_conf = {};
		io_conf.mode = GPIO_MODE_INPUT;
		io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
		io_conf.pin_bit_mask = (1ULL << channels[i].channelPin);
		gpio_config(&(io_conf));

		
		//attachInterrupt(digitalPinToInterrupt(channels[i].channelPin), channels[i].interruptHandler, CHANGE);
		gpio_isr_handler_add((gpio_num_t)channels[i].channelPin, handleInterrupt, &channels[i]);		
		gpio_set_intr_type((gpio_num_t)channels[i].channelPin, GPIO_INTR_ANYEDGE);
	}
}

/** 
 *  Bonus: at some point setup a watchdog if the receiver is not receiving any pulses
 */
void Reciever::handleInterrupt(void* arg){
  portENTER_CRITICAL_ISR(&mux);

  RCChannel* currentChannel = (RCChannel*)arg;
  unsigned long currentTime = esp_timer_get_time();

  if(digitalRead(currentChannel->channelPin) == LOW ){
	unsigned long difference = currentTime - currentChannel->lastRisingEdge;
	if(difference > 900 && difference < 2100){
		currentChannel->pulseWidth = difference;
	}
  } else {
	currentChannel->lastRisingEdge = currentTime;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void Reciever::getData(double* buffer){ 
	buffer[0] = channels[0].pulseWidth;
	buffer[1] = channels[1].pulseWidth;
	buffer[2] = channels[2].pulseWidth;
	buffer[3] = channels[3].pulseWidth;
	buffer[4] = channels[4].pulseWidth;
	buffer[5] = channels[5].pulseWidth;
}

void Reciever::print(){
	Serial.print("Value 1: ");
	Serial.println(channels[0].pulseWidth);
	Serial.print("Value 2: ");
	Serial.println(channels[1].pulseWidth);
	Serial.print("Value 3: ");
	Serial.println(channels[2].pulseWidth);
	Serial.print("Value 4: ");
	Serial.println(channels[3].pulseWidth);
	Serial.print("Value 5: ");
	Serial.println(channels[4].pulseWidth);
	Serial.print("Value 6: ");
	Serial.println(channels[5].pulseWidth);
}

