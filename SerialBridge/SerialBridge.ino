//#include <SoftModem.h>
//SoftModem modem = SoftModem();
//
//void setup() {
//	Serial.begin(115200);
//	Serial.println("Booting");
//	delay(100);
//	modem.begin();
//}
//
//void loop() {
//	while(modem.available()){
//		int c = modem.read();
//		if(isprint(c)){
//			Serial.print((char)c);
//		}
//		else{
//			Serial.print("(");
//			Serial.print(c,HEX);
//			Serial.println(")");      
//		}
//	}
//	if(Serial.available()){
//		modem.write(0xff);
//		while(Serial.available()){
//			char c = Serial.read();
//			modem.write(c);
//		}
//	}
//}

#include <SoftModem.h>
volatile uint32_t CaptureCountA, CaptureCountB, TimerCount;
volatile boolean CaptureFlag;


void setup() {
  Serial.begin(250000);                                   // initilize serial port to 250000 baud

/**************          Generate simply a PWM signal                   ***************/
  analogWriteResolution(12); // From 0 to 2exp12  - 1 = 4095
  analogWrite(7, 1024);                                   // Duty cycle is 25% with 12 bits resolution
  
/*************         Capture a PWM frequency and duty cycle          ****************/
  PMC->PMC_PCER0 |= PMC_PCER0_PID28;                      // Timer Counter 0 channel 1 IS TC1, TC1 power ON

  TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 // capture mode, MCK/2 = 42 MHz, clk on rising edge
                              | TC_CMR_ABETRG              // TIOA is used as the external trigger
                              | TC_CMR_LDRA_RISING         // load RA on rising edge of trigger input
                              | TC_CMR_LDRB_FALLING;       // load RB on falling edge of trigger input
                              
  // If you want to capture PWM data from TC1_Handler()
  TC0->TC_CHANNEL[1].TC_IER |= TC_IER_LDRAS | TC_IER_LDRBS; // Trigger interruption on Load RA and load RB
  
  TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Reset TC counter and enable

  NVIC_EnableIRQ(TC1_IRQn);                                // Enable TC1 interrupts

  Serial.println("Timer Capture");

}

void loop() {
  if (CaptureFlag) {
    CaptureFlag = 0;
    printf("\r %d , %d \n", CaptureCountA, CaptureCountB);
  }
  
}

// Note that you could either test status register by polling in loop()
void TC0_Handler() {

  //Registers RA and RB are used as capture registers. They are loaded with
  //the counter value TC_CV when a programmable event occurs on the signal TIOA1.
  TimerCount = TC0->TC_CHANNEL[1].TC_CV;            // save the timer counter register, for testing

  uint32_t status = TC0->TC_CHANNEL[1].TC_SR;       // Read & Save satus register -->Clear status register

  // If TC_SR_LOVRSRA is set, RA or RB have been loaded at least twice without any read
  // of the corresponding register since the last read of the Status Register,
  // We are losing some values,trigger of TC_Handler is not fast enough !!
  //if (status & TC_SR_LOVRS) abort();

  // TODO: calculate frequency and duty cycle from data below *****************
  if (status & TC_SR_LDRAS) {  // If ISR is fired by LDRAS then ....
    CaptureCountA = TC0->TC_CHANNEL[1].TC_RA;        // get data from capture register A for TC0 channel 1
  }
  else { /* if (status && TC_SR_LDRBS)*/  // If ISR is fired by LDRBS then ....
    CaptureCountB = TC0->TC_CHANNEL[1].TC_RB;         // get data from capture register B for TC0 channel 1

    CaptureFlag = 1;                      // set flag indicating a new capture value is present
  }

}
