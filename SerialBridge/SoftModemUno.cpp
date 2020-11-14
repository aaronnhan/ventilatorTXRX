#include "SoftModem.h" 

#define TX_PIN  (3)
#define RX_PIN1 (6)  // AIN0
#define RX_PIN2 (7)  // AIN1

SoftModem *SoftModem::activeObject = 0;

SoftModem::SoftModem() {
}

SoftModem::~SoftModem() {
	end();
}

//TIMER_CLOCK_SELECT sets the clock prescaling and clock settings (ex: counting up or down, oneshot, clear timer on compare match, etc.)
//BAUD_RATE affects the rate at which the highs/lows are sampled to decode the data/used to determine speed of transmission.
//MICROS_PER_TIMER_COUNT is the number of microseconds per clock cycle. this is used so when we count microseconds to time sampling of data, we can do it in terms of clock cycles instead of microseconds.

//Also should note(this is with uno, but may be applicable to due):  The ATmega328P uses a hardware divisor from it's clock-rate to generate the base-clock for the serial interface. 
//If there is no integer ratio from the main clock to the bit-time of the desired baud rate, the MCU will not be able to exactly produce the desired rate. This can lead to errors.

#if F_CPU == 16000000
#if SOFT_MODEM_BAUD_RATE <= 126
#define TIMER_CLOCK_SELECT	   (7)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(1024))
#elif SOFT_MODEM_BAUD_RATE <= 315
#define TIMER_CLOCK_SELECT	   (6)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(256))
#elif SOFT_MODEM_BAUD_RATE <= 630
#define TIMER_CLOCK_SELECT	   (5)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(128))
#elif SOFT_MODEM_BAUD_RATE <= 1225
#define TIMER_CLOCK_SELECT	   (4)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(64))
#else
#define TIMER_CLOCK_SELECT	   (3)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(32))
#endif
#else
#if SOFT_MODEM_BAUD_RATE <= 126
#define TIMER_CLOCK_SELECT	   (6)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(256))
#elif SOFT_MODEM_BAUD_RATE <= 315
#define TIMER_CLOCK_SELECT	   (5)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(128))
#elif SOFT_MODEM_BAUD_RATE <= 630
#define TIMER_CLOCK_SELECT	   (4)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(64))
#else
#define TIMER_CLOCK_SELECT	   (3)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(32))
#endif
#endif

//BIT_PERIOD is the period of one bit AKA time to fully send 1 bit
//HIGH_FREQ_MICROS is the period of one high freq signal or time for one high freq signal to complete
//LOW_FREQ_MICROS is the period of one low freq signal or time for one low freq signal to complete
#define BIT_PERIOD            (1000000/SOFT_MODEM_BAUD_RATE)
#define HIGH_FREQ_MICROS      (1000000/SOFT_MODEM_HIGH_FREQ)
#define LOW_FREQ_MICROS       (1000000/SOFT_MODEM_LOW_FREQ)

//HIGH_FREQ_CNT is how many high frequency signals should be sent to signal 1 high bit
//LOW_FREQ_CNT is how many low frequency signals should be sent to signal 1 low bit
#define HIGH_FREQ_CNT         (BIT_PERIOD/HIGH_FREQ_MICROS)
#define LOW_FREQ_CNT          (BIT_PERIOD/LOW_FREQ_MICROS)

//MAX_CARRIR_BITS is the max length of the message the softmodem sends including preamble.
#define MAX_CARRIR_BITS	      (40000/BIT_PERIOD)

//TCNT_BIT_PERIOD is the number of clock cycles to fully send 1 bit
//TCNT_HIGH_FREQ is the number of clock cycles for one high frequency signal to complete
//TCNT_LOW_FREQ is the number of clock cycles for one low frequency signal to complete
#define TCNT_BIT_PERIOD		  (BIT_PERIOD/MICROS_PER_TIMER_COUNT)
#define TCNT_HIGH_FREQ		  (HIGH_FREQ_MICROS/MICROS_PER_TIMER_COUNT)
#define TCNT_LOW_FREQ		  (LOW_FREQ_MICROS/MICROS_PER_TIMER_COUNT)

//Low/high thresholds for the number of clock cycles for one high/low frequency signal to complete 
#define TCNT_HIGH_TH_L		  (TCNT_HIGH_FREQ * 0.90)
#define TCNT_HIGH_TH_H		  (TCNT_HIGH_FREQ * 1.15)
#define TCNT_LOW_TH_L		  (TCNT_LOW_FREQ * 0.85)
#define TCNT_LOW_TH_H		  (TCNT_LOW_FREQ * 1.10)

#if SOFT_MODEM_DEBUG_ENABLE
static volatile uint8_t *_portLEDReg;
static uint8_t _portLEDMask;
#endif

enum { START_BIT = 0, DATA_BIT = 8, STOP_BIT = 9, INACTIVE = 0xff };

void SoftModem::begin(void)
{
	//initalize first rx pin as input pin.
	//If the pin is configured as an INPUT, digitalWrite() will enable (HIGH) or disable (LOW) the internal pullup on the input pin.
	pinMode(RX_PIN1, INPUT);
	digitalWrite(RX_PIN1, LOW);
	
	//initalize second rx pin as input pin.
    //If the pin is configured as an INPUT, digitalWrite() will enable (HIGH) or disable (LOW) the internal pullup on the input pin.
	pinMode(RX_PIN2, INPUT);
	digitalWrite(RX_PIN2, LOW);
	
	//initalize third rx pin as output pin.
    //pin value/voltage set to 0.
	pinMode(TX_PIN, OUTPUT);
	digitalWrite(TX_PIN, LOW);
	
	//Set _txPortReg as the register that contains the TX pin
	//Set _txPortMask as the bitmask to access the specific pin bits we need of the register
	_txPortReg = portOutputRegister(digitalPinToPort(TX_PIN));
	_txPortMask = digitalPinToBitMask(TX_PIN);
	
#if SOFT_MODEM_DEBUG_ENABLE
	_portLEDReg = portOutputRegister(digitalPinToPort(13));
	_portLEDMask = digitalPinToBitMask(13);
	pinMode(13, OUTPUT);
#endif
	
    // _recvstat keeps track of if bits are being received and if so, which bit is being processed (start, data, stop)
	//recvBufferHead/Tail are used as pointers in the recvBuffer buffer
	_recvStat = INACTIVE;
	_recvBufferHead = _recvBufferTail = 0;
	
	SoftModem::activeObject = this;
	
	//_lastTCNT keeps track of time since modem was started
	//_lastdiff is the time since the last message was demodulated
	_lastTCNT = TCNT2;
	_lastDiff = _lowCount = _highCount = 0;
	
	//TCCR2A sets timer2 to normal mode
	//TCCR2B sets which prescaling setting to use
	//ACSR is set which enables comparator interrupts for the clock and sets the interrupt to be on the falling edge of the clock
	//DIDR1 is set to disable digital input on the AIN1/0 pins. Likely to reduce power consumption.
	TCCR2A = 0;
	TCCR2B = TIMER_CLOCK_SELECT;
	ACSR   = _BV(ACIE) | _BV(ACIS1);
	DIDR1  = _BV(AIN1D) | _BV(AIN0D);
}

void SoftModem::end(void)
{
	//ACSR is set to disable comparator interrupts
	//TIMSK2 sets the timer interrupt mask bit on timer 2 off
	//DIDR1 enables digital input on AIN1/0 pins
	//dereferences instance of SoftModem object
	ACSR   &= ~(_BV(ACIE));
	TIMSK2 &= ~(_BV(OCIE2A));
	DIDR1  &= ~(_BV(AIN1D) | _BV(AIN0D));
	SoftModem::activeObject = 0;
}

void SoftModem::demodulate(void)
{
	uint8_t t = TCNT2;
	uint8_t diff;
	
	//diff is time since last demodulation
	diff = t - _lastTCNT;
	
	//if demodulation is happening too quickly after last (last should still be going on), abort
	if(diff < 4)
		return;
	
	_lastTCNT = t;
	
	//TODO not sure
	if(diff > (uint8_t)(TCNT_LOW_TH_H))
		return;
	
	// Calculating the moving average
#if SOFT_MODEM_MOVING_AVERAGE_ENABLE
	_lastDiff = (diff >> 1) + (diff >> 2) + (_lastDiff >> 2);
#else
	_lastDiff = diff;
#endif

	if(_lastDiff >= (uint8_t)(TCNT_LOW_TH_L)){
		_lowCount += _lastDiff;
		if(_recvStat == INACTIVE){
			// Start bit detection
			if(_lowCount >= (uint8_t)(TCNT_BIT_PERIOD * 0.5)){
				_recvStat = START_BIT;
				_highCount = 0;
				_recvBits  = 0;
        // set timer compare register value to t + (uint8_t)(TCNT_BIT_PERIOD) - _lowCount;
				OCR2A = t + (uint8_t)(TCNT_BIT_PERIOD) - _lowCount;
        //Clear compare match flag for compare match
				TIFR2 |= _BV(OCF2A);
        //Enable TC compare match interrupt
				TIMSK2 |= _BV(OCIE2A);
			}
		}
	}
	else if(_lastDiff <= (uint8_t)(TCNT_HIGH_TH_H)){
		if(_recvStat == INACTIVE){
			_lowCount = 0;
			_highCount = 0;
		}
		else{
			_highCount += _lastDiff;
		}
	}
}

// Analog comparator interrupt
ISR(ANALOG_COMP_vect)
{
	SoftModem::activeObject->demodulate();
}

void SoftModem::recv(void)
{
	uint8_t high;
	
	// Bit logic determination
	if(_highCount > _lowCount){
		_highCount = 0;
		high = 0x80;
	}
	else{
		_lowCount = 0;
		high = 0x00;
	}
	
	// Start bit reception
	if(_recvStat == START_BIT){
		if(!high){
			_recvStat++;
		}
		else{
			goto end_recv;
		}
	}
	// Data bit reception
	else if(_recvStat <= DATA_BIT) {
		_recvBits >>= 1;
		_recvBits |= high;
		_recvStat++;
	}
	// Stop bit reception
	else if(_recvStat == STOP_BIT){
		if(high){
			// Stored in the receive buffer
			uint8_t new_tail = (_recvBufferTail + 1) & (SOFT_MODEM_RX_BUF_SIZE - 1);
			if(new_tail != _recvBufferHead){
				_recvBuffer[_recvBufferTail] = _recvBits;
				_recvBufferTail = new_tail;
			}
			else{
				;// Overrun error detection
			}
		}
		else{
			;// Fleming error detection
		}
		goto end_recv;
	}
	else{
	end_recv:
		_recvStat = INACTIVE;
		TIMSK2 &= ~_BV(OCIE2A);
	}
}

// Timer 2 compare match interrupt A
ISR(TIMER2_COMPA_vect)
{
	OCR2A += (uint8_t)TCNT_BIT_PERIOD;
	SoftModem::activeObject->recv();
#if SOFT_MODEM_DEBUG_ENABLE
	*_portLEDReg ^= _portLEDMask;
#endif
}

int SoftModem::available()
{
	return (_recvBufferTail + SOFT_MODEM_RX_BUF_SIZE - _recvBufferHead) & (SOFT_MODEM_RX_BUF_SIZE - 1);
}

int SoftModem::read()
{
	if(_recvBufferHead == _recvBufferTail)
		return -1;
	int d = _recvBuffer[_recvBufferHead];
	_recvBufferHead = (_recvBufferHead + 1) & (SOFT_MODEM_RX_BUF_SIZE - 1);
	return d;
}

int SoftModem::peek()
{
	if(_recvBufferHead == _recvBufferTail)
		return -1;
	return _recvBuffer[_recvBufferHead];
}

void SoftModem::flush()
{
}

void SoftModem::modulate(uint8_t b)
{
	uint8_t cnt,tcnt,tcnt2;
	if(b){
		cnt = (uint8_t)(HIGH_FREQ_CNT);
		tcnt2 = (uint8_t)(TCNT_HIGH_FREQ / 2);
		tcnt = (uint8_t)(TCNT_HIGH_FREQ) - tcnt2;
	}else{
		cnt = (uint8_t)(LOW_FREQ_CNT);
		tcnt2 = (uint8_t)(TCNT_LOW_FREQ / 2);
		tcnt = (uint8_t)(TCNT_LOW_FREQ) - tcnt2;
	}
	do {
		cnt--;
		{
			OCR2B += tcnt;
			TIFR2 |= _BV(OCF2B);
			while(!(TIFR2 & _BV(OCF2B)));
		}
		*_txPortReg ^= _txPortMask;
		{
			OCR2B += tcnt2;
			TIFR2 |= _BV(OCF2B);
			while(!(TIFR2 & _BV(OCF2B)));
		}
		*_txPortReg ^= _txPortMask;
	} while (cnt);
}

//  Preamble bit before transmission
//  1 start bit (LOW)
//  8 data bits, LSB first
//  1 stop bit (HIGH)
//  ...
//  Postamble bit after transmission

size_t SoftModem::write(const uint8_t *buffer, size_t size)
{
	// To calculate the preamble bit length
	uint8_t cnt = ((micros() - _lastWriteTime) / BIT_PERIOD) + 1;
	if(cnt > MAX_CARRIR_BITS){
		cnt = MAX_CARRIR_BITS;
	}
	// Preamble bit transmission
	for(uint8_t i = 0; i<cnt; i++){
		modulate(HIGH);
	}
	size_t n = size;
	while (size--) {
		uint8_t data = *buffer++;
		// Start bit transmission
		modulate(LOW);
		// Data bit transmission
		for(uint8_t mask = 1; mask; mask <<= 1){
			if(data & mask){
				modulate(HIGH);
			}
			else{
				modulate(LOW);
			}
		}
		// Stop bit transmission
		modulate(HIGH);
	}
	// Postamble bit transmission
	modulate(HIGH);
	_lastWriteTime = micros();
	return n;
}

size_t SoftModem::write(uint8_t data)
{
	return write(&data, 1);
}
