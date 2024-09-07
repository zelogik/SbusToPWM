const int ledPin = LED_BUILTIN;  // the number of the LED pin

// TODO:
// - add channel number D8 or D16
// ie: so could be digital or analogic
// - add time based set output
// - add time based failsafe
// - add failsafe logic
// - TEST with hardware :D
// - 

#define F_CPU 16000000UL  // Define CPU frequency as 16 MHz (typical for Arduino Uno)
#define PRESCALER 8        // Assuming Timer1 is configured with a prescaler of 8
#define DEBUG 1
// Not the best, precise millis class, but enought for that code
class MyMillis {
public:
    MyMillis() : __millis(0) {}

    void time(uint8_t addModulo) {
      // TODO, /1000 != >> 10 ... make a compensation
      // uint8_t compensation = (24 * addModulo) >> 3;
      __millis += addModulo;
    }

    uint32_t time() {
        return __millis;
    }

private:
    uint32_t __millis; // Shared variable
};

MyMillis millis_;

struct t_rx_sbus
{
    uint8_t Sbuffer[28];
    uint8_t Sindex = 0;	
    uint8_t Slast_rcv_ms = 0;
    uint8_t need_process = 0;
} volatile sbus;

struct t_state
{
    uint8_t failsafe = 0;
    uint8_t swap = 0; // allow double PCB with 'same wiring'
    uint8_t fast = 0; // digital vs analog
} volatile state;

uint16_t pulseTimes[17] = {1500}; // set default PWM

volatile uint16_t pseudo16bitCounter = 0;

struct PinConfig {
    volatile uint8_t *port;  // Pointer to the port register (PORTB, PORTD, etc.)
    uint8_t pin;             // Pin number (0-7)
};

PinConfig outputs[16] = {
    { &PORTD, 2 },
    { &PORTD, 3 },
    { &PORTD, 4 },
    { &PORTD, 5 },
    { &PORTB, 0 },
    { &PORTB, 1 },
    { &PORTB, 2 },
    { &PORTB, 3 },
    { &PORTC, 0 },
    { &PORTC, 1 },
    { &PORTC, 2 },
    { &PORTC, 3 },
    { &PORTD, 6 },
    { &PORTD, 7 },
    { &PORTB, 4 },
    { &PORTB, 5 }
};

volatile uint32_t millis_inc = 0;
ISR(TIMER0_COMPA_vect)
{
  OCR0A += 250; // Advance The COMPA Register
  millis_inc++;
}

volatile uint8_t tick_ppm = 0;
ISR(TIMER2_COMPA_vect)
{
  OCR2A += 138; // Advance The COMPA Register

}


ISR(USART0_RX_vect) {
    // ISR triggered when data is received
    uint8_t rx_byte = UDR0;  // Read the received data from the UART data register

    // SBUS messages take 3ms to communicate. Typical repeat intervals are: 14ms (analog mode), 7ms (high speed mode).
    // TODO: add if Sto_proceed is false...
    if ( ! sbus.need_process )
    {
        // Start only if delay passed.
        // give 5ms for postprocessing that sbus
        if ( millis_.time() - sbus.Slast_rcv_ms > 5 )
        {
            // TODO: check corrupted ?
            uint8_t idx = sbus.Sindex;

            sbus.Sbuffer[idx] = rx_byte;
            if ( idx > 24 ) //Sbus = 25bytes
            {
                sbus.Slast_rcv_ms == millis_.time();
                sbus.need_process = 1;
            }
        }
    }
}


void bubbleSort(uint8_t arr[], uint8_t n) {
    for (uint8_t i = 0; i < n-1; i++) {
        for (uint8_t j = 0; j < n-i-1; j++) {
            if (arr[j] > arr[j+1]) {
                // Swap arr[j] and arr[j+1]
                uint8_t temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
}


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
#ifdef DEBUG
  Serial.begin(57600);
#endif

    // Timer0 : millis
    TCCR0A = 0;  // Normal mode
    TCCR0B |= (1 << CS01) | (1 << CS00);  // Set prescaler to 64

    OCR0A = 250;
    // TIMSK0 |= 0x02;
    TIMSK0 |= (1 << TOIE0);  // Enable Timer0 overflow interrupt


    // Timer2 : ppm tick
    TCCR2A = 0;  // Normal mode
    TCCR2B |= (1 << CS22);  // Set prescaler to 64
    OCR2A = 138;        // Timer Compare2A Register
    TIMSK2 |= (1 << TOIE2); //B00000010;  // Enable Timer COMPA Interrupt


    // Timer1
    // Clear Timer1 Control Registers
    TCCR1A = 0x00;    //Init.
    // TCCR1B = 0x00; // //|= (1 << CS11);
    // Set prescaler to 8 (CS11 bit set)
    TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11); 
    // Set Timer1 to Normal mode (no PWM, no CTC)
    TIFR1 = (1 << OCF1A);
    TIMSK1 |= ( 1<< OCIE1A );

    uint16_t baud_rate = 103;  // Assuming 16 MHz clock and 9600 baud
    UBRR0H = (uint8_t)(baud_rate >> 8);
    UBRR0L = (uint8_t)baud_rate;

    // Enable receiver
    // UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0B &= ~TXEN0 ;

    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01 ) | (1<<UPM01) ;
    // Enable global interrupts
    sei();


    // TODO: set pin swap AND speed ana/dig

}

// the loop function runs over and over again forever
void loop() {
    static uint32_t outputLoopTimeout = millis_inc;
    // ie: make a "mean" operation with two value
    // int mean = (a & b) + ((a ^ b) >> 1);

    // disable failsafe switch after 10sec...
    // if switch failsafe
    //   set failsafe (write to eeprom)

    // if no new value after 500ms 
    //   go to failsafe...
    // 
    // TODO: change by DEFINE variable
	if ( millis_inc - outputLoopTimeout < 20 )
	{
		outputLoopTimeout = millis_inc;
        setOutput();
	}

    processISR();
#ifdef DEBUG
    debugLoop();
#endif
    micros_();
}


void setOutput()
{
    static uint8_t ch_tick[16] = {127};
    static uint8_t pin_on = 0;
    static uint8_t current_channel = 0;

    const uint8_t size = sizeof(ch_tick) / sizeof(ch_tick[0]);
    static uint8_t ch_tick_sorted[size];
    // set all Pin high (ppm)
    if ( ! pin_on )
    {
        memcpy(ch_tick_sorted, ch_tick, sizeof(ch_tick_sorted));  // Copy unsorted into sorted
        bubbleSort(ch_tick_sorted, size);

        for (size_t i = 0; i < 16; i++)
        {
            // set all pin HIGH
            // TODO:
        }
        pin_on = 1;
        tick_ppm = 0;
    }

    if ( tick_ppm > ch_tick_sorted[current_channel])
    {
        // set [current_channel] low
        // TODO:

        current_channel++;
    }
    
    // uint16_t *pulsePtr = PulseTimes;
    // uint16_t times[4];				// The 4 pulse lengths to process
    // uint8_t j = 0, k;
    // uint16_t m;

    // pulsePtr += move_on;
    // k = move_on;

    // if ( ChannelSwap )		// Link on AD5
    // {
    //     // swap chanels 1-8 and 9-16
    //     ( k >= 8 )? k -= 8 :  k += 8;
    // }

    // for (uint8_t i = 0; i < 4; i += 1 )
    // {
    //     times[i] = pulsePtr[i];                   // Make local copy of pulses to process
    // }
   
}

uint8_t processISR()
{
    // asm volatile ("nop\n\t");
    // https://os.mbed.com/users/Digixx/code/SBUS-Library_16channel
    if ( sbus.need_process )
    {
        uint8_t *sbus_ptr = sbus.Sbuffer;

        if ( *sbus_ptr++ != 0x0F ) // sbus.Sbuffer[0]
        {
            return 0;
        }
        else if (sbus.Sbuffer[24] == 0x00 )
        {
            uint8_t inputbitsavailable = 0;
            uint32_t inputbits = 0;

            for (uint8_t i = 0 ; i < 16 ; i += 1 )
            {
                uint16_t value = 0;
                while ( inputbitsavailable < 11 )
                {
                    inputbits |= (uint32_t)*sbus_ptr++ << inputbitsavailable;
                    inputbitsavailable += 8;
                }
                value = (int16_t)(inputbits & 0x7FF) - 0x3E0;
                value = (value << 2) + value; // Equivalent to value * 5
                value = (value >> 3) + 1500;
                if ( ( value > 800 ) && ( value < 2200 ) )
                {
                    pulseTimes[i] = value;
                }
                // else
                // {
                //     // don't change value?
                // }
                inputbitsavailable -= 11;
                inputbits >>= 11;
            }
            // DigiChannel 1
            if (sbus.Sbuffer[23] & (1<<0)) {
                pulseTimes[16] = 1;
            }else{
                pulseTimes[16] = 0;
            }
            // DigiChannel 2
            if (sbus.Sbuffer[23] & (1<<1)) {
                pulseTimes[17] = 1;
            }else{
                pulseTimes[17] = 0;
            }
            // Failsafe
            // failsafe_status = SBUS_SIGNAL_OK;
            if (sbus.Sbuffer[23] & (1<<2)) {
                // failsafe_status = SBUS_SIGNAL_LOST;
            }
            if (sbus.Sbuffer[23] & (1<<3)) {
                // failsafe_status = SBUS_SIGNAL_FAILSAFE;
            }

                // TODO: Test and should be more readable
           // uint8_t byte_in_sbus = 1;
            // uint8_t bit_in_sbus = 0;
            // uint8_t channel = 0;
            // uint8_t bit_in_channel = 0;

            // for (uint8_t i=0; i<176; i++ ) {
            //     if ( sbus.Sbuffer[byte_in_sbus] & (1<<bit_in_sbus) ) {
            //         pulseTimes[channel] |= (1<<bit_in_channel);
            //     }
            //     bit_in_sbus++;
            //     bit_in_channel++;

            //     if ( bit_in_sbus == 8 ) {
            //         bit_in_sbus =0;
            //         byte_in_sbus++;
            //     }
            //     if ( bit_in_channel == 11 ) {
            //         bit_in_channel =0;
            //         channel++;
            //     }
            // }
        }
        else
        {
            return 0;
        }
        sbus.need_process = 0;
    }
}

void debugLoop() {
  static const uint16_t interval = 1000;  // interval at which to blink (milliseconds)
  static uint32_t previousMillis = 0;  // will store last time LED was updated
  uint32_t currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    PINB |= (1 << 5);

    static uint16_t old_micros;
    Serial.print(F("micros delay: "));
    uint16_t delayMicrosOriginal = micros();
    Serial.println(delayMicrosOriginal);
    Serial.print(F("micro lag: "));
    Serial.println(delayMicrosOriginal - old_micros);
    old_micros = delayMicrosOriginal;

    Serial.print(F("delay millis: "));
    uint32_t delayMillisOriginal = millis();
    Serial.println(delayMillisOriginal);

    Serial.print(F("delay micros_: "));
    Serial.println(micros_());
    Serial.print(F("delay millis_: "));
    Serial.println(millis_.time());


    Serial.print(F("Delay millis_inc: "));
    Serial.println(millis_inc);
    Serial.println("-----------"); 
  }
}


uint16_t micros_() {
    // Calculate how many clock cycles occur per microsecond
    // F_CPU / 1,000,000 gives the number of clock cycles per microsecond
    // With a prescaler of 8, you divide that value by 8
    // static const uint32_t cycles_per_microsecond = (F_CPU / 1000000UL) / PRESCALER;

    // Get the current value of Timer1
    // Should be 2... so >> 1
    uint16_t timer_value = TCNT1 >> 1;

    static uint8_t old_add_millis = 0;
    uint8_t add_millis = timer_value >> 10;

    // Serial.println(modulo);

    if (add_millis > 0)
    {
      if (add_millis != old_add_millis)
      {
        millis_.time(add_millis - old_add_millis);
        old_add_millis = add_millis;
      }
      if (add_millis > 31)
      {
        // ERROR, loop too long ?
        // CHECK: buffer overflow ie:65535?
      }
      else if (add_millis > 24) // It give 30ms for a loop in micros_
      {
        old_add_millis = 0;
        cli();
        TCNT1 = 0;
        sei();
      }
    }


    return timer_value;
}