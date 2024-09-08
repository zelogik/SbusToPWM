// const int ledPin = LED_BUILTIN;  // the number of the LED pin

// TODO:
// - add channel number swap D8 or D16
// ie: so could be digital or analogic
// - add failsafe logic
// - test sbus to struct...
// - 
#include <avr/eeprom.h>

#define F_CPU 16000000UL  // Define CPU frequency as 16 MHz (typical for Arduino Uno)
// #define PRESCALER 8        // Assuming Timer1 is configured with a prescaler of 8
// #define DEBUG

// #define ENABLE_TICK_INTERRUPT( )       ( TIMSK2 |= ( 1<< OCIE2A ) )
// #define DISABLE_TICK_INTERRUPT( )      ( TIMSK2 &= ~( 1<< OCIE2A ) )
// #define CLEAR_TICK_INTERRUPT( )        ( TIFR2 = (1 << OCF2A) )

// struct t_rx_sbus
// {
//     uint8_t Sbuffer[28];
//     uint8_t Sindex = 0;	
//     uint8_t Slast_rcv_ms = 0;
//     uint8_t need_process = 0;
// } volatile sbus;


struct t_pinConfig {
    uint8_t *port;  // Pointer to the port register (PORTB, PORTD, etc.)
    uint8_t pin;    // Pin number (0-7)
    uint16_t pulse;
    uint16_t failpulse;
};

t_pinConfig **outputs_sorted[16]; // Swapped and 8/16 channel set by switch
t_pinConfig outputs_unsorted[16] = {
    { &PORTD, 2 , 127},
    { &PORTD, 3 , 127},
    { &PORTD, 4 , 127},
    { &PORTD, 5 , 127},
    { &PORTB, 0 , 127},
    { &PORTB, 1 , 127},
    { &PORTB, 2 , 127},
    { &PORTB, 3 , 127},
    { &PORTC, 0 , 127},
    { &PORTC, 1 , 127},
    { &PORTC, 2 , 127},
    { &PORTC, 3 , 127},
    { &PORTD, 6 , 127},
    { &PORTD, 7 , 127},
    { &PORTB, 4 , 127},
    { nullptr, 0 , 127}
    // { &PORTB, null }
};

uint16_t FailsafeTimes[16] ;


volatile uint32_t millis_inc = 0;
ISR(TIMER0_COMPA_vect)
{
  OCR0A += 250; // Advance The COMPA Register
  millis_inc++;
}

volatile uint16_t tick_ppm = 0;
ISR(TIMER2_COMPA_vect)
{
  OCR2A += 88; // Advance The COMPA Register
  tick_ppm++;
}

// ISR(USART0_RX_vect) {
//     // ISR triggered when data is received
//     uint8_t rx_byte = UDR0;  // Read the received data from the UART data register

//     // SBUS messages take 3ms to communicate. Typical repeat intervals are: 14ms (analog mode), 7ms (high speed mode).
//     // TODO: add if Sto_proceed is false...
//     if ( ! sbus.need_process )
//     {
//         // Start only if delay passed.
//         // give 5ms for postprocessing that sbus
//         if ( millis_() - sbus.Slast_rcv_ms > 5 )
//         {
//             // TODO: check corrupted ?
//             uint8_t idx = sbus.Sindex;

//             sbus.Sbuffer[idx] = rx_byte;
//             if ( idx > 24 ) //Sbus = 25bytes
//             {
//                 sbus.Slast_rcv_ms == millis_();
//                 sbus.need_process = 1;
//             }
//         }
//     }
// }

// loop over outputs_unsorted, return i_position, by low pulse -> high pulse
uint8_t sortByPulse(const t_pinConfig arr[], uint8_t order[], uint8_t size) {
    // Initialize the order array
    for (uint8_t i = 0; i < size; i++) {
        order[i] = i;
    }

    // Simple Bubble Sort to sort indices based on the pulse element
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = 0; j < size - i - 1; j++) {
            if (arr[order[j]].pulse > arr[order[j + 1]].pulse) {
                // Swap the indices in the order array
                uint8_t temp = order[j];
                order[j] = order[j + 1];
                order[j + 1] = temp;
            }
        }
    }
}


struct SbusData {
  bool lost_frame;
  bool failsafe;
  bool ch17, ch18;
  static constexpr int8_t NUM_CH = 16;
  int16_t ch[NUM_CH];
};

class SbusRx {
 public:
  explicit SbusRx(HardwareSerial *bus, const bool fast = false)
   : uart_(bus), fast_(fast) {}

  void Begin()
  {
    baud_ = fast_ ? 200000 : 100000;
    uart_->begin(baud_, SERIAL_8E2);
    uart_->flush();
  }
  bool Read()
  {
    /* Read through all available packets to get the newest */
    new_data_ = false;
    while (uart_->available())
    {
        if (Parse()) {
            new_data_ = true;
        }
    }
    return new_data_;
  }
  inline SbusData data() const {return data_;}

 private:
  /* Communication */
  HardwareSerial *uart_;
  bool fast_ = false;
  int32_t baud_ = 100000;
  /* Message len */
  static constexpr int8_t PAYLOAD_LEN_ = 23;
  static constexpr int8_t HEADER_LEN_ = 1;
  static constexpr int8_t FOOTER_LEN_ = 1;
  /* SBUS message defs */
  static constexpr int8_t NUM_SBUS_CH_ = 16;
  static constexpr uint8_t HEADER_ = 0x0F;
  static constexpr uint8_t FOOTER_ = 0x00;
  static constexpr uint8_t FOOTER2_ = 0x04;
  static constexpr uint8_t CH17_MASK_ = 0x01;
  static constexpr uint8_t CH18_MASK_ = 0x02;
  static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
  static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
  /* Parsing state tracking */
  uint8_t state_ = 0;
  uint8_t prev_byte_ = FOOTER_;
  uint8_t cur_byte_;
  /* Buffer for storing messages */
  uint8_t buf_[25];
  /* Data */
  bool new_data_;
  SbusData data_;
  bool Parse()
    {
    /* Parse messages */
    while (uart_->available()) {
        cur_byte_ = uart_->read();
        if (state_ == 0) {
        if ((cur_byte_ == HEADER_) && ((prev_byte_ == FOOTER_) ||
            ((prev_byte_ & 0x0F) == FOOTER2_))) {
            buf_[state_++] = cur_byte_;
        } else {
            state_ = 0;
        }
        } else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_) {
            buf_[state_++] = cur_byte_;
        } else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_ + FOOTER_LEN_) {
        state_ = 0;
        prev_byte_ = cur_byte_;
        if ((cur_byte_ == FOOTER_) || ((cur_byte_ & 0x0F) == FOOTER2_)) {
            /* Grab the channel data */
            ParseChannels();
            return true;
        } else {
            return false;
        }
        } else {
        state_ = 0;
        }
        prev_byte_ = cur_byte_;
    }
    return false;
  }
  void ParseChannels() {
        data_.ch[0]  = static_cast<int16_t>(buf_[1] | ((buf_[2] << 8) & 0x07FF));
        data_.ch[1]  = static_cast<int16_t>((buf_[2] >> 3) | ((buf_[3] << 5) & 0x07FF));
        data_.ch[2]  = static_cast<int16_t>((buf_[3] >> 6) | (buf_[4] << 2) | ((buf_[5] << 10) & 0x07FF));
        data_.ch[3]  = static_cast<int16_t>((buf_[5] >> 1) | ((buf_[6] << 7) & 0x07FF));
        data_.ch[4]  = static_cast<int16_t>((buf_[6] >> 4) | ((buf_[7] << 4) & 0x07FF));
        data_.ch[5]  = static_cast<int16_t>((buf_[7] >> 7) | (buf_[8] << 1) | ((buf_[9] << 9) & 0x07FF));
        data_.ch[6]  = static_cast<int16_t>((buf_[9] >> 2) | ((buf_[10] << 6) & 0x07FF));
        data_.ch[7]  = static_cast<int16_t>((buf_[10] >> 5) | ((buf_[11] << 3) & 0x07FF));
        data_.ch[8]  = static_cast<int16_t>(buf_[12] | ((buf_[13] << 8) & 0x07FF));
        data_.ch[9]  = static_cast<int16_t>((buf_[13] >> 3) | ((buf_[14] << 5) & 0x07FF));
        data_.ch[10] = static_cast<int16_t>((buf_[14] >> 6) | (buf_[15] << 2) | ((buf_[16] << 10) & 0x07FF));
        data_.ch[11] = static_cast<int16_t>((buf_[16] >> 1) | ((buf_[17] << 7) & 0x07FF));
        data_.ch[12] = static_cast<int16_t>((buf_[17] >> 4) | ((buf_[18] << 4) & 0x07FF));
        data_.ch[13] = static_cast<int16_t>((buf_[18] >> 7) | (buf_[19] << 1) | ((buf_[20] << 9) & 0x07FF));
        data_.ch[14] = static_cast<int16_t>((buf_[20] >> 2) | ((buf_[21] << 6) & 0x07FF));
        data_.ch[15] = static_cast<int16_t>((buf_[21] >> 5) | ((buf_[22] << 3) & 0x07FF));
        data_.ch17 = buf_[23] & CH17_MASK_;
        data_.ch18 = buf_[23] & CH18_MASK_;
        data_.lost_frame = buf_[23] & LOST_FRAME_MASK_;
        data_.failsafe = buf_[23] & FAILSAFE_MASK_;
    }
};
SbusRx sbus(&Serial);

// the setup function runs once when you press reset or power the board
void init_board() {
#ifdef DEBUG
   Serial.begin(57600);
#endif

    // Timer0 : millis
  TCCR0A = 0;           // Init Timer0A
  TCCR0B = 0;           // Init Timer0B
  TCCR0B |= B00000011;  // Prescaler = 64
  OCR0A = 250;        // Timer Compare0A Register
  TIMSK0 |= B00000010;  // Enable Timer COMPA Interrupt
//     TCCR0A = 0;  // Normal mode
//     TCCR0B |= (1 << CS01) | (1 << CS00);  // Set prescaler to 64
// //   TCCR0B = 0;           // Init Timer0B
// //   TCCR0B |= B00000011;  // Prescaler = 64
//   OCR0A = 250;        // Timer Compare0A Register
// //   TIMSK0 |= B00000010;  // Enable Timer COMPA Interrupt
//     // TIMSK0 |= 0x02;
//     TIMSK0 |= (1 << TOIE0);  // Enable Timer0 overflow interrupt

    // Timer2 : ppm tick
    // TCCR2A = 0;  // Normal mode
    // TCCR2B |= (1 << CS22);  // Set prescaler to 64
    // OCR2A = 138;        // Timer Compare2A Register
    // TIMSK2 |= B00000010;  // (1 << TOIE2); Enable Timer COMPA Interrupt
  TCCR2A = 0;           // Init Timer2A
  TCCR2B = 0;           // Init Timer2B
  TCCR2B |= B00000001;  // Prescaler = 1
  OCR2A = 88;        // Timer Compare2A Register
  TIMSK2 |= B00000010;  // Enable Timer COMPA Interrupt

    // // Timer1
    // // Clear Timer1 Control Registers
    // TCCR1A = 0x00;    //Init.
    // // TCCR1B = 0x00; // //|= (1 << CS11);
    // // Set prescaler to 8 (CS11 bit set)
    // TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11); 
    // // Set Timer1 to Normal mode (no PWM, no CTC)
    // TIFR1 = (1 << OCF1A);
    // TIMSK1 |= ( 1<< OCIE1A );

    // uint16_t baud_rate = 103;  // Assuming 16 MHz clock and 9600 baud
    // UBRR0H = (uint8_t)(baud_rate >> 8);
    // UBRR0L = (uint8_t)baud_rate;

    // // Enable receiver
    // // UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    // UCSR0B &= ~TXEN0 ;

    // // Set frame format: 8 data bits, 1 stop bit
    // UCSR0C = (1<<UCSZ00) | (1<<UCSZ01 ) | (1<<UPM01) ;
    // // Enable global interrupts
    // // DDRD |= (1<<2) ;
    // // DDRD |= (1<<3) ;
    DDRD |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);  // Set PORTD pins as output
    DDRB |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5);  // Set PORTB pins as output
    DDRC |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);  // Set PORTC pins as output

    sbus.Begin();

    uint8_t ChannelSwap = 0;
    if ( ( PINC & 0x20 ) == 0 )		// Link on AD5
    {
      ChannelSwap = 1 ;
    }
      uint8_t EightOnly = 0;
    if ( ( PIND & 0x02 ) == 0 )		// Link on PD1
    {
      EightOnly = 1 ;
    }

    t_pinConfig *port_pin_ptr = outputs_unsorted;

    for (uint8_t i = 0; i < 16; i++)
    {
        // ( i >= 8 && ChannelSwap ) ? outputs_sorted[i] = &port_pin_ptr++ : outputs_sorted[i] = &port_pin_ptr++;
        if ( i >= 8 && ChannelSwap)
        {
          outputs_sorted[i-8] = &port_pin_ptr;
        }
        else
        {
          // port_pin[i] -= 8;
          outputs_sorted[i+8] = &port_pin_ptr;
        }
        port_pin_ptr++;
    }

    // TODO: set pin swap AND speed ana/dig
    sei();

}

// the loop function runs over and over again forever
int main() {

    // ie: make a "mean" operation with two value
    // int mean = (a & b) + ((a ^ b) >> 1);

    // disable failsafe switch after 10sec...
    // if switch failsafe
    //   set failsafe (write to eeprom)

    // if no new value after 500ms 
    //   go to failsafe...
    // 
    // if ( ! init_ok )
    // {
    //     init_board();
    //     init_ok = 1;
    // }
    init_board();

    while(1)
    {
        static uint32_t outputLoopTimeout = 0;
    // static uint8_t init_ok = 0;
        static uint32_t blinkLoopTimeout = 0;
        static uint32_t sbusLoopTimeout = 0;
        static uint16_t setFailsafeLoopTimeout = 0;

        static uint8_t check_failsafe = 0;

        if ( millis_() - sbusLoopTimeout > 3 )
        {
            if (sbus.Read())
            {
              // .. do some actualisation sbus -> pulse
              check_failsafe = 0;
            }
            else
            {
              check_failsafe++;
              if (check_failsafe > 150)
              {
                readFailpulseFromEEPROM();
                for ( uint8_t i = 0 ; i < 16; i++)
                {
                  outputs_unsorted[i].pulse = outputs_unsorted[i].failpulse;
                }
              }
            }
            sbusLoopTimeout = millis_();
        }

        if ( millis_() - outputLoopTimeout > 19 )
        {
            setOutput(1);
            outputLoopTimeout = millis_();
        }

        if ( millis_() - blinkLoopTimeout > 1000 )
        {
            blinkLoopTimeout = millis_();
            PINB |= (1 << PINB5); // PINB = PORTB ^ B00100000; //(1 << PINB5);
        }

        if ( millis_() - setFailsafeLoopTimeout < 500 * 1000 ) // ie: 500ms
        {
          if (( PINC & 0x10 ) == 0 ) // Failsafe Switch
          {
            // ie: bufferoverflow after 54days?
            // After 500ms, disable posibility to set failsafe with switch
            writeFailpulseToEEPROM();
          }
        }

        setOutput(0);
        // processISR();
    #ifdef DEBUG
        debugLoop();
    #endif
    }
    return 0;
}

void setOutput(uint8_t init)
{
    static uint8_t current_channel = 0;
    static uint8_t order_pulse_down[16];
    static uint8_t tick_ready = 0;
    static uint8_t *port;
    static uint8_t pin;
    // TODO: Make the swap here ?
    // static t_pinConfig *port_pin_ptr = &outputs_sorted;

    if ( init ) // ! pin_on && 
    {
        constexpr uint8_t size = sizeof(order_pulse_down) / sizeof(order_pulse_down[0]);
        cli();
        sortByPulse(outputs_unsorted, order_pulse_down, size);
        // memcpy(ch_tick_sorted, ch_tick, sizeof(ch_tick_sorted));  // Copy unsorted into sorted
        sei();
        // bubbleSort(ch_tick_sorted, size);

        // t_pinConfig **output_sorted_ptr[16];
        for (size_t i = 0; i < 16; i++)
        {
            // output_sorted_ptr[i] = &outputs_sorted[i];
            port = outputs_unsorted[i].port; //(*outputs_sorted[i])->port;
            pin = outputs_unsorted[i].pin; //(*outputs_sorted[i])->pin;
            *port |= (1 << pin);
        }
        // pin_on = 1;
        current_channel = 0;
        tick_ready = 0;
        // DISABLE_TICK_INTERRUPT( );
        // CLEAR_TICK_INTERRUPT( );
        tick_ppm = 0;
        // ENABLE_TICK_INTERRUPT( );
    }
    if ( current_channel < 16 )
    {
      // Set "neutral" by low offset, approx 800us
      if ( tick_ppm > ( 118 ) && ! tick_ready )
      {
        tick_ready = 1;
        tick_ppm = 0;
      }

      uint8_t order = order_pulse_down[current_channel];
      if (( tick_ppm > outputs_unsorted[order].pulse ) && tick_ready )
      {
          // set [current_channel] low
    //   uint8_t order = order_pulse_down[i];
    //   uint8_t port = &outputs_unsorted[order].port;
    //   uint8_t pin = outputs_unsorted[order].pin; //(*outputs_sorted[i])->pin;
    //   uint8_t pulse = outputs_unsorted[order].pulse; //(*outputs_sorted[i])->pulse;
          port = outputs_unsorted[order].port; //(*outputs_sorted[current_channel])->port;
          pin = outputs_unsorted[order].pin; //(*outputs_sorted[current_channel])->pin;
          *port &= ~(1 << pin);
          current_channel++;
      }
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

// uint8_t processISR()
// {
//     // asm volatile ("nop\n\t");
//     // https://os.mbed.com/users/Digixx/code/SBUS-Library_16channel
//     if ( sbus.need_process )
//     {
//         uint8_t *sbus_ptr = sbus.Sbuffer;

//         if ( *sbus_ptr++ != 0x0F ) // sbus.Sbuffer[0]
//         {
//             return 0;
//         }
//         else if (sbus.Sbuffer[24] == 0x00 )
//         {
//             uint8_t inputbitsavailable = 0;
//             uint32_t inputbits = 0;

//             for (uint8_t i = 0 ; i < 16 ; i += 1 )
//             {
//                 uint16_t value = 0;
//                 while ( inputbitsavailable < 11 )
//                 {
//                     inputbits |= (uint32_t)*sbus_ptr++ << inputbitsavailable;
//                     inputbitsavailable += 8;
//                 }
//                 value = (int16_t)(inputbits & 0x7FF) - 0x3E0;
//                 value = (value << 2) + value; // Equivalent to value * 5
//                 value = (value >> 3) + 1500;
//                 if ( ( value > 800 ) && ( value < 2200 ) )
//                 {
//                     pulseTimes[i] = value;
//                 }
//                 // else
//                 // {
//                 //     // don't change value?
//                 // }
//                 inputbitsavailable -= 11;
//                 inputbits >>= 11;
//             }
//             // DigiChannel 1
//             if (sbus.Sbuffer[23] & (1<<0)) {
//                 pulseTimes[16] = 1;
//             }else{
//                 pulseTimes[16] = 0;
//             }
//             // DigiChannel 2
//             if (sbus.Sbuffer[23] & (1<<1)) {
//                 pulseTimes[17] = 1;
//             }else{
//                 pulseTimes[17] = 0;
//             }
//             // Failsafe
//             // failsafe_status = SBUS_SIGNAL_OK;
//             if (sbus.Sbuffer[23] & (1<<2)) {
//                 // failsafe_status = SBUS_SIGNAL_LOST;
//             }
//             if (sbus.Sbuffer[23] & (1<<3)) {
//                 // failsafe_status = SBUS_SIGNAL_FAILSAFE;
//             }

//                 // TODO: Test and should be more readable
//            // uint8_t byte_in_sbus = 1;
//             // uint8_t bit_in_sbus = 0;
//             // uint8_t channel = 0;
//             // uint8_t bit_in_channel = 0;

//             // for (uint8_t i=0; i<176; i++ ) {
//             //     if ( sbus.Sbuffer[byte_in_sbus] & (1<<bit_in_sbus) ) {
//             //         pulseTimes[channel] |= (1<<bit_in_channel);
//             //     }
//             //     bit_in_sbus++;
//             //     bit_in_channel++;

//             //     if ( bit_in_sbus == 8 ) {
//             //         bit_in_sbus =0;
//             //         byte_in_sbus++;
//             //     }
//             //     if ( bit_in_channel == 11 ) {
//             //         bit_in_channel =0;
//             //         channel++;
//             //     }
//             // }
//         }
//         else
//         {
//             return 0;
//         }
//         sbus.need_process = 0;
//     }
// }

#define EEPROM_START_ADDRESS 0x10  // Starting EEPROM address

void eeprom_write_byte_cmp (uint8_t dat, uint16_t pointer_eeprom)
{
  //see /home/thus/work/avr/avrsdk4/avr-libc-1.4.4/libc/misc/eeprom.S:98 143
  while(EECR & (1<<EEPE)) /* make sure EEPROM is ready */
  {
  }
  EEAR  = pointer_eeprom ;

  EECR |= 1<<EERE ;
  if(dat == EEDR) return ;

  EEDR  = dat ;
  uint8_t flags=SREG ;
  cli() ;
  EECR |= 1<<EEMPE ;
  EECR |= 1<<EEPE ;
  SREG = flags ;
}


void writeFailpulseToEEPROM() {
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t eepromAddress = EEPROM_START_ADDRESS + (i * 2);  // Two bytes per failpulse

        // Split the 16-bit failpulse into two bytes
        uint8_t lowByte = (outputs_unsorted[i].pulse & 0xFF);
        uint8_t highByte = (outputs_unsorted[i].pulse >> 8);

        // Write the low byte and high byte of failpulse to EEPROM
        eeprom_write_byte_cmp((uint8_t *)eepromAddress, lowByte);
        eeprom_write_byte_cmp((uint8_t *)(eepromAddress + 1), highByte);
    }
}

void readFailpulseFromEEPROM() {
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t eepromAddress = EEPROM_START_ADDRESS + (i * 2);  // Two bytes per failpulse

        // Read the low byte and high byte from EEPROM
        uint8_t lowByte = eeprom_read_byte((uint8_t *)eepromAddress);
        uint8_t highByte = eeprom_read_byte((uint8_t *)(eepromAddress + 1));

        // Combine the two bytes into a 16-bit value
        outputs_unsorted[i].failpulse = (highByte << 8) | lowByte;
    }
}


uint32_t millis_()
{
    uint32_t millis_tmp;
    cli();
    millis_tmp = millis_inc;
    sei();
    return millis_tmp;
}


// void debugLoop() {
//   static const uint16_t interval = 1000;  // interval at which to blink (milliseconds)
//   static uint32_t previousMillis = 0;  // will store last time LED was updated
//   uint32_t currentMillis = millis();

//   if (currentMillis - previousMillis > interval) {
//     previousMillis = currentMillis;

//     static uint16_t old_micros;
//     Serial.print(F("micros delay: "));
//     uint16_t delayMicrosOriginal = micros();
//     Serial.println(delayMicrosOriginal);
//     Serial.print(F("micro lag: "));
//     Serial.println(delayMicrosOriginal - old_micros);
//     old_micros = delayMicrosOriginal;

//     Serial.print(F("delay millis: "));
//     uint32_t delayMillisOriginal = millis();
//     Serial.println(delayMillisOriginal);

//     // Serial.print(F("delay micros_: "));
//     // Serial.println(micros_());
//     // Serial.print(F("delay millis_: "));
//     // Serial.println(millis_.time());


//     Serial.print(F("Delay millis_inc: "));
//     Serial.println(millis_inc);
//     Serial.println("-----------"); 
//   }
// }
