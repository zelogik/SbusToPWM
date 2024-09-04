const int ledPin = LED_BUILTIN;  // the number of the LED pin

#define F_CPU 16000000UL  // Define CPU frequency as 16 MHz (typical for Arduino Uno)
#define PRESCALER 8        // Assuming Timer1 is configured with a prescaler of 8

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


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(57600);

    // Timer1
    // Clear Timer1 Control Registers
    TCCR1A = 0x00;    //Init.
    // TCCR1B = 0x00; // //|= (1 << CS11);
    // Set prescaler to 8 (CS11 bit set)
    TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11); 
    // Set Timer1 to Normal mode (no PWM, no CTC)
    TIFR1 = (1 << OCF1A);
    TIMSK1 |= ( 1<< OCIE1A );

}

// the loop function runs over and over again forever
void loop() {
  debugLoop();
  micros_();
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