#include <IRremote.h>

#include "WORLD_IR_CODES.h"
#include <avr/sleep.h>

void xmitCodeElement(uint16_t ontime, uint16_t offtime, uint8_t PWM_code);
void quickflashLEDx(uint8_t x);
void delay_ten_us(uint16_t us);
void quickflashLED(void);
uint8_t read_bits(uint8_t count);

#define putstring_nl(s) Serial.println(s)
#define putstring(s) Serial.print(s)
#define putnum_ud(n) Serial.print(n, DEC)
#define putnum_uh(n) Serial.print(n, HEX)

#define MAX_WAIT_TIME 65535  //tens of us (ie: 655.350ms)


extern const IrCode* const NApowerCodes[] PROGMEM;
extern const IrCode* const EUpowerCodes[] PROGMEM;
extern uint8_t num_NAcodes, num_EUcodes;

/* This function is the 'workhorse' of transmitting IR codes.
 Given the on and off times, it turns on the PWM output on and off
 to generate one 'pair' from a long code. Each code has ~50 pairs! */
void xmitCodeElement(uint16_t ontime, uint16_t offtime, uint8_t PWM_code) {
  TCNT2 = 0;
  if (PWM_code) {
    pinMode(IRLED, OUTPUT);
    // Fast PWM, setting top limit, divide by 8
    // Output to pin 3
    TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(WGM22) | _BV(CS21);
  } else {
    // However some codes dont use PWM in which case we just turn the IR
    // LED on for the period of time.
    digitalWrite(IRLED, HIGH);
  }

  // Now we wait, allowing the PWM hardware to pulse out the carrier
  // frequency for the specified 'on' time
  delay_ten_us(ontime);

  // Now we have to turn it off so disable the PWM output
  TCCR2A = 0;
  TCCR2B = 0;
  // And make sure that the IR LED is off too (since the PWM may have
  // been stopped while the LED is on!)
  digitalWrite(IRLED, LOW);

  // Now we wait for the specified 'off' time
  delay_ten_us(offtime);
}

/* This is kind of a strange but very useful helper function
 Because we are using compression, we index to the timer table
 not with a full 8-bit byte (which is wasteful) but 2 or 3 bits.
 Once code_ptr is set up to point to the right part of memory,
 this function will let us read 'count' bits at a time which
 it does by reading a byte into 'bits_r' and then buffering it. */

uint8_t bitsleft_r = 0;
uint8_t bits_r = 0;
PGM_P code_ptr;

// we cant read more than 8 bits at a time so dont try!
uint8_t read_bits(uint8_t count) {
  uint8_t i;
  uint8_t tmp = 0;

  // we need to read back count bytes
  for (i = 0; i < count; i++) {
    // check if the 8-bit buffer we have has run out
    if (bitsleft_r == 0) {
      // in which case we read a new byte in
      bits_r = pgm_read_byte(code_ptr++);
      // and reset the buffer size (8 bites in a byte)
      bitsleft_r = 8;
    }
    // remove one bit
    bitsleft_r--;
    // and shift it off of the end of 'bits_r'
    tmp |= (((bits_r >> (bitsleft_r)) & 1) << (count - 1 - i));
  }
  // return the selected bits in the LSB part of tmp
  return tmp;
}


/*
The C compiler creates code that will transfer all constants into RAM when
 the microcontroller resets.  Since this firmware has a table (powerCodes)
 that is too large to transfer into RAM, the C compiler needs to be told to
 keep it in program memory space.  This is accomplished by the macro PROGMEM
 (this is used in the definition for powerCodes).  Since the C compiler assumes
 that constants are in RAM, rather than in program memory, when accessing
 powerCodes, we need to use the pgm_read_word() and pgm_read_byte macros, and
 we need to use powerCodes as an address.  This is done with PGM_P, defined
 below.
 For example, when we start a new powerCode, we first point to it with the
 following statement:
 PGM_P thecode_p = pgm_read_word(powerCodes+i);
 The next read from the powerCode is a byte that indicates the carrier
 frequency, read as follows:
 const uint8_t freq = pgm_read_byte(code_ptr++);
 After that is a byte that tells us how many 'onTime/offTime' pairs we have:
 const uint8_t numpairs = pgm_read_byte(code_ptr++);
 The next byte tells us the compression method. Since we are going to use a
 timing table to keep track of how to pulse the LED, and the tables are
 pretty short (usually only 4-8 entries), we can index into the table with only
 2 to 4 bits. Once we know the bit-packing-size we can decode the pairs
 const uint8_t bitcompression = pgm_read_byte(code_ptr++);
 Subsequent reads from the powerCode are n bits (same as the packing size)
 that index into another table in ROM that actually stores the on/off times
 const PGM_P time_ptr = (PGM_P)pgm_read_word(code_ptr);
 */

#define BUTTON_PRESSED 0

uint16_t ontime, offtime;
uint8_t i, num_codes;
uint8_t region;

IRsend sender;

void setup() {
  Serial.begin(9600);

  TCCR2A = 0;
  TCCR2B = 0;

  digitalWrite(LED, LOW);
  digitalWrite(IRLED, LOW);
  pinMode(LED, OUTPUT);
  pinMode(IRLED, OUTPUT);
  pinMode(REGIONSWITCH, INPUT_PULLUP);
  pinMode(TRIGGER, INPUT_PULLUP);
  pinMode(PTVPOWER, INPUT_PULLUP);
  pinMode(FREEZE, INPUT_PULLUP);

  delay_ten_us(5000);  //50ms (5000x10 us) delay: let everything settle for a bit

  // determine region
  if (digitalRead(REGIONSWITCH)) {
    region = NA;
    DEBUGP(putstring_nl("NA"));
  } else {
    region = EU;
    DEBUGP(putstring_nl("EU"));
  }

  // Debug output: indicate how big our database is
  DEBUGP(putstring("\n\rNA Codesize: ");
         putnum_ud(num_NAcodes););
  DEBUGP(putstring("\n\rEU Codesize: ");
         putnum_ud(num_EUcodes););

  // Tell the user what region we're in  - 3 flashes is NA, 6 is EU
  if (region == NA)
    quickflashLEDx(3);
  else  //region == EU
    quickflashLEDx(6);
}

void sendAllCodes() {
  bool endingEarly = false;  //will be set to true if the user presses the button during code-sending

  // determine region from REGIONSWITCH: 1 = NA, 0 = EU (defined in main.h)
  if (digitalRead(REGIONSWITCH)) {
    region = NA;
    num_codes = num_NAcodes;
  } else {
    region = EU;
    num_codes = num_EUcodes;
  }

  // for every POWER code in our collection
  for (i = 0; i < num_codes; i++) {
    PGM_P data_ptr;

    // print out the code # we are about to transmit
    DEBUGP(putstring("\n\r\n\rCode #: ");
           putnum_ud(i));

    // point to next POWER code, from the right database
    if (region == NA) {
      data_ptr = (PGM_P)pgm_read_word(NApowerCodes + i);
    } else {
      data_ptr = (PGM_P)pgm_read_word(EUpowerCodes + i);
    }

    // print out the address in ROM memory we're reading
    DEBUGP(putstring("\n\rAddr: ");
           putnum_uh((uint16_t)data_ptr));

    // Read the carrier frequency from the first byte of code structure
    const uint8_t freq = pgm_read_byte(data_ptr++);
    // set OCR for Timer1 to output this POWER code's carrier frequency
    OCR2A = freq;
    OCR2B = freq / 3;  // 33% duty cycle

    // Print out the frequency of the carrier and the PWM settings
    DEBUGP(putstring("\n\rOCR1: ");
           putnum_ud(freq););
    DEBUGP(uint16_t x = (freq + 1) * 2;
           putstring("\n\rFreq: ");
           putnum_ud(F_CPU / x););

    // Get the number of pairs, the second byte from the code struct
    const uint8_t numpairs = pgm_read_byte(data_ptr++);
    DEBUGP(putstring("\n\rOn/off pairs: ");
           putnum_ud(numpairs));

    // Get the number of bits we use to index into the timer table
    // This is the third byte of the structure
    const uint8_t bitcompression = pgm_read_byte(data_ptr++);
    DEBUGP(putstring("\n\rCompression: ");
           putnum_ud(bitcompression);
           putstring("\n\r"));

    // Get pointer (address in memory) to pulse-times table
    // The address is 16-bits (2 byte, 1 word)
    PGM_P time_ptr = (PGM_P)pgm_read_word(data_ptr);
    data_ptr += 2;
    code_ptr = (PGM_P)pgm_read_word(data_ptr);

    // Transmit all codeElements for this POWER code
    // (a codeElement is an onTime and an offTime)
    // transmitting onTime means pulsing the IR emitters at the carrier
    // frequency for the length of time specified in onTime
    // transmitting offTime means no output from the IR emitters for the
    // length of time specified in offTime

//DEVELOPMENTAL TESTING:
#if 0
    // print out all of the pulse pairs
    for (uint8_t k=0; k<numpairs; k++) {
      uint8_t ti;
      ti = (read_bits(bitcompression)) * 4;
      // read the onTime and offTime from the program memory
      ontime = pgm_read_word(time_ptr+ti);
      offtime = pgm_read_word(time_ptr+ti+2);
      DEBUGP(putstring("\n\rti = ");
      putnum_ud(ti>>2);
      putstring("\tPair = ");
      putnum_ud(ontime));
      DEBUGP(putstring("\t");
      putnum_ud(offtime));
    }
    continue;
#endif

    // For EACH pair in this code....
    cli();
    for (uint8_t k = 0; k < numpairs; k++) {
      uint16_t ti;

      // Read the next 'n' bits as indicated by the compression variable
      // The multiply by 4 because there are 2 timing numbers per pair
      // and each timing number is one word long, so 4 bytes total!
      ti = (read_bits(bitcompression)) * 4;

      // read the onTime and offTime from the program memory
      ontime = pgm_read_word(time_ptr + ti);       // read word 1 - ontime
      offtime = pgm_read_word(time_ptr + ti + 2);  // read word 2 - offtime
      // transmit this codeElement (ontime and offtime)
      xmitCodeElement(ontime, offtime, (freq != 0));
    }
    sei();

    //Flush remaining bits, so that next code starts
    //with a fresh set of 8 bits.
    bitsleft_r = 0;

    // visible indication that a code has been output.
    quickflashLED();

    // delay 205 milliseconds before transmitting next POWER code
    delay_ten_us(20500);

    // if user is pushing (holding down) TRIGGER button, stop transmission early
    if (digitalRead(TRIGGER) == BUTTON_PRESSED) {
      endingEarly = true;
      delay_ten_us(50000);  //500ms delay
      quickflashLEDx(4);
      //pause for ~1.3 sec to give the user time to release the button so that the code sequence won't immediately start again.
      delay_ten_us(MAX_WAIT_TIME);  // wait 655.350ms
      delay_ten_us(MAX_WAIT_TIME);  // wait 655.350ms
      break;                        //exit the POWER code "for" loop
    }
  }  //end of POWER code for loop

  if (endingEarly == false) {
    //pause for ~1.3 sec, then flash the visible LED 8 times to indicate that we're done
    delay_ten_us(MAX_WAIT_TIME);  // wait 655.350ms
    delay_ten_us(MAX_WAIT_TIME);  // wait 655.350ms
    quickflashLEDx(8);
  }
}  //end of sendAllCodes

void loop() {
  //Super "ghetto" (but decent enough for this application) button debouncing:
  //-if the user pushes the Trigger button, then wait a while to let the button stop bouncing, then start transmission of all POWER codes
  if (digitalRead(TRIGGER) == BUTTON_PRESSED) {
    delay_ten_us(50000);  // delay 500ms to give the user time to release the button before the code sequence begins
    sendAllCodes();
  }
  if (digitalRead(FREEZE) == BUTTON_PRESSED) {
    quickflashLEDx(3);
    uint32_t data = 0xC1AA49B6;
    uint32_t data1 = 0x40FF44BB;
    uint32_t data2 = 0xFFFFFFFF;
    uint8_t len = 32;
    for (int i = 0; i < 10; i++) {
      sender.sendNEC(data, len);
    }
    for (int i = 0; i < 10; i++) {
      sender.sendNEC(data1, len);
      delay(100);
      sender.sendNEC(data2, len);
    }
  }

  if (digitalRead(PTVPOWER) == BUTTON_PRESSED) {
    quickflashLEDx(15);
    uint32_t data = 0xC1AA09F6;
    uint32_t data1 = 0x40FFD02F;
    uint32_t data2 = 0xFFFFFFFF;
    uint8_t len = 32;
    for (int i = 0; i < 10; i++) {
      sender.sendNEC(data, len);
    }
    for (int i = 0; i < 10; i++) {
      sender.sendNEC(data1, len);
      delay(100);
      sender.sendNEC(data2, len);
    }
  }
}


/****************************** LED AND DELAY FUNCTIONS ********/


// This function delays the specified number of 10 microseconds
// it is 'hardcoded' and is calibrated by adjusting DELAY_CNT
// in main.h Unless you are changing the crystal from 8MHz, dont
// mess with this.
//-due to uint16_t datatype, max delay is 65535 tens of microseconds, or 655350 us, or 655.350 ms.
//-NB: DELAY_CNT has been increased in main.h from 11 to 25 (last I checked) in order to allow this function
// to work properly with 16MHz Arduinos now (instead of 8MHz).
void delay_ten_us(uint16_t us) {
  uint8_t timer;
  while (us != 0) {
    // for 8MHz we want to delay 80 cycles per 10 microseconds
    // this code is tweaked to give about that amount.
    for (timer = 0; timer <= DELAY_CNT; timer++) {
      NOP;
      NOP;
    }
    NOP;
    us--;
  }
}


// This function quickly pulses the visible LED (connected to PB0, pin 5)
// This will indicate to the user that a code is being transmitted
void quickflashLED(void) {
  digitalWrite(LED, HIGH);
  delay_ten_us(3000);  // 30 ms ON-time delay
  digitalWrite(LED, LOW);
}

// This function just flashes the visible LED a couple times, used to
// tell the user what region is selected
void quickflashLEDx(uint8_t x) {
  quickflashLED();
  while (--x) {
    delay_ten_us(25000);  // 250 ms OFF-time delay between flashes
    quickflashLED();
  }
}



/****************************** SLEEP and WAKE FUNCTIONS ********/
// from kaqkjz:
// http://www.ka1kjz.com/561/adding-sleep-to-tv-b-gone-code/

void sleepNow() {
  set_sleep_mode(TRIGGER);  // sleep mode is set here

  sleep_enable();  // enables the sleep bit in the mcucr register

  attachInterrupt(0, wakeUpNow, LOW);  // use interrupt 0 (pin 2) and run function
  // wakeUpNow when pin 2 gets LOW

  sleep_mode();  // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE ON WAKE

  sleep_disable();  // first thing after waking, disable sleep

  detachInterrupt(0);  // disables int 0 as the wakeupnow code will
                       // not be executed during normal runtime
}

void wakeUpNow() {
  // any needed wakeup code can be placed here
}
