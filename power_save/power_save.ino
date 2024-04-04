//// This variable is made volatile because it is changed inside
//// an interrupt function
volatile int f_wdt = 1;
const int LED = 5;
int count = 0;

ISR(WDT_vect) {
  if (f_wdt == 0) {
    // here we can implement a counter the can set the f_wdt to true if
    // the watchdog cycle needs to run longer than the maximum of eight
    // seconds.
    count = count + 1;
    if (count == 4) {
      f_wdt = 1;
      count = 0;
    }
  }
}
void setup() {
  // put your setup code here, to run once:
  pinMode(LED,  OUTPUT);//Declaring that the LED is an output.
  pinMode(13,  OUTPUT);//Declaring that the LED is an output.
  digitalWrite(13,  LOW);//Declaring that the LED is an output.
  setupWatchDogTimer();
  //setClockFrequency(10);
}

void loop() {
  //initialize_timer2();
  // Wait until the watchdog have triggered a wake up.
  if (f_wdt != 1) {
    return;
  }
  //sleep_off();

  // Toggle the LED on
  digitalWrite(LED, 1);
  // wait
  delay(5000);
  // Toggle the LED off
  digitalWrite(LED, 0);

  // clear the flag so we can run above code again after the MCU wake up
  f_wdt = 0;

  // Re-enter sleep mode.
  start_sleep();
}

void start_sleep() {
  //SMCR = ((SMCR & ~(_BV(SM0) | _BV(SM1) | _BV(SM2))) | (_BV(SM1)));
  MCUCR &= (1 << BODS) | (1 << BODSE);//to turn off brown out detections first
  MCUCR &= (1 << BODS) | ~(1 << BODSE);//second step

  ADCSRA &= ~(1 << ADEN);
  PRR = 0xFF;

  //  SMCR &= (0 << SM0);
  //  SMCR &= (1 << SM1);
  //  //enable
  ////  delay(10);
  //  SMCR &= (1 << SM2);



  SMCR |= (uint8_t)_BV(SE);
  SMCR = ((SMCR & ~(_BV(SM0) | _BV(SM1) | _BV(SM2))) | (_BV(SM1)));
}

void sleep_off() {
  SMCR &= (uint8_t)(~_BV(SE));//said to do this just after waking upS

  ADCSRA &= (1 << ADEN);
  PRR = 0x00;
}
// Setup the Watch Dog Timer (WDT)
void setupWatchDogTimer() {
  // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
  MCUSR &= ~(1 << WDRF);

  WDTCSR |= (1 << WDCE) | (1 << WDE);

  WDTCSR  = (1 << WDP3) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0); //prescalar
  // Enable the WD interrupt (note: no reset).
  WDTCSR |= _BV(WDIE);
}


//Timer
//void initialize_timer2() {
//  TCCR2B = 0;  // stop Timer 2
//  TIMSK2 = 0; // disable Timer 2 interrupts
//  ASSR = (1 << AS2);  // select asynchronous operation of Timer2
//  TCNT2 = 0;      // clear Timer 2 counter
//  TCCR2A = 0;     // normal count up mode, no port output
//  TCCR2B = (1 << CS22) | (1 << CS20);   // select prescaler 128 => 1 sec between each overflow
//
// while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB)|(1<<TCR2AUB))); // wait for TCN2UB and TCR2A/BUB to be cleared
//
//  TIFR2 = (1 << TOV2);      // clear interrupt-flag
//  TIMSK2 = (1 << TOIE2);  // enable Timer2 overflow interrupt
//}
