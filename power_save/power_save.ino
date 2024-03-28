//// This variable is made volatile because it is changed inside
//// an interrupt function
volatile int f_wdt=1;
const int LED = 5;
int count = 0;

ISR(WDT_vect) {
  if(f_wdt == 0) {
    // here we can implement a counter the can set the f_wdt to true if
    // the watchdog cycle needs to run longer than the maximum of eight
    // seconds.
    count=count+1;
    if (count ==4){
      f_wdt=1;
      count = 0;
    }
  }
}
void setup() {
  // put your setup code here, to run once:
    pinMode(LED,  OUTPUT);//Declaring that the LED is an output.
    setupWatchDogTimer();
}

void loop() {
    //initialize_timer2();
    // Wait until the watchdog have triggered a wake up.
    if(f_wdt != 1) {
      return;
    }
  
    // Toggle the LED on
    digitalWrite(LED, 1);
    // wait
    delay(200);
    // Toggle the LED off
    digitalWrite(LED, 0);
  
    // clear the flag so we can run above code again after the MCU wake up
    f_wdt = 0;
  
    // Re-enter sleep mode.
    start_sleep();
}

void start_sleep() {
  //TCCR2B = 1;  // start Timer 2
  SMCR = (0 << SM0);  
  SMCR = (1 << SM1);
  //enable
  delay(10);
  SMCR = (0 << SM2);

  
}

void sleep_off(){
    SMCR = (0 << SM0);//said to do this just after waking upS
}

void initialize_timer2() {
  TCCR2B = 0;  // stop Timer 2
  TIMSK2 = 0; // disable Timer 2 interrupts
  ASSR = (1 << AS2);  // select asynchronous operation of Timer2
  TCNT2 = 0;      // clear Timer 2 counter
  TCCR2A = 0;     // normal count up mode, no port output
  TCCR2B = (1 << CS22) | (1 << CS20);   // select prescaler 128 => 1 sec between each overflow

 while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB)|(1<<TCR2AUB))); // wait for TCN2UB and TCR2A/BUB to be cleared

  TIFR2 = (1 << TOV2);      // clear interrupt-flag
  TIMSK2 = (1 << TOIE2);  // enable Timer2 overflow interrupt
}

// Setup the Watch Dog Timer (WDT)
void setupWatchDogTimer() {
  // The MCU Status Register (MCUSR) is used to tell the cause of the last
  // reset, such as brown-out reset, watchdog reset, etc.
  // NOTE: for security reasons, there is a timed sequence for clearing the
  // WDE and changing the time-out configuration. If you don't use this
  // sequence properly, you'll get unexpected results.

  // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
  MCUSR &= ~(1<<WDRF);

  // Configure the Watchdog timer Control Register (WDTCSR)
  // The WDTCSR is used for configuring the time-out, mode of operation, etc

  // In order to change WDE or the pre-scaler, we need to set WDCE (This will
  // allow updates for 4 clock cycles).

  // Set the WDCE bit (bit 4) and the WDE bit (bit 3) of the WDTCSR. The WDCE
  // bit must be set in order to change WDE or the watchdog pre-scalers.
  // Setting the WDCE bit will allow updates to the pre-scalers and WDE for 4
  // clock cycles then it will be reset by hardware.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  WDTCSR  = (1<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
  // Enable the WD interrupt (note: no reset).
  WDTCSR |= _BV(WDIE);
}
