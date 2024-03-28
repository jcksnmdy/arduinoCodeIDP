void initialize_timer2() {
  TCCR2B = 0;  // stop Timer 2
  TIMSK2 = 0;	// disable Timer 2 interrupts
  ASSR = (1 << AS2);	// select asynchronous operation of Timer2
  TCNT2 = 0;			// clear Timer 2 counter
  TCCR2A = 0; 		// normal count up mode, no port output
  TCCR2B = (1 << CS22) | (1 << CS20);		// select prescaler 128 => 1 sec between each overflow

 while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB)|(1<<TCR2AUB))); // wait for TCN2UB and TCR2A/BUB to be cleared

  TIFR2 = (1 << TOV2);			// clear interrupt-flag
  TIMSK2 = (1 << TOIE2);	// enable Timer2 overflow interrupt
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
