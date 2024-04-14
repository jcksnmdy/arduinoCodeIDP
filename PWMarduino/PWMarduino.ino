#include <avr/sleep.h>      // powerdown library
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX | 

int battery = A1;
int inputVoltage = A2;
const int wakeUpPin = 2;
const int led = 8;

// Set 'TOP' for PWM resolution.  Assumes 16 MHz clock.
const unsigned int TOP = 0x013F; //
//const unsigned int TOP = 0x028F; //
float maxA = 600;
float compare1 = 0; // current reading
float compare2 = 0; // previous reading
float compare3 = 0; // compare1 - compare2
byte keep_ADCSRA; 

int ThermistorPin = 3;
int initialReading;
float R1 = 100000;
float logR2, R2, T;
float c1 = 6.88216e-04, c2 = 2.894356e-04;

int maxReached = 0;
//ISR(INT0_vect) {
//  wakeUp();
//}

void wakeUp()
{
  stopSleep();
//  sleep_disable();
  // Clear interrupt flag for INT0
//  EIFR |= (1 << INTF0); // Set INTF0 bit in EIFR to clear interrupt flag
  detachInterrupt (0);
  ADCSRA = keep_ADCSRA;

}

void PWM16Begin()
{
  // Stop Timer/Counter1
  TCCR1A = 0;  // Timer/Counter1 Control Register A
  TCCR1B = 0;  // Timer/Counter1 Control Register B
  TIMSK1 = 0;   // Timer/Counter1 Interrupt Mask Register
  TIFR1 = 0;   // Timer/Counter1 Interrupt Flag Register
  ICR1 = TOP;
  OCR1A = 0;  // Default to 0% PWM
  OCR1B = 0;  // Default to 0% PWM
  // Set clock prescale to 1 for maximum PWM frequency
  TCCR1B |= (1 << CS10);
  //TCCR1A |= (1 << CS10);


  // Set to Timer/Counter1 to Waveform Generation Mode 14: Fast PWM with TOP set by ICR1
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12) ;
}


void PWM16Enable()
{
  // Enable Fast PWM on Pin 9: Set OC1A at BOTTOM and clear OC1A on OCR1A compare
  TCCR1A |= (1 << COM1A1);
  pinMode(9, OUTPUT);
}

void PWM16Disable()
{
  // Enable Fast PWM on Pin 9: Set OC1A at BOTTOM and clear OC1A on OCR1A compare
  TCCR1A &= ~(1 << COM1A1);
  pinMode(9, INPUT);
}

inline void PWM16(unsigned int PWMValue)
{
  OCR1A = constrain(PWMValue, 0, TOP);
}



void setupSleep() {
  // Configure wake up pin as input.
  // This will consumes few uA of current.

//  Serial.begin(9600);
//  Serial.println("start");
//  delay(100);
  // Setup external interrupt pin as input
  DDRD &= ~(1 << wakeUpPin);

  // Configure external interrupt 0 (INT0) to trigger on low level
  EICRA &= ~((1 << ISC01) | ~(1 << ISC00)); // Clear ISC01 and ISC00 bits

  // Enable external interrupt INT0
  EIMSK |= (1 << INT0);

  // Enable global interrupts
  asm volatile ("sei");
}
void stopSleep() {
//  SMCR &= ~(1 << SE); // Enable sleep mode
//  //finished sleeping and been interupted
//  asm volatile ("cli"); // Disable global interrupts
//  SMCR &= ~(1 << SM0) & ~(1 << SM1) & ~(1 << SM2);
//  MCUCR |= ~(1 << BODS) | ~(1 << BODSE);//to turn on brown out detections first
//  MCUCR = bit (BODS); //second step
  sleep_disable();
  //ADCSRA = keep_ADCSRA;
//  PRR &= ~(1 << PRTWI) & ~(1 << PRTIM2) & ~(1 << PRTIM0) & ~(1 << PRSPI) & ~(1 << PRUSART0) & ~(1 << PRTIM1) & ~(1 << PRSPI) & ~(1 << PRADC);
  //PRR = 0;
}

void goSleep() {
  keep_ADCSRA = ADCSRA;
  ADCSRA = 0;  
//  asm volatile ("cli"); // Disable global interrupts
  noInterrupts ();
  digitalWrite(4, LOW);
  delay(500);
  digitalWrite(4, HIGH);
  delay(500);
  attachInterrupt (0, wakeUp, LOW);
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  sleep_enable();  
//  // Enter power-down mode
//  SMCR |= (1 << SM1); // Set bit SM1 in SMCR (Sleep Mode Control Register) for power-down mode
//  SMCR |= (1 << SE); // Enable sleep mode
//  PRR = 0xFF;
  MCUCR &= (1 << BODS) | (1 << BODSE);//to turn off brown out detections first
  MCUCR &= (1 << BODS) | ~(1 << BODSE);//second step
////  ADCSRA &= ~(1 << ADEN);
//  asm volatile ("sei");    // Enable global interrupts
//  asm volatile ("sleep");  // Enter sleep mode
  //PRR= 0xFF;
  interrupts ();  // one cycle
  sleep_cpu ();   // one cycle
}

void setup()
{
  stopSleep();
  Serial.begin(9600);  
    mySerial.begin(9600);
      mySerial.print("Jackson and Callie's Wireless Sensor Node Temp Readings");

  pinMode(wakeUpPin, INPUT_PULLUP);
  
  pinMode(12, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  digitalWrite(12, HIGH);
  PWM16Begin();
  // On the Arduino UNO T1A is Pin 9
  PWM16(0);  // Set initial PWM value for Pin 9
  PWM16Enable();  // Turn PWM on for Pin 9

}

void adjustDuty(int value, int drop) {
  if ( (value - drop) < 60) {
    maxA = maxA + 1;
  } else {
    maxA = maxA - 1;
  }
}

void adjustDutyTrickle(int value, int drop) {
  if ((value - drop) < 60) { // may need to change 100 - haven't tested trickle yet
    maxA += 1;
  }
  else {
    maxA -= 1;
  }
}

int getTemp() {
  
  initialReading = analogRead(ThermistorPin);
    R2 = R1 * (1023.0 / (float)initialReading - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2*logR2));
    T = T - 273.15;
    T = (T * 9.0)/ 5.0 + 32.0; 
    return T;
}
void sendTemp(int T) {

    mySerial.print("Temperature: "); 
    mySerial.print(T);
    mySerial.println(" F");
}

void loop()
{
  int analogValue = analogRead(A0); // reading current being outputted by boost converter
  int batteryCharge = analogRead(battery); // reading battery charge


  //  if(millis()%200==0 && maxReached == 1){
  //    adjustDutyTrickle(analogValue, batteryCharge);
  //  }
  //
  //  if(millis()%30000 == 0){ // negative voltage detection
  //    compare2 = compare1; // privious charge
  //    compare1 = batteryCharge; // current charge
  //    compare3 = compare1 - compare2;
  //  }
  //
  //  if(compare3 < 0){ // 480 should be the reading at around 10V
  //    maxReached = 1; // the max voltage has been met
  //  }
  //
  //  if(batteryCharge < 400){ // reading at 8.5V
  //    maxReached = 0; // the battery should resume normal charging
  //  }
  //
  if (millis() % 200 == 0 && analogRead(inputVoltage) < 100 ) { // less then 1.5V
    //    digitalWrite(led, HIGH);
    //    delay(1000);
    //    digitalWrite(led, LOW);
    //    delay(500);
    PWM16Disable();
    
  pinMode(led, OUTPUT);
  digitalWrite(12, LOW);
  pinMode(12, INPUT);
    digitalWrite(led, HIGH);
    delay(2000);
    digitalWrite(led, LOW);
    
  pinMode(led, INPUT);
    delay(50);
//    setupSleep();
//    Serial.flush();
    goSleep();// now go to Sleep and waits/*
    //stopSleep();
    int test = analogRead(inputVoltage);
    if (analogRead(inputVoltage) > 100) {
      
  pinMode(led, OUTPUT);
      digitalWrite(led, HIGH);
      delay(500);
      digitalWrite(led, LOW);
      delay(500);
      digitalWrite(led, HIGH);
      delay(500);
      digitalWrite(led, LOW);
      delay(500);
      PWM16Enable();
      
  pinMode(12, OUTPUT);
      digitalWrite(12, HIGH);
    }
  } else if (millis() % 200 == 0 && maxReached == 0) {
    adjustDuty(analogValue, batteryCharge);
  }

  if (millis() %1000 == 0) {
    
    sendTemp(getTemp());
    
  }


  if (maxA > 910) {
    maxA = 910;
  }
  if (maxA < 200) {
    maxA = 200;
  }
  unsigned int PWMValue = map(maxA, 0, 1023, 0, TOP);
  PWM16(PWMValue);  // Update the PWM at the end of the current cycle
}
