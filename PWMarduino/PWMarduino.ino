#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX |
//Constants for inputs
int battery = A2;
int boost = A1;
int inputVoltage = A0;
const int wakeUpPin = 2;
const int clockTrig = 4;
const int led = 8;//delete eventually

//constants and variables for thermistor
int ThermistorPin = 3;
int initialReading;
float R1 = 100000;
float logR2, R2, T;
float c1 = 6.88216e-04, c2 = 2.894356e-04;
const int ten_volt = 480;
const int ten_four_volt = 500;
const int eight_five_volt = 400;
const int one_volt = 100;
const int constantCurrent = 100;
// Set 'TOP' for PWM resolution.  Assumes 16 MHz clock.
const unsigned int TOP = 0x013F; //

//variables for logic and PWM adjustments and comming out of sleep mode
float maxA = 000;
const int maxPWM = 910;
float compare1 = 0; // current reading
float compare2 = 0; // previous reading
float compare3 = 0; // compare1 - compare2
byte keep_ADCSRA;
int maxReached = 0;

//interupt handler for when digitla pin two detects a log change from high to low
ISR(INT0_vect) {
  wakeUp();
}
void wakeUp()
{
  stopSleep();
  // Clear interrupt flag for INT0
  EIFR |= (1 << INTF0); // Set INTF0 bit in EIFR to clear interrupt flag
  //detachInterrupt (0);
  ADCSRA = keep_ADCSRA;
}

void setup()
{
  stopSleep();//precaution
  Serial.begin(9600);//DELETE
  mySerial.begin(9600);//start bluetooth
  mySerial.print("Jackson and Callie's Wireless Sensor Node Temp Readings");
  //Initialize sleep pins used
  pinMode(wakeUpPin, INPUT_PULLUP);
  pinMode(led, OUTPUT);//delete eventually
  pinMode(clockTrig, OUTPUT);
  digitalWrite(clockTrig, HIGH);
  PWM16Begin();
  // On the Arduino UNO T1A is Pin 9
  PWM16(0);  // Set initial PWM value for Pin 9
  PWM16Enable();  // Turn PWM on for Pin 9
}
void loop()
{
  int analogValue = analogRead(boost); // reading current being outputted by boost converter
  int batteryCharge = analogRead(battery); // reading battery charge
  int supply = analogRead(inputVoltage); // reading battery charge
//    if(millis()%30000 == 0){ // negative voltage detection
//      compare2 = compare1; // privious charge
//      compare1 = batteryCharge; // current charge
//      compare3 = compare1 - compare2;
//    }
//    if(compare3 < 0 && batteryCharge > ten_four_volt){ // 480 should be the reading at around 10V
//      maxReached = 1; // the max voltage has been met
//    }
    if(batteryCharge > ten_four_volt){ // 480 should be the reading at around 10V
        maxReached = 1; // the max voltage has been met
    }
    if(batteryCharge < eight_five_volt ){ // reading at 8.5V
      maxReached = 0; // the battery should resume normal charging
    }
  //check if the device should go into sleep mode becuase there is no input power formt the "sun"
  if (millis() % 200 == 0 && supply < one_volt ) { // less then 1V
    PWM16Disable();
    setupSleep();
    goSleep();// now go to Sleep and waits/*
    //stopSleep();
    int test = analogRead(inputVoltage);
    ///check if it shoudl wake up and turn the PWM wave back on
    if (analogRead(inputVoltage) > one_volt) {
      maxA = 0;
      PWM16Enable();
    }
  } else if (millis() % 200 == 0 && maxReached == 0) {
    adjustDuty(analogValue, batteryCharge);
  }
  if (millis() % 1000 == 0) {
    sendTemp(getTemp());
  }
//  if (maxReached == 1 && maxA >=5) {
//    maxA-=5;
//  }
  //constrain PWM just incase, to avoid always on
  if (maxA > maxPWM) {
    maxA = maxPWM;
  }
  unsigned int PWMValue = map(maxA, 0, 1023, 0, TOP);
  PWM16(PWMValue);  // Update the PWM at the end of the current cycle
}
//////////PWM FUNCTIONS/////////////////////
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
void adjustDuty(int value, int drop) {
  if ( (value - drop) < constantCurrent) {
    maxA = maxA + 2;
  } else {
    maxA = maxA - 2;
  }
}
//////////SLEEP FUNCTIONS/////////////////////
void setupSleep() {
  // Setup external interrupt pin as input
  DDRD &= ~(1 << wakeUpPin);
  // Configure external interrupt 0 (INT0) to trigger on low level
  EICRA &= ~((1 << ISC01) | ~(1 << ISC00)); // Clear ISC01 and ISC00 bits
  // Enable external interrupt INT0
  EIMSK |= (1 << INT0);
  asm volatile ("sei");
}
void goSleep() {
  keep_ADCSRA = ADCSRA;
  ADCSRA = 0;
    asm volatile ("cli"); // Disable  interrupts
  digitalWrite(clockTrig, LOW);
  delay(500);
  digitalWrite(clockTrig, HIGH);
  delay(500);
    // Enter power-down mode
    SMCR |= (1 << SM1); // Set bit SM1 in SMCR (Sleep Mode Control Register) for power-down mode
    SMCR |= (1 << SE); // Enable sleep mode
  MCUCR &= (1 << BODS) | (1 << BODSE);//to turn off brown out detections first
  MCUCR &= (1 << BODS) | ~(1 << BODSE);//second step
  ////  ADCSRA &= ~(1 << ADEN);
    asm volatile ("sei");    // Enable global interrupts
    asm volatile ("sleep");  // Enter sleep mode
  //////PRR= 0xFF;///breaks code
}
void stopSleep() {
    SMCR &= (uint8_t)~(1 << SE); // Enable sleep mode
  //  //finished sleeping and been interupted
    asm volatile ("cli"); // Disable global interrupts
    //SMCR &= ~(1 << SM0) & ~(1 << SM1) & ~(1 << SM2);
  //  MCUCR |= ~(1 << BODS) | ~(1 << BODSE);//to turn on brown out detections first
  //  MCUCR = bit (BODS); //second step
// ////   PRR &= ~(1 << PRTWI) & ~(1 << PRTIM2) & ~(1 << PRTIM0) & ~(1 << PRSPI) & ~(1 << PRUSART0) & ~(1 << PRTIM1) & ~(1 << PRSPI) & ~(1 << PRADC);
//  ////PRR = 0; breaks code
}
//////////TEMPERATURE FUNCTIONS///////////////////////
int getTemp() {
  initialReading = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)initialReading - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2));
  T = T - 273.15;
  T = (T * 9.0) / 5.0 + 32.0;
  return T;
}
/////////////luetooth transmit, only used not in power down mode//////////////

void sendTemp(int T) {
  mySerial.print("Temperature: ");
  mySerial.print(T);
  mySerial.println(" F");
}
