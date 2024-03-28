/*
  Demonstration of 16-bit PWM on Timer1
  Written by John Wasser
  
  PWM16Begin(): Set up Timer1 for PWM.
  PWM16EnableA(): Start the PWM output on Pin 9
  PWM16A(unsigned int value): Set the PWM value for Pin 9.
*/


// Set 'TOP' for PWM resolution.  Assumes 16 MHz clock.
 const unsigned int TOP = 0x013F; // 
 //const unsigned int TOP = 0x028F; // 
float maxA = 600;

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

inline void PWM16(unsigned int PWMValue)
{
  OCR1A = constrain(PWMValue, 0, TOP);
}

void setup()
{
  Serial.begin(9600);
  PWM16Begin();
  // On the Arduino UNO T1A is Pin 9
  PWM16(0);  // Set initial PWM value for Pin 9
  PWM16Enable();  // Turn PWM on for Pin 9
}

void adjustDuty(int value) {
if ( value < 490) {
    maxA = maxA + 1;
  } else {
        maxA = maxA - 1;
  }
}

void loop()
{
  int analogValue = analogRead(A0);
  if (millis()%200==0) {
    adjustDuty(analogValue);
  }
  
  
  if (maxA > 810) {
    maxA = 810;
  }
  if (maxA < 200) {
    maxA = 200;
  }
  unsigned int PWMValue = map(maxA, 0, 1023, 0, TOP);
  PWM16(PWMValue);  // Update the PWM at the end of the current cycle
}
