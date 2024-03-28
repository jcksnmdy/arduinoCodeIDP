//Bluetooth  uses serial communication. So, we use many serial functions 
#include <SoftwareSerial.h>
const int LED = 5;
SoftwareSerial mySerial(10, 11); // RX, TX
char switchstate;
void setup()  {//Here the code only runs once.
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(LED,  OUTPUT);//Declaring that the LED is an output.
//  sendCommand("AT");
//  sendCommand("AT+IMME?");
//  sendCommand("AT+IMME0");
//  sendCommand("AT+NOTI1");
//  sendCommand("AT+NOTP1");
//  sendCommand("AT+NAMEmySanity");
//  sendCommand("AT+ROLE1");
  digitalWrite(LED, LOW);
//  delay(1000);
//  digitalWrite(LED, HIGH);
//  delay(1000);
//  digitalWrite(LED, LOW);
}
void loop() {
//  while(mySerial.available()>0){ 
//
//    switchstate = mySerial.read();
//
//    Serial.print(switchstate);
//
//    Serial.println("\n");
//    delay(15);
//
//
//    if(switchstate  == '1'){//Checking if the value from app is '1'
//     digitalWrite(LED, HIGH);
//
//    }
//    else if(switchstate == '0'){//Else,  if the vaue from app is '0',
//     digitalWrite(LED, LOW);//Write the component on pin  5(LED) low.
//}}

  //mySerial.print("CONNECTED"); 
}
void sendCommand(const char * command){
  Serial.print("Command send: ");
  Serial.print(command);
  mySerial.print(command);  
  Serial.println(" :Command end");
  delay(1000);
  char reply[100];
  int i = 0;
  while (mySerial.available()) {
    reply[i] = mySerial.read();
    i += 1;
  }
  //end the string
  reply[i] = '\0';
  Serial.print("Reply start: ");
  Serial.print(reply);
  Serial.println(" :Reply end");
}
