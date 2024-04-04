void setup() {
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
    delay(65000);
}

void loop() {
  if(millis()%3000 == 0){
    digitalWrite(5, LOW);
  }
  int value = analogRead(A3);
  Serial.println(value);
}
