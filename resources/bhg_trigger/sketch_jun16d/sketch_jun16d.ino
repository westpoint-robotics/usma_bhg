void setup()
{ 
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
} // Set digital pins 12 and 13 to outputs
int T = 8279;

void loop()
{
  digitalWrite(3, !digitalRead(3));
  digitalWrite(4, !digitalRead(4));

  delayMicroseconds(T);  //30 microsecond delay
  for (int i = 0; i <= 4; i++) {
    digitalWrite(3, !digitalRead(3));
    delayMicroseconds(T);  //30 microsecond delay
  }
   
}
