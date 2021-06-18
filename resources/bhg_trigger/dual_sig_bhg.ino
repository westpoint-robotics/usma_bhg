void setup()
{ 
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
} // Set digital pins 12 and 13 to outputs
int offset = 32000;
int halfP = 49700 - offset;

void loop()
{
  digitalWrite(4, !digitalRead(4));  
  delayMicroseconds(offset);  //30 microsecond delay  
  digitalWrite(3, !digitalRead(3));  
  delayMicroseconds(halfP);  //30 microsecond delay   
}
