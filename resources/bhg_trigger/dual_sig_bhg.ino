
int rgb_pin = 4;
int lwir_pin = 3;

void setup()
{ 
  pinMode(rgb_pin, OUTPUT);
  pinMode(lwir_pin, OUTPUT);
} // Set digital pins 12 and 13 to outputs
int offset = 32000;
int halfP = 49700 - offset;

void loop()
{
  digitalWrite(rgb_pin, !digitalRead(rgb_pin));  
  delayMicroseconds(offset);  //30 microsecond delay  
  digitalWrite(lwir_pin, !digitalRead(lwir_pin));  
  delayMicroseconds(halfP);  //30 microsecond delay   
}
