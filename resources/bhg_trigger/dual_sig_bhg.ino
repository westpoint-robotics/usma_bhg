
int rgb_pin = 4;
int lwir_pin = 3;
// You must use the matching offset for each frequency
//int offset = 32000; // 10 hz & 5hz
//int halfP = 99400 - offset; // 5hz
//int halfP = 49700 - offset; // 10 hz
//int offset = 24400; // 15 hz & 20 hz
//int halfP = 33000 - offset; // 15 hz
//int halfP = 24850 - offset; // 20 hz
int offset = 0; // 25 hz & 20 hz
int halfP = 19880 - offset; // 25 hz
//int halfP = 16560 - offset; // 30 hz


void setup()
{ 
  pinMode(rgb_pin, OUTPUT);
  pinMode(lwir_pin, OUTPUT);
  Serial.begin(9600);
} // Set digital pins 12 and 13 to outputs

void loop()
{
  Serial.write(45); // send a byte with the value 45
  digitalWrite(rgb_pin, !digitalRead(rgb_pin));  
  delayMicroseconds(offset);  //30 microsecond delay  
  digitalWrite(lwir_pin, !digitalRead(lwir_pin));  
  delayMicroseconds(halfP);  //30 microsecond delay   
}
