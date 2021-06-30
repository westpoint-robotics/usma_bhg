#include <Adafruit_DotStar.h>
#define NUMPIXELS 1 
#define DATAPIN    7
#define CLOCKPIN   8
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

int rgb_on = 0;
int rgb_pin = 4; // red wire
int lwir_pin = 3; // orange wire
// You must use the matching offset for each frequency
int offset = 32000; // 10 hz & 5hz
//int halfP = 99400 - offset; // 5hz
//int offset = 20800; // 25 hz & 20 hz
int halfP = 49700 - offset; // 10 hz
//int offset = 24400; // 15 hz & 20 hz
//int halfP = 33000 - offset; // 15 hz
//int halfP = 24850 - offset; // 20 hz
//int offset = 19800; // 25 hz & 20 hz
//int halfP = 19880 - offset; // 25 hz
//int halfP = 16560 - offset; // 30 hz


void setup()
{ 
  pinMode(rgb_pin, OUTPUT);
  pinMode(lwir_pin, OUTPUT);
  Serial.begin(9600);
    strip.begin(); // Initialize pins for output
  strip.setBrightness(70);
  strip.show();  // Turn off LED
  strip.setPixelColor(0,255,0,255); // cyan color
  strip.show();  // Turn on LED
} 

void loop()
{
  rgb_on = digitalRead(rgb_pin);
  if (rgb_on){
    Serial.write(45); // send a byte with the value 45 before start of high signal
  }
  digitalWrite(rgb_pin, !rgb_on);  
  delayMicroseconds(offset);  // Delay between singles
  digitalWrite(lwir_pin, !digitalRead(lwir_pin));  
  delayMicroseconds(halfP);  // Delay to achieve desired Hz  
}
