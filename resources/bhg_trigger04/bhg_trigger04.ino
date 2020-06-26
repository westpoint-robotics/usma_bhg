/*
 * bhg_trigger04
 * 
 * Sends a square wave trigger to the cameras in order to synchronize the taking of pictures
 * 
 * Based on blink example code.
 */

unsigned int count = 0;
int hz = 20;
int pulse_width = 1.0/hz/2*1000; // Pulse width in milli seconds

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(3, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  if (count % 2 == 0){
    digitalWrite(3, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else{
    digitalWrite(3, LOW);   // turn the LED on (HIGH is the voltage level)    
  }
  count = count + 1;
  if (count > 65000){ // Reset to avoid overflow of the uint.
    count = 0;
  }
  delay(pulse_width);                       // wait for a milliseconds
}
