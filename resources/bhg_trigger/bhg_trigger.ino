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

void loop() {  // 201 hz max speed
  if (count % 2 == 0){
    digitalWrite(3, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else{
    digitalWrite(3, LOW);   // turn the LED on (HIGH is the voltage level)    
  }
  count = count + 1;
  delay(pulse_width);
}

//void loop() {  // 277 hz
//    digitalWrite(3, HIGH);   // turn the LED on (HIGH is the voltage level)
//    digitalWrite(3, LOW);   // turn the LED on (HIGH is the voltage level)    
//}

//void loop() {  // 207 hz but hi variance
//  if (value == HIGH){
//    value = LOW;
//  }
//  else{
//    value = HIGH;
//  }
//  digitalWrite(3, value);   // turn the LED on (HIGH is the voltage level)
//}

//void loop() {  // 201 hz more stable then above solution
//  if (count % 2 == 0){
//    digitalWrite(3, HIGH);   // turn the LED on (HIGH is the voltage level)
//  }
//  else{
//    digitalWrite(3, LOW);   // turn the LED on (HIGH is the voltage level)    
//  }
//  count = count + 1;
//}

//void loop() {  // 157 hz
//  if (digitalRead(3)) {
//    digitalWrite(3, LOW);   // turn the LED on (HIGH is the voltage level)   
//  } else {
//    digitalWrite(3, HIGH);   // turn the LED on (HIGH is the voltage level)
//  }     
//}
