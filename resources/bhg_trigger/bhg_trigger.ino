/*
 * bhg_trigger04
 * 
 * Sends a square wave trigger to the cameras in order to synchronize the taking of pictures
 * 
 * Based on blink example code.
 */

unsigned long count = 0;
float hz = 59.50;
float pulse_width = 1.0/hz/2.0*1000.0; // Pulse width in milli seconds
int pin60hz = 3;
int pin10hz = 4;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(pin60hz, OUTPUT);
  pinMode(pin10hz, OUTPUT);
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps

}

void loop() {  // 201 hz max speed
  if (count % 2 == 0){
    digitalWrite(pin60hz, HIGH);   // turn the LED on (HIGH is the voltage level)
    if (count % 6 == 0){
      digitalWrite(pin10hz, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
  }
  else{
    digitalWrite(pin60hz, LOW);   // turn the LED on (HIGH is the voltage level)
    if (count % 6 == 0){
      digitalWrite(pin10hz, LOW);   // turn the LED on (HIGH is the voltage level)
    }    
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
