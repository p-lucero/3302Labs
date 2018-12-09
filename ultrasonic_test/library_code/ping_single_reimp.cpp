/* sparki should be able to see 1m in front of him, but doesn't care about anything further
change header to be float ping_single
change ping_single implementation to the below

TODO run experiment with the below code and measure consistency
TODO make some play-doh candle holders?
INVESTIGATE: what if we change it so that it returns when the pulse starts??? */

float SparkiClass::ping_single(){
  long duration; 
  float cm;
  digitalWrite(ULTRASONIC_TRIG, LOW); 
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(ULTRASONIC_TRIG, LOW); 

  uint8_t bit = digitalPinToBitMask(ULTRASONIC_ECHO);
  uint8_t port = digitalPinToPort(ULTRASONIC_ECHO);
  uint8_t stateMask = (HIGH ? bit : 0);
  
  unsigned long startCount = 0;
  unsigned long endCount = 0;
  unsigned long width = 0; // keep initialization out of time critical area
  
  // convert the timeout from microseconds to a number of times through
  // the initial loop; it takes 16 clock cycles per iteration.
  unsigned long numloops = 0;
  unsigned long maxloops = 5000;
	
  // wait for any previous pulse to end
  while ((*portInputRegister(port) & bit) == stateMask)
    if (numloops++ == maxloops)
      return -1;
	
  // wait for the pulse to start
  while ((*portInputRegister(port) & bit) != stateMask)
    if (numloops++ == maxloops)
      return -1;
  
  startCount = micros();
  // wait for the pulse to stop
  while ((*portInputRegister(port) & bit) == stateMask) {
    if (numloops++ == maxloops)
      return -1;
    delayMicroseconds(10); //loop 'jams' without this
    if((micros() - startCount) > 5800 ){ // 5800 = 100CM
      return -1;
      break;
    }
  }
  duration = micros() - startCount;
  //--------- end pulsein
  cm = (float)duration / 58.0;
  return cm;
}