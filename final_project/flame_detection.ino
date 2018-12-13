// Code kept as reference in this .ino file, but already integrated into the main final_project.ino

// #include <Sparki.h>
// #include <Radio.h> 

// #define flame_sensor 8   // Used:  Radio Pin  NRF_IRQ 8
// int flame_detected; 


// void setup()
// {
//   Serial.begin(9600);
//  // pinMode(buzzer, OUTPUT);
//   //pinMode(LED, OUTPUT);
//   pinMode(flame_sensor, INPUT);
// }

// void loop()
// {
//   flame_detected = digitalRead(flame_sensor);
//   if (flame_detected == 0)
//   {
//     Serial.println("Flame detected...! take action immediately.");
//   //  digitalWrite(LED, HIGH);
//   //  delay(200);
//  //   digitalWrite(LED, LOW);
//   //  delay(200);
//     sparki.beep(); 
//     /*We can add a sginal ->  explore & save */
//   //  delay(1000);
//   }
//   else
//   {
//     Serial.println("No flame detected. stay cool");
//    // digitalWrite(LED, LOW);
//   }
//   delay(1000);
