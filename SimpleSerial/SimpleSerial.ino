#include <CapacitiveSensor.h>
#define SetAutoFilltoZero 50

//Defining pins 2 and 4 to be the capacitive sensor
CapacitiveSensor capsense4_2 = CapacitiveSensor(4,2);

void setup() {
  capsense4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);
  // Setting up serial to write to terminal log
  Serial.begin(9600);
}

void loop() {
  //Raw cap sense data
  long autofill;
  //Set time since program started
  long start_time = millis();
  
  autofill = capsense4_2.capacitiveSensor(30); //30 Raw sensor samples
  autofill -= SetAutoFilltoZero;
  if(autofill <=5) {
    Serial.print(millis() - start_time);
    Serial.print("\t");
    Serial.print("Autofill: ");
    Serial.print(autofill);
    Serial.println("\t AUTOFILL!");
  }
  else{
    Serial.print(millis() - start_time);
    Serial.print("\t");
    Serial.print("Autofill: ");
    Serial.print(autofill);
    Serial.println("\t .....!");
  }
}
