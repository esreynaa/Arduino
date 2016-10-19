#include <CapacitiveSensor.h>
#define SetAutoFilltoZero 35

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
  
  autofill = capsense4_2.capacitiveSensor(50); //30 Raw sensor samples
  autofill -= SetAutoFilltoZero;
  autofill /= 90;
  if(autofill <=0) {
    Serial.print(millis() - start_time);
    Serial.print("\t");
    Serial.print("Data: ");
    Serial.print(autofill);
    Serial.println("\t AUTOFILL!");
  }
  
  
  else{
    Serial.print(millis() - start_time);
    Serial.print("\t");
    Serial.print("Data: ");
    Serial.print(autofill);
    Serial.println("\t .....!");
  }
}
