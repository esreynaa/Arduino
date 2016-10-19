
/*VL6180X(void)  
    Constructor. 
  #define I2C_defaddress 0x29

  void setAddress(uint8_t new_addr)
    Changes the I²C slave device address of the VL6180X to the given value (7-bit).
  void init(void)
    Loads required settings onto the VL6180X to initialize the sensor
  void configureDefault(void)
     Configures some settings for the sensor's default behavior. See the comments in VL6180X.cpp for a full explanation of the settings.

  void writeReg(uint16_t reg, uint8_t value)
     Writes an 8-bit sensor register with the given value.
     Register address constants are defined by the regAddr enumeration type in VL6180X.h.
     Example use: sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  uint8_t readReg(uint16_t reg)
     Reads an 8-bit sensor register and returns the value read.
  
  void startRangeContinuous(uint16_t period)
     Starts continuous ranging measurements with the given period in milliseconds (10 ms resolution; defaults to 100 ms if not specified).
     In all continuous modes, the period must be greater than the time it takes to perform the measurement(s). See section 2.4.4 ("Continuous mode limits") in the datasheet for details.
  void stopContinuous(void)
     Stops continuous mode.
  uint16_t readRangeContinuousMillimeters(void)
     Returns a range reading in millimeters, taking the range scaling setting into account, when continuous mode is active.
*/
#include <Wire.h>
#include <VL6180X.h>
//#include "TimerOne.h"
#define I2C_defaddress 0x29

/*******BELOW IS DEFINING THE DIGITS AND SEGMENTS FOR 7 SEG DISPLAY******/
#define  DIGIT1  A0  // Rightmost digit (Active LOW)
#define  DIGIT2  A1
#define  DIGIT3  A2
#define  DIGIT4  A3  // Leftmost digit
#define  SEG_A   10  // Active LOW
#define  SEG_B   9
#define  SEG_C   8
#define  SEG_D   7
#define  SEG_E   6
#define  SEG_F   5
#define  SEG_G   4
#define  SEG_DP  3
unsigned char outPins[] = {DIGIT1, DIGIT2, DIGIT3, DIGIT4, SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G, SEG_DP};
unsigned char sDecode[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
unsigned char digits[4];
int digit = 0, dp = 0;
/************************************************************************/
VL6180X sensor; 

void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  
  for (int ii = 0; ii < sizeof(outPins); ii++) {
     pinMode(outPins[ii], OUTPUT);
     digitalWrite(outPins[ii], HIGH);
  }
  
  //Initialize VL6180X
  pinMode(13,OUTPUT);    //Active HI
  digitalWrite(13, HIGH);
  
//  // Attach timer interrupt for viewing 7 seg display
//  Timer1.initialize(5000);
//  Timer1.attachInterrupt(callback);

  // Mandatory : private registers
  WriteByte(0x0207, 0x01);
  WriteByte(0x0208, 0x01);
  WriteByte(0x0096, 0x00);
  WriteByte(0x0097, 0xfd);
  WriteByte(0x00e3, 0x00);
  WriteByte(0x00e4, 0x04);
  WriteByte(0x00e5, 0x02);
  WriteByte(0x00e6, 0x01);
  WriteByte(0x00e7, 0x03);
  WriteByte(0x00f5, 0x02);
  WriteByte(0x00d9, 0x05);
  WriteByte(0x00db, 0xce);
  WriteByte(0x00dc, 0x03);
  WriteByte(0x00dd, 0xf8);
  WriteByte(0x009f, 0x00);
  WriteByte(0x00a3, 0x3c);
  WriteByte(0x00b7, 0x00);
  WriteByte(0x00bb, 0x3c);
  WriteByte(0x00b2, 0x09);
  WriteByte(0x00ca, 0x09);
  WriteByte(0x0198, 0x01);
  WriteByte(0x01b0, 0x17);
  WriteByte(0x01ad, 0x00);
  WriteByte(0x00ff, 0x05);
  WriteByte(0x0100, 0x05);
  WriteByte(0x0199, 0x05);
  WriteByte(0x01a6, 0x1b);
  WriteByte(0x01ac, 0x3e);
  WriteByte(0x01a7, 0x1f);
  WriteByte(0x0030, 0x00);
  
  // Recommended : Public registers - See data sheet for more detail
  WriteByte(0x0011, 0x10); // Enables polling for ‘New Sample ready’ when measurement completes
  WriteByte(0x010a, 0x30); // Set the averaging sample period (compromise between lower noise and increased execution time)
  WriteByte(0x003f, 0x46); // Sets the light and dark gain (upper nibble). Dark gain should not be changed.
  WriteByte(0x0031, 0xFF); // sets the # of range measurements after which auto calibration of system is performed
  WriteByte(0x0040, 0x63); // Set ALS integration time to 100ms
  WriteByte(0x002e, 0x01); // perform a single temperature calibration of the ranging sensor
  
  /*OPTIONAL*/
  WriteByte(0x001b, 0x09); // Set default ranging inter-measurement period to 100ms
  WriteByte(0x003e, 0x31); // Set default ALS inter-measurement period to 500ms
  WriteByte(0x0014, 0x24); // Configures interrupt on ‘New Sample Ready threshold event’ 

}

void loop() {
  //Starting continuous range measurments
  WriteByte(VL6180X::SYSRANGE__START,0x03);
  //Now wait range measurement (poll) to complete
  while(ReadByte(VL6180X::RESULT__INTERRUPT_STATUS_GPIO) &0x07 != 4)
    delay(10);
  //Read the range in mm
  int range = ReadByte(VL6180X::RESULT__RANGE_VAL);
  //Print out to serial
  Serial.println(range);
  //Clear interrupts
  WriteByte(VL6180X::SYSTEM__INTERRUPT_CLEAR,0x07);
  //TODO: write number to display
  writeNumber(range);
  delay(100);

}

/**************Functions for outputting to display**************/
//Sets each 7 segment display
void setSegments (unsigned char digit, unsigned char num) {
  unsigned char pattern = sDecode[num];
  for (int ii = 0; ii < 4; ii++) {
    digitalWrite(outPins[ii], digit == ii ? LOW : HIGH);
  }
  for (int jj = 4; jj < sizeof(outPins) - 1; jj++) {
    unsigned char mask = 1 << (jj - 4);
    digitalWrite(outPins[jj], (pattern & mask) != 0 ? LOW : HIGH);
  }
  digitalWrite(SEG_DP, digit == dp ? LOW : HIGH);
}
// Scans 7 Segment display
void callback () {
  setSegments(digit, digits[digit]);
  digit = (digit + 1) % 4;
}
//Writes to 7 Segment display
void writeNumber (int num) {
  for (int ii = 0; ii < sizeof(digits); ii++) {
    digits[ii] = num % 10;
    num /= 10;
  }
}
/***************************************************************/

/**************Internal functions for wire transmission**************/
uint8_t ReadByte (uint16_t subAddress) {
  Wire.beginTransmission(I2C_defaddress);
  Wire.write((subAddress >> 8) & 0xFF);
  Wire.write(subAddress & 0xFF);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_defaddress, 1);
  return Wire.read();
}
void WriteByte (uint16_t subAddress, uint8_t data) {
  Wire.beginTransmission(I2C_defaddress);
  Wire.write((subAddress >> 8) & 0xFF);
  Wire.write(subAddress & 0xFF);
  Wire.write(data);
  Wire.endTransmission();
}
/********************************************************************/
