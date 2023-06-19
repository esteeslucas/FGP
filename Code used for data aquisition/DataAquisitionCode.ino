//Pins for the absolute encoder
const int CSn = 10;
const int CLK= 8;
const int DO = 9; 

//sampling Frequency modifying
const int samplingPeriod = 49; //Given in terms of steps of the measuring wheel encoder.

//Wheel diameter:
const int wheelDiameter = 35; //Unit: mm

//Cable end detection
const int cableEndPin = 5;
//Pins for the incremental encoder. Cannot change because they're interrupts.
const int encoderPin1 = 2;
const int encoderPin2 = 3;

//Variables used later for the incremental encoder
volatile int lastEncoded = 0;
volatile float encoderValue = 0;
volatile long int encoderUnmodified = 0;
float angle = 0;
int lastMSB = 0;
int lastLSB = 0;
float distance = 0;
long int lastEncoderUnmodified = 0; //"Long" as errors occured when lsb turned into 1 the number was interpreted as negative and started going down

//Variables used later for the absolute encoder
int swivel = 0;
float swivelAngle = 0.0;
//Variables used for filtering of absolute encoder data
const int N = 100;
float movingAverageAngle[N] = {};
float movingAverageAngleValue = 0;
float usedAngle = 0;

//Distance between presence-absence sensors:
float distanceBetSensors = 26.94;

//For SDCard reading
const int chipSelect = 4;
/*
SD card reading from:
 SD card read/write
  
 This example shows how to read and write data to and from an SD card file 	
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 
 created   Nov 2010
 by David A. Mellis
 updated 2 Dec 2010
 by Tom Igoe
 modified by Bill Greiman 11 Apr 2011
 This example code is in the public domain.
 	 
 */
#include <SdFat.h>
SdFat sd;
SdFile myFile;
void setup() {
  //Serial UART communication
  Serial.begin(115200);
  while (!Serial) {} //Wait for serial communication to begin
  // Setting up the absolute encoder's pins
  pinMode(CSn, OUTPUT);
	pinMode(CLK, OUTPUT);
	pinMode(DO, INPUT);

  //Setting up the the cable end detector
  pinMode(cableEndPin,INPUT_PULLUP);
  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  //This is so the absolute encoder doesn't lose steps.
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  //Start the reading sequence for the absolute encoder
	digitalWrite(CLK, HIGH);
	digitalWrite(CSn, HIGH);
  delay(400);  // catch Due reset problem
  //Initial configuration of the SD module and specifying this is a new experiment
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
  if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening test.txt for write failed");
  }
  //Line that specifies it's a new experiment
  myFile.println("Time; Swivel angle (E-03); Filtered swivel angle (E-03);Distance");
  // close the file:
  myFile.close();
}

void loop() {  
  //Wait for cable detection to count new distance
  if (digitalRead(cableEndPin) == HIGH) encoderValue = 0;
  if (abs(encoderUnmodified-lastEncoderUnmodified)>samplingPeriod){
    lastEncoderUnmodified = encoderUnmodified;
    //Incremental encoder:
    angle = encoderValue*3.141592/(1000); 
    //Read value from the absolute encoder:
    swivel = readSensor()-2669;
    //Update distance
    distance = angle*wheelDiameter/2 + distanceBetSensors; //Takes into account that the distance is not updated until it reaches the second presence/absence switch
    //According to calibration curve:
    distance = distance*0.8517+6.9124+4;
    //swivel angle using relation from the gearbox. Used here to avoid problems by changing speed
    swivelAngle = -swivel*2*3.141592/(4096*5) - 0.030837201; //Additive value is result of calibration
    //Updating the movingAverageAngle array
    movingAverageAngleValue = 0;
    for (int i = 0; i<N-1; i++){ //Moving all the data to the left and obtaining the average of the moved values
        movingAverageAngle[i] = movingAverageAngle[i+1];
        movingAverageAngleValue = movingAverageAngleValue + movingAverageAngle[i];
    }
    if (swivelAngle >0) movingAverageAngle[N-1] = swivelAngle; //Adding the new swivelAngle only if its positive to avoid excesive initial bias.
    movingAverageAngleValue = (movingAverageAngleValue + swivelAngle)/N; //Obtaining average using the newly obtained value
    usedAngle = movingAverageAngleValue; //Used to make sure that the interrupt doesn't occur while calculating the moving average.
    
    if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
      sd.errorHalt("opening test.txt for write failed");
    }
    // Write in SD card:
    Serial.print(millis());Serial.print(";");Serial.print(swivelAngle*1000);Serial.print(";");Serial.print(usedAngle*1000);Serial.print(";");Serial.println(distance);
    myFile.print(millis());myFile.print(";");myFile.print(swivelAngle*1000);myFile.print(";");myFile.print(usedAngle*1000);myFile.print(";");myFile.println(distance);
    // close the file:
    myFile.close();
    
  }
}

//Absolute encoder reading.
unsigned int readSensor(){ //Basado en: https://forum.arduino.cc/t/read-in-10-bits-magnetic-encoder-aeat-6010-a06-into-arduino-uno/160575
  unsigned int readValue = 0;
  digitalWrite(CSn, LOW);
  delayMicroseconds(1);//Wait for 2 TCLFE to pass

  //Reading each bit:
  for(int x=0;x<12;x++){
    digitalWrite(CLK,LOW);
    delayMicroseconds(1); //Tclk/2
    digitalWrite(CLK,HIGH);
    delayMicroseconds(1); //Tclk/2
    readValue = (readValue<<1) | digitalRead(DO); //Shift and add
  }
    digitalWrite(CSn, HIGH);
  return readValue;
}   
//Incremental encoder reading (Working as an interrupt routine), based on: https://gist.github.com/pdp7/0d7d17522dc892569eee091a8df23266 
void updateEncoder(){
  
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011){
   encoderValue = encoderValue + sin(usedAngle);
   encoderUnmodified = encoderUnmodified + 1;
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000){
    encoderValue = encoderValue - sin(usedAngle);
    encoderUnmodified = encoderUnmodified - 1;
  }
  
  lastEncoded = encoded; //store this value for next time
  
}
