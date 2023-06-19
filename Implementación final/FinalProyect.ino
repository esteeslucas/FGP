#include "SevSeg.h"
#include "math.h"
SevSeg sevseg; //Instantiate a seven segment object

//Buzzer
const int Buzzer = 1;
const int buzzerSeconds = 5; //Modifiable: sets ON Time of initial alarm

//Saves past millis to use a sampling period of 30 ms
const int samplingPeriod = 30; //in encoder steps

//Setting distance of cut
const int DoC = A6; //Analogue pin for potentiometer
float actualDoC = 0; //Saves the current value of DoC
const int Enter = 4; //Enter or start button pin
float distanceOfCut = 0; //Saves normalized actualDoC to accepted range
bool distanceSelected = LOW; //Bool that enables to change of state once distance has been selected

//Pins for the absolute encoder
const int CSn = 11;
const int CLK= 13;
const int DO = 12; 
//Variables used later for the absolute encoder
int sensorValue = 0;
float swivelAngle = 0.0;

//Pins for known distance detection:
const int zeroPin = 0;
//Variables used for known distance detection:
bool inCable = LOW;

//Wheel diameter:
int wheelDiameter = 35; //Unit: mm

//Pins for the incremental encoder. Cannot change because they're interrupts.
const int encoderPin1 = 2;
const int encoderPin2 = 3;

//Variables used later for the incremental encoder
volatile int lastEncoded = 0;
volatile float encoderValue = 0;
float angle = 0;
int lastMSB = 0;
int lastLSB = 0;
volatile float distance = 0;

//Special signal processing variables:
//Used to measure the amount of steps of the measuring wheel between samples of the swivel angle:
long int lastEncoderUnmodified = 0; //"Long" as errors occured when lsb turned into 1 the number was interpreted as negative and started going down
volatile long int encoderUnmodified = 0;

//Variables used for filtering of absolute encoder data
//Moving average filter:
const int N = 100;
const int typicalAngle = 0; //Initial value for mean average filter
float movingAverageAngle[N] = {};
float movingAverageAngleValue = 0;
float usedAngle = 0;
const int minimumSteps = 22; //Modify according to the sampling speed.
// Distance between presence-absence sensors:
float distanceBetSensors = 26.26;

// Final position reached:
const int StopPosPin = 10; //Pin for signal 'stop'
//Setup:
void setup() {
  //Reset mean average filter values (otherwise errors were observed)
  for (int i = 0; i <N; i++){
    movingAverageAngle[i] = typicalAngle;
  }

  //Known position detection setup:
  pinMode(zeroPin, INPUT_PULLUP);
  inCable = digitalRead(zeroPin);

  //Buzzer setup:
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, LOW); 

  // Setting up the absolute encoder's pins
  pinMode(CSn, OUTPUT);
	pinMode(CLK, OUTPUT);
	pinMode(DO, INPUT);

  //Start the reading sequence for the absolute encoder
	digitalWrite(CLK, HIGH);
	digitalWrite(CSn, HIGH);
  //Delay to avoid problems in reset
  delay(400);
////////////////////////////////////////
  //Incremental encoder
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  //This is so no steps are lost
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  //Setup for the 7-segment, based on SevSeg's library documentation
  byte numDigits = 4;
  byte digitPins[] = {9,5,8,7};
  byte segmentPins[] = {A2,A1,A4,A5,6,A3,A0}; 
  bool resistorsOnSegments = true; // 'false' means resistors are on digit pins
  byte hardwareConfig = COMMON_ANODE; // See README.md for options
  bool updateWithDelays = false; // Default 'false' is Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  bool disableDecPoint = true; // Use 'true' if your decimal point doesn't exist or isn't connected. Then, you only need to specify 7 segmentPins[]
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,
  updateWithDelays, leadingZeros, disableDecPoint);

  //Setup for HMI in distance of cut selection
  pinMode(Enter, INPUT_PULLUP); 

  //Final position indicators
  pinMode(StopPosPin, OUTPUT);
  digitalWrite(StopPosPin, LOW);//Stop the system (~stop)
  digitalWrite(Buzzer, LOW);  //Set the buzzer to silent
  
  //One-time setup for cutting distance selection
  while(distanceSelected == LOW){
    actualDoC = analogRead(DoC); //Read distance of cut from potentiometer
    distanceOfCut = round(actualDoC/1023 *90 + 150); //Normalize values from potentiometer
    digitalWrite(Buzzer, LOW);   //
    sevseg.setNumberF(distanceOfCut,1);
    sevseg.refreshDisplay();
    if (digitalRead(Enter) == LOW){
      distanceSelected = HIGH;
      for (int n = 5*buzzerSeconds; n = n-1; n==0){
        digitalWrite(Buzzer, HIGH);
        delay(100);
        digitalWrite(Buzzer, LOW);
        delay(100);
      }
      digitalWrite(Buzzer, LOW);   
      digitalWrite(StopPosPin, HIGH); //Start moving of the robot
    }    
  }

  
}

void loop() {
  //If the final position is reached
  //Set ~stop output (represented with a led) as low
  if (distance >= distanceOfCut){
    digitalWrite(StopPosPin, LOW);
  }

  //If the known position hasn't been reached, set the encoderValue as 0
  inCable = digitalRead(zeroPin);
  if (inCable == HIGH){
      encoderValue = 0;
    }
  if (abs(encoderUnmodified-lastEncoderUnmodified)>samplingPeriod){
    lastEncoderUnmodified = encoderUnmodified; //Saves last encoder step value
    //Absolute encoder:
    sensorValue = readSensor() - 2669; //Calibrated using steady state and comparing with real life
    swivelAngle = -sensorValue*2*3.141592/(4096*5)-0.030837201;//Normalizing signal, taking into account the gear relation of 1:5
    
    //Incremental encoder (variable encoderValue updated through interrupts):
    angle = encoderValue*3.141592/(1000); 
    //Update distance
    distance = angle*wheelDiameter/2 + distanceBetSensors;
    
    //////////////////////////////////////////
    //Calibration Eq./////////////////////////
    //////////////////////////////////////////
    distance = distance*0.8441+5.8389; //Result of calibration curve adjustment + distance where it starts measuring
    /////////////////////////////////////////

    //moving Average Filter
    movingAverageAngleValue = 0;
    for (int i = 0; i<N-1; i++){ //Moving all the data to the left and obtaining the average of the moved values
        movingAverageAngle[i] = movingAverageAngle[i+1];
        movingAverageAngleValue = movingAverageAngleValue + movingAverageAngle[i];
    }
    movingAverageAngle[N-1] = swivelAngle; //Adding the new swivelAngle
    movingAverageAngleValue = (movingAverageAngleValue + swivelAngle)/N; //Obtaining average using the newly obtained value
    usedAngle = movingAverageAngleValue; //The angle used for the calculus of current position
  }
  //Output distance on the 7 seg. display  
  sevseg.setNumberF(distance,1);
  //Refresh screen
  sevseg.refreshDisplay();
}

//Absolute encoder reading function
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

//Incremental encoder reading function (Working as an interrupt routine)
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
