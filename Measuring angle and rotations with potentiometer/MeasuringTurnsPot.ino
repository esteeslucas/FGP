#define POT_PIN A0  // Pin connected to the potentiometer
#define MAX_RAD 6.28 // Maximum angle in radians

int potValue = 0;   // Variable to store the potentiometer reading
int turnAmount = 0; //Amount of turns done
float rad = 0;      // Variable to store the angle in radians
float prevRad = 0;  // Variable to store the previous angle in radians
int distance = 0; //Total distance
int counter = 0; //Amount of times the same distance is taken
float initialPoint = 0; //Initial point (the angle at which the wheel is in 0 pos)
float rotDisp = 0; //Initial rotational displacement values
float relRotDisp = 0; //Relative to the initial position displacement
float linearDisp = 0;

void setup() {
  Serial.begin(9600); // Start the serial communication
  potValue = analogRead(POT_PIN); // Read the potentiometer value
  rad = MAX_RAD*potValue/1023;
  initialPoint = rad;
}
void loop(){
  Serial.begin(9600); // Start the serial communication
  potValue = analogRead(POT_PIN); // Read the potentiometer value
  rad = MAX_RAD*potValue/1023;
  if (abs(rad-prevRad)>5 && rad<prevRad) prevRad = prevRad-MAX_RAD;
  else if (abs(rad-prevRad)>5 && rad>prevRad) prevRad = prevRad+MAX_RAD;
  rotDisp = rotDisp + (rad-prevRad);
  prevRad = rad;
  relRotDisp = rotDisp - initialPoint;
  linearDisp = relRotDisp*12.8/2;
  Serial.println(linearDisp);
  delay(50);
}
