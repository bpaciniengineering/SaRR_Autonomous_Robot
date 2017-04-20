/*
  Testing RC Connection
  4/12/15 by Brian Patton
  Captures the Pulse signal from a RC controller
  Feel free to do whatever you want with this code example
*/
#include <Servo.h>

// Create Variables to hold the Receiver signals
int Ch1, Ch2, Ch3, Ch4, Ch5, Ch6;
int CalcHold;        //Variable to temp hold calculations for steering stick corections
int Rwheel;
int Lwheel;
int Cwheel;

// Create Servo Objects as defined in the Servo.h files
Servo L_Servo;  // Servo DC Motor Driver (Designed for RC cars)Servo L_Servo;  // Servo DC Motor Driver (Designed for RC cars)
Servo R_Servo;  // Servo DC Motor Driver (Designed for RC cars)Servo R_Servo;  // Servo DC Motor Driver (Designed for RC cars)
Servo C_Servo;  // Servo motor for the cross wheel

int L_SharpValue; // Variable to hold Left Sharp data
int C_SharpValue; // Variable to hold Center Sharp data
int R_SharpValue; // Variable to hold Right Sharp data
const int L_SharpPin = A9; //Pin connecting the sharp
const int C_SharpPin = A8; //Pin connecting the sharp
const int R_SharpPin = A7; //Pin connecting the sharp

//**************************************************************
//*****************  Setup  ************************************
//**************************************************************
void setup() {
  // Set the pins that the transmitter will be connected to all to input
  pinMode(12, INPUT); //I connected this to Chan1 of the Receiver
  pinMode(11, INPUT); //I connected this to Chan2 of the Receiver
  pinMode(10, INPUT); //I connected this to Chan3 of the Receiver
  pinMode(9, INPUT); //I connected this to Chan4 of the Receiver
  pinMode(8, INPUT); //I connected this to Chan5 of the Receiver
  pinMode(7, INPUT); //I connected this to Chan6 of the Receiver
  pinMode(13, OUTPUT); //Onboard LED to output for diagnostics
  // Attach Speed controller that acts like a servo to the board
  R_Servo.attach(2); //Pin 2
  L_Servo.attach(3); //Pin 3
  C_Servo.attach(1); //Pin 1
  //Flash the LED on and Off 10x Start
  for (int i = 0; i < 10; i++) {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }
  //Flash the LED on and Off 10x End
  Serial.begin(9600);
}

//********************** MixLimits() ***************************
//*******  Make sure values never exceed ranges  ***************
//******  For most all servos and like controlers  *************
//****   control must fall between 1000uS and 2000uS  **********
//**************************************************************
void SetLimits() {
  if (Lwheel < 1260) {// Can be set to a value you don't wish to exceed
    Lwheel = 1260;    // to adjust maximums for your own robot
  }
  if (Lwheel > 1740) {// Can be set to a value you don't wish to exceed
    Lwheel = 1740;    // to adjust maximums for your own robot
  }
  if (Rwheel < 1420) {// Can be set to a value you don't wish to exceed
    Rwheel = 1420;    // to adjust maximums for your own robot
  }
  if (Rwheel > 1570) {// Can be set to a value you don't wish to exceed
    Rwheel = 1570;    // to adjust maximums for your own robot
  }

  if (Cwheel < 1300) {// Can be set to a value you don't wish to exceed
    Cwheel = 1300;    // to adjust maximums for your own robot
  }
  if (Cwheel > 1700) {// Can be set to a value you don't wish to exceed
    Cwheel = 1700;    // to adjust maximums for your own robot
  }
  
  R_Servo.writeMicroseconds(Rwheel);
  L_Servo.writeMicroseconds(Lwheel);
  C_Servo.writeMicroseconds(Cwheel);

   PrintWheelCalcs(); //REMEMBER: printing values slows reaction times
}


//*****************  PrintWheelCalcs()  ************************
//*******  Prints calculated wheel output values  **************
//**************************************************************
void PrintWheelCalcs() {
 Serial.print("Rwheel = ");
 Serial.println(Rwheel);
 Serial.print("Lwheel = ");
 Serial.println(Lwheel);
 Serial.println(" ");
 Serial.print("Cwheel = ");
 Serial.println(Cwheel);
 Serial.println(" ");
// Serial.println(L_SharpValue);
// Serial.println(" ");
}
//********************  TestWheels()  **************************
//*  Direct call to Servos to test wheel speed and direction  **
//**************************************************************
void TestWheels() {
 // R_Servo.writeMicroseconds(1500); // 1000-2000, 1500 should be stop
 // L_Servo.writeMicroseconds(1500); // 1000-2000, 1500 should be stop
}
//*******************  DriveServosRC()  ************************
//******  Use the value collected from Ch1 and Ch2  ************
//******  on a single stick to relatively calculate  ***********
//****  speed and direction of two servo driven wheels *********
//**************************************************************
void DriveServosRC()
{
  // Drive wheels
  if (Ch3 <= 1400) {
    Lwheel = Ch4 + Ch3 - 1500;
    Rwheel = Ch4 - Ch3 + 1500;
    SetLimits(); 
  }
  else if (Ch3 > 1600) {
    int Ch4_mod = map(Ch4, 1000, 2000, 2000, 1000); // Invert the Ch1 axis to keep the math similar
    int Ch3_mod = map(Ch3, 1000, 2000, 1000, 2000); // Slow reaction time
    Lwheel = Ch4_mod + Ch3 - 1500;
    Rwheel = Ch4_mod - Ch3 + 1500;
    SetLimits();
  }
  else{
    Lwheel = 1500;
    Rwheel = 1500;
    SetLimits();
  }


   // Cross wheels
  if (Ch1 <= 1460) {
    Cwheel = Ch1;
    SetLimits();
  }
  else if (Ch1 > 1540) {
    int Ch1_mod = map(Ch1, 1000, 2000, 1000, 2000); // Invert the Ch1 axis to keep the math similar
    Cwheel = Ch1_mod;
    SetLimits();
  }
  else{
    Cwheel = 1500;
    SetLimits();
  }
}
//*******************   pulseMotors  ***************************
//pulses either mapped or direct signals generated from Mixlimits
//**************************************************************
void pulseMotors() {
  //un-comment the next two line to drive the wheels directly with the MaxLimits Set
  //  R_Servo.writeMicroseconds(Rwheel);
  //  L_Servo.writeMicroseconds(Lwheel);

  //un-comment the next two to map a control range.
  //*** Take the standard range of 1000 to 2000 and frame it to your own minimum and maximum
  //*** for each wheel.
  Rwheel = map(Rwheel, 1000, 2000, 1350, 1650);
  Lwheel = map(Lwheel, 1000, 2000, 1350, 1650);
  R_Servo.writeMicroseconds(Rwheel);
  L_Servo.writeMicroseconds(Lwheel);

  // un-comment this line do display the value being sent to the motors
  //  PrintWheelCalcs(); //REMEMBER: printing values slows reaction times
}
//**********************  PrintRC()  ***************************
//***  Simply print the collected RC values for diagnostics  ***
//**************************************************************
void PrintRC()
{ // print out the values you read in:
//  Serial.println(" RC Control Mode ");
  Serial.print("Value Ch1 = ");
  Serial.println(Ch1);
//  Serial.print("Value Ch2 = ");
//  Serial.println(Ch2);
  Serial.print("Value Ch3 = ");
  Serial.println(Ch3);
  Serial.print("Value Ch4 = ");
  Serial.println(Ch4);
  //Serial.print("Control = ");
  //Serial.println(Ch5);
//  Serial.println(L_SharpValue);
//  Serial.println(" ");
//  Serial.println(C_SharpValue);
//  Serial.println(" ");
  //Serial.print("Value Ch6 = ");
  //Serial.println(Ch6);
  //Serial.println(" ");
  //delay(1000);
}
//************************  loop()  ****************************
//**********************  Main Loop  ***************************
//**************************************************************
void loop()
{
       Ch1 = pulseIn(12, HIGH, 21000); // Capture pulse width on Channel 1
 // Ch2 = pulseIn(11, HIGH, 21000); // Capture pulse width on Channel 2
       Ch3 = pulseIn(10, HIGH, 21000);  // Capture pulse width on Channel 3
       Ch4 = pulseIn(9, HIGH, 21000);  // Capture pulse width on Channel 4
 // Ch5 = pulseIn(8, HIGH, 21000); // Capture pulse width on Channel 5
  //      Ch6 = pulseIn(7, HIGH, 21000); // Capture pulse width on Channel 6
    L_SharpValue = analogRead(L_SharpPin); //Read the value of the sharp sensor
    C_SharpValue = analogRead(C_SharpPin); //Read the value of the sharp sensor
    R_SharpValue = analogRead(R_SharpPin); //Read the value of the sharp sensor
  //  TestWheels();
  DriveServosRC(); // Drive Motors under RC control
   PrintRC(); //Print Values for RC Mode Diagnostics

}
