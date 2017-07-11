//**********************************************************************************************************************************************************
//  GLOBAL VARIABLES AND CONSTANTS
//**********************************************************************************************************************************************************
//*********** RC GLOBAL VARIABLES ********************
#include <Servo.h>

// Create Variables to hold the Receiver signals
int Ch1, Ch2, Ch3, Ch4, Ch5, Ch6;
int CalcHold;      //Variable to temp hold calculations for steering stick corrections
int Rwheel;
int Lwheel;
int Cwheel;
int Awheel;

// Create Servo Objects as defined in the Servo.h files
Servo L_Servo;  // Servo DC Motor Driver (Designed for RC cars)Servo L_Servo;  // Servo DC Motor Driver (Designed for RC cars)
Servo R_Servo;  // Servo DC Motor Driver (Designed for RC cars)Servo R_Servo;  // Servo DC Motor Driver (Designed for RC cars)
Servo C_Servo;  // Servo motor for the cross wheel
Servo A_Servo;  // Servo motor for arm


//************ WALL GLOBAL VARIABLES ****************

//************ CHUTE GLOBAL VARIABLES ***************
int SharpVF; // Front sharp sensor Variable to hold Sharp data
int SharpVB; // Back sharp sensor
int SharpVL; // Left sharp sensor
int SharpVR; // Right sharp sensor

//arrays for sharp sensors
int SharpAvgL[5];
int SharpAvgR[5];
int SharpAvgF[5];
int SharpAvgB[5];
int i = 0;

const int SharpF = A1; //Pin connecting the sharp
const int SharpB = A2; //Pin connecting the sharp
const int SharpL = A3; //Pin connecting the sharp
const int SharpR = A4; //Pin connecting the sharp

int SharpThreshold = 350; // set threshold for left and right sharp sensors

//**********************************************************************************************************************************************************
//  MAIN SETUP
//**********************************************************************************************************************************************************

void setup() {
  RCsetup();
  // Wallsetup();
  Chutesetup();
  // Lightsetup();

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

//************************************** SET UP CODES ******************************

//********************** RCsetup() ***************************
//**************************************************************
void RCsetup() {
  // Set the pins that the transmitter will be connected to all to input
  pinMode(12, INPUT); //I connected this to Chan1 of the Receiver
  pinMode(11, INPUT); //I connected this to Chan2 of the Receiver
  pinMode(10, INPUT); //I connected this to Chan3 of the Receiver
  pinMode(9, INPUT); //I connected this to Chan4 of the Receiver
  pinMode(8, INPUT); //I connected this to Chan5 of the Receiver
  pinMode(7, INPUT); //I connected this to Chan6 of the Receiver
  pinMode(13, OUTPUT); //Onboard LED to output for diagnostics

  // Attach Speed controller that acts like a servo to the board
  R_Servo.attach(3); //Pin 3    // Right wheel
  L_Servo.attach(5); //Pin 5    // Left wheel
  C_Servo.attach(1); //Pin 1    // Cross wheel
  A_Servo.attach(4); //Pin 4    // Arm

}

//********************** Chutesetup() ***************************
//**************************************************************
void Chutesetup() {

  //  pinMode(A1, INPUT); // for front sharp sensor
  //  pinMode(A2, INPUT); // for back sharp sensor
  //  pinMode(A3, INPUT); // for left sharp sensor
  //  pinMode(A4, INPUT); // for right sharp sensor

}

//**********************************************************************************************************************************************************
//  MAIN DRIVE CONTROL CODES
//**********************************************************************************************************************************************************

//********************** RCdrive() ***************************
//************  drive code for RC control  ***************
//**************************************************************

void RCdrive() {
  Ch1 = pulseIn(12, HIGH, 21000); // Capture pulse width on Channel 1
  Ch2 = pulseIn(11, HIGH, 21000); // Capture pulse width on Channel 2
  Ch3 = pulseIn(10, HIGH, 21000);  // Capture pulse width on Channel 3
  Ch4 = pulseIn(9, HIGH, 21000);  // Capture pulse width on Channel 4
  // Ch5 = pulseIn(8, HIGH, 21000); // Capture pulse width on Channel 5
  //    Ch6 = pulseIn(7, HIGH, 21000); // Capture pulse width on Channel 6
  DriveServosRC(); // Drive Motors under RC control
  PrintRC(); //Print Values for RC Mode Diagnostics
}

//********************** Chutedrive() ***************************
//************  drive code for chute traversal  ***************
//**************************************************************

void Chutedrive() {

  readSharpsEdgy();
 // Serial.println("done reading Sharp Edgy!");
  //printSharps();
  //forwardRobot();

  // if the robot is not close to either side of the chute, go forward
  while (SharpVR < SharpThreshold && SharpVL < SharpThreshold) {
  SharpThreshold = 350;
  forwardRobot();
  readSharpsEdgy();
  Serial.println("Forward!");
  }

  // robot is too close to the right wall, speed up right wheel to turn away
  if (SharpVR > SharpThreshold) {
  Serial.println("Turning left!");
  Rwheel = 1500 + 70;
  Lwheel = 1500 - 70;
  updateWheels();
  }

  // robot is too close to the left wall, speed up left wheel to turn away
  if (SharpVL > SharpThreshold) {
  Serial.println("Turning right!");
  Rwheel = 1500 + 40;
  Lwheel = 1500 - 110;
  updateWheels();
  }

  // if robot for some reason is too close on both sides, stop the robot
  if (SharpVR > SharpThreshold && SharpVL > SharpThreshold) {
  SharpThreshold = 400;
  stopRobot();
  delay(500);
  }
 
}

//**************************************************************


//**********************************************************************************************************************************************************
// HELPING FUNCTIONS
//**********************************************************************************************************************************************************

//**************************************************** DRIVING ****************************************************

//*************************************************************************
//  forwardRobot()
//*************************************************************************
void forwardRobot() {
  //roll forward slowly with even speeds
  Rwheel = 1500 + 55;
  Lwheel = 1500 - 80;
  updateWheels();
}

//*************************************************************************
//  stopRobot()
//*************************************************************************
void stopRobot() {
  //set wheels to zero
  Rwheel = 1500;
  Lwheel = 1500;
  updateWheels();
}

//*************************************************************************
//  updateWheels
//*************************************************************************
void updateWheels() {

  //set limits
  if (Rwheel > 2000)
  Rwheel = 2000;
  if (Rwheel < 1000)
  Rwheel = 1000;

  if (Lwheel > 2000)
  Lwheel = 2000;
  if (Lwheel < 1000)
  Lwheel = 1000;

  //update the wheels
  R_Servo.writeMicroseconds(Rwheel); // 1000-2000, 1500 should be stop
  L_Servo.writeMicroseconds(Lwheel); // 1000-2000, 1500 should be stop

  //delay(5000);
  // Rwheel = 1500;
  // Lwheel = 1500;
  // R_Servo.writeMicroseconds(Rwheel); // 1000-2000, 1500 should be stop
  // L_Servo.writeMicroseconds(Lwheel); // 1000-2000, 1500 should be stop
}


//********************** SetLimits() ***************************
//*******  Make sure values never exceed ranges  ***************
//******  For most all servos and like controlers  *************
//****   control must fall between 1000uS and 2000uS  **********
//**************************************************************
void SetLimits() {
  if (Lwheel < 1280) {// Can be set to a value you don't wish to exceed
  Lwheel = 1280;  // to adjust maximums for your own robot
  }
  if (Lwheel > 1760) {// Can be set to a value you don't wish to exceed
  Lwheel = 1760;  // to adjust maximums for your own robot
  }
  if (Rwheel < 1420) {// Can be set to a value you don't wish to exceed
  Rwheel = 1420;  // to adjust maximums for your own robot
  }
  if (Rwheel > 1570) {// Can be set to a value you don't wish to exceed
  Rwheel = 1570;  // to adjust maximums for your own robot
  }

  if (Cwheel < 1100) {// Can be set to a value you don't wish to exceed
  Cwheel = 1100;  // to adjust maximums for your own robot
  }
  if (Cwheel > 1900) {// Can be set to a value you don't wish to exceed
  Cwheel = 1900;  // to adjust maximums for your own robot
  }

  if (Awheel < 1460) {// Can be set to a value you don't wish to exceed
  Awheel = 1460;  // to adjust maximums for your own robot
  }
  if (Awheel > 1540) {// Can be set to a value you don't wish to exceed
  Awheel = 1540;  // to adjust maximums for your own robot
  }

  R_Servo.writeMicroseconds(Rwheel);
  L_Servo.writeMicroseconds(Lwheel);
  C_Servo.writeMicroseconds(Cwheel);
  A_Servo.writeMicroseconds(Awheel);

  PrintWheelCalcs(); //REMEMBER: printing values slows reaction times
}

//*******************  DriveServosRC()  ************************
//******  Use the value collected from Ch1 and Ch2  ************
//******  on a single stick to relatively calculate  ***********
//****  speed and direction of two servo driven wheels *********
//**************************************************************
void DriveServosRC()
{

  ///////// Drive wheels ///////////////
  if (Ch3 <= 1450 || (Ch4 <= 1450 || Ch4 > 1550)) {
  Lwheel = Ch4 + Ch3 - 1500;
  Rwheel = Ch4 - Ch3 + 1500;
  //  Serial.println("Going down!");
  SetLimits();
  }
  else if (Ch3 > 1550 || (Ch4 <= 1450 || Ch4 > 1550)) {
  int Ch4_mod = map(Ch4, 1000, 2000, 2000, 1000); // Invert the Ch1 axis to keep the math similar
  int Ch3_mod = map(Ch3, 1000, 2000, 1000, 2000); // Slow reaction time
  Lwheel = Ch4_mod + Ch3 - 1500;
  Rwheel = Ch4_mod - Ch3 + 1500;
  //  Serial.println("Going up!");
  SetLimits();
  }
  else {
  Lwheel = 1500;
  Rwheel = 1500;
  SetLimits();
  //  Serial.println("Stopped!");

  }


  /////////// Cross wheels //////////////
  if (Ch1 <= 1460) {
  Cwheel = Ch1;
  SetLimits();
  }
  else if (Ch1 > 1540) {
  int Ch1_mod = map(Ch1, 1000, 2000, 1000, 2000); // Invert the Ch1 axis to keep the math similar
  Cwheel = Ch1_mod;
  SetLimits();
  }
  else {
  Cwheel = 1500;
  SetLimits();
  }

  /////////// Arm //////////////
  if (Ch2 <= 1300) {
  Awheel = Ch2;
  SetLimits();
  }
  else if (Ch2 > 1700) {
  int Ch2_mod = map(Ch2, 1000, 2000, 1000, 2000); // Invert the Ch1 axis to keep the math similar
  Awheel = Ch2_mod;
  SetLimits();
  }
  else {
  Awheel = 1500;
  SetLimits();
  }

}

//**************************************************** SENSORS ****************************************************
 
//*************************************************************************
//  readSharpsEdgy
//*************************************************************************
void readSharpsEdgy() {
  //use arrays to calculate average
  SharpAvgL[i] = analogRead(SharpL);
  SharpAvgR[i] = analogRead(SharpR);
  SharpAvgF[i] = analogRead(SharpF);
  SharpAvgB[i] = analogRead(SharpB);

  //iterate through arrays
  i++;
  i = i % 5; //if i = 5, then restart at i = 0

  int j;
  int sumL = 0;
  int sumR = 0;
  int sumF = 0;
  int sumB = 0;
  for (j = 0; j < 5; j++) {
  sumL = sumL + SharpAvgL[j];
  sumR = sumR + SharpAvgR[j];
  sumF = sumF + SharpAvgF[j];
  sumB = sumB + SharpAvgB[j];
  }

  SharpVL = sumL / 5;
  SharpVR = sumR / 5;
  SharpVF = sumF / 5;
  SharpVB = sumB / 5;
}


//*************************************************************************
//  readSharps()
//*************************************************************************
void readSharps() {
  SharpVF = analogRead(SharpF); //Read the value of the sharp sensor
  Serial.println("Sharp front =  " + (String)SharpVF);
  Serial.println(" ");

  SharpVB = analogRead(SharpB); //Read the value of the sharp sensor
  Serial.println("Sharp back =  " + (String)SharpVB);
  Serial.println(" ");

  SharpVL = analogRead(SharpL); //Read the value of the sharp sensor
  Serial.println("Sharp left =  " + (String)SharpVL);
  Serial.println(" ");

  SharpVR = analogRead(SharpR); //Read the value of the sharp sensor
  Serial.println("Sharp right =  " + (String)SharpVR);
  Serial.println(" ");
  delay(500);
  Serial.println("*************************************** ");
  Serial.println(" ");

}

//**************************************************** PRINTING ****************************************************

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
  Serial.print("Awheel = ");
  Serial.println(Awheel);
  Serial.println(" ");
}

//*************************************************************************
//  printSharps()
//*************************************************************************
void printSharps() {
  Serial.println("Sharp front =  " + (String)SharpVF);
  Serial.println(" ");

  Serial.println("Sharp back =  " + (String)SharpVB);
  Serial.println(" ");

  Serial.println("Sharp left =  " + (String)SharpVL);
  Serial.println(" ");

  Serial.println("Sharp right =  " + (String)SharpVR);
  Serial.println(" ");

  delay(500);
  Serial.println("*************************************** ");
  Serial.println(" ");
}


//**********************  PrintRC()  ***************************
//***  Simply print the collected RC values for diagnostics  ***
//**************************************************************
void PrintRC()
{ // print out the values you read in:
  //  Serial.println(" RC Control Mode ");
  Serial.print("Value Ch1 = ");
  Serial.println(Ch1);
  Serial.print("Value Ch2 = ");
  Serial.println(Ch2);
  Serial.print("Value Ch3 = ");
  Serial.println(Ch3);
  Serial.print("Value Ch4 = ");
  Serial.println(Ch4);
}



//***********************************************`***********************************************************************************************************
//  MAIN LOOP - EDGY
//**********************************************************************************************************************************************************


void loop() {
  //RCdrive();
  Chutedrive();
}


