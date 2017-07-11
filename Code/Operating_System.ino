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

int ChuteVar = 0;    // 0 is off, 1 is on
int RCVar = 0;
int LightVar = 0;
int WallVar = 0;

//************ WALL GLOBAL VARIABLES ****************

const int ClickSwitch = A6;
int Switch;

//************ CHUTE GLOBAL VARIABLES ***************
int SharpVF; // Front sharp sensor (cross wheel side) Variable to hold Sharp data
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

int SharpThreshold = 340; // set threshold for left and right sharp sensors
int SharpSum;    // sum of left and right sharp sensors

int firstPush = 0;

//************ LIGHT TRACKING GLOBAL VARIABLES ****************
//variables to hold data
//these hold values related by the sensors
int LeftLightSensor;
int RightLightSensor;
int dist;       // combined sensor output, indicates distance from light source
int direc;      // left-right sensor output, indicates whether robot is facing towards light or not
int workingDirecThreshold;
int workingDistThreshold;


//has the first spin happened?
int firstSpin = 0;

//hard constants for when to stop initial spin
const int direcThreshold = 200;
const int distThreshold = 750;

//**********************************************************************************************************************************************************
//  MAIN SETUP
//**********************************************************************************************************************************************************

void setup() {
  RCsetup();
  Wallsetup();
  Chutesetup();
  Lightsetup();

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
  R_Servo.attach(1); //Pin 1    // Right wheel
  L_Servo.attach(2); //Pin 2    // Left wheel
  C_Servo.attach(3); //Pin 3    // Cross wheel
  A_Servo.attach(4); //Pin 4    // Arm

}

//********************** Wallsetup() ***************************
//**************************************************************
void Wallsetup() {


}

//********************** Chutesetup() ***************************
//**************************************************************
void Chutesetup() {


}

//********************** Lightsetup() ***************************
//**************************************************************
void Lightsetup() {
  pinMode(A9, INPUT); // right light sensor should be connected to pin A9, it needs to be reading inputs
  pinMode(A8, INPUT); // left sensor
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
  Ch5 = pulseIn(8, HIGH, 21000); // Capture pulse width on Channel 5
  Switch = analogRead(ClickSwitch);
  //    Ch6 = pulseIn(7, HIGH, 21000); // Capture pulse width on Channel 6

  // if safety on RC controller is off
  while (abs(1500 - Ch5) < 20) {
    Ch1 = pulseIn(12, HIGH, 21000); // Capture pulse width on Channel 1
    Ch2 = pulseIn(11, HIGH, 21000); // Capture pulse width on Channel 2
    Ch3 = pulseIn(10, HIGH, 21000);  // Capture pulse width on Channel 3
    Ch4 = pulseIn(9, HIGH, 21000);  // Capture pulse width on Channel 4
    Ch5 = pulseIn(8, HIGH, 21000); // Capture pulse width on Channel 5
    Switch = analogRead(ClickSwitch);
    //  WallVar = 0;
    //  ChuteVar = 0;
    //  LightVar = 0;
    DriveServosRC(); // Drive Motors under RC control
    PrintRC(); //Print Values for RC Mode Diagnostics
  }

  // if robot has not yet completed chute and light traversal
  if (ChuteVar == 0 && LightVar == 0) {
    // once the RC safety is turned off, begin wall traversal
    WallVar = 1;    // start Wall traversal segment
  }
}

//**********************  Walldrive() **************************
//************  drive code for chute traversal  ***************
//*************************************************************

void Walldrive() {

  //read in the values of the sharp sensors
  Serial.println("I'm in wall drive!");
  Switch = analogRead(ClickSwitch);
  readSharpsEdgy();

//  forwardRobot();

  if (SharpVF > 600) {

//        forwardCross();
//        delay(500);

    // do over-the-wall stuff here
    while (Switch > 20) {
      Switch = analogRead(ClickSwitch);
      Serial.println("Crossing wall!");
//            forwardRobot();
//            forwardCross();

      //regain RC control
      Ch5 = pulseIn(8, HIGH, 21000);
      if (abs(1500 - Ch5) < 20) {
        break;
      }

      if (Switch < 20) {
        stopRobot();
        break;
      }
    }
  }

//   forwardRobot();

  // if the front sensor sees nothing in front of it and the back sensor sees something nearby
  if (SharpVB > 400 && SharpVF < 100) {
    Serial.println("I think I'm over the wall!");
//        stopRobot();
//        delay(2000);
    WallVar = 0;
    ChuteVar = 1;  // initiate the chute code segment
  }

}

//********************** Chutedrive() ***************************
//************  drive code for chute traversal  ***************
//**************************************************************

void Chutedrive() {
  if (ChuteVar == 1) {

    if (firstPush == 0) {
      forwardRobot();    // make sure we're inside the chute
      delay(2000);
      firstPush = 1;
      stopRobot();
    }

    readSharps();
    Serial.print("SharpSum = ");
    Serial.println(SharpSum);
    //printSharps();
    //forwardRobot();

    // if the robot is not close to either side of the chute, go forward
    while (SharpVR < SharpThreshold && SharpVL < SharpThreshold) {
      SharpThreshold = SharpThreshold - 0;
      forwardRobot();
      readSharps();

      // is the robot out of the shoot
      if (SharpSum < 200) {
        break;
      }

      //regain RC control
      Ch5 = pulseIn(8, HIGH, 21000);
      if (abs(1500 - Ch5) < 20) {
        break;
      }
    }

    // robot is too close to the right wall, speed up right wheel to turn away
    if (SharpVR > SharpThreshold + 0) {
      //Serial.println("Turning left!");
      Rwheel = 1500 + 55;
      Lwheel = 1500;
      updateWheels();
    }

    // robot is too close to the left wall, speed up left wheel to turn away
    if (SharpVL > SharpThreshold - 10) {
      //Serial.println("Turning right!");
      Rwheel = 1500;
      Lwheel = 1500 - 48;
      updateWheels();
    }

    // if robot for some reason is too close on both sides, stop the robot
    if (SharpVR > 700 && SharpVL > 700) {
      SharpThreshold = SharpThreshold + 100;
      stopRobot();
      delay(500);
    }

    // if robot sees that it's out of the chute
    if (SharpSum < 200) {
      stopRobot();
      Serial.println("Out of the chute!");
      delay(1000);
      forwardRobot();    // move all the way out of the chute
      delay(2500);
      ChuteVar = 0;
      LightVar = 1;
      stopRobot();
    }

  }
}

//********************** Lightdrive() ***************************
//************  drive code for light sensing  ***************
//**************************************************************
void Lightdrive() {
  SharpVB = analogRead(SharpB); //Read the value of the sharp sensor

  //this spin will only get executed the first time around.
  if (firstSpin == 0) {
    spinRobot();
    firstSpin = 1;
  }

  Serial.println("Entering loop()");

  //Robot has performed initial spin, and it's pointing at the light
  readLightSensors();
  readSharpSensors();

  //Robot is pointing at light
  while (dist < workingDistThreshold && abs(direc) < workingDirecThreshold) {
    Serial.println("I'm pointing at light! Doing nothing");
    backwardRobot();
    readLightSensors();
    readSharpSensors();

    //regain RC control
    Ch5 = pulseIn(8, HIGH, 21000);
    if (abs(1500 - Ch5) < 20) {
      break;
    }
  }

  //Robot is NOT pointing at light
  while (dist > workingDistThreshold || abs(direc) > workingDirecThreshold) {
    turnRobot();
    readLightSensors();
    readSharpSensors();

    //regain RC control
    Ch5 = pulseIn(8, HIGH, 21000);
    if (abs(1500 - Ch5) < 20) {
      break;
    }
  }

  readSharpSensors();
}


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
  Lwheel = 1500 - 47;
  updateWheels();
}

//*************************************************************************
//  forwardStrong()
//*************************************************************************
void forwardStrong() {
  //roll forward slowly with even speeds
  Rwheel = 1500 + 70;
  Lwheel = 1500 - 70;
  updateWheels();
}

//*************************************************************************
//  forwardCross()
//*************************************************************************
void forwardCross() {
  Cwheel = 1500 - 490;
  updateWheels();
}

//*************************************************************************
//  backwardRobot
//*************************************************************************
void backwardRobot() {
  //roll forward slowly with even speeds
  Lwheel = 1500 + 58; //1500 + 40;
  Rwheel = 1500 - 67; //1500 - 40;
  updateWheels();
}

//*************************************************************************
//  killRobot
//*************************************************************************
void killRobot() {
  //set wheels to zero
  Rwheel = 1500;
  Lwheel = 1500;
  updateWheels();

  armPlace();
  //pause forever
  while (SharpVB > 640) {
    stopRobot();
    SharpVB = analogRead(SharpB);
  }
}

//*************************************************************************
//  stopRobot()
//*************************************************************************
void stopRobot() {
  //set wheels to zero
  Rwheel = 1500;
  Lwheel = 1500;
  Cwheel = 1500;
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

  if (Awheel > 1540)
    Awheel = 1540;
  if (Awheel < 1460)
    Awheel = 1460;

  if (Cwheel > 1510)
    Cwheel = 1510;
  if (Cwheel < 1020)
    Cwheel = 1020;

  //update the wheels
  R_Servo.writeMicroseconds(Rwheel); // 1000-2000, 1500 should be stop
  L_Servo.writeMicroseconds(Lwheel); // 1000-2000, 1500 should be stop
  A_Servo.writeMicroseconds(Awheel);
  C_Servo.writeMicroseconds(Cwheel);
}

//*************************************************************************
//  turnRobot
//*************************************************************************
void turnRobot() {

  while (abs(direc) > direcThreshold || dist > distThreshold) {
    Serial.println("turning");

    //regain RC control
    Ch5 = pulseIn(8, HIGH, 21000);
    if (abs(1500 - Ch5) < 20) {
      break;
    }

    //check to see where we are pointing: + = L, - = R
    readLightSensors();
    readSharpSensors();
    Serial.println("Point = " + (String)direc);

    //if robot is pointing right, spin left
    if (direc < 0) {
      Serial.println("I'm pointing right! turn Left.");
      //      Lwheel = 1500 + 70;
      //      Rwheel = 1500 - 57;
      Rwheel = 1500;
      Lwheel = 1500 + 40; //1500 + 45; // left wheel going forwards
      updateWheels();
      readLightSensors();
      readSharpSensors();
    }

    //if robot is pointing left, spin right
    if (direc > 0) {
      Serial.println("I'm pointing left! turn right.");
      //      Lwheel = 1500 + 48;
      //      Rwheel = 1500 - 80;
      Rwheel = 1500 - 42; //1500 - 45; // right wheel going backwards
      Lwheel = 1500;
      updateWheels();
      readLightSensors();
      readSharpSensors();
    }

    //try reading again
    readLightSensors();
    readSharpSensors();
  }
}
//********************** SetLimits() ***************************
//*******  Make sure values never exceed ranges  ***************
//******  For most all servos and like controlers  *************
//****   For RC drive code  **********
//**************************************************************
void SetLimits() {
  if (Lwheel < 1420) {// Can be set to a value you don't wish to exceed
    Lwheel = 1420;  // to adjust maximums for your own robot
  }
  if (Lwheel > 1580) {// Can be set to a value you don't wish to exceed
    Lwheel = 1580;  // to adjust maximums for your own robot
  }
  if (Rwheel < 1400) {// Can be set to a value you don't wish to exceed
    Rwheel = 1400;  // to adjust maximums for your own robot
  }
  if (Rwheel > 1600) {// Can be set to a value you don't wish to exceed
    Rwheel = 1600;  // to adjust maximums for your own robot
  }

  if (Cwheel < 1050) {// Can be set to a value you don't wish to exceed
    Cwheel = 1050;  // to adjust maximums for your own robot
  }
  if (Cwheel > 1510) {// Can be set to a value you don't wish to exceed
    Cwheel = 1510;  // to adjust maximums for your own robot
  }

  if (Awheel < 1410) {// Can be set to a value you don't wish to exceed
    Awheel = 1410;  // to adjust maximums for your own robot
  }
  if (Awheel > 1590) {// Can be set to a value you don't wish to exceed
    Awheel = 1590;  // to adjust maximums for your own robot
  }

  R_Servo.writeMicroseconds(Rwheel);
  L_Servo.writeMicroseconds(Lwheel);
  C_Servo.writeMicroseconds(Cwheel);
  A_Servo.writeMicroseconds(Awheel);

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

//*************************************************************************
//  spinRobot - light code
//*************************************************************************
void spinRobot() {

  Serial.println("Performing spinRobot()");

  stopRobot();

  // for initial spin
  readLightSensors();

  //if either condition is bad, spin
  while (abs(direc) > direcThreshold || dist > distThreshold) {

    //regain RC control
    Ch5 = pulseIn(8, HIGH, 21000);
    if (abs(1500 - Ch5) < 20) {
      break;
    }

    Serial.println("spinning");
    //try reading again
    readLightSensors();

    Rwheel = 1500 - 50; //1500 - 40; // right wheel going forwards
    Lwheel = 1500 - 50; //1500 - 40; // left wheel going backwards
    updateWheels();
  }

  //success, robot is looking at light
  Serial.println("Success! I'm pointing at light! Stopping.");
  workingDirecThreshold = direcThreshold;
  workingDistThreshold = distThreshold;
  Serial.println("point =  " + (String)direc);

  stopRobot();
  delay(1000);
}


//*************************************************************************
//  armPlace
//*************************************************************************
void armPlace() {
  // lower arm
  Awheel = 1500 + 100;
  A_Servo.writeMicroseconds(Awheel);
  delay(3500);
  Awheel = 1500;
  A_Servo.writeMicroseconds(Awheel);

  delay(1000);
  forwardRobot();
  delay(2000);
  stopRobot();

  // go back up
  Awheel = 1500 - 100;
  A_Servo.writeMicroseconds(Awheel);
  delay(2200);
  Awheel = 1500;
  A_Servo.writeMicroseconds(Awheel);

  delay(1000);
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

  int n = 5;

  //iterate through arrays
  i++;
  i = i % n; //if i = 5, then restart at i = 0

  int j;
  int sumL = 0;
  int sumR = 0;
  int sumF = 0;
  int sumB = 0;
  for (j = 0; j < n; j++) {
    sumL = sumL + SharpAvgL[j];
    sumR = sumR + SharpAvgR[j];
    sumF = sumF + SharpAvgF[j];
    sumB = sumB + SharpAvgB[j];
  }


  SharpVL = sumL / n;
  SharpVR = sumR / n;
  SharpVF = sumF / n;
  SharpVB = sumB / n;

  SharpSum = SharpVL + SharpVR;
}


//*************************************************************************
//  readSharps() - for chute traversal and wall code
//*************************************************************************
void readSharps() {
  SharpVF = analogRead(SharpF); //Read the value of the sharp sensor
  //  Serial.println("Sharp front =  " + (String)SharpVF);
  //  Serial.println(" ");

  SharpVB = analogRead(SharpB); //Read the value of the sharp sensor
  //  Serial.println("Sharp back =  " + (String)SharpVB);
  //  Serial.println(" ");

  SharpVL = analogRead(SharpL); //Read the value of the sharp sensor
  //  Serial.println("Sharp left =  " + (String)SharpVL);
  //  Serial.println(" ");

  SharpVR = analogRead(SharpR); //Read the value of the sharp sensor
  //  Serial.println("Sharp right =  " + (String)SharpVR);
  //  Serial.println(" ");
  //  Serial.println("*************************************** ");
  //  Serial.println(" ");
  SharpSum = SharpVL + SharpVR;


}

//*************************************************************************
//  readLightSensors
//*************************************************************************
void readLightSensors() {
  //read and print values from both light sensors

  LeftLightSensor = analogRead(A9);
  RightLightSensor = analogRead(A8);
  dist = LeftLightSensor + RightLightSensor;  // associated with distance to light
  direc = LeftLightSensor - RightLightSensor; // associated with direction to light
  //
  //  Serial.println("dist = " + (String)dist);
  //  Serial.println("");
  //  Serial.println("direc = " + (String)direc);
  //delay(200);

  // Serial.println("L = " + (String)LeftLightSensor + " | " + (String)RightLightSensor + " = R");
}

//*************************************************************************
//  readSharpSensors - light sensor code, triggers the stop in front of the box
//*************************************************************************
void readSharpSensors() {
  //read and print values from front sharp sensor
  SharpVB = analogRead(SharpB); //Read the value of the sharp sensor
  Serial.println("Sharp =  " + (String)SharpVB);
  Serial.println("****************************");

  if (SharpVB > 650) {
    killRobot();
  }
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

  Serial.println("*************************************** ");
  Serial.println(" ");
}


//**********************  PrintRC()  ***************************
//***  Simply print the collected RC values for diagnostics  ***
//**************************************************************
void PrintRC()
{ // print out the values you read in:
  Serial.println(" RC Control Mode ");
  Serial.print("Value Ch1 = ");
  Serial.println(Ch1);
  Serial.print("Value Ch2 = ");
  Serial.println(Ch2);
  Serial.print("Value Ch3 = ");
  Serial.println(Ch3);
  Serial.print("Value Ch4 = ");
  Serial.println(Ch4);
  Serial.print("Value Ch5 = ");
  Serial.println(Ch5);
  Serial.print("Switch = ");
  Serial.println(Switch);

}



//***********************************************`***********************************************************************************************************
//  MAIN LOOP - EDGY
//**********************************************************************************************************************************************************
void loop() {

  //  readSharps();
  //  printSharps();
  //  delay(200);


  RCdrive();

  if (WallVar == 1) {
    Walldrive();
  }

  if (ChuteVar == 1) {
    Chutedrive();
  }

  if (LightVar == 1) {
    Lightdrive();
  }

}



