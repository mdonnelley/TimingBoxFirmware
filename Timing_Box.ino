#include <TimerOne.h>

// ----------------------------------------------------------------------------------------------
//
// i - ROI Search Mode
// r - Run mode
// a - Acquire one block (standard block)
// f - Acquire flat images (short block, 1 image per breath)
// s - Toggle shutter status (use to keep shutter open permanently)
// v - Toggle timing status (External trigger vs Internal timing)
// x - Stop
//
// n - Aeroneb on
// m - Aeroneb off
//
// 0 - No treatment
// 1 - Deliver Treatment (default)
//
// ----------------------------------------------------------------------------------------------
// Setup timing box input BNC interrupts
#define IN1Interrupt 0                             // Interrupt 0 is on IN1
#define IN2Interrupt 1                             // Interrupt 1 is on IN2

// Arduino pin to timing box BNC mappings
#define OUT1 4
#define OUT2 5
#define OUT3 6
#define OUT4 7
#define PWM1 9
#define PWM2 10
#define IN1 2
#define IN2 3
#define LED 13

// Timing box BNC to variable mappings
const int cameraOutput = OUT1;
const int shutterOutput = OUT2;
const int rxOutput = OUT3;
const int inspirationInput = IN1;
const int indicator = LED;

// ----------------------------------------------------------------------------------------------

long rate = 500;                                    // Cycle rate (in msec) for free breathing (no external trigger)

// Set timing options (make sure all camera and shutter delays add up to less than the breath period, i.e. 500 msec)
int initialDelay = 425;                             // Delay to appropriate point in breath (in msec)
int shutterOpenDelay = 5;                           // Time required for shutter to open (in msec)
int cameraPulseShort = 5;                           // Short exposure length (in msec)
int cameraPulseLong = 25;                           // Long exposure length (in msec)
int cameraDelay = 25;                               // Delay between exposures (in msec, only used if imagingExposures > 1, or for flat correction)
int shutterCloseDelay = 15;                         // Delay before closing shutter (in msec)

// Set block repeat options
int imagingExposures = 1;                           // Number of camera triggers per breath
int imagingFlats = 20;
int imagingRepeats = 50;                            // Number of sequential breaths for which to repeat imaging
int imagingBlocks = 12;                             // Number of imaging blocks (should be equal to the number of elements in imagingStarts)
int imagingStarts[] = {
  0,60,120,180,240,360,480,600,840,1200,1560,1920}; // Imaging start times (in breaths; the difference between each element should be greater than repeat)

// Set treatment options
int rxDelay = 0;                                    // Rx delay to appropriate point in breath (in msec)
int rxPulse = 15;                                   // Rx pulse length (msec)
int rxRepeats = 180;                                // Number of sequential breaths for which to repeat Rx delivery in each block
int rxBlocks = 1;                                   // Number of Rx blocks (should be equal to the number of elements in rxStart)
int rxStarts[] = {
  120};                                             // Rx start times (in breaths; the difference between each element should be greater than repeat)
 
// ----------------------------------------------------------------------------------------------

// Global Variables
int ventilate = true, mode = 3, acquire = false, rx = false, cameraPulse, shutterStatus = LOW;
int imBlock, imStart, imStage = 1, rxBlock, rxStart, rxStage = 1;
int i, e, r, a;
long elapsedTime, stageTime;

volatile int breath;
volatile unsigned long inspTime;

// ----------------------------------------------------------------------------------------------

void setup()
{

  // Setup serial communication
  Serial.begin(9600);

  // Setup the Arduino pins
  pinMode(cameraOutput, OUTPUT);
  pinMode(shutterOutput, OUTPUT);
  pinMode(rxOutput, OUTPUT);
  pinMode(inspirationInput, INPUT);
  pinMode(indicator, OUTPUT);

  // Setup the interrupts 
  Timer1.initialize(rate*1000);
  if(ventilate) 
    attachInterrupt(IN1Interrupt, interrupt, RISING);
  else
    Timer1.attachInterrupt(interrupt);
}

// ----------------------------------------------------------------------------------------------

void interrupt()
{

  // Increment the breath counter on inspiration
  breath++;

  // Record the time of inspiration
  inspTime = millis(); 

}

// ----------------------------------------------------------------------------------------------

void loop()
{
  
  // ------------------------------------------------
  // Serial communication
  // ------------------------------------------------

  if (Serial.available() > 0) {

    int serialData = Serial.read();

    switch(serialData) {
      
    case 'v':
      ventilate = !ventilate;
      if(ventilate) {
        Serial.println("External trigger");
        Timer1.detachInterrupt(); 
        attachInterrupt(IN1Interrupt, interrupt, RISING);
      }
      else {
        Serial.println("Internal timing");
        detachInterrupt(IN1Interrupt);
        Timer1.attachInterrupt(interrupt);
      }
      break;
//        Timer1.setPeriod(rate*1000);  // Use this to modify the period once serial comms are written

    case 'i':
      Serial.println("Search mode on");
      cameraPulse = cameraPulseShort;
      e = 1;
      mode = 1;
      breath = -1;                  // Need to add 1 here to allow time to finish serial comms for accurate timing
      break;

    case 'r':
      Serial.println("Running Script");
      cameraPulse = cameraPulseLong;
      e = imagingExposures;
      r = imagingRepeats;
      imBlock = 0;
      imStart = imagingStarts[imBlock];
      
      a = rxRepeats;
      rxBlock = 0;
      rxStart = rxStarts[rxBlock];
      
      mode = 2;
      breath = -1;
      break;
  
   case 'a':
      Serial.println("Acquiring one block");
      cameraPulse = cameraPulseLong;
      e = imagingExposures;
      r = imagingRepeats;
      imBlock = imagingBlocks - 1;
      imStart = 0;
      
      rxBlock = rxBlocks;  // Added to ensure the aerosol/insufflation code is never executed
      
      mode = 2;
      breath = -1;
      break;
      
   case 'f':
      Serial.println("Acquiring flats / darks");
      cameraPulse = cameraPulseLong;
      e = 1;
      r = imagingFlats;
      imBlock = imagingBlocks - 1;
      imStart = 0;
      
      rxBlock = rxBlocks;  // Added to ensure the aerosol/insufflation code is never executed
      
      mode = 2;
      breath = -1;
      break;

    case 's':
      shutterStatus = !shutterStatus;
      digitalWrite(shutterOutput, shutterStatus);
      Serial.println("Shutter status toggled");
      break;
      
    case 'x':
      digitalWrite(rxOutput, LOW);
      Serial.println("Complete");
      mode = 3;
      break;
      
    case 'n':
      digitalWrite(rxOutput, HIGH);
      Serial.println("Treatment on");
      a = rxRepeats;
      rxBlock = 0;
      rxStart = breath;
      break;
      
    case 'm':
      digitalWrite(rxOutput, LOW);
      Serial.println("Treatment off");
      rxBlock = rxBlocks;
      break;

    case '0':
      rx = 0;
      Serial.println("No Rx selected");
      break;

    case '1':
      rx = 1;
      Serial.println("Rx selected");
      break;
      
    }
  }

  // ------------------------------------------------
  // Mode selection
  // ------------------------------------------------

  switch(mode)  {

  case 1: // MODE 1: ROI Search mode
    if(breath == 0) {
      breath = -1;
      acquire = true;
    }
    break;

  case 2: // MODE 2: Normal run mode
    if(imBlock < imagingBlocks) {
      if(breath == imStart) {
        acquire = true;                            // Start image acquisition state machine
        if(r > 1) {
          r--;
          imStart++;
        } 
        else {
          imBlock++;
          r = imagingRepeats;
          imStart = imagingStarts[imBlock];
        }
      }
      
      switch(rx) {
        
      case 0: // No Rx
        digitalWrite(rxOutput, LOW);
        break;
        
      case 1: // Rx
        if(rxBlock < rxBlocks) {
          if(breath == rxStart) {
            rx = true;                     // Start treatment state machine
            if(a > 1) {
              a--;
              rxStart++;
            }
            else {
              rxBlock++;
              a = rxRepeats;
              rxStart = rxStarts[rxBlock];
            }
          }
        }
        break;
           
      }
    }
    else {
      Serial.println("Complete");
      mode = 3;
    } 
    break;

  case 3: // MODE 3: Stop mode
    break;

  }

  // ------------------------------------------------
  // Image acquisition state machine
  // ------------------------------------------------

  if(acquire) {

    elapsedTime = millis() - inspTime;

    switch(imStage)  {

    case 1: // Wait until the appropriate point in the breath & open the shutter
      if(elapsedTime >= initialDelay)  {
        digitalWrite(shutterOutput, HIGH);
        stageTime = initialDelay;
        imStage = 2;
      }
      break;

    case 2: // Wait for the shutter to open & start the camera trigger
      if(elapsedTime >= shutterOpenDelay + stageTime)  {
        digitalWrite(cameraOutput, HIGH);
        digitalWrite(indicator, HIGH);
        stageTime += shutterOpenDelay;
        imStage = 3;
        i = 1;
      }
      break;

    case 3: // Wait for the camera exposure length & end the camera trigger
      if(elapsedTime >= cameraPulse + stageTime)  {
        digitalWrite(cameraOutput, LOW);
        digitalWrite(indicator, LOW);
        stageTime += cameraPulse;
        imStage = 4;
      }
      break;

    case 4: // Repeat for the required number of exposures
      if(i < e) {
        if(elapsedTime >= cameraDelay + stageTime)  {
          digitalWrite(cameraOutput, HIGH);
          digitalWrite(indicator, HIGH);
          stageTime += cameraDelay;
          imStage = 3;
          i++;
        }
      }
      else imStage = 5;
      break;

    case 5: // Wait for the shutter delay & close the shutter
      if(elapsedTime >= shutterCloseDelay + stageTime)  {
        if(!shutterStatus) digitalWrite(shutterOutput, LOW);
        imStage = 1;
        acquire = false;
      }
      break;

    }
  }
  
  // ------------------------------------------------
  // Treatment state machine
  // ------------------------------------------------

  if(rx) {

    elapsedTime = millis() - inspTime;

    switch(rxStage)  {

    case 1: // Wait until the appropriate point in the breath & activate
      if(elapsedTime >= rxDelay)  {
        digitalWrite(rxOutput, HIGH);
        rxStage = 2;
      }
      break;
        
    case 2: // Wait for the delivery time
      if(elapsedTime >= rxDelay + rxPulse)  {
        digitalWrite(rxOutput, LOW);
        rxStage = 1;
        rx = false;
      }
      break;
    }
  }
}



