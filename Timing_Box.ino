#include <TimerOne.h>

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
const int insufflatorOutput = OUT4;
const int aeronebOutput = OUT3;
const int inspirationInput = IN1;
const int indicator = LED;

// ----------------------------------------------------------------------------------------------

// Set mode to internal timer (timer1) or external trigger (ventilator pin interrupt)
const int ventilate = true;

// Set timing options (make sure all camera and shutter delays add up to less than the breath period, i.e. 500 msec)
long rate = 500000;                                 // Cycle rate (in microsec) for free breathing (no external trigger)

int initialDelay = 400;                             // Delay to appropriate point in breath (in msec)
int shutterOpenDelay = 5;                           // Time required for shutter to open (in msec)
int cameraPulseShort = 5;                           // Short exposure length (in msec)
int cameraPulseLong = 50;                           // Long exposure length (in msec)
int cameraDelay = 25;                               // Delay between exposures (in msec, only used if imagingExposures > 1, or for flat correction)
int shutterCloseDelay = 25;                         // Delay before closing shutter (in msec)

int imagingExposures = 1;                           // Number of camera triggers per breath
int imagingFlats = 20;
int imagingRepeats = 50;                            // Number of sequential breaths for which to repeat imaging
int imagingBlocks = 12;                             // Number of imaging blocks (should be equal to the number of elements in imagingStart)
int imagingStart[] = {
  0,60,120,180,240,360,480,600,840,1200,1560,1920}; // Imaging start times (in breaths; the difference between each element should be greater than repeat)

int aeronebDelay = 0;                               // Aeroneb delay to appropriate point in breath (in msec)
int aeronebPulse = 15;                              // Aeroneb pulse length (msec)
int aeronebRepeats = 180;                           // Number of sequential breaths for which to repeat aerosol delivery in each block
int aeronebBlocks = 1;                              // Number of aeroneb blocks (should be equal to the number of elements in aeronebStart)
int aeronebStart[] = {
  120};                                             // Aeroneb start times (in breaths; the difference between each element should be greater than repeat)

int insufflatorDelay = 25;                          // Insufflator delay to appropriate point in breath (in msec)
int insufflatorPulse = 225;                         // Air valve open time (in msec)
int insufflatorRepeats = 3;                         // Number of sequential breaths for which to actuate insufflator in each block
int insufflatorBlocks = 2;                          // Number of insufflator blocks (should be equal to the number of elements in insufflatorStart)
int insufflatorStart[] = {
  65,125};                                          // Insufflator start times (in breaths; the difference between each element should be greater than repeat)
 
// ----------------------------------------------------------------------------------------------

// Global Variables
int mode = 3, acquire = false, aerosolise = false, insufflate = false, Rx = 1, cameraPulse;
int imBlock, imStart, imStage = 1, anBlock, anStart, anStage = 1, dpiBlock, dpiStart, dpiStage = 1;
int i, e, r, a, d;
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
  pinMode(insufflatorOutput, OUTPUT);
  pinMode(aeronebOutput, OUTPUT);
  pinMode(inspirationInput, INPUT);
  pinMode(indicator, OUTPUT);

  // Setup the interrupts 
  if(ventilate) 
    attachInterrupt(IN1Interrupt, interrupt, RISING);
  else {
    Timer1.initialize(rate);
    Timer1.attachInterrupt(interrupt);
  }
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
      imStart = imagingStart[imBlock];
      
      a = aeronebRepeats;
      anBlock = 0;
      anStart = aeronebStart[anBlock];
      
      d = insufflatorRepeats;
      dpiBlock = 0;
      dpiStart = insufflatorStart[dpiBlock];
      
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
      
      anBlock = aeronebBlocks;  // Added to ensure the aerosol/insufflation code is never executed
      dpiBlock = insufflatorBlocks;
      
      mode = 2;
      breath = -1;
      break;
      
   case 'f':
      Serial.println("Acquiring flats / darks");
      cameraPulse = cameraPulseLong;
      e = imagingFlats;
      r = 1;
      imBlock = imagingBlocks - 1;
      imStart = 0;
      
      anBlock = aeronebBlocks;  // Added to ensure the aerosol/insufflation code is never executed
      dpiBlock = insufflatorBlocks;
      
      mode = 2;
      breath = -1;
      break;

    case 's':
      Serial.println("Complete");
      mode = 3;
      break;
      
    case 'n':
      digitalWrite(aeronebOutput, HIGH);
      Serial.println("Aeroneb on");
      a = aeronebRepeats;
      anBlock = 0;
      anStart = breath;
      break;
      
    case 'm':
      digitalWrite(aeronebOutput, LOW);
      Serial.println("Aeroneb off");
      anBlock = aeronebBlocks;
      break;

    case '0':
      Rx = 0;
      Serial.println("No Rx selected");
      break;

    case '1':
      Rx = 1;
      Serial.println("Aeroneb Rx selected");
      break;

    case '2':
      Rx = 2;
      Serial.println("Insufflator Rx selected");
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
          imStart = imagingStart[imBlock];
        }
      }
      
      switch(Rx) {
        
      case 0: // No Rx
        digitalWrite(aeronebOutput, LOW);
        digitalWrite(insufflatorOutput, LOW);
        break;
        
      case 1: // Aeroneb
        if(anBlock < aeronebBlocks) {
          if(breath == anStart) {
            aerosolise = true;                     // Start aeroneb state machine
            if(a > 1) {
              a--;
              anStart++;
            }
            else {
              anBlock++;
              a = aeronebRepeats;
              anStart = aeronebStart[anBlock];
            }
          }
        }
        break;
        
      case 2: // Dry Powder Insufflator
        if(dpiBlock < insufflatorBlocks) {
          if(breath == dpiStart) {
            insufflate = true;                     // Start aeroneb state machine
            if(d > 1) {
              d--;
              dpiStart++;
            }
            else {
              dpiBlock++;
              d = insufflatorRepeats;
              dpiStart = insufflatorStart[dpiBlock];
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
    digitalWrite(insufflatorOutput, LOW);
    digitalWrite(aeronebOutput, LOW);
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
        digitalWrite(shutterOutput, LOW);
        imStage = 1;
        acquire = false;
      }
      break;

    }
  }
  
  // ------------------------------------------------
  // Aerosol state machine
  // ------------------------------------------------

  if(aerosolise) {

    elapsedTime = millis() - inspTime;

    switch(anStage)  {

    case 1: // Wait until the appropriate point in the breath & activate aeroneb
      if(elapsedTime >= aeronebDelay)  {
        digitalWrite(aeronebOutput, HIGH);
        anStage = 2;
      }
      break;
        
    case 2: // Wait for the aeroneb delivery time
      if(elapsedTime >= aeronebDelay + aeronebPulse)  {
        digitalWrite(aeronebOutput, LOW);
        anStage = 1;
        aerosolise = false;
      }
      break;
    }
  }
    
  // ------------------------------------------------
  // Insufflator state machine
  // ------------------------------------------------

  if(insufflate) {

    elapsedTime = millis() - inspTime;

    switch(dpiStage)  {

    case 1: // Wait until the appropriate point in the breath & activate aeroneb
      if(elapsedTime >= insufflatorDelay)  {
        digitalWrite(insufflatorOutput, HIGH);
        dpiStage = 2;
      }
      break;
        
    case 2: // Wait for the insufflator actuation time
      if(elapsedTime >= insufflatorDelay + insufflatorPulse)  {
        digitalWrite(insufflatorOutput, LOW);
        dpiStage = 1;
        insufflate = false;
      }
      break;
    }
  }

}


