#include <TimerOne.h>

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
boolean instructionComplete = false, acquire = false, rx = false, shutterStatus = LOW;
int count, serialStage, command = 0, parameter = 0, value = 0;
char incomingByte, outgoing[20];
int mode = 3, cameraPulse, imBlock, imStart, imStage = 1, rxBlock, rxStart, rxStage = 1;
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

  // Setup the interrupts (Default is externally triggered timing)
  Timer1.initialize(rate*1000);
  attachInterrupt(IN1Interrupt, interrupt, RISING);
  //  Timer1.attachInterrupt(interrupt);
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

  // Check if the serial buffer contains data
  if(Serial.available()) {

    incomingByte = Serial.read();

    // Reset instruction code if # received: Indicates communication start
    if(incomingByte == 35)  {

      command = 0;         // Throw away previous command
      parameter = 0;
      value = 0;
      count = 0;
      serialStage = 1;     // Reset serialStage state machine

    } 

    else {

      switch(serialStage)  {

      case 1: // Get command
        command = incomingByte - 48;
        serialStage = 2;
        break;

      case 2: // Check for comma
        if(incomingByte == 44) 
          serialStage = 3;
        else {
          sprintf(outgoing,"#%.2d,%.4d", 0, 0);
          Serial.println(outgoing);
          serialStage = 0; // Add error notification
        }
        break;

      case 3: // Get parameter
        parameter *= 10;
        parameter = ((incomingByte - 48) + parameter);
        count++;
        if(count == 2) {
          count = 0;
          serialStage = 4;
        }
        break;

      case 4: // Check for comma
        if(incomingByte == 44) 
          serialStage = 5;
        else {
          sprintf(outgoing,"#%.2d,%.4d", 0, 0);
          Serial.println(outgoing);
          serialStage = 0; // Add error notification
        }
        break;

      case 5: // Get value
        value *= 10;
        value = ((incomingByte - 48) + value);
        count++;
        if(count == 4) {
          serialStage = 0;
          instructionComplete = true;
        }
        break;

      }
    }
  }
  
  // ------------------------------------------------
  // Instruction execution
  // ------------------------------------------------

  if(instructionComplete) {

    // 1: Set parameter
    // 2: Get parameter
    // 3: Control timing box

    switch(command)  {

      // 1: rate                     Cycle rate (in msec) for free breathing (no external trigger)

      // 11: initialDelay            Delay to appropriate po// in breath (in msec)
      // 12: shutterOpenDelay        Time required for shutter to open (in msec)
      // 13: cameraPulseShort        Short exposure length (in msec)
      // 14: cameraPulseLong         Long exposure length (in msec)
      // 15: cameraDelay             Delay between exposures (in msec, only used if imagingExposures > 1, or for flat correction)
      // 16: shutterCloseDelay       Delay before closing shutter (in msec)

      // 21: imagingExposures        Number of camera triggers per breath
      // 22: imagingFlats            Number of flat images to acquire
      // 23: imagingRepeats          Number of sequential breaths for which to repeat imaging
      // 24: imagingBlocks           Number of imaging blocks (should be equal to the number of elements in imagingStarts)
      // 25: imagingStarts[]         Imaging start times (in breaths; the difference between each element should be greater than repeat)

      // 31: rxDelay                 Rx delay to appropriate po// in breath (in msec)
      // 32: rxPulse                 Rx pulse length (msec)
      // 33: rxRepeats               Number of sequential breaths for which to repeat Rx delivery in each block
      // 34: rxBlocks                Number of Rx blocks (should be equal to the number of elements in rxStart)
      // 35: rxStarts[]              Rx start times (in breaths; the difference between each element should be greater than repeat)

    case 1: // Set a parameter

      switch(parameter) {

      case 1:
        rate = value;
        Timer1.setPeriod(rate*1000);
        break;

      case 11:
        initialDelay = value;
        break;

      case 12:
        shutterOpenDelay = value;
        break;

      case 13:
        cameraPulseShort = value;
        break;

      case 14:
        cameraPulseLong = value;
        break;

      case 15:
        cameraDelay = value;
        break;

      case 16:
        shutterCloseDelay = value;
        break;

      case 21:
        imagingExposures = value;
        break;

      case 22:
        imagingFlats = value;
        break;

      case 23:
        imagingRepeats = value;
        break;

      case 31:
        rxDelay = value;
        break;

      case 32:
        rxPulse = value;
        break;

      case 33:
        rxRepeats = value;
        break;

      }

    case 2: // Get a parameter

      switch(parameter) {

      case 1:
        sprintf(outgoing,"#%.2d,%.4d", parameter, rate);
        break;

      case 11:
        sprintf(outgoing,"#%.2d,%.4d", parameter, initialDelay);
        break;

      case 12:
        sprintf(outgoing,"#%.2d,%.4d", parameter, shutterOpenDelay);
        break;

      case 13:
        sprintf(outgoing,"#%.2d,%.4d", parameter, cameraPulseShort);
        break;

      case 14:
        sprintf(outgoing,"#%.2d,%.4d", parameter, cameraPulseLong);
        break;

      case 15:
        sprintf(outgoing,"#%.2d,%.4d", parameter, cameraDelay);
        break;

      case 16:
        sprintf(outgoing,"#%.2d,%.4d", parameter, shutterCloseDelay);
        break;

      case 21:
        sprintf(outgoing,"#%.2d,%.4d", parameter, imagingExposures);
        break;

      case 22:
        sprintf(outgoing,"#%.2d,%.4d", parameter, imagingFlats);
        break;

      case 23:
        sprintf(outgoing,"#%.2d,%.4d", parameter, imagingRepeats);
        break;

      case 31:
        sprintf(outgoing,"#%.2d,%.4d", parameter, rxDelay);
        break;

      case 32:
        sprintf(outgoing,"#%.2d,%.4d", parameter, rxPulse);
        break;

      case 33:
        sprintf(outgoing,"#%.2d,%.4d", parameter, rxRepeats);
        break;

      default:
        sprintf(outgoing,"#%.2d,%.4d", 0, 0);

      }
      Serial.println(outgoing);
      break;

      // 51: External trigger
      // 52: Internal timing

      // 61: Search mode on
      // 62: Run script
      // 63: Acquire one block
      // 64: Acquire flats / darks
      // 65: Stop all

      // 71: Force shutter open
      // 72: Normal shuttering
      // 73: Treatment on manual
      // 74: Treatment off manual
      // 75: Schedule treatment to deliver on script run
      // 76: Schedule treatment to NOT deliver on script run

    case 3: // Control the timing box

      switch(parameter) {

      case 51: // External trigger
        Timer1.detachInterrupt(); 
        attachInterrupt(IN1Interrupt, interrupt, RISING);
        break;

      case 52: // Internal timing
        detachInterrupt(IN1Interrupt);
        Timer1.attachInterrupt(interrupt);
        break;

      case 61: // Search mode on
        cameraPulse = cameraPulseShort;
        e = 1;
        mode = 1;
        breath = -1;                  // Need to add 1 here to allow time to finish serial comms for accurate timing
        sprintf(outgoing,"#%.2d,%.4d", parameter, 0);
        break;

      case 62: // Run script
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

      case 63: // Acquire one block
        cameraPulse = cameraPulseLong;
        e = imagingExposures;
        r = imagingRepeats;
        imBlock = imagingBlocks - 1;
        imStart = 0;

        rxBlock = rxBlocks;  // Added to ensure the aerosol/insufflation code is never executed

        mode = 2;
        breath = -1;
        break;

      case 64: // Acquire flats / darks
        cameraPulse = cameraPulseLong;
        e = 1;
        r = imagingFlats;
        imBlock = imagingBlocks - 1;
        imStart = 0;

        rxBlock = rxBlocks;  // Added to ensure the aerosol/insufflation code is never executed

        mode = 2;
        breath = -1;
        break;

      case 65: // Stop all
        digitalWrite(rxOutput, LOW);
        mode = 3;
        break;

      case 71: // Force shutter open
        shutterStatus = HIGH;
        digitalWrite(shutterOutput, shutterStatus);
        break;

      case 72: // Normal shuttering
        shutterStatus = LOW;
        digitalWrite(shutterOutput, shutterStatus);
        break;

      case 73: // Treatment on manual
        digitalWrite(rxOutput, HIGH);
        a = rxRepeats;
        rxBlock = 0;
        rxStart = breath;
        break;

      case 74: // Treatment off manual
        digitalWrite(rxOutput, LOW);
        rxBlock = rxBlocks;
        break;

      case 75: // Schedule treatment to deliver on script run
        rx = false;
        break;

      case 76: // Schedule treatment to NOT deliver on script run
        rx = true;
        break;

      }

      sprintf(outgoing,"#%.2d,%.4d", parameter, 0);
      Serial.println(outgoing);
      break;

    default:
      sprintf(outgoing,"#%.2d,%.4d", 0, 0);
      Serial.println(outgoing);

    }

    instructionComplete = false;

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





