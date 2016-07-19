#include <TimerOne.h>

// Arduino pin to timing box BNC mappings
#define OUT1 4
#define OUT2 5
#define OUT3 6
#define OUT4 7
#define PWM1 9
#define PWM2 10
#define IN1 2                                       // Interrupt 0
#define IN2 3                                       // Interrupt 1
#define LED 13

// Timing box BNC to variable mappings
const int cameraOutput = OUT1;
const int shutterOutput = OUT2;
const int rxOutput = OUT3;                          // Set to OUT3 for Aeroneb
const int inspirationInput = IN1;
const int indicator = LED;

// ----------------------------------------------------------------------------------------------

// Set timing options (make sure all camera and shutter delays add up to less than the breath period, i.e. 500 msec)
long rate = 500;                                    // Cycle rate (in msec) for free breathing (no external trigger)
int initialDelay = 425;                             // Delay to appropriate point in breath (in msec)
int shutterOpenDelay = 5;                           // Time required for shutter to open (in msec)
int cameraPulse = 25;                               // Long exposure length (in msec)
int cameraDelay = 25;                               // Delay between exposures (in msec, only used if imagingExposures > 1, or for flat correction)
int shutterCloseDelay = 15;                         // Delay before closing shutter (in msec)

// Set block repeat options
int imagingExposures = 1;                           // Number of camera triggers per breath
int imagingFlats = 20;
int imagingRepeats = 50;                            // Number of sequential breaths for which to repeat imaging
int imagingBlocks = 12;                             // Number of imaging blocks (should be equal to the number of elements in imagingStarts)
int imagingStarts[100] = {
  0, 60, 120, 180, 240, 360, 480, 600, 840, 1200, 1560, 1920}; // Imaging start times (in breaths; the difference between each element should be greater than repeat)

// Set treatment options
int rxDelay = 0;                                    // Rx delay to appropriate point in breath (in msec)
int rxPulse = 15;                                   // Rx pulse length (msec)
int rxRepeats = 180;                                // Number of sequential breaths for which to repeat Rx delivery in each block
int rxBlocks = 1;                                   // Number of Rx blocks (should be equal to the number of elements in rxStart)
int rxStarts[100] = {
  120
};                                             // Rx start times (in breaths; the difference between each element should be greater than repeat)

// ----------------------------------------------------------------------------------------------

// Global Variables
boolean instructionComplete = false, acquire = false, rx = true, rxActive = false, shutterStatus = LOW;
int byteCount, arrayCount, serialElement = 0, serialLength = 0, serialParameters[100], serialStage = 0;
char incomingByte, outgoing[20];
int mode = 3, imBlock, imStart, imStage = 1, rxBlock, rxStart, rxStage = 1;
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
  pinMode(indicator, OUTPUT);
  pinMode(inspirationInput, INPUT_PULLUP);

  // Setup the interrupts (Default is externally triggered timing)
  Timer1.initialize(rate * 1000);
  attachInterrupt(digitalPinToInterrupt(inspirationInput), interrupt, RISING);

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

  // Serial protocol: All communication starts with the # indicator and consists of 2 or more comma separated values (all zero padded to length 4, ie $.4d formatted): length, parameter and optional values.
  //
  // Examples:
  //
  // #0001,0052               Use internal timing
  // #0001,0061               Start search mode
  // #0002,0001,1000          Set rate to 1 sec
  // #0002,0011,0400          Set initial delay to 400 msec
  // #0001,0065               Stop
  // #0003,0034,0120,0240     Set Rx start times to 120 and 240 breaths
  // #0002,0023,0005          Set number of flats to acquire to 5
  // #0001,0064               Acquire one block of flats

  // Check if the serial buffer contains data
  if (Serial.available()) {

    incomingByte = Serial.read();

    switch (serialStage)  {

      // Error state to wait for new # for instruction start
      case 0:
        byteCount = 0;
        serialLength = 0;
        serialElement = 0;
        memset(serialParameters, 0, sizeof(serialParameters));  // Reset array to zero
        if (incomingByte == 35) serialStage = 1;                // Start serialStage state machine
        break;

      // Get serialLength to determine how many extra values to read
      case 1:
        serialLength *= 10;
        serialLength += (incomingByte - 48);
        byteCount++;
        if (byteCount == 4) serialStage = 2;
        break;

      // Check for comma
      case 2:
        byteCount = 0;
        if (incomingByte == 44) serialStage = 3;
        else serialStage = 0;
        break;

      // Get the parameter
      case 3:
        serialParameters[serialElement] *= 10;
        serialParameters[serialElement] += (incomingByte - 48);
        byteCount++;
        if (byteCount == 4) {
          serialElement++;
          if (serialElement == serialLength) {
            serialStage = 0;
            instructionComplete = true;
          } else {
            serialStage = 2;
          }
        }
        break;

    }
  }

  // ------------------------------------------------
  // Instruction execution
  // ------------------------------------------------

  if (instructionComplete) {

    switch (serialParameters[0]) {

      // 0001: rate - Cycle rate (in msec) for free breathing (no external trigger)
      case 1:
        rate = serialParameters[1];
        Timer1.setPeriod(rate * 1000);
        break;

      // 0011: initialDelay - Delay to appropriate point in breath (in msec)
      case 11:
        initialDelay = serialParameters[1];
        break;

      // 0012: shutterOpenDelay - Time required for shutter to open (in msec)
      case 12:
        shutterOpenDelay = serialParameters[1];
        break;

      // 0013: cameraPulse - Exposure length (in msec)
      case 13:
        cameraPulse = serialParameters[1];
        break;

      // 0014: cameraDelay - Delay between exposures (in msec, only used if imagingExposures > 1)
      case 14:
        cameraDelay = serialParameters[1];
        break;

      // 0015: shutterCloseDelay - Delay before closing shutter (in msec)
      case 15:
        shutterCloseDelay = serialParameters[1];
        break;

      // 0021: imagingExposures - Number of camera triggers per breath
      case 21:
        imagingExposures = serialParameters[1];
        break;

      // 0022: imagingRepeats - Number of sequential breaths for which to repeat imaging
      case 22:
        imagingRepeats = serialParameters[1];
        break;

      // 0023: imagingFlats - Number of flat images to acquire
      case 23:
        imagingFlats = serialParameters[1];
        break;

      // 0024: imagingStarts[] - Imaging start times (in breaths; the difference between each element should be greater than repeat)
      case 24:
        imagingBlocks = serialLength - 1;
        for (arrayCount = 0; arrayCount < imagingBlocks; arrayCount++)  imagingStarts[arrayCount] = serialParameters[arrayCount + 1];
        break;

      // 0031: rxDelay - Rx delay to appropriate point in breath (in msec)
      case 31:
        rxDelay = serialParameters[1];
        break;

      // 0032: rxPulse - Rx pulse length (msec)
      case 32:
        rxPulse = serialParameters[1];
        break;

      // 0033: rxRepeats - Number of sequential breaths for which to repeat Rx delivery in each block
      case 33:
        rxRepeats = serialParameters[1];
        break;

      // 0034: rxStarts[] - Rx start times (in breaths; the difference between each element should be greater than repeat)
      case 34:
        rxBlocks = serialLength - 1;
        for (arrayCount = 0; arrayCount < rxBlocks; arrayCount++)  rxStarts[arrayCount] = serialParameters[arrayCount + 1];
        break;

      // 0051: External trigger - Switch to external (ie ventilator) trigger
      case 51:
        Timer1.detachInterrupt();
        attachInterrupt(digitalPinToInterrupt(inspirationInput), interrupt, RISING);
        break;

      // 0052: Internal timing - Use internal timing
      case 52:
        detachInterrupt(digitalPinToInterrupt(inspirationInput));
        Timer1.attachInterrupt(interrupt);
        break;

      // Search mode on
      case 61:
        e = 1;
        mode = 1;
        breath = -1;
        break;

      // Run script
      case 62:
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

      // Acquire one block
      case 63:
        e = imagingExposures;
        r = imagingRepeats;
        imBlock = imagingBlocks - 1;
        imStart = 0;
        rxBlock = rxBlocks;  // Added to ensure the aerosol/insufflation code is never executed
        mode = 2;
        breath = -1;
        break;

      // Acquire flats / darks
      case 64:
        e = 1;
        r = imagingFlats;
        imBlock = imagingBlocks - 1;
        imStart = 0;
        rxBlock = rxBlocks;  // Added to ensure the aerosol/insufflation code is never executed
        mode = 2;
        breath = -1;
        break;

      // Stop all
      case 65:
        digitalWrite(rxOutput, LOW);
        mode = 3;
        break;

      // 0071: Force shutter open - Keep the shutter open
      case 71: // Force shutter open
        shutterStatus = HIGH;
        digitalWrite(shutterOutput, shutterStatus);
        break;

      // 0072: Normal shuttering - Return to normal shutter operation
      case 72:
        shutterStatus = LOW;
        digitalWrite(shutterOutput, shutterStatus);
        break;

      // 0073: Treatment on manual
      case 73:
        digitalWrite(rxOutput, HIGH);
        a = rxRepeats;
        rxBlock = 0;
        rxStart = breath;
        break;

      // 0074: Treatment off manual
      case 74:
        digitalWrite(rxOutput, LOW);
        rxBlock = rxBlocks;
        break;

      // 0075: Schedule treatment to deliver on script run
      case 75:
        rx = true;
        break;

      // 0076: Schedule treatment to NOT deliver on script run
      case 76:
        rx = false;
        break;

      // 0099: Do nothing: Use to check box active / run complete
      case 99:
        break;

    }

    sprintf(outgoing, "#%.4d", serialParameters[0]);
    Serial.println(outgoing);

    instructionComplete = false;

  }

  // ------------------------------------------------
  // Mode selection
  // ------------------------------------------------

  switch (mode)  {

    // MODE 1: ROI Search mode
    case 1:
      if (breath == 0) {
        breath = -1;
        acquire = true;
      }
      break;

    // MODE 2: Normal run mode
    case 2:
      if (imBlock < imagingBlocks) {
        if (breath == imStart) {
          acquire = true;                            // Start image acquisition state machine
          if (r > 1) {
            r--;
            imStart++;
          }
          else {
            imBlock++;
            r = imagingRepeats;
            imStart = imagingStarts[imBlock];
          }
        }

        if (rx) {
          if (rxBlock < rxBlocks) {
            if (breath == rxStart) {
              rxActive = true;                       // Start treatment state machine
              if (a > 1) {
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
        }
        else digitalWrite(rxOutput, LOW);
      }
      else {
        sprintf(outgoing, "#99");
        Serial.println(outgoing);
        mode = 3;
      }
      break;

    // MODE 3: Stop mode
    case 3:
      break;

  }

  // ------------------------------------------------
  // Image acquisition state machine
  // ------------------------------------------------

  if (acquire) {

    elapsedTime = millis() - inspTime;

    switch (imStage)  {

      // Wait until the appropriate point in the breath & open the shutter
      case 1:
        if (elapsedTime >= initialDelay)  {
          digitalWrite(shutterOutput, HIGH);
          stageTime = initialDelay;
          imStage = 2;
        }
        break;

      // Wait for the shutter to open & start the camera trigger
      case 2:
        if (elapsedTime >= shutterOpenDelay + stageTime)  {
          digitalWrite(cameraOutput, HIGH);
          digitalWrite(indicator, HIGH);
          stageTime += shutterOpenDelay;
          imStage = 3;
          i = 1;
        }
        break;

      // Wait for the camera exposure length & end the camera trigger
      case 3:
        if (elapsedTime >= cameraPulse + stageTime)  {
          digitalWrite(cameraOutput, LOW);
          digitalWrite(indicator, LOW);
          stageTime += cameraPulse;
          imStage = 4;
        }
        break;

      // Repeat for the required number of exposures
      case 4:
        if (i < e) {
          if (elapsedTime >= cameraDelay + stageTime)  {
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
        if (elapsedTime >= shutterCloseDelay + stageTime)  {
          if (!shutterStatus) digitalWrite(shutterOutput, LOW);
          imStage = 1;
          acquire = false;
        }
        break;

    }
  }

  // ------------------------------------------------
  // Treatment state machine
  // ------------------------------------------------

  if (rxActive) {

    elapsedTime = millis() - inspTime;

    switch (rxStage)  {

      // Wait until the appropriate point in the breath & activate
      case 1:
        if (elapsedTime >= rxDelay)  {
          digitalWrite(rxOutput, HIGH);
          rxStage = 2;
        }
        break;

      // Wait for the delivery time
      case 2:
        if (elapsedTime >= rxDelay + rxPulse)  {
          digitalWrite(rxOutput, LOW);
          rxStage = 1;
          rxActive = false;
        }
        break;
    }
  }
}
