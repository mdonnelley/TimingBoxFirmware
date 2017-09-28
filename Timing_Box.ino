#include <TimerOne.h>

#define TimingBoxVersion 133

// Arduino pin to timing box BNC mappings
#define OUT1 4
#define OUT2 5
#define OUT3 6
#define OUT4 7
#define PWM1 9
#define PWM2 10
#define IN1 2                                       // Interrupt 0
#define IN2 3                                       // Interrupt 1

// Timing box BNC to variable mappings
const int cameraOutput = OUT1;                      // Camera output
const int shutterOutput = OUT2;                     // Shutter output
const int rx1Output = OUT3;                         // Rx1 output (i.e. Aeroneb)
const int rx2Output = OUT4;                         // Rx2 output (i.e. Pneumatic valve)
const int inspirationInput = IN1;                   // Inspiration input from ventilator
const int indicator = LED_BUILTIN;                  // LED indicator

// ----------------------------------------------------------------------------------------------

#define BAUD 9600
#define CHECK_CONNECTION 0
#define INTERNAL_TRIGGER 10
#define FORCE_SHUTTER_OPEN 11
#define RX1_MANUAL 12
#define RX1_ACTIVE 13
#define RX2_MANUAL 14
#define RX2_ACTIVE 15
#define RATE 20
#define INITIAL_DELAY 21
#define SHUTTER_OPEN 22
#define CAMERA 23
#define CAMERA_DELAY 24
#define SHUTTER_CLOSE 25
#define IMAGING_EXPOSURES 26
#define IMAGING_REPEATS 27
#define IMAGING_FLATS 28
#define RX1_DELAY 29
#define RX1_PULSE 30
#define RX1_REPEATS 31
#define RX2_DELAY 32
#define RX2_PULSE 33
#define RX2_REPEATS 34
#define IMAGING_STARTS 40
#define RX1_STARTS 41
#define RX2_STARTS 42
#define SHUTTER_MODE 45
#define SEARCH 50
#define ONE_SHOT 51
#define ACQUIRE_ONE 52
#define RUN 53
#define ACQUIRE_FLATS 54
#define STOP 55

// ----------------------------------------------------------------------------------------------

// Set timing options (make sure all camera and shutter delays add up to less than the breath period, i.e. 500 msec)
long rate = 500;                                    // Cycle rate (in msec) for free breathing (no external trigger)
int shutterMode = 0;                                // Set shutter mode to breath (default), image, or block
int initialDelay = 425;                             // Delay to appropriate point in breath (in msec)
int shutterOpenDelay = 5;                           // Time required for shutter to open (in msec)
int cameraPulse = 25;                               // Long exposure length (in msec)
int cameraDelay = 25;                               // Delay between exposures (in msec, only used if imagingExposures > 1, or for flat correction)
int shutterCloseDelay = 15;                         // Delay before closing shutter (in msec)                              

// Set block repeat options
int imagingExposures = 1;                           // Number of camera triggers per breath
int imagingFlats = 20;                              // Number of flats to acquire
int imagingRepeats = 50;                            // Number of sequential breaths for which to repeat imaging
int imagingBlocks = 12;                             // Number of imaging blocks (should be equal to the number of elements in imagingStarts)
int imagingStarts[100] = {
  0, 60, 120, 180, 240, 360, 480, 600, 840, 1200, 1560, 1920
};                                                  // Imaging start times (in breaths; the difference between each element should be greater than repeat)

// Set treatment options
int rx1Delay = 0, rx2Delay = 0;                     // Rx delay to appropriate point in breath (in msec)
int rx1Pulse = 15, rx2Pulse = 50;                   // Rx pulse length (msec)
int rx1Repeats = 180, rx2Repeats = 1;               // Number of sequential breaths for which to repeat Rx delivery in each block
int rx1Blocks = 1, rx2Blocks = 1;                   // Number of Rx blocks (should be equal to the number of elements in rx1Start)
int rx1Starts[100] = {120}, rx2Starts[100] = {10};  // Rx start times (in breaths; the difference between each element should be greater than repeat)

// ----------------------------------------------------------------------------------------------

// Timing variables
volatile int breath;
volatile unsigned long inspTime;
long elapsedTime;

// Serial communication & instruction execution variables
char incomingByte;
int byteCount, arrayCount, serialElement = 0, serialLength = 0, serialParameters[100], serialStage = 0;
boolean instructionComplete = false;

// Image acquisition state machine variables
boolean acquire = false, shutterStatus = LOW;
int imBlock, imStart, imStage = 1;
int mode = 3, i, e, r, a1, a2, stageTime;

// Treatment state machine variables
boolean rx1Deliver = true, rx1Active = false, rx2Deliver = true, rx2Active = false;
int rx1Block, rx1Start, rx1Stage = 1, rx2Block, rx2Start, rx2Stage = 1;

// ----------------------------------------------------------------------------------------------

void setup()
{

  // Setup serial communication; 8 data bits, no parity, 1 stop bit
  Serial.begin(BAUD,SERIAL_8N1);

  // Setup the Arduino pins
  pinMode(cameraOutput, OUTPUT);
  pinMode(shutterOutput, OUTPUT);
  pinMode(rx1Output, OUTPUT);
  pinMode(rx2Output, OUTPUT);
  pinMode(indicator, OUTPUT);
  pinMode(inspirationInput, INPUT_PULLUP);

  // Setup the interrupts (Default is externally triggered timing)
  Timer1.initialize(rate * 1000);
  attachInterrupt(digitalPinToInterrupt(inspirationInput), interrupt, RISING);

}

// ----------------------------------------------------------------------------------------------
 // Record the time of inspiration and increment the breath counter
 
void interrupt()
{
  inspTime = millis();
  breath++;
}

// ----------------------------------------------------------------------------------------------
// Send a serial command

void sendCommand(int parameter)
{
  char outgoing[20];
  sprintf(outgoing, "<0001,%.4d", parameter);
  Serial.println(outgoing);
}

// ----------------------------------------------------------------------------------------------
// Send a serial parameter

void sendParameter(int parameter, int value)
{
  char outgoing[20];
  sprintf(outgoing, "<0002,%.4d,%.4d", parameter, value);
  Serial.println(outgoing);
}

// ----------------------------------------------------------------------------------------------
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
  
void checkSerial()
{
  incomingByte = Serial.read();

  switch (serialStage)  {

    // Error state to wait for new > for instruction start
    case 0:
      byteCount = 0;
      serialLength = 0;
      serialElement = 0;
      memset(serialParameters, 0, sizeof(serialParameters));  // Reset array to zero
      if (incomingByte == 62) serialStage = 1;                // If > received then start serialStage state machine
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

// ----------------------------------------------------------------------------------------------
// Decode an incoming serial instruction

void decodeInstruction()
{
  switch (serialParameters[0]) {

    // Do nothing - Use to check serial connection successful
    case CHECK_CONNECTION:
      break;

    // --------------- CHECKBOXES ---------------

    // Switch between internal and external trigger
    case INTERNAL_TRIGGER:
      if (serialParameters[1]) {
        detachInterrupt(digitalPinToInterrupt(inspirationInput));
        Timer1.attachInterrupt(interrupt);
      }
      else {
        Timer1.detachInterrupt();
        attachInterrupt(digitalPinToInterrupt(inspirationInput), interrupt, RISING);
      }
      break;

    // Force shutter open / normal shuttering
    case FORCE_SHUTTER_OPEN:
      if (serialParameters[1]) {
        shutterStatus = HIGH;
        digitalWrite(shutterOutput, shutterStatus);
      }
      else {
        shutterStatus = LOW;
        digitalWrite(shutterOutput, shutterStatus);
      }
      break;

    // Rx1 on/off manual
    case RX1_MANUAL:
      if (serialParameters[1]) {
        digitalWrite(rx1Output, HIGH);
        a1 = rx1Repeats;
        rx1Block = 0;
        rx1Start = breath;
      }
      else {
        digitalWrite(rx1Output, LOW);
        rx1Block = rx1Blocks;
      }
      break;

    // Schedule Rx1 to deliver on script run
    case RX1_ACTIVE:
      if (serialParameters[1]) rx1Deliver = true;
      else rx1Deliver = false;
      break;

    // Rx2 on/off manual
    case RX2_MANUAL:
      if (serialParameters[1]) {
        digitalWrite(rx2Output, HIGH);
        a2 = rx2Repeats;
        rx2Block = 0;
        rx2Start = breath;
      }
      else {
        digitalWrite(rx2Output, LOW);
        rx2Block = rx2Blocks;
      }
      break;

    // Schedule Rx2 to deliver on script run
    case RX2_ACTIVE:
      if (serialParameters[1]) rx2Deliver = true;
      else rx2Deliver = false;
      break;

    // --------------- NUMERIC UP/DOWN BOXES ---------------

    // Cycle rate (in msec) for free breathing (no external trigger)
    // Note that the maximum cycle length is 8388.608mS (8.3 seconds)
    case RATE:
      rate = serialParameters[1];
      Timer1.setPeriod(rate * 1000);
      break;

    // Delay to appropriate point in breath (in msec)
    case INITIAL_DELAY:
      initialDelay = serialParameters[1];
      break;

    // Time required for shutter to open (in msec)
    case SHUTTER_OPEN:
      shutterOpenDelay = serialParameters[1];
      break;

    // Exposure length (in msec)
    case CAMERA:
      cameraPulse = serialParameters[1];
      break;

    // Delay between exposures (in msec, only used if imagingExposures > 1)
    case CAMERA_DELAY:
      cameraDelay = serialParameters[1];
      break;

    // Delay before closing shutter (in msec)
    case SHUTTER_CLOSE:
      shutterCloseDelay = serialParameters[1];
      break;

    // Number of camera triggers per breath
    case IMAGING_EXPOSURES:
      imagingExposures = serialParameters[1];
      break;

    // Number of sequential breaths for which to repeat imaging
    case IMAGING_REPEATS:
      imagingRepeats = serialParameters[1];
      break;

    // Number of flat images to acquire
    case IMAGING_FLATS:
      imagingFlats = serialParameters[1];
      break;

    // Rx1 delay to appropriate point in breath (in msec)
    case RX1_DELAY:
      rx1Delay = serialParameters[1];
      break;

    // Rx1 pulse length (msec)
    case RX1_PULSE:
      rx1Pulse = serialParameters[1];
      break;

    // Number of sequential breaths for which to repeat Rx1 delivery in each block
    case RX1_REPEATS:
      rx1Repeats = serialParameters[1];
      break;

    // Rx2 delay to appropriate point in breath (in msec)
    case RX2_DELAY:
      rx2Delay = serialParameters[1];
      break;

    // Rx2 pulse length (msec)
    case RX2_PULSE:
      rx2Pulse = serialParameters[1];
      break;

    // Number of sequential breaths for which to repeat Rx2 delivery in each block
    case RX2_REPEATS:
      rx2Repeats = serialParameters[1];
      break;

    // --------------- TEXT BOXES ---------------

    // Imaging start times (in breaths; the difference between each element should be greater than repeat)
    case IMAGING_STARTS:
      imagingBlocks = serialLength - 1;
      for (arrayCount = 0; arrayCount < imagingBlocks; arrayCount++)  imagingStarts[arrayCount] = serialParameters[arrayCount + 1];
      break;

    // Rx start times (in breaths; the difference between each element should be greater than repeat)
    case RX1_STARTS:
      rx1Blocks = serialLength - 1;
      for (arrayCount = 0; arrayCount < rx1Blocks; arrayCount++)  rx1Starts[arrayCount] = serialParameters[arrayCount + 1];
      break;

    // Rx start times (in breaths; the difference between each element should be greater than repeat)
    case RX2_STARTS:
      rx2Blocks = serialLength - 1;
      for (arrayCount = 0; arrayCount < rx2Blocks; arrayCount++)  rx2Starts[arrayCount] = serialParameters[arrayCount + 1];
      break;

    // --------------- COMBO BOXES ---------------

    // Set shutter mode to breath (default), image or block
    case SHUTTER_MODE:
      shutterMode = serialParameters[1];
      break;

    // --------------- BUTTONS ---------------

    // Search mode
    case SEARCH:
      e = 1;
      mode = 1;
      breath = -1;
      break;

    // One shot
    case ONE_SHOT:
      e = 1;
      r = 1;
      imBlock = imagingBlocks - 1;
      imStart = 0;
      rx1Block = rx1Blocks;  // Added to ensure the aerosol/insufflation code is never executed
      rx2Block = rx2Blocks;  // Added to ensure the aerosol/insufflation code is never executed
      mode = 2;
      breath = -1;
      break;

    // One block
    case ACQUIRE_ONE:
      e = imagingExposures;
      r = imagingRepeats;
      imBlock = imagingBlocks - 1;
      imStart = 0;
      rx1Block = rx1Blocks;
      rx2Block = rx2Blocks;
      mode = 2;
      breath = -1;
      break;

    // Run script
    case RUN:
      e = imagingExposures;
      r = imagingRepeats;
      imBlock = 0;
      imStart = imagingStarts[imBlock];
      a1 = rx1Repeats;
      a2 = rx2Repeats;
      rx1Block = 0;
      rx1Start = rx1Starts[rx1Block];
      rx2Block = 0;
      rx2Start = rx2Starts[rx2Block];
      mode = 2;
      breath = -1;
      break;

    // Acquire flats / darks
    case ACQUIRE_FLATS:
      e = 1;
      r = imagingFlats;
      imBlock = imagingBlocks - 1;
      imStart = 0;
      rx1Block = rx1Blocks;
      rx2Block = rx2Blocks;
      mode = 2;
      breath = -1;
      break;

    // Stop all
    case STOP:
      if (!shutterStatus) digitalWrite(shutterOutput, LOW);
      digitalWrite(rx1Output, LOW);
      digitalWrite(rx2Output, LOW);
      digitalWrite(cameraOutput, LOW);
      digitalWrite(indicator, LOW);
      imStage = 1;
      acquire = false;
      mode = 3;
      break;

  }
  sendCommand(serialParameters[0]);
  instructionComplete = false;
}

// ----------------------------------------------------------------------------------------------
// Select the mode

void selectMode()
{
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

        if (rx1Deliver) {
          if (rx1Block < rx1Blocks) {
            if (breath == rx1Start) {
              rx1Active = true;                       // Start Rx1 state machine
              if (a1 > 1) {
                a1--;
                rx1Start++;
              }
              else {
                rx1Block++;
                a1 = rx1Repeats;
                rx1Start = rx1Starts[rx1Block];
              }
            }
          }
        }
        else digitalWrite(rx1Output, LOW);

        if (rx2Deliver) {
          if (rx2Block < rx2Blocks) {
            if (breath == rx2Start) {
              rx2Active = true;                       // Start Rx2 state machine
              if (a2 > 1) {
                a2--;
                rx2Start++;
              }
              else {
                rx2Block++;
                a2 = rx2Repeats;
                rx2Start = rx2Starts[rx2Block];
              }
            }
          }
        }
        else digitalWrite(rx2Output, LOW);
      }
      else {
        sendCommand(55);
        mode = 3;
      }
      break;

    // MODE 3: Stop mode
    case 3:
      break;
  }
}

// ----------------------------------------------------------------------------------------------
// Perform image acquisition

void imageAcquisitionSM()
{
  elapsedTime = millis() - inspTime;

  // Shutter open for breath
  if(mode == 1 || shutterMode == 0)  {

    switch (imStage)  {

      // Wait until the appropriate point in the breath & open the shutter
      case 1:
        if (elapsedTime >= initialDelay)  {
          digitalWrite(shutterOutput, HIGH);          
          stageTime = initialDelay;
          i = 1;
          imStage = 2;
        }
        break;

      // Wait for the shutter to open & start the camera trigger
      case 2:
        if (elapsedTime >= stageTime + shutterOpenDelay)  {
          digitalWrite(cameraOutput, HIGH);
          digitalWrite(indicator, HIGH);
          stageTime += shutterOpenDelay;
          imStage = 3;
        }
        break;

      // Wait for the camera exposure length & end the camera trigger
      case 3:
        if (elapsedTime >= stageTime + cameraPulse)  {
          digitalWrite(cameraOutput, LOW);
          digitalWrite(indicator, LOW);
          stageTime += cameraPulse;
          imStage = 4;
        }
        break;

      // Repeat for the required number of exposures
      case 4:
        if (i < e) {
          if (elapsedTime >= stageTime + cameraDelay)  {
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
        if (elapsedTime >= stageTime + shutterCloseDelay)  {
          if (!shutterStatus) {
            digitalWrite(shutterOutput, LOW);
          }
          imStage = 1;
          acquire = false;
        }
        break;

    }
  }

  // Shutter open for image
  else if(mode != 1 && shutterMode == 1) {

    switch (imStage)  {

      // Wait until the appropriate point in the breath & open the shutter
      case 1:
        if (elapsedTime >= initialDelay)  {
          digitalWrite(shutterOutput, HIGH);
          stageTime = initialDelay;
          i = 1;
          imStage = 2;
        }
        break;

      // Wait for the shutter to open & start the camera trigger
      case 2:
        if (elapsedTime >= stageTime + shutterOpenDelay)  {
          digitalWrite(cameraOutput, HIGH);
          digitalWrite(indicator, HIGH);
          stageTime += shutterOpenDelay;
          imStage = 3;
        }
        break;

      // Wait for the camera exposure length & end the camera trigger
      case 3:
        if (elapsedTime >= stageTime + cameraPulse)  {
          digitalWrite(cameraOutput, LOW);
          digitalWrite(indicator, LOW);
          stageTime += cameraPulse;
          imStage = 4;
        }
        break;

      case 4: // Wait for the shutter delay & close the shutter
        if (elapsedTime >= stageTime + shutterCloseDelay)  {
          if (!shutterStatus) {
            digitalWrite(shutterOutput, LOW);
            Serial.println("#0002,0080,0000");
          }
          stageTime += shutterCloseDelay;
          imStage = 5;
        }
      break;

      // Repeat for the required number of exposures
      case 5:
        if (i < e) {
          if (elapsedTime >= stageTime + cameraDelay)  {
            digitalWrite(shutterOutput, HIGH);
            stageTime += cameraDelay;
            imStage = 2;
            i++;
          }
        }
        else {
          imStage = 1;
          acquire = false;
        }
        break;

    }
  }

  // Shutter open for block
  else if(mode != 1 && shutterMode == 2) {

    switch (imStage)  {

      // Open the shutter
      case 1:
        digitalWrite(shutterOutput, HIGH);
        i = 1;
        imStage = 2;
        break;

      // Wait until the appropriate point in the breath & start the camera trigger
      case 2:
        if (elapsedTime >= initialDelay)  {
          digitalWrite(cameraOutput, HIGH);
          digitalWrite(indicator, HIGH);
          stageTime = initialDelay;
          imStage = 3;
        }
        break;

      // Wait for the camera exposure length & end the camera trigger
      case 3:
        if (elapsedTime >= stageTime + cameraPulse)  {
          digitalWrite(cameraOutput, LOW);
          digitalWrite(indicator, LOW);
          stageTime += cameraPulse;
          imStage = 4;
        }
        break;

      // Repeat for the required number of exposures
      case 4:
        if (i < e) {
          if (elapsedTime >= stageTime + cameraDelay)  {
            digitalWrite(cameraOutput, HIGH);
            digitalWrite(indicator, HIGH);
            stageTime += cameraDelay;
            imStage = 3;
            i++;
          }
        }
        else {
          if (r == imagingRepeats && !shutterStatus)  {
            digitalWrite(shutterOutput, LOW);
            Serial.println("#0002,0080,0000");
          }
          imStage = 1;
          acquire = false;
        }
        break;
          
    }
  }
}

// ----------------------------------------------------------------------------------------------
// Run treatment output 1

void rx1SM()
{
    elapsedTime = millis() - inspTime;
  
  switch (rx1Stage)  {
  
    // Wait until the appropriate point in the breath & activate
    case 1:
      if (elapsedTime >= rx1Delay)  {
        digitalWrite(rx1Output, HIGH);
        rx1Stage = 2;
      }
      break;
  
    // Wait for the delivery time
    case 2:
      if (elapsedTime >= rx1Delay + rx1Pulse)  {
        digitalWrite(rx1Output, LOW);
        rx1Stage = 1;
        rx1Active = false;
      }
      break;
  }
}

// ----------------------------------------------------------------------------------------------
// Run treatment output 2

void rx2SM()
{
  elapsedTime = millis() - inspTime;

  switch (rx2Stage)  {

    // Wait until the appropriate point in the breath & activate
    case 1:
      if (elapsedTime >= rx2Delay)  {
        digitalWrite(rx2Output, HIGH);  
        rx2Stage = 2;
      }
      break;

    // Wait for the delivery time
    case 2:
      if (elapsedTime >= rx2Delay + rx2Pulse)  {
        digitalWrite(rx2Output, LOW);
        rx2Stage = 1;
        rx2Active = false;
      }
      break;
  }
}

// ----------------------------------------------------------------------------------------------
// Main function

void loop()
{
  if (Serial.available()) checkSerial();
  if (instructionComplete) decodeInstruction();
  selectMode();
  if (acquire) imageAcquisitionSM();
  if (rx1Active) rx1SM();
  if (rx2Active) rx2SM();
}
