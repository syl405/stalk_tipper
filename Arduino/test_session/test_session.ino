#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

//define pins and ports for rotary encoder
#define ENC_A 14
#define ENC_B 15
#define ENC_PORT PINC


Adafruit_ADS1115 ads1115; //instantiate ADS object using default address 0x48
Adafruit_7segment sevenSeg = Adafruit_7segment(); // instantiate 7 segment display object

//constants to set pin numbers
const int readyLedPin = 4;
const int testStatusLedPin = 3;
const int testStartStopButtonPin = 2;
const int encoderButtonPin = 5;
const int encoderGreenLedPin = 11;
const int encoderRedLedPin = 12;

//constants to set preload and termination criterion
const int preload = 240; //magnitude change in ADC code corresponding to 2N preload (120 codes per N)
const float dropTerm = 0.3; //percentage change from max load to terminate test at

//16-bit variables for ADC outputs (force to 16-bits, equivalent to normal integer declaration on 16-bit boards)
int16_t loadCellVal = 0;
int16_t potVal = 0;

//general variables
boolean testUnderway = false;
boolean preloadAttained = false;
int zeroCode;
int testStartStopButtonState = 0;
int lastTestStartStopButtonState = 0;
int testId = -1;
unsigned int height = 0; //height of force applicator above ground in mm
int maxLoad; //ADC code corresponding to max load seen in current test

void setup() {
  Serial.begin(115200); //initialise hardware serial port
  ads1115.begin(); //initialise ADS object at 0x48
  sevenSeg.begin(0x70); //initialise 7 segment display object at 0x70
  sevenSeg.println(height);
  sevenSeg.writeDisplay();
  pinMode(readyLedPin, OUTPUT);
  pinMode(testStatusLedPin, OUTPUT);
  pinMode(testStartStopButtonPin, INPUT);
  pinMode(encoderButtonPin, INPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(encoderGreenLedPin, OUTPUT);
  pinMode(encoderRedLedPin, OUTPUT);
  
  //pullup encoder input pins and encoder LED pins (to turn them off, common anode)
  digitalWrite(ENC_A, HIGH);
  digitalWrite(ENC_B, HIGH);
  digitalWrite(encoderGreenLedPin, HIGH);
  digitalWrite(encoderRedLedPin, HIGH);
  //============================================================================
  //SEND "WAITING" SIGNAL REPEATEDLY WHILE LISTENING FOR READY SIGNAL FROM RASPI
  //============================================================================
  waitForRasPi();
  
}

void loop() { 
  //===============================
  //READ BUTTON AND START STOP TEST
  //***to-do: introduce debounce here to ignore noise from button presses (potential problem with test stopping prematurely when loop delay reduced)
  //===============================
  testStartStopButtonState = digitalRead(testStartStopButtonPin); //read button state
  if (testStartStopButtonState != lastTestStartStopButtonState) { //check if button state has changed
    if (testStartStopButtonState == HIGH) { //if button is newly depressed
      if (testUnderway == false) { //if no test was underway
        //=================
        //PRE-TEST SEQUENCE
        //=================
        
        //=====================
        //SET OR CONFIRM HEIGHT
        //=====================
        if (height ==  0) { //check if test height is set
          setHeight(encoderButtonPin); //call function to set height
        }
        confirmHeight(encoderButtonPin, encoderGreenLedPin, encoderRedLedPin); //prompt user to confirm currently set height value

        //==============
        //ZERO LOAD CELL
        //==============
        long int codeSum = 0; //sum of read ADC codes 
        long int lastReadTime = millis();
        for (int i = 0; i < 100; i++) {
          long int curTime = millis();
          while (curTime - lastReadTime < 10) {
            curTime = millis(); //update current time
          }
          lastReadTime = curTime; //reset timer to read next value
          codeSum = codeSum + ads1115.readADC_Differential_0_1(); //read ADC code for zero load
        }
        zeroCode = codeSum / 100; //calculate average for zero baseline
        preloadAttained = false; //boolean variable for whether preload of 2N reached
        maxLoad = zeroCode; //set current maxLoad to zero point
        testUnderway = true; //start test
        digitalWrite(testStatusLedPin, HIGH); //turn on test status LED
        digitalWrite(readyLedPin, LOW); //turn off READY status LED
        testId ++; //increment testId
        Serial.println("BEGIN"); //keyword to start test
        Serial.print("TESTID="); //testID on new line
        Serial.println(testId); //unique test identifier
        Serial.print("NOLOAD="); //no-load ADC code on new line
        Serial.println(zeroCode); //ADC code for "zero" load condition
        Serial.print("HEIGHT="); //force application height on new line
        Serial.println(height); //current force applicator height
      }
      else { //if test was underway
        testUnderway = false; //stop test
        preloadAttained = false; //reset preload attainment toggle
        maxLoad = 0; //reset maxLoad variable
        digitalWrite(testStatusLedPin, LOW); //turn off test status LED
        Serial.println("END"); //keyword to end test
        Serial.print("TESTID=");  //testID on new line
        Serial.println(testId); //unique test identifier
        
        if (promptAcceptReject(encoderButtonPin, encoderGreenLedPin, encoderRedLedPin) == true) {
          acceptData();
        }
        else {
          rejectData();
        }
      }
    }
    lastTestStartStopButtonState = testStartStopButtonState; //save current button state as last button state for next iteration
  }
  
  //================================================================
  // READ SENSORS AND SEND DATA TO SERIAL OUTPUT WHILE TEST UNDERWAY
  //================================================================
  if (testUnderway == true) {
    loadCellVal = ads1115.readADC_Differential_0_1(); //differential signal between channels 0 and 1
    potVal = ads1115.readADC_Differential_2_3(); //differential signal between channels 2 and 3
    //========================================
    //CHECK IF PRE-LOAD ATTAINED AND SEND DATA
    //========================================
    if (preloadAttained) { //if preload already attained, just send data
      sendData();
    }
    else if (loadCellVal > zeroCode + 240) { //if preload not previously attained, check if new point attains preload
      preloadAttained = true; //start sending data once preload attained
      sendData(); //send first data point
    }

    //==================================================
    //UPDATE MAX LOAD AND CHECK FOR TERMINATION CRITERIA
    //==================================================
    if (loadCellVal > maxLoad) {
      maxLoad = loadCellVal; //set current load cell value as new maximum
    }
    else if (loadCellVal < (1-dropTerm) * float(maxLoad)) {
      testUnderway = false; //stop test
      preloadAttained = false; //reset preload attainment toggle
      maxLoad = 0; //reset maxLoad variable
      digitalWrite(testStatusLedPin, LOW); //turn off test status LED
      Serial.println("END"); //keyword to end test
      Serial.print("TESTID=");  //testID on new line
      Serial.println(testId); //unique test identifier
        
      if (promptAcceptReject(encoderButtonPin, encoderGreenLedPin, encoderRedLedPin) == true) {
        acceptData();
      }
      else {
        rejectData();
      }
    }
  }
 
 
 //=========
 //DEBUGGING
 //=========
 //Serial.print("test=");
 //Serial.print(testUnderway);
 //Serial.print(" ,buttonState=");
 //Serial.println(testStartStopButtonState);
 
  delay(50); //***TO-DO: Eliminate this and figure out a more elegant way to control sampling rate
}


void waitForRasPi() {
  
  String raspiStringReceived = ""; //line received from RasPi
  boolean raspiReady = false;
  long int waitStartTime = millis(); //save starting time of the loop
  
  flushMainIncoming(); //flush main serial port in
  
  while (raspiReady == false) {
    long int curTime = millis(); //time at the start of this loop iteration
    if (curTime - waitStartTime > 1000) {
      Serial.println("WAITING"); //listen for signal on every iteration but only send waiting signal every half-second
      waitStartTime = curTime; //reset timer
    }
    if (Serial.available() > 0) { //if data is available in the serial buffer
      raspiStringReceived += char(Serial.read()); //read next byte from buffer and append to string
    }
    if (raspiStringReceived.endsWith("READY")) { //check whether full "READY" signal received from RasPi
      raspiReady = true;
    }
    //TO-DO: Add error handling for cases when RasPi is not sending correct signal, either based on stringReceived.length() or on a timer 
  }
  Serial.println("READY"); //return "READY" signal to RasPi
  digitalWrite(readyLedPin, HIGH); //turn on READY status LED
}

void sendData() { //writes load and angle data to serial output, separated by comma
  Serial.print(loadCellVal);
  Serial.print(",");
  Serial.println(potVal);
}  

//boolean promptAcceptReject(int acceptPin, int rejectPin) { //prompts user to indicate whether to accept or discard data from last test
boolean promptAcceptReject(int pushbuttonPin, int acceptLedPin, int rejectLedPin) { //prompts user to indicate whether to accept or discard data from last test
  int lastPushbuttonState = digitalRead(pushbuttonPin);
  int pushbuttonState = digitalRead(pushbuttonPin);
  //int lastAcceptState = 0;
  //int lastRejectState = 0;
  //int acceptState = digitalRead(acceptPin);
  //int rejectState = digitalRead(rejectPin);
  int accepted = 0; //+1 if accepted, -1 if rejected, 0 if not yet indicated
  int curSelection = 1;
  
  //enter loop to wait for user to press either the accept or reject button
  while (accepted == 0) {
    int8_t tmpdata;
    /**/
    tmpdata = read_encoder();
    
    if (tmpdata) {
      if (tmpdata > 0) {
        curSelection = 1;
      }
      else{
        curSelection = -1;
      }
    }
    
    //LED output values reversed due to common anode LED design on encoder
    if (curSelection == 1) {
      digitalWrite(acceptLedPin, LOW);
      digitalWrite(rejectLedPin, HIGH);
    }
    else {
      digitalWrite(acceptLedPin, HIGH);
      digitalWrite(rejectLedPin, LOW);
    }
    
    pushbuttonState = digitalRead(pushbuttonPin);
    if (pushbuttonState != lastPushbuttonState && pushbuttonState == HIGH) { //if confirm button is newly depressed
      accepted = curSelection;
    }
    lastPushbuttonState = pushbuttonState;
  }
  
  if (accepted == 1) {
    //turn off selection indicator LEDS
    digitalWrite(acceptLedPin, HIGH);
    digitalWrite(rejectLedPin, HIGH);
    return true;
  }
  else if (accepted == -1) {
    //turn off selection indicator LEDS
    digitalWrite(acceptLedPin, HIGH);
    digitalWrite(rejectLedPin, HIGH);
    return false;
  }
}

void acceptData() {
  Serial.println("ACCEPT");
  //testing - blink test status led 3 times
  for (int i = 0; i < 3; i++) {
    digitalWrite(testStatusLedPin, HIGH);
    delay(100);
    digitalWrite(testStatusLedPin, LOW);
    delay(100);
  }
  //testing
  waitForRasPi();
}

void rejectData() {
  Serial.println("REJECT");
  //testing - blink ready status led 3 times
  for (int i = 0; i < 3; i++) {
    digitalWrite(readyLedPin, HIGH);
    delay(100);
    digitalWrite(readyLedPin, LOW);
    delay(100);
  }
  //testing
  testId --; //decrement by 1 since last test rejected
  waitForRasPi();
}

void setHeight(int pushbuttonPin) {
  boolean heightSet = false;
  unsigned int tempHeight;
  if (height == 0) {
    tempHeight = 1500;
  }
  else {
    tempHeight = height;
  }
  int pushbuttonState = digitalRead(pushbuttonPin); //state of encoder pushbutton
  int lastPushbuttonState = pushbuttonState;
  int8_t tmpdata; //8-bit data for to store encoder state
  int encoderRotationCounter = 0;
  int8_t lastTmpdata;
  sevenSeg.blinkRate(1); //blink height display while setting in progress
  
  while (heightSet == false) { //loop while user selects height
    sevenSeg.println(tempHeight);
    sevenSeg.writeDisplay();
    tmpdata = read_encoder(); //listen for rotation on encoder
    if (tmpdata > 0 && (tmpdata*lastTmpdata) >= 0) {
      encoderRotationCounter++;
    }
    else if (tmpdata < 0 && (tmpdata*lastTmpdata) >= 0) {
      encoderRotationCounter--;
    }
    lastTmpdata = tmpdata;
    if (encoderRotationCounter >= 3) {
      tempHeight = tempHeight + 1;
      sevenSeg.println(tempHeight); //update display immediately
      sevenSeg.writeDisplay();
      encoderRotationCounter = 0; //reset counter
    }
    else if (encoderRotationCounter <= -3) {
      tempHeight = tempHeight - 1;
      sevenSeg.println(tempHeight); //update display immediately
      sevenSeg.writeDisplay();
      encoderRotationCounter = 0; //reset counter
    }

    pushbuttonState = digitalRead(pushbuttonPin); //read encoder pushbutton
    if (pushbuttonState != lastPushbuttonState && pushbuttonState == HIGH) { //if confirm button is newly depressed
      height = tempHeight; //save currently selected height
      sevenSeg.println(height); //update display again to make sure correct height is reflected
      sevenSeg.writeDisplay();
      sevenSeg.blinkRate(0); //set height display to not blink
      heightSet = true; //toggle heightSet to true to exit on next loop iteration
    }

    lastPushbuttonState = pushbuttonState;
  }
}

void confirmHeight(int pushbuttonPin, int acceptLedPin, int rejectLedPin) {
  int heightConfirmed = 0;
  int pushbuttonState = digitalRead(pushbuttonPin); //state of encoder pushbutton
  int lastPushbuttonState =pushbuttonState;
  int8_t tmpdata; //8-bit int to store encoder state
  int curSelection = 0; //current encoder focus state (accept or reject)
  
  sevenSeg.println(height); //update display to reflect currently set height
  sevenSeg.writeDisplay();
  
  while (heightConfirmed == 0) {
    int8_t tmpdata;
    /**/
    tmpdata = read_encoder();
    
    if (tmpdata) {
      if (tmpdata > 0) {
        curSelection = 1;
      }
      else{
        curSelection = -1;
      }
    }
    
    //LED output values reversed due to common anode LED design on encoder
    if (curSelection == 1) {
      digitalWrite(acceptLedPin, LOW);
      digitalWrite(rejectLedPin, HIGH);
    }
    else {
      digitalWrite(acceptLedPin, HIGH);
      digitalWrite(rejectLedPin, LOW);
    }
    
    pushbuttonState = digitalRead(pushbuttonPin);
    if (pushbuttonState != lastPushbuttonState && pushbuttonState == HIGH) { //if confirm button is newly depressed
      heightConfirmed = curSelection;
    }
    
    lastPushbuttonState = pushbuttonState;
  }

  //turn both LEDs off
  digitalWrite(acceptLedPin, HIGH);
  digitalWrite(rejectLedPin, HIGH);

  if (heightConfirmed < 0) { //if user rejects currently set height value
    setHeight(pushbuttonPin); //prompt user to set new height value    
  }
}

void flushMainIncoming() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

/* returns change in encoder state (-1,0,1) */
int8_t read_encoder()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}
