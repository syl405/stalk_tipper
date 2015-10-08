#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <SoftwareSerial.h>


Adafruit_ADS1115 ads1115; //instantiate ADS bject using default address 0x48
SoftwareSerial blueSerial(10,11); //instantiate software serial port


//constants to set pin numbers
const int readyLedPin = 4;
const int testStatusLedPin = 3;
const int testStartStopButtonPin = 2;
const int dataAcceptButtonPin = 5;
const int dataRejectButtonPin = 6;

//16-bit variables for ACDC outputs (force to 16-bits, equivalent to normal integer declaration on 16-bit boards)
int16_t loadCellVal = 0;
int16_t potVal = 0;

//general variables
boolean testUnderway = false;
int testStartStopButtonState = 0;
int lastTestStartStopButtonState = 0;
unsigned int testId = -1;

void setup() {
  Serial.begin(57600); //initialise hardware serial port
  blueSerial.begin(9600); //initialise software serial port
  ads1115.begin(); //initialise ADS object
  pinMode(readyLedPin, OUTPUT);
  pinMode(testStatusLedPin, OUTPUT);
  pinMode(testStartStopButtonPin, INPUT);
  pinMode(dataAcceptButtonPin, INPUT);
  pinMode(dataRejectButtonPin, INPUT);
  
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
      //Serial.println("button state changed");
      if (testUnderway == false) { //if no test was underway
        testUnderway = true; //start test
        digitalWrite(testStatusLedPin, HIGH); //turn on test status LED
        digitalWrite(readyLedPin, LOW); //turn off READY status LED
        testId ++; //increment testId
        Serial.println("BEGIN"); //keyword to start test
        blueSerial.println("BEGIN");
        Serial.print("TESTID="); //testID on new line
        blueSerial.print("TESTID=");
        Serial.println(testId); //unique test identifier
        blueSerial.println(testId);
      }
      else { //if test was underway
        testUnderway = false; //stop test
        digitalWrite(testStatusLedPin, LOW); //turn off test status LED
        Serial.println("END"); //keyword to end test
        blueSerial.println("END");
        Serial.print("TESTID=");  //testID on new line
        blueSerial.print("TESTID=");
        Serial.println(testId); //unique test identifier
        blueSerial.println(testId);
        
        if (promptAcceptReject(dataAcceptButtonPin, dataRejectButtonPin) == true) {
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
    sendData();
  }
 
 
 //=========
 //DEBUGGING
 //=========
 //Serial.print("test=");
 //Serial.print(testUnderway);
 //Serial.print(" ,buttonState=");
 //Serial.println(testStartStopButtonState);
 
  delay(100); //***TO-DO: Eliminate this and figure out a more elegant way to control sampling rate
}


void waitForRasPi() {
  
  String raspiStringReceived = ""; //line received from RasPi
  String blueStringReceived = ""; //line received from tablet
  boolean raspiReady = false;
  boolean btReady = false;
  long int waitStartTime = millis(); //save starting time of the loop
  
  flushMainIncoming(); //flush main serial port in
  flushSoftwareIncoming(); //flush software serial port incoming buffer
  
  while (raspiReady == false || btReady == false) {
    long int curTime = millis(); //time at the start of this loop iteration
    if (curTime - waitStartTime > 1000) {
      Serial.println("WAITING"); //listen for signal on every iteration but only send waiting signal every half-second
      blueSerial.println("WAITING");
      waitStartTime = curTime; //reset timer
    }
    if (Serial.available() > 0) { //if data is available in the serial buffer
      raspiStringReceived += char(Serial.read()); //read next byte from buffer and append to string
    }
    if (blueSerial.available() > 0) { //if data is available in the software serial buffer
      blueStringReceived += char(blueSerial.read()); //read next byte from bluetooth buffer and append to string
    }
    if (raspiStringReceived.endsWith("READY")) { //check whether full "READY" signal received from RasPi
      raspiReady = true;
    }
    if (blueStringReceived.endsWith("READY")) { //check whether full "READY" signal received from bluetooth device
      btReady = true;
    }
    //TO-DO: Add error handling for cases when RasPi is not sending correct signal, either based on stringReceived.length() or on a timer 
  }
  Serial.println("READY"); //return "READY" signal to RasPi
  blueSerial.println("READY");
  digitalWrite(readyLedPin, HIGH); //turn on READY status LED
}

void sendData() { //writes load and angle data to serial output, separated by comma
  Serial.print(loadCellVal);
  Serial.print(",");
  Serial.println(potVal);
  blueSerial.print(loadCellVal);
  blueSerial.print(",");
  blueSerial.println(potVal);
}  

boolean promptAcceptReject(int acceptPin, int rejectPin) { //prompts user to indicate whether to accept or discard data from last test
  int lastAcceptState = 0;
  int lastRejectState = 0;
  int acceptState = digitalRead(acceptPin);
  int rejectState = digitalRead(rejectPin);
  int accepted = 0; //+1 if accepted, -1 if rejected, 0 if not yet indicated
  
  //enter loop to wait for user to press either the accept or reject button
  while (accepted == 0) {
    acceptState = digitalRead(acceptPin);
    rejectState = digitalRead(rejectPin);
    //NOTE: this if...else block means that data will be accepted if both buttons are depressed at the same time
    //TO-DO: come up with alternative structure that does not make this arbitrary (albeit conservative) assumption and instead ignores simultaneous presses
    if (acceptState != lastAcceptState && acceptState == HIGH) { //if accept button is newly depressed
      accepted = 1;
    }
    else if (rejectState != lastRejectState && rejectState == HIGH) { //if reject button is newly depressed
      accepted = -1;
    }
    lastAcceptState = acceptState;
    lastRejectState = rejectState;
  }
  
  if (accepted == 1) {
    return true;
  }
  else if (accepted == -1) {
    return false;
  }
}

void acceptData() {
  Serial.println("ACCEPT");
  blueSerial.println("ACCEPT");
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
  blueSerial.println("REJECT");
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

void flushMainIncoming() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void flushSoftwareIncoming() {
  while (blueSerial.available() > 0) {
    blueSerial.read();
  }
}
