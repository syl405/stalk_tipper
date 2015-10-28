#include <Wire.h>
#include <Adafruit_ADS1015.h>

//define pins and ports for rotary encoder
#define ENC_A 14
#define ENC_B 15
#define ENC_PORT PINC


Adafruit_ADS1115 ads1115; //instantiate ADS bject using default address 0x48


//constants to set pin numbers
const int readyLedPin = 4;
const int testStatusLedPin = 3;
const int testStartStopButtonPin = 2;
const int encoderButtonPin = 5;
const int encoderGreenLedPin = 6;
const int encoderRedLedPin = 7;

//16-bit variables for ADC outputs (force to 16-bits, equivalent to normal integer declaration on 16-bit boards)
int16_t loadCellVal = 0;
int16_t potVal = 0;

//general variables
boolean testUnderway = false;
int testStartStopButtonState = 0;
int lastTestStartStopButtonState = 0;
unsigned int testId = -1;

void setup() {
  Serial.begin(115200); //initialise hardware serial port
  ads1115.begin(); //initialise ADS object
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
      //Serial.println("button state changed");
      if (testUnderway == false) { //if no test was underway
        testUnderway = true; //start test
        digitalWrite(testStatusLedPin, HIGH); //turn on test status LED
        digitalWrite(readyLedPin, LOW); //turn off READY status LED
        testId ++; //increment testId
        Serial.println("BEGIN"); //keyword to start test
        Serial.print("TESTID="); //testID on new line
        Serial.println(testId); //unique test identifier
      }
      else { //if test was underway
        testUnderway = false; //stop test
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
