#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads1115; //instantiate ADS bject using default address 0x48

//constants to set pin numbers
const int readyLedPin = 4;
const int testStatusLedPin = 3;
const int testStartStopButtonPin = 2;

//16-bit variables for ACDC outputs (force to 16-bits, equivalent to normal integer declaration on 16-bit boards)
int16_t loadCellVal = 0;
int16_t potVal = 0;

//general variables
boolean testUnderway = false;
int testStatusLedState = 0;
int testStartStopButtonState = 0;
int lastTestStartStopButtonState = 0;
unsigned int testId = 0;


void setup() {
  Serial.begin(115200);
  ads1115.begin(); //initialise ADS object
  pinMode(readyLedPin, OUTPUT);
  pinMode(testStatusLedPin, OUTPUT);
  pinMode(testStartStopButtonPin, INPUT);
  
  //============================================================================
  //SEND "WAITING" SIGNAL REPEATEDLY WHILE LISTENING FOR READY SIGNAL FROM RASPI
  //============================================================================
  String stringReceived; //line received from RasPi
  boolean rasPiReady = false;
  
  while (rasPiReady == false) {
    Serial.println("WAITING");
    stringReceived += Serial.read(); //read next byte from buffer and append to string
    if (stringReceived.endsWith("READY")) { //check whether full "READY" signal received from RasPi
      rasPiReady = true;
      Serial.println("READY"); //return "READY" signal to RasPi
      digitalWrite(readyLedPin, HIGH); //turn on READY status LED
    }
    //TO-DO: Add error handling for cases when RasPi is not sending correct signal, either based on stringReceived.length() or on a timer
  }
}

void loop() {
  loadCellVal = ads1115.readADC_Differential_0_1(); //differential signal between channels 0 and 1
  potVal = ads1115.readADC_Differential_2_3(); //differential signal between channels 2 and 3
  
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
        testStatusLedState = HIGH; //turn on test status LED
        testId ++; //increment testId
        Serial.print("BEGIN TESTID="); //keyword to start test
        Serial.println(testId); //unique test identifier
      }
      else { //if test was underway
        testUnderway = false; //stop test
        testStatusLedState = LOW; //turn off test status LED
        Serial.print("END TESTID=");
        Serial.println(testId);
      }
    }
    lastTestStartStopButtonState = testStartStopButtonState; //save current button state as last button state for next iteration
  }
  
  //==============================================
  //SEND DATA TO SERIAL OUTPUT WHILE TEST UNDERWAY
  //==============================================
  if (testUnderway == true) {
    sendData();
  }
  
  //===========================
  //UPDATE INDICATOR LED STATES
  //***note: only update LEDs after first datapoint has been sent to avoid missing start of test
  //=========================== 
  digitalWrite(testStatusLedPin, testStatusLedState);
 
 
 //=========
 //DEBUGGING
 //=========
 //Serial.print("test=");
 //Serial.print(testUnderway);
 //Serial.print(" ,buttonState=");
 //Serial.println(testStartStopButtonState);
 
  delay(100); //***TO-DO: Eliminate this and figure out a more elegant way to control sampling rate
}

void sendData() { // writes load and angle data to serial output, separated by comma
  Serial.print(loadCellVal);
  Serial.print(",");
  Serial.println(potVal);
}
