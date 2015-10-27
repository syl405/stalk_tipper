#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1015.h>
#include <SoftwareSerial.h>

//define pins and ports for rotary encoder
#define ENC_A 14
#define ENC_B 15
#define ENC_PORT PINC


Adafruit_ADS1115 ads1115; //instantiate ADS bject using default address 0x48
//SoftwareSerial blueSerial(10,11); //instantiate software serial port


//constants to set pin numbers
const int readyLedPin = 4;
const int testStatusLedPin = 3;
const int testStartStopButtonPin = 2;
const int encoderButtonPin = 5;
const int encoderGreenLedPin = 6;
const int encoderRedLedPin = 7;
const int digitalPotCSPin = 12;
byte digitalPotAddress = 0x00;

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
  ads1115.setGain(GAIN_ONE);
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
  
}

void loop() { 
  //================================================================
  // READ SENSORS AND SEND DATA TO SERIAL OUTPUT WHILE TEST UNDERWAY
  //================================================================
  loadCellVal = ads1115.readADC_Differential_0_1(); //differential signal between channels 0 and 1
  potVal = ads1115.readADC_Differential_2_3(); //differential signal between channels 2 and 3
  sendData();

 
 //=========
 //DEBUGGING
 //=========
 //Serial.print("test=");
 //Serial.print(testUnderway);
 //Serial.print(" ,buttonState=");
 //Serial.println(testStartStopButtonState);
 
  delay(100); //***TO-DO: Eliminate this and figure out a more elegant way to control sampling rate
}

void sendData() { //writes load and angle data to serial output, separated by comma
  Serial.print(loadCellVal);
  Serial.print(",");
  Serial.println(potVal);
}  

