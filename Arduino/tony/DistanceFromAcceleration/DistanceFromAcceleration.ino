#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

uint32_t timer;
#define PRINT_CALCULATED
float velocity;
float acceleration;
float distance;
float g = 9.80665;
float offset;
const int buttonPin = 2;
int buttonState = 0; 
void setup()
{
  Serial.begin(115200); // Start serial at 115200 bps
  // Use the begin() function to initialize the LSM9DS0 library.
  // You can either call it with no parameters (the easy way):
  uint16_t status = dof.begin(dof.G_SCALE_2000DPS, 
                             dof.A_SCALE_6G, dof.M_SCALE_2GS);
  // Or call it with declarations for sensor scales and data rates:  
  //uint16_t status = dof.begin(dof.G_SCALE_2000DPS, 
  //                            dof.A_SCALE_6G, dof.M_SCALE_2GS);
  dof.readAccel();
  offset = dof.calcAccel(dof.ax);
    pinMode(buttonPin, INPUT);
  timer = micros();
}

void loop()
{
  double looptime = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  dof.readAccel();
  acceleration = dof.calcAccel(dof.ax)-offset;
  velocity = velocity + acceleration*looptime*g;
  distance = distance + velocity*looptime;
  Serial.println(distance);
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    // turn LED on:
    distance = 0;
    velocity = 0;
    offset = dof.calcAccel(dof.ax);
  }
}

