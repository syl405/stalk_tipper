// The SFE_LSM9DS0 requires both the SPI and Wire libraries.
// Unfortunately, you'll need to include both in the Arduino
// sketch, before including the SFE_LSM9DS0 library.
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>

///////////////////////
// Example I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

// Do you want to print calculated values or raw ADC ticks read
// from the sensor? Comment out ONE of the two #defines below
// to pick:
#define PRINT_CALCULATED
//#define PRINT_RAW

#define PRINT_SPEED 500 // 500 ms between prints
uint32_t timer;
void setup()
{
  Serial.begin(115200); // Start serial at 115200 bps
  // Use the begin() function to initialize the LSM9DS0 library.
  // You can either call it with no parameters (the easy way):
  uint16_t status = dof.begin();
  // Or call it with declarations for sensor scales and data rates:  
  //uint16_t status = dof.begin(dof.G_SCALE_2000DPS, 
  //                            dof.A_SCALE_6G, dof.M_SCALE_2GS);
  
  // begin() returns a 16-bit value which includes both the gyro 
  // and accelerometers WHO_AM_I response. You can check this to
  // make sure communication was successful.
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();
  timer = micros();
}

// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()

float tau=0.075;
float a=0.0;

float Complementary(float newAngle, float newRate,double looptime) {
 double dtC = looptime;
  a=tau/(tau+dtC);
  float x_angleC= a* (x_angleC + newRate * dtC) + (1-a) * (newAngle);
  return x_angleC;
}

void loop()
{
  double looptime = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  dof.readGyro();
  dof.readAccel();
  float ay = dof.calcAccel(dof.ay);
  float ax = dof.calcAccel(dof.ax);
  float newAngle = atan2 (ay,ax);
  float gz = dof.calcGyro(dof.gz);
  float gx = dof.calcGyro(dof.gx);
  float newRate = gz;
  Serial.println(Complementary(newAngle,newRate,looptime));
}
