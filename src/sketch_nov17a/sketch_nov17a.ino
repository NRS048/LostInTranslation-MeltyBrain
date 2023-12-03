#include "Accelerometer.h"
#include "kalman.h"

#include <SD.h>
#include <SPI.h>

#define meltLED 23

//#define SDDebug //if you want data on the teensy 4.1's sd card
//#define SerialDebug //if you want data when connected to a computer
#define DoHeading //if you want a heading light on your test stand

KalmanFilter kf(0.0, 1.0, 0.1, 1.0);

#ifdef SDDebug
  //CS for sd card
  File myFile;
  const int chipSelect = BUILTIN_SDCARD;
#endif

using namespace Melty::Navigation;

// TODO: Verify this is the real chip select pin
Accelerometer::Accelerometer *accel1;
Accelerometer::Accelerometer *accel2;

float A45, B45, combined45, kalman45, rpm45;
int radiusCm = 3;

int pastTime = 0;       // time from past measurement. 
int currentTime = 0;    // use millis(), these two create Δt, or elapsed time
float deltaTimeSec = 0.000; // test variable to help do the first calculation
float velocity = 0.000;   // deg/sec, uses rpm calculation * 6
float pastAngle = 0.000;  // degrees, used to add the most recent rotations to the actual robot angle
float angle = 0.000;      // degrees, current robot angle

bool firstLoop = true;

void setup() {
  pinMode(meltLED, OUTPUT);
  digitalWrite(meltLED, HIGH);
  delay(500);
  digitalWrite(meltLED, LOW);

  #ifdef SerialDebug
    // setup for the first accelerometer
    Serial.println("Before construction 1");
    accel1 = new Accelerometer::Accelerometer(10);
    Serial.println("Before config 1");
    accel1->configure();
    Serial.println("Before full scale 1");
    accel1->setFullScale(Accelerometer::Config::FullScale::G400);
    Serial.println("Before ODR 1");
    accel1->setOutputDataRate(Accelerometer::Config::DataRates::HZ_100);
    Serial.println("Before Calibrate 1");
    accel1->calibrate();

    Serial.println("Before construction 2");
    accel2 = new Accelerometer::Accelerometer(9);
    Serial.println("Before config 2");
    accel2->configure();
    Serial.println("Before full scale 2");
    accel2->setFullScale(Accelerometer::Config::FullScale::G400);
    Serial.println("Before ODR 2");
    accel2->setOutputDataRate(Accelerometer::Config::DataRates::HZ_100);
    Serial.println("Before Calibrate 2");
    accel2->calibrate();

    #ifdef SDDebug
      Serial.print("Initializing SD card...");

      if (!SD.begin(chipSelect)) {
        Serial.println("SD initialization failed!");
        return;
      }
      Serial.println("SD initialization done.");

      myFile = SD.open("datalogs/data.txt", FILE_WRITE);
      
      // if the file opened okay, write to it:
      if (myFile) {
        Serial.print("Writing to data.txt...");
        myFile.println("time, 1x, 1y, 1z, 1xgs, 1ygs, 1zgs, 2x, 2y, 2z, 2xgs, 2ygs, 2zgs, kalman45, rpm45, velocity, angle, LED");
      // close the file:
        myFile.close();
        Serial.println("done.");
      } else {
        // if the file didn't open, print an error:
        Serial.println("error opening test.txt");
      }
    #endif
  #else
    // setup for the first accelerometer
    accel1 = new Accelerometer::Accelerometer(10);
    accel1->configure();
    accel1->setFullScale(Accelerometer::Config::FullScale::G400);
    accel1->setOutputDataRate(Accelerometer::Config::DataRates::HZ_100);
    accel1->calibrate();

    accel2 = new Accelerometer::Accelerometer(9);
    accel2->configure();
    accel2->setFullScale(Accelerometer::Config::FullScale::G400);
    accel2->setOutputDataRate(Accelerometer::Config::DataRates::HZ_100);
    accel2->calibrate();

    #ifdef SDDebug
      if (!SD.begin(chipSelect)) {
        //Serial.println("SD initialization failed!");
        return;
      }

      myFile = SD.open("datalogs/data.txt", FILE_WRITE);
      
      // if the file opened okay, write to it:
      if (myFile) {
        myFile.println("time, 1x, 1y, 1z, 1xgs, 1ygs, 1zgs, 2x, 2y, 2z, 2xgs, 2ygs, 2zgs, kalman45, rpm45, velocity, angle");
      // close the file:
        myFile.close();
      } //else, error opening file
    #endif
  #endif
  
}

void loop() {
  static long loopTimer = 0;
  if (millis() - loopTimer > 10){
    loopTimer = millis();
    // put your main code here, to run repeatedly:

    Accelerometer::AccelerometerReading a1 = accel1->read(); //read the first accelerometer
    Accelerometer::AccelerometerReading a2 = accel2->read(); //read the second accelerometer

    A45 = ((abs(accel1->convertToG(a1.accel_x))+abs(accel1->convertToG(a1.accel_y)))/2)/0.7071; //convert the x and y values from accelerometer 1 into one adjusted value (allows a higher maximum measured g value)
    B45 = ((abs(accel2->convertToG(a2.accel_x))+abs(accel2->convertToG(a2.accel_y)))/2)/0.7071; //convert the x and y values from accelerometer 2 into one adjusted value
    
    combined45 = (A45+B45)/2; //finds the average of the two sensors' g value

    kf.predict(); //prepare kalman filter
    kf.update(combined45); //run g value through a kalman filter

    rpm45 = sqrt(kf.getState()/(radiusCm * 11.18)) * 1000; //convert the filtered g value to RPM

    currentTime = millis(); //record time for calculations
    if(firstLoop = true){ //if this is the first loop, the time since beginning will be so high, that it will claim it is at multiple thousand degrees.
      deltaTimeSec = 0.011; // by setting the time to an accurate number, we keep the equation reasonable
      firstLoop = false;
    }else{
      deltaTimeSec = (currentTime - pastTime)/1000; //find the time in seconds since the last iteration of the loop (should be ~10ms, but is usually closer to 11ms)
    }
    
    velocity = rpm45 * 6; //convert rpm to degrees per second
    angle = (velocity*(deltaTimeSec))+pastAngle; //fun the velocity through equation with the elapsed time and the past calculated angle to figure out our direction.

    if(angle > 360){ //keep the angle value between 0 and 360 degrees
      angle -=360;
    }else if(angle < 0){
      angle += 360;
    }

    pastTime = currentTime; //save the values we just used fore the next equation
    pastAngle = angle;

    #ifdef DoHeading //enable at the top if you want to toggle the led when spinning.
      if(angle >= 20 && angle < 70){
        digitalWrite(meltLED, HIGH);
      }else{
        digitalWrite(meltLED, LOW);
      }
    #endif
    
    //debug options:
    #ifdef SerialDebug
      Serial.print(millis());
      Serial.print(" ");
      Serial.print(a1.accel_x);
      Serial.print(" ");
      Serial.print(a1.accel_y);
      Serial.print(" ");
      Serial.print(a1.accel_x);
      Serial.print(" ");
      Serial.print(accel1->convertToG(a1.accel_x));
      Serial.print(" ");
      Serial.print(accel1->convertToG(a1.accel_y));
      Serial.print(" ");
      Serial.print(accel1->convertToG(a1.accel_z));;
      Serial.print(" ");
      Serial.print(a2.accel_x);
      Serial.print(" ");
      Serial.print(a2.accel_y);
      Serial.print(" ");
      Serial.print(a2.accel_x);
      Serial.print(" ");
      Serial.print(accel2->convertToG(a2.accel_x));
      Serial.print(" ");
      Serial.print(accel2->convertToG(a2.accel_y));
      Serial.print(" ");
      Serial.println(accel2->convertToG(a2.accel_z));
      Serial.print(" ");
      Serial.print(kf.getState());
      Serial.print(" ");
      Serial.print(rpm45);
      Serial.print(" ");
      Serial.print(velocity);
      Serial.print(" ");
      Serial.print(angle);
      Serial.println(" ");
    #endif
    
    #ifdef SDDebug
      myFile = SD.open("datalogs/data.txt", FILE_WRITE);
      if (myFile) {
        //Serial.print("Writing to data.txt...");
        myFile.print(millis());
        myFile.print(" ");
        myFile.print(a1.accel_x);
        myFile.print(" ");
        myFile.print(a1.accel_y);
        myFile.print(" ");
        myFile.print(a1.accel_x);
        myFile.print(" ");
        myFile.print(accel1->convertToG(a1.accel_x));
        myFile.print(" ");
        myFile.print(accel1->convertToG(a1.accel_y));
        myFile.print(" ");
        myFile.print(accel1->convertToG(a1.accel_z));
        myFile.print(" ");
        myFile.print(a2.accel_x);
        myFile.print(" ");
        myFile.print(a2.accel_y);
        myFile.print(" ");
        myFile.print(a2.accel_x);
        myFile.print(" ");
        myFile.print(accel2->convertToG(a2.accel_x));
        myFile.print(" ");
        myFile.print(accel2->convertToG(a2.accel_y));
        myFile.print(" ");
        myFile.print(accel2->convertToG(a2.accel_z));
        myFile.print(" ");
        myFile.print(kf.getState());
        myFile.print(" ");
        myFile.print(rpm45);
        myFile.print(" ");
        myFile.print(velocity);
        myFile.print(" ");
        myFile.print(angle);
        myFile.println(" ");
        // close the file:
        myFile.close();
      }
    #endif
  }
}
