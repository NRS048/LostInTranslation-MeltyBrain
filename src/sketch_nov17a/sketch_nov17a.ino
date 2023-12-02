#include "Accelerometer.h"
#include "kalman.h"

#include <SD.h>
#include <SPI.h>

#define meltLED 23

KalmanFilter kf(0.0, 1.0, 0.1, 1.0);

//CS for sd card
File myFile;
const int chipSelect = BUILTIN_SDCARD;

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
}

void loop() {
  static long loopTimer = 0;
  if (millis() - loopTimer > 10)
  {
    loopTimer = millis();
    // put your main code here, to run repeatedly:

    //Serial.println("Before accel read");
    Accelerometer::AccelerometerReading a1 = accel1->read();
    Accelerometer::AccelerometerReading a2 = accel2->read();
    /*
    Serial.printf("Valid: %d, x: %f, y: %f, z: %f\r\n",
        reading.valid,
        accel->convertToG(reading.accel_x),
        accel->convertToG(reading.accel_y),
        accel->convertToG(reading.accel_z));
    Serial.printf("x: 0x%x, y: 0x%x, z: 0x%x\r\n",
        reading.accel_x,
        reading.accel_y,
        reading.accel_z);
    Serial.print(" 45* calc ");
    kf.predict();
    kf.update(accel->convertToG(((reading.accel_x+reading.accel_y)/2)/0.7071));
    Serial.println(kf.getState());
    */
    /*
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
    */
    

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
      //myFile.print(" ");
      //myFile.println(kf.getState()); //kalman filter datalog
      //myFile.println(accel->convertToG(((reading.accel_x+reading.accel_y)/2)/0.7071));
      //kf.predict();
      //kf.update(accel->convertToG(((reading.accel_x+reading.accel_y)/2)/0.7071));
      //myFile.print(kf.getState());

      A45 = ((abs(accel1->convertToG(a1.accel_x))+abs(accel1->convertToG(a1.accel_y)))/2)/0.7071;
      B45 = ((abs(accel2->convertToG(a2.accel_x))+abs(accel2->convertToG(a2.accel_y)))/2)/0.7071;
      combined45 = (A45+B45)/2;
      kf.predict();
      kf.update(combined45);

      myFile.print(" ");
      myFile.print(kf.getState());
      myFile.print(" ");

      rpm45 = sqrt(kf.getState()/(radiusCm * 11.18)) * 1000;

      myFile.print(rpm45);
      myFile.print(" ");

      currentTime = millis();
      if(firstLoop = true){
        deltaTimeSec = 0.011;
        firstLoop = false;
      }else{
        deltaTimeSec = (currentTime - pastTime)/1000;
      }
      
      velocity = rpm45 * 6; //convert rpm to degrees/second
      angle = (velocity*(deltaTimeSec))+pastAngle;
      Serial.print(angle);
      Serial.print(" ");
      Serial.print(velocity);
      Serial.print(" ");
      Serial.print(deltaTimeSec);
      Serial.print(" ");
      Serial.println(pastAngle);

      if(angle > 360){
        angle -=360;
      }else if(angle < 0){
        angle += 360;
      }
      myFile.print(velocity);
      myFile.print(" ");
      myFile.print(angle);
      myFile.print(" ");

      pastTime = currentTime;
      pastAngle = angle;

      if(angle >= 20 && angle < 70){
        digitalWrite(meltLED, HIGH);
        myFile.print(1);
      }else{
        digitalWrite(meltLED, LOW);
        myFile.print(0);
      }

      // close the file:
      
      myFile.println(" ");
      
      myFile.close();
    }

    //heading light calculations
  }
}