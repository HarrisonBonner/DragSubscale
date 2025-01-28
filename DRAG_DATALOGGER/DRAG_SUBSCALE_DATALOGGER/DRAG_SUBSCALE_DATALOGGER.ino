/* DRAG DATA COLLECTION
First subscale launch data collection script

  
   Connections
   ===========
   

   Harrison Bonner
*/

#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_BNO055.h"
#include "./utility/imumaths.h"
#include "PWMServo.h"

// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
const int chipSelect = BUILTIN_SDCARD;

//Define Pin connections
#define buzzerPin 3
#define keySwitchPin 34
#define servoPin 7


#define SEALEVELPRESSURE_HPA (1013.25)

//Define Devices
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno;
PWMServo dragServo;

//Define global Variables
double altitude;
double previousAltitude;

double zAccel;

double deltaTime = 0;
double previousTime = 0;

double velocityThreshold;
int velocityAboveCountThreshold;
bool launchDetected = false;

double burnoutThreshold;
int burnoutBelowCountThreshold;
bool burnoutDetected = false;


void setup() {
  //TODO REMOVE ME FOR ACTUAL ON BOARD TEST!!
  // Open serial communications and wait for port to open:
  //Serial.begin(9600);
  //while (!Serial) {
  //  ;  // wait for serial port to connect.
  //}

  pinMode(buzzerPin, OUTPUT);
  pinMode(keySwitchPin, INPUT);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  Serial.println("card initialized.");
  makeFileHeader();


  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  //Initlize starting altitude
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
  }
  previousAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  bno.setMode(OPERATION_MODE_AMG);
  //bno.setGRange(16);


  dragServo.attach(servoPin);
  dragServo.write(90);

  for (int i = 0; i < 10; i++) {
    beepBuzzer(500);
  }
}


void makeFileHeader() {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("TIME\tALTITUDE\tORIENTATION_X\tORIENTATION_Y\tORIENTATION_Z\tANGVELOCITY_X\tANGVELOCITY_Y\tANGVELOCITY_Z\tLINEARACCEL_X\tLINEARACCEL_Y\tLINEARACCEL_Z\tMAGNETOMETER_X\tMAGNETOMETER_Y\tMAGNETOMETER_Z\tACCELEROMETER_X\tACCELEROMETER_Y\tACCELEROMETER_Z\tGRAVITY_X\tGRAVITY_Y\tGRAVITY_Z\t");
    dataFile.close();
    Serial.println("Header made");
    // print to the serial port too:
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
}


String altitudeRecording() {
  String result = "\t";
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return "ERROR READING BMP";
  }
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  result += altitude;
  result += "\t";

  return result;
}

String printBNOEvent(sensors_event_t* event) {
  String result = "";
  double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print("Unk:");
  }

  result += x;
  result += "\t";
  result += y;
  result += "\t";
  result += z;
  result += "\t";

  return result;
}

String bnoRecording() {
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  String result = "";

  result += printBNOEvent(&orientationData);
  result += printBNOEvent(&angVelocityData);
  result += printBNOEvent(&linearAccelData);
  result += printBNOEvent(&magnetometerData);
  result += printBNOEvent(&accelerometerData);
  result += printBNOEvent(&gravityData);
  return result;
}

void beepBuzzer(int length) {
  tone(buzzerPin, 262);
  delay(length >> 2);
  noTone(buzzerPin);
  delay(length >> 2);
}


int numVelocityAboveThreshold = 0;
bool checkIfLaunch() {

  //Setup dt
  double currentTime = millis();
  deltaTime = currentTime - previousTime;
  previousTime = currentTime;

  //Setup dZ for velocity
  double velocity = (altitude - previousAltitude) * deltaTime;
  previousAltitude = altitude;

  if (velocity > velocityThreshold) {
    numVelocityAboveThreshold++;
  } else {
    numVelocityAboveThreshold = 0;
  }

  if (numVelocityAboveThreshold > velocityAboveCountThreshold) {
    return true;
  }

  return false;
}

int numBurnoutBelowThreshold = 0;
bool checkIfMotorBurnout() {

  //Setup dt
  double currentTime = millis();
  deltaTime = currentTime - previousTime;
  previousTime = currentTime;

  //Checking that velocity is decreasing for X samples

  //Setup dZ for velocity
  double velocity = (altitude - previousAltitude) * deltaTime;
  previousAltitude = altitude;


  if (velocity < burnoutThreshold) {
    numBurnoutBelowThreshold++;
  } else {
    numBurnoutBelowThreshold = 0;
  }

  if (numBurnoutBelowThreshold > burnoutBelowCountThreshold) {
    return true;
  }

  return false;
}

void dataLog() {
  // make a string for assembling the data to log:
  String dataString = "";
  dataString += millis();
  dataString += altitudeRecording();
  dataString += bnoRecording();

  // open the file.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
}

void setDragPercent(int goal) {
  //Math of angle rotation to percentage
}


void loop() {
  if (digitalRead(keySwitchPin)) {


    dataLog();
    beepBuzzer(100);  // Buzzer has built in delay using parameter

    if (launchDetected) {
      if (burnoutDetected) {
        //Control logic
      } else {
        checkIfMotorBurnout();
      }
    } else {
      checkIfLaunch();
    }
  }
}
