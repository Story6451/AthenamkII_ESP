#include <ESP32Servo.h>
//#include <Arduino.h>
//#include <SD.h>
#include <LPS.h>
#include <MyLSM6.h>
#include <BasicLinearAlgebra.h>
//#include <LIS3MDL.h> WE DONT NEED YAW DATA ANYWAY GRRR
#include <Wire.h>
#include <SPI.h>
using namespace BLA;
float kalmanAltitude, kalmanVerticalVelocity;
float timestep = 0.001;
BLA::Matrix<2, 2, float> sstateTransitionMatrix;
BLA::Matrix<2, 2, float> predictionUncertaintyVector;
BLA::Matrix<2, 1, float> stateVector;
BLA::Matrix<2, 2, float> unityMatrix;
BLA::Matrix<2, 1, float> kalmanGain;
BLA::Matrix<1, 1, float> pintermediateMatrix;
BLA::Matrix<2, 1, float> controlMatrix;
BLA::Matrix<2, 2, float> processUncertainty;
BLA::Matrix<1, 2, float> observationMatrix;
BLA::Matrix<1, 1, float> Acc;
BLA::Matrix<1, 1, float> measurementUncertainty;
BLA::Matrix<1, 1, float> measurementVector;

LPS barometer;
LSM6 imu;
//LIS3MDL compass;
//File fileHandler;

Servo servo;

float pressure, altitude, temperature;
float accelZInertial;
double accelX, accelY, accelZ;
double pitchRate, rollRate, yawRate;
float pitch, roll, yaw;
float kalmanRoll = 0, kalmanUncertaintyRoll = 2 * 2;
float kalmanPitch = 0, kalmanUncertaintyPitch = 2 * 2;
float kalman1DOutput[] = {0, 0};

float maxAltitude = 0;
float startingAltitude = 0;
uint64_t launchTime;

bool SDPresent = false;
bool launched = false;
bool filePresent = true;


void PeripheralInitialisation()
{
  if (imu.init() == false)
  {
    Serial.print("Failed to initialise imu");
  }
  imu.enableHighSensitivity();

  if (barometer.init() == false)
  {
    Serial.print("Failed to initialise barometer");
  }
  barometer.enableDefault();
  /*
  if (compass.init() == false)
  {
    Serial.print("Failed to initialise compass");
  }
  compass.enableDefault();
  */
}

void SDCheck()
{
  /*
  fileHandler = SD.open("DATA.txt", FILE_WRITE | O_TRUNC);
  if (fileHandler == true)
  {
    fileHandler.print("Starting Altitude/m:,"); fileHandler.println(startingAltitude);
    fileHandler.print("Time/ms,");
    fileHandler.print("Altitude/m,");
    fileHandler.print("Relative Altitude/m,");
    fileHandler.print("Pressure/Pa,");
    fileHandler.print("Temperature/'C,");
    fileHandler.print("AccelerationX/g,");
    fileHandler.print("AccelerationY/g,");
    fileHandler.print("AccelerationZ/g,");
    fileHandler.print("GyroscopeX,");
    fileHandler.print("GyroscopeY,");
    fileHandler.print("GyroscopeZ,");
    fileHandler.println("Heading,");
    Serial.println("file present");
  }
  else
  {
    filePresent = false;
    Serial.println("file not present");
  }
  fileHandler.close();
  */
}

void ServoSetup()
{
  servo.attach(4);
  delay(500);
  servo.write(0);
}

void ReadBarometer()
{
  pressure = barometer.readPressureMillibars();
  altitude = barometer.pressureToAltitudeMeters(pressure);
  temperature = barometer.readTemperatureC();
}

void DefineMatrices()
{
  sstateTransitionMatrix = {
    1,  timestep,
    0,  1
  };  

  controlMatrix = {
    0.5*timestep*timestep,
    timestep
  };

  observationMatrix = {1, 0};

  unityMatrix = {
    1,  0,
    0,  1
  };

  processUncertainty = controlMatrix * ~controlMatrix * 10.0f * 10.0f;

  measurementUncertainty = {30*30};

  predictionUncertaintyVector = {
    0,  0,
    0,  0
  };

  stateVector = {
    0,
    0
  };
}

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  PeripheralInitialisation();
  ReadBarometer();
  startingAltitude = altitude;
  SDCheck();
  ServoSetup();
  DefineMatrices();


  delay(500);
  Serial.print("Time/ms,");
  Serial.print("MaxAltitude/m");
  Serial.print("Altitude/m,");
  Serial.print("Relative Kalman Altitude/m,");
  Serial.print("Pressure/Pa,");
  Serial.print("Temperature/'C,");
  Serial.print("AccelerationX/g,");
  Serial.print("AccelerationY/g,");
  Serial.print("AccelerationZ/g,");
  Serial.print("Pitch/',");
  Serial.print("Roll/',");
  Serial.println("Verical Velocity/ms^-1");


}

void ReadIMU()
{
  imu.read();
  accelX = imu.a.x * 0.000476;
  accelY = imu.a.y * 0.000476; 
  accelZ = imu.a.z * 0.000476;
  pitchRate = imu.g.x * 0.00875;
  rollRate = imu.g.y * 0.00875;
  yawRate = imu.g.z * 0.00875;

  pitch = atan(-accelX/(sqrt(accelY * accelY + accelZ * accelZ))) * 180/PI;

  roll = atan(accelY/(sqrt(accelX * accelX + accelZ * accelZ))) * 180/PI;
}

void ReadCompass()
{
}

void PrintData()
{
  
  Serial.print(millis()); Serial.print("ms ");
  Serial.print(maxAltitude); Serial.print("m ");
  Serial.print(altitude); Serial.print("m ");
  Serial.print(kalmanAltitude); Serial.print("m ");
  Serial.print(pressure); Serial.print("hPa ");
  Serial.print(temperature); Serial.print("'C ");
  Serial.print(accelX); Serial.print("g ");
  Serial.print(accelY); Serial.print("g ");
  Serial.print(accelZ); Serial.print("g ");
  Serial.print(kalmanPitch); Serial.print("' ");
  Serial.print(kalmanRoll); Serial.print("' ");
  Serial.print(kalmanVerticalVelocity); Serial.println("m/s ");
}

void LogData()
{
  /*
  fileHandler = SD.open("DATA.txt", FILE_WRITE);
  if (fileHandler == true)
  {
    fileHandler.print(millis() - launchTime); fileHandler.print(",");
    fileHandler.print(altitude); fileHandler.print(",");
    fileHandler.print(altitude - startingAltitude); fileHandler.print(",");
    fileHandler.print(pressure); fileHandler.print(",");
    fileHandler.print(temperature); fileHandler.print(",");
    fileHandler.print(accelX); fileHandler.print(",");
    fileHandler.print(accelY); fileHandler.print(",");
    fileHandler.print(accelZ); fileHandler.print(",");
    fileHandler.print(pitch); fileHandler.print(",");
    fileHandler.print(roll); fileHandler.print(",");
    fileHandler.print(yaw); fileHandler.println(",");
  }
  else
  {
    filePresent = false;
  }
  fileHandler.close();
  */
}

void Kalman1d(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement)
{
  kalmanState = kalmanState + timestep * kalmanInput;
  kalmanUncertainty = kalmanUncertainty + timestep * timestep * 4 * 4;
  float kalmanGain = kalmanUncertainty * 1/(kalmanUncertainty + 3 * 3);
  kalmanState = kalmanState + kalmanGain * (kalmanMeasurement - kalmanState);
  kalmanUncertainty = (1 - kalmanGain) * kalmanUncertainty;
  kalman1DOutput[0] = kalmanState;
  kalman1DOutput[1] = kalmanUncertainty;
}

void kalman2d()
{
  Acc = {accelZInertial};
  stateVector = sstateTransitionMatrix * stateVector + controlMatrix * Acc;
  predictionUncertaintyVector = sstateTransitionMatrix * predictionUncertaintyVector * ~sstateTransitionMatrix + processUncertainty;
  pintermediateMatrix = observationMatrix * predictionUncertaintyVector * ~observationMatrix + measurementUncertainty;
  kalmanGain = predictionUncertaintyVector * ~observationMatrix * Inverse(pintermediateMatrix);
  measurementVector = {(altitude-startingAltitude)};
  stateVector = stateVector + kalmanGain * (measurementVector - observationMatrix * stateVector);
  kalmanAltitude = stateVector(0, 0);
  kalmanVerticalVelocity = stateVector(1, 0);
  predictionUncertaintyVector = (unityMatrix - kalmanGain * observationMatrix) * predictionUncertaintyVector;
}

uint64_t loopTimer = 0;
void loop() 
{
  ReadIMU();
  ReadBarometer();
  ReadCompass();
  Kalman1d(kalmanRoll, kalmanUncertaintyRoll, rollRate, roll);
  kalmanRoll = kalman1DOutput[0];
  kalmanUncertaintyRoll = kalman1DOutput[1];
  Kalman1d(kalmanPitch, kalmanUncertaintyPitch, pitchRate, pitch);
  kalmanPitch = kalman1DOutput[0];
  kalmanUncertaintyPitch = kalman1DOutput[1];
  //accelZInertial = -sin(kalmanPitch * 3.142/180) * accelX + cos(kalmanPitch * 3.142/180) * sin(kalmanRoll * 3.142/180) * accelY + cos(kalmanPitch * 3.142/180) * cos(kalmanRoll * 3.142/180) * accelZ;
  accelZInertial = -sin(pitch * 3.142/180) * accelX + cos(pitch * 3.142/180) * sin(roll * 3.142/180) * accelY + cos(pitch * 3.142/180) * cos(roll * 3.142/180) * accelZ;
  accelZInertial = (accelZInertial - 1) * 9.81;
  kalman2d();

  if (launched == true)
  {
    PrintData();


    if (filePresent == true)
    {
      LogData();
    }

    if ((millis - launchTime))
    if (kalmanAltitude > maxAltitude)
    {
      maxAltitude = kalmanAltitude;
    }

    if ((kalmanAltitude < (maxAltitude-0.5)) && ((kalmanVerticalVelocity > -1) && (kalmanVerticalVelocity < 1)))
    {
      //Serial.println("RECOVER!!!");
      servo.write(90);
    }
  }
  else if (accelZ > 1.5)
  {
    launched = true;
    launchTime = millis();
  }

  while ((micros() - loopTimer) < timestep * 1000000);
  {
    loopTimer = micros();
  }
}
