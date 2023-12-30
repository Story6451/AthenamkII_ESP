//#include <ESP32Servo.h>
#include <Servo.h>
#include <LPS.h>
#include <MyLSM6.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
using namespace BLA;

const uint8_t warningLED = 1;
const float timestep = 0.001;
const float apogeeSensitivity = 0.1;

float kalmanAltitude, kalmanVerticalVelocity;
BLA::Matrix<2, 2, float> sstateTransitionMatrix;
BLA::Matrix<2, 2, float> predictionUncertaintyVector;
BLA::Matrix<2, 1, float> stateVector;
BLA::Matrix<2, 2, float> unitMatrix;
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
File fileHandler;

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
uint64_t lastLogged = 0;

bool SDPresent = false;
bool launched = false;
bool filePresent = true;
bool apogee = false;
bool error = false;

void PeripheralInitialisation()
{
  if (imu.init() == false)
  {
    Serial.print("Failed to initialise imu");
    error = true;
  }
  imu.enableHighSensitivity();

  if (barometer.init() == false)
  {
    Serial.print("Failed to initialise barometer");
    error = true;
  }
  barometer.enableDefault();
}

void SDCheck()
{
  if (SD.begin(21) == true)
  {
    fileHandler = SD.open("/DATA.txt", FILE_WRITE);
    if (fileHandler == true)
    {
      fileHandler.print("Starting Altitude/m:,");
      fileHandler.print("Time/ms,");
      fileHandler.print("Max Relative Altitude/m,");
      fileHandler.print("Altitude/m,");
      fileHandler.print("Kalman Altitude/m,");
      fileHandler.print("Pressure/Pa,");
      fileHandler.print("Temperature/'C,");
      fileHandler.print("AccelerationX/g,");
      fileHandler.print("AccelerationY/g,");
      fileHandler.print("AccelerationZ/g,");
      fileHandler.print("Pitch,");
      fileHandler.print("Roll,");
      fileHandler.print("Inertial Vertical Acceleration,");
      fileHandler.println("Kalman Vertical Velocity,");
      Serial.println("file present");
    }
    else
    {
      filePresent = false;
    }
    fileHandler.close();
    
  }
  else
  {
    error = true;
  }
  
}

void ServoSetup()
{
  //servo.attach(4);
  delay(500);
  servo.write(3, 0);
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

  unitMatrix = {
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
  

  pinMode(warningLED, OUTPUT);
  digitalWrite(warningLED, LOW);
  
  Serial.begin(9600);
  Serial.setTxTimeoutMs(0);

  Wire.begin();
  PeripheralInitialisation();
  ReadBarometer();
  SDCheck();
  ServoSetup();
  DefineMatrices();




  if (error == true)
  {
    while(true)
    {
      digitalWrite(warningLED, HIGH);
      Serial.print((float)SD.cardSize()/(1024*1024*1024)); Serial.print(" "); Serial.println(SD.cardType());
    }
  }

  Serial.print("Starting Altitude/m,");
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
  Serial.print("Inertial Verical Acceleration/ms^-2");
  Serial.println("Verical Velocity/ms^-1");

  startingAltitude = altitude;
}

void ReadIMU()
{
  imu.read();
  accelX = imu.a.x * 0.000488;
  accelY = imu.a.y * 0.000488; 
  accelZ = imu.a.z * 0.000488;
  pitchRate = imu.g.x * 0.00875;
  rollRate = imu.g.y * 0.00875;
  yawRate = imu.g.z * 0.00875;

  pitch = atan(-accelX/(sqrt(accelY * accelY + accelZ * accelZ))) * 180/PI;

  roll = atan(accelY/(sqrt(accelX * accelX + accelZ * accelZ))) * 180/PI;
}

void PrintData()
{
  if (apogee == true)
  {
    Serial.print("APOGEE!!! ");
  }
  Serial.print(startingAltitude); Serial.print("m ");
  Serial.print(millis() -launchTime); Serial.print("ms ");
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
  Serial.print(accelZInertial); Serial.print("m/s^2 ");
  Serial.print(kalmanVerticalVelocity); Serial.println("m/s ");
}

void LogData()
{
  
  if ((millis() - lastLogged)> 100)
  {
    digitalWrite(warningLED, HIGH);
    fileHandler = SD.open("/DATA.txt", FILE_APPEND);
    if (fileHandler == true)
    {
      fileHandler.print(startingAltitude); fileHandler.print(",");
      fileHandler.print(millis() -launchTime); fileHandler.print(",");
      fileHandler.print(maxAltitude); fileHandler.print(",");
      fileHandler.print(altitude); fileHandler.print(",");
      fileHandler.print(kalmanAltitude); fileHandler.print(",");
      fileHandler.print(pressure); fileHandler.print(",");
      fileHandler.print(temperature); fileHandler.print(",");
      fileHandler.print(accelX); fileHandler.print(",");
      fileHandler.print(accelY); fileHandler.print(",");
      fileHandler.print(accelZ); fileHandler.print(",");
      fileHandler.print(kalmanPitch); fileHandler.print(",");
      fileHandler.print(kalmanRoll); fileHandler.print(",");
      fileHandler.print(accelZInertial); fileHandler.print(",");
      fileHandler.print(kalmanVerticalVelocity); fileHandler.println(",");
    }
    else
    {
      filePresent = false;
    }
    fileHandler.close();

    lastLogged = millis();
  }
  digitalWrite(warningLED, LOW);
  
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

  predictionUncertaintyVector = (unitMatrix - kalmanGain * observationMatrix) * predictionUncertaintyVector;
}

uint64_t loopTimer = 0;
void loop() 
{
  if (error == false)
  {



    ReadIMU();
    ReadBarometer();


    Kalman1d(kalmanRoll, kalmanUncertaintyRoll, rollRate, roll);
    kalmanRoll = kalman1DOutput[0];
    kalmanUncertaintyRoll = kalman1DOutput[1];
    Kalman1d(kalmanPitch, kalmanUncertaintyPitch, pitchRate, pitch);
    kalmanPitch = kalman1DOutput[0];
    kalmanUncertaintyPitch = kalman1DOutput[1];

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

      if (kalmanAltitude > maxAltitude)
      {
        maxAltitude = kalmanAltitude;
      }

      
      if ((kalmanAltitude < (maxAltitude - apogeeSensitivity)) && ((kalmanVerticalVelocity < 1)))
      {
        apogee = true;

        servo.write(3, 180);
      }
    }
    else if (accelZ > 1.5)
    {
      launched = true;
      launchTime = millis();
      startingAltitude = altitude;

      if ((kalmanAltitude > 1000) || (kalmanAltitude < -1000))
      {
        error = true;
      }
      if ((kalmanVerticalVelocity > 400) || (kalmanVerticalVelocity < -400))
      {
        error = true;
      }
    }

    while ((micros() - loopTimer) < (timestep * 1000000));
    {
      loopTimer = micros();
    }
  }
  else
  {
    digitalWrite(warningLED, HIGH);
    Serial.println("ERROR");
  }
}
