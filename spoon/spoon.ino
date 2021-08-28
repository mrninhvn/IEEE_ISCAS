#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "arduinoFFT.h"
#include <ServoSpeedControl.h>
#include "BluetoothSerial.h"

#define HANDLE_SENSOR 1
#define SPOON_SENSOR 0

#define pitchBalance 50
#define rollBalance  86

ServoSpeedControl pitch;
ServoSpeedControl roll;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 handSen = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 spoonSen = Adafruit_BNO055(55, 0x29);

sensors_event_t orientationData, h_linearAccelData, s_linearAccelData;
double accX, accY, accZ;
double accX_S, accY_S, accZ_S;
double angX, angY, angZ;

#define SAMPLES 64             // Must be a power of 2
#define SAMPLING_FREQUENCY 20  // Hz
 
arduinoFFT FFT_X = arduinoFFT();
arduinoFFT FFT_Y = arduinoFFT();
arduinoFFT FFT_Z = arduinoFFT();
 
unsigned int sampling_period_us;
unsigned long microseconds;
 
double vRealX[SAMPLES];
double vImagX[SAMPLES];
double vRealY[SAMPLES];
double vImagY[SAMPLES];
double vRealZ[SAMPLES];
double vImagZ[SAMPLES];

double fx;
double fy;
double fz;

TaskHandle_t sensorTask;
//TaskHandle_t firebaseTask;
TaskHandle_t pitchTask;
TaskHandle_t rollTask;
TaskHandle_t servoRunTask;

BluetoothSerial SerialBT;

String frequency = "";

void setup(void)
{
  Serial.begin(115200);
  SerialBT.begin("Magic Spoon");

  if (!handSen.begin())
  {
    Serial.print("Ooops, no handle sensor detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  if (!spoonSen.begin())
  {
    Serial.print("Ooops, no spoon sensor detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  pitch.attach(23);
  pitch.setSpeed(10);
  roll.attach(18);
  roll.setSpeed(10);
  
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  
  /* pin task to core 0 */  
  xTaskCreatePinnedToCore(sensorTaskCode,    "Read Sensor",     4000, NULL, 1, &sensorTask,    0);
  delay(10);
//  xTaskCreatePinnedToCore(firebaseTaskCode,  "Send data",       2000, NULL, 0, &firebaseTask,  0);
//  delay(10);
  /* pin task to core 1 */
  xTaskCreatePinnedToCore(pitchTaskCode,     "Pitch control",   1000, NULL, 1, &pitchTask,     1);
  delay(10);
  xTaskCreatePinnedToCore(rollTaskCode,      "Roll control",    1000, NULL, 1, &rollTask,      1);
  delay(10);
  xTaskCreatePinnedToCore(servoRunTaskCode,  "Run Servo",       1000, NULL, 1, &servoRunTask,  1);
  delay(10);
}

void pitchTaskCode(void *pvParameters)
{
  while(1)
  {
//    float servo = pitchBalance-angZ-accZ*3;
    float servo = pitchBalance-accZ*1.5;
    if (servo > 89.9 && servo < 90.1) servo = 90.1;
    pitch.setGoalAngle(servo);
  }
}

void servoRunTaskCode(void *pvParameters)
{
  while(1)
  {    
    pitch.doStep();
    roll.doStep();
  }
}

void rollTaskCode(void *pvParameters)
{
  while(1)
  {
    float servo = rollBalance-accX*3;
    if (servo > 89.9 && servo < 90.1) servo = 90.1;
    roll.setGoalAngle(servo);
  }
}

void sensorTaskCode(void *pvParameters)
{
  while(1)
  {
    /*SAMPLING*/
    for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();
        handSen.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        handSen.getEvent(&h_linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        getData(&h_linearAccelData, HANDLE_SENSOR);
        getData(&orientationData, HANDLE_SENSOR);
        
        vRealX[i] = accX;
        vImagX[i] = 0;
        vRealY[i] = accY;
        vImagY[i] = 0;
        vRealZ[i] = accZ;
        vImagZ[i] = 0;

        spoonSen.getEvent(&s_linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        getData(&s_linearAccelData, SPOON_SENSOR);
//        Serial.println(accZ_S);

        while(micros() < (microseconds + sampling_period_us)){
        }
    }
 
    /*FFT*/
    FFT_X.Windowing(vRealX, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT_X.Compute(vRealX, vImagX, SAMPLES, FFT_FORWARD);
    FFT_X.ComplexToMagnitude(vRealX, vImagX, SAMPLES);
    fx = FFT_X.MajorPeak(vRealX, SAMPLES, SAMPLING_FREQUENCY, 50);

    FFT_Y.Windowing(vRealY, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT_Y.Compute(vRealY, vImagY, SAMPLES, FFT_FORWARD);
    FFT_Y.ComplexToMagnitude(vRealY, vImagY, SAMPLES);
    fy = FFT_Y.MajorPeak(vRealY, SAMPLES, SAMPLING_FREQUENCY, 50);

    FFT_Z.Windowing(vRealZ, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT_Z.Compute(vRealZ, vImagZ, SAMPLES, FFT_FORWARD);
    FFT_Z.ComplexToMagnitude(vRealZ, vImagZ, SAMPLES);
    fz = FFT_Z.MajorPeak(vRealZ, SAMPLES, SAMPLING_FREQUENCY, 50);

    if (fx != 0 || fy != 0 || fz != 0)
    {
      frequency = "{fx:" + String(fx) + ",fy:" + String(fy) + ",fz:" + String(fz) + "}";
//      Serial.println(frequency);
      SerialBT.println(frequency);
    }
  }
}

void firebaseTaskCode(void *pvParameters)
{
  while(1)
  {
    /*PRINT RESULTS*/
//    frequency = "{fx:" + String(fx) + ",fy:" + String(fy) + ",fz:" + String(fz) + "}";
////    Serial.println(frequency);
//    SerialBT.println(frequency);
    
//    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void getData(sensors_event_t* event, uint8_t senNum) {
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION && senNum == HANDLE_SENSOR) {
    accX = event->acceleration.x;
    accY = event->acceleration.y;
    accZ = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION && senNum == HANDLE_SENSOR) {
    angX = event->orientation.x;
    angY = event->orientation.y;
    angZ = event->orientation.z;
  }
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION && senNum == SPOON_SENSOR) {
    accX_S = event->acceleration.x;
    accY_S = event->acceleration.y;
    accZ_S = event->acceleration.z;
  }
}

void loop(void){}
