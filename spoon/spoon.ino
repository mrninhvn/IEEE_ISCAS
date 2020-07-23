#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "arduinoFFT.h"
#include "BluetoothSerial.h"
#include <ServoSpeedControl.h>

ServoSpeedControl pitch;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

double accX, accY, accZ;
double angX, angY, angZ;

#define SAMPLES 64             //Must be a power of 2
#define SAMPLING_FREQUENCY 20 //Hz, must be less than 10000 due to ADC
 
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
TaskHandle_t bluetoothTask;
TaskHandle_t controlTask;
TaskHandle_t servoTask;

BluetoothSerial SerialBT;
String frequency = "";

void setup(void)
{
  Serial.begin(115200);
  SerialBT.begin("Magic Spoon");

  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  pitch.attach(23);
  
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  
  /* pin task to core 0 */  
  xTaskCreatePinnedToCore(sensorTaskCode, "Read Sensor", 4000, NULL, 1, &sensorTask, 0);
  delay(100);
  xTaskCreatePinnedToCore(bluetoothTaskCode, "Send bluetooth", 10]00, NULL, 1, &bluetoothTask, 0);
  delay(100);
  /* pin task to core 1 */
  xTaskCreatePinnedToCore(controlTaskCode, "Controller", 4000, NULL, 1, &controlTask, 1);
  delay(100);
  xTaskCreatePinnedToCore(servoTaskCode, "Run Servo", 1000, NULL, 1, &servoTask, 1);
  delay(100);
}

void controlTaskCode(void *pvParameters)
{
  while(1)
  {
//    sensors_event_t orientationData;
//    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//    getData(&orientationData);
//
//    String angle = "";
//    angle = "{AngX:" + String(angX) + ",AngY:" + String(angY) + ",AngZ:" + String(angZ) + "}";
//    Serial.println(angle);
    float servo = 100-angZ;
    if (servo > 89.9 && servo < 90.1) servo = 90.1;
    pitch.setSpeed(10);
    pitch.setGoalAngle(servo);
    pitch.doStep();
//    vTaskDelay(200/portTICK_PERIOD_MS);
  }
}

void servoTaskCode(void *pvParameters)
{
  while(1)
  {
    pitch.doStep();
  }
}

void sensorTaskCode(void *pvParameters)
{
  while(1)
  {
    /*SAMPLING*/
    for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();    //Overflows after around 70 minutes!
        sensors_event_t orientationData, linearAccelData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        getData(&linearAccelData);
        getData(&orientationData);
        
        vRealX[i] = accX;
        vImagX[i] = 0;
        vRealY[i] = accY;
        vImagY[i] = 0;
        vRealZ[i] = accZ;
        vImagZ[i] = 0;
     
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
  }
}

void bluetoothTaskCode(void *pvParameters)
{
  while(1)
  {
    /*PRINT RESULTS*/
    frequency = "{fx:" + String(fx) + ",fy:" + String(fy) + ",fz:" + String(fz) + "}";
    Serial.println(frequency);
    SerialBT.println(frequency);
    
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void getData(sensors_event_t* event) {
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    accX = event->acceleration.x;
    accY = event->acceleration.y;
    accZ = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    angX = event->orientation.x;
    angY = event->orientation.y;
    angZ = event->orientation.z;
  }
}

void loop(void){}
