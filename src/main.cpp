#include <Arduino.h>
#include "Arduino_FreeRTOS.h"
#include "Servo.h"
#include <Wire.h>

// defines pins numbers
const int trigPin = 22;
const int echoPin = 24;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// defines variables
long duration;
int distance;
int blinks = 0;
Servo servotje;
int pos = 0;

TaskHandle_t TaskHandle_Ultrasoon;
TaskHandle_t TaskHandle_Servo;
TaskHandle_t TaskHandle_Alarm;
TaskHandle_t TaskHandle_MPU;


void UltrasoonTaak(void* pvParameters);
void ServoTaak(void* pvParameters);
void AlarmTaak(void* pvParameters);
void MPUTaak(void* pvParameters);


void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(LED_BUILTIN, OUTPUT);
  servotje.attach(9);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.begin(9600); // Starts the serial communication


  xTaskCreate(
    UltrasoonTaak
    , (const portCHAR*)"UltrasoonTaak"
    , 128
    , NULL
    , 1
    , &TaskHandle_Ultrasoon );

  xTaskCreate(
    ServoTaak
    , (const portCHAR*)"ServoTaak"
    , 128
    , NULL
    , 1
    , &TaskHandle_Servo );

  xTaskCreate(
    AlarmTaak
    , (const portCHAR*)"AlarmTaak"
    , 128
    , NULL
    , 1
    , &TaskHandle_Alarm  );
  
  xTaskCreate(
    MPUTaak
    , (const portCHAR*)"MPUTaak"
    , 128
    , NULL
    , 1
    , &TaskHandle_MPU  );
}

void UltrasoonTaak(void* pvParameters) {
  vTaskSuspend(TaskHandle_Alarm);

  while(1) {
    // Clear trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    
    // Calculating the distance
    distance= duration*0.034/2;
    
    // Prints the distance on the Serial Monitor
    Serial.print("Afstand: ");
    Serial.println(distance);

    if (distance < 10 && distance != 0) {
      vTaskSuspend(TaskHandle_Servo);
      vTaskResume(TaskHandle_Alarm);
    }
  }
}

void ServoTaak(void* pvParameters) {
  while(1) {
    for (pos = 0; pos <= 180; pos += 2) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
      servotje.write(pos);              // tell servo to go to position in variable 'pos'
      vTaskDelay(15 / portTICK_PERIOD_MS);                       
    }

    for (pos = 180; pos >= 0; pos -= 2) { // goes from 180 degrees to 0 degrees
      servotje.write(pos);              // tell servo to go to position in variable 'pos'
      vTaskDelay(15 / portTICK_PERIOD_MS);
    }
  }
  
}

void AlarmTaak(void* pvParameters) {
  while(1) {
    if (blinks >= 5) {
      //vTaskResume(TaskHandle_Ultrasoon);
      vTaskResume(TaskHandle_Servo);
      blinks = 0;
      vTaskSuspend(TaskHandle_Alarm);
    }

    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    blinks++;
  }
}

void MPUTaak(void* pvParameters) {
  while(1) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    // Serial.print("AcX = "); Serial.print(AcX);
    // Serial.print(" | AcY = "); Serial.print(AcY);
    // Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    // Serial.print(" | GyX = "); Serial.print(GyX);
    // Serial.print(" | GyY = "); Serial.print(GyY);
    // Serial.print(" | GyZ = "); Serial.println(GyZ);
    vTaskDelay(333 / portTICK_PERIOD_MS);
  }
}



void loop() {
    
  Serial.println("Idle taak..");
  delay(1000);
  
}