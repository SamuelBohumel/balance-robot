//Bluetooth
#include <DabbleESP32.h>
#include "PIDController.h"
#include <PID_v1.h>


//Gyroscope
#include <stdbool.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <stdio.h>
Adafruit_MPU6050 mpu;


//macros
#define LED_PIN 23 
#define BUTTON_PIN  19
#define SHORT_PRESS_TIME 500 // 500 milliseconds
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//PID setup
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=1, Ki=1, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//MOTOR PINS 
int ena = 14;
int in1 = 27;
int in2 = 26;
int in3 = 25;
int in4 = 33;
int enb = 32;

// Motor parameters
int min_speed = 90; // Minimum motor speed
int max_speed = 255; // Maximum motor speed

void keep_balance(void* pvParameters );

void bluetooth_input(void* pvParameters );


//remember last motor state: false=LOW, true=HIGH
void motor_A_forward(int speed){
  if(digitalRead(in1) == HIGH){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  analogWrite(ena, speed);
}

void motor_B_forward(int speed){
  if(digitalRead(in3) == HIGH){
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
  }
  analogWrite(enb, speed);
}

void motor_A_backward(int speed){
  if(digitalRead(in1) == LOW){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  analogWrite(ena, speed);
}

void motor_B_backward(int speed){
  if(digitalRead(in3) == LOW){
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
  }
  analogWrite(enb, speed);
}

void stop_motors(){
  if(digitalRead(in1) == HIGH){
    digitalWrite(in1,LOW);
  }
  if(digitalRead(in2) == HIGH){
    digitalWrite(in2,LOW);
  }
  if(digitalRead(in3) == HIGH){
    digitalWrite(in3,LOW);
  }
  if(digitalRead(in4) == HIGH){
    digitalWrite(in4,LOW);
  }
}

int mapValue(float y) {
    // Ensure y is within the range of 0 to 7
    if (y < 0.0) {
        y = 0.0;
    } else if (y > 7.0) {
        y = 7.0;
    }

    // Map y from the range [0, 7] to the range [150, 255]
    return 50 + int(y * ((255 - 50) / 7.0));
}


void setup() {
  Serial.begin(9600);      // make sure your Serial Monitor is also set at this baud rate.
  Dabble.begin("Samo ESP");       //set bluetooth name of your device
  
  pinMode(LED_PIN, OUTPUT); 

  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Try to initialize! MPU gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  //setup gyroscpoe
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    break;
  case MPU6050_RANGE_4_G:
    break;
  case MPU6050_RANGE_8_G:
    break;
  case MPU6050_RANGE_16_G:
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    break;
  case MPU6050_RANGE_500_DEG:
    break;
  case MPU6050_RANGE_1000_DEG:
    break;
  case MPU6050_RANGE_2000_DEG:
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    break;
  case MPU6050_BAND_184_HZ:
    break;
  case MPU6050_BAND_94_HZ:
    break;
  case MPU6050_BAND_44_HZ:
    break;
  case MPU6050_BAND_21_HZ:
    break;
  case MPU6050_BAND_10_HZ:
    break;
  case MPU6050_BAND_5_HZ:
    break;
  }

    //turn the PID on
  myPID.SetMode(AUTOMATIC);
  Setpoint = 128;

  uint32_t variable = 1000;
  // Set up two tasks to run independently.
  // ARFGS: function name, name for humans, the stack size,  Task parameter, priority - higher-better, Task 
  xTaskCreate(
  keep_balance
  ,  "Task for keeping the robot in equilibium" // A name just for humans
  ,  4096        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
  ,  (void*) &variable // Task parameter which can modify the task behavior. This must be passed as pointer to void.
  ,  1  // Priority
  ,  NULL // Task handle is not used here - simply pass NULL
  );
  // xTaskCreate(
  // bluetooth_input
  // ,  "For Controlling robot with bluetooth" // A name just for humans
  // ,  4096        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
  // ,  (void*) &variable // Task parameter which can modify the task behavior. This must be passed as pointer to void.
  // ,  1  // Priority
  // ,  NULL // Task handle is not used here - simply pass NULL
  // );
}

void loop() {

}

//transer -10:10 to 0:255 for PID controller 
int scale_input(float y_acc){
  int scaled = 0;

  scaled = 128 + (y_acc* (128 / 12));

  if(scaled < 0){
    scaled = 0;
  }
  if(scaled > 255){
    scaled = 255;
  }
  return scaled;
}

void keep_balance(void* pvParameters ){
    bool robot_do_balance = false;
    //for button
    int lastState = HIGH;  // the previous state from the input pin
    int currentState = HIGH;     // the current reading from the input pin
    unsigned long pressedTime  = 0;
    unsigned long releasedTime = 0;
    uint32_t variable = *((uint32_t*)pvParameters);
    float action_point=0.3;
    float stop_angle=8.0;
    int motor_speed = 0;
    for(;;){
      // read the state of the switch/button:
      currentState = digitalRead(BUTTON_PIN);
      if (lastState == HIGH && currentState == LOW)       // button is pressed
        pressedTime = millis();
      else if (lastState == LOW && currentState == HIGH) { // button is released
        releasedTime = millis();
        long pressDuration = releasedTime - pressedTime;
        if ( pressDuration > SHORT_PRESS_TIME ){
          robot_do_balance = !robot_do_balance;
          digitalWrite(LED_PIN, robot_do_balance);	// turn on the LED
        }
      }
      lastState = currentState;
      

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      float gyro_reading = a.acceleration.y;

      //motor_speed = mapValue(gyro_reading);
      // Compute PID output
      Input = scale_input(a.acceleration.y);
      myPID.Compute();
      motor_speed = 20 + (abs(128-Output));

      if (motor_speed < min_speed) {
        motor_speed = min_speed;
      } else if (motor_speed > max_speed) {
        motor_speed = max_speed;
      }
      if (robot_do_balance && gyro_reading < -action_point && gyro_reading > -stop_angle){
        motor_A_forward(motor_speed);
        motor_B_forward(motor_speed);
        printf("%f, Forw, speed %d, Output: %f\n", gyro_reading, motor_speed, Output);
        Terminal.print(" Forw, speed");
        Terminal.print(motor_speed);

      } else if (robot_do_balance && gyro_reading > action_point && gyro_reading < stop_angle) {
        motor_A_backward(motor_speed);
        motor_B_backward(motor_speed); 
        printf("%f, Forw, speed %d, Output: %f\n", gyro_reading, motor_speed, Output);
        Terminal.print("Backw, speed");
        Terminal.print(motor_speed);
      } else {
        stop_motors();
      }
    }
}

void bluetooth_input(void* pvParameters ){
    int motor_speed = 150;
    (void) pvParameters;
    for(;;){
      Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.
      if (GamePad.isUpPressed()){
        Serial.println("Up");
      }

      if (GamePad.isDownPressed()){
        Serial.println("Down");     
      }

      if (GamePad.isLeftPressed()){
        Serial.println("Left");
      }

      if (GamePad.isRightPressed()){

      }

      if (GamePad.isSquarePressed()){
        Serial.println("Square");
      }

      if (GamePad.isCirclePressed()){
        Serial.println("Circle");
        
      }

      if (GamePad.isCrossPressed()){

      }

      if (GamePad.isTrianglePressed()){
        Serial.println("Triangle");
      }

      if (GamePad.isStartPressed()){
        Serial.println("Start");
      }

      if(GamePad.isSelectPressed()){
        Serial.println("Select");
      }
      //int a = GamePad.getAngle();
      //int b = GamePad.getRadius();

      float x = GamePad.getXaxisData();
      float y = GamePad.getYaxisData();
      float point = 5.0;
      if (x > point){
        motor_A_backward(motor_speed);
        motor_B_forward(motor_speed);
      }
      if (x < point){
        motor_A_forward(motor_speed);
        motor_B_backward(motor_speed); 
      }
      if (y > point){
        motor_A_forward(motor_speed);
        motor_B_forward(motor_speed);
      }
      if (y < point){
        motor_A_backward(motor_speed);
        motor_B_backward(motor_speed);
      }
      if(-0.5 < x && x < 0.5 && -0.5 < y && y < 0.5)
        stop_motors();
    }
}
