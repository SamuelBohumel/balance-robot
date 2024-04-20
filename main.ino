//Bluetooth
#include <DabbleESP32.h>
#include "PID_v1.h"


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
double Setpoint=0, Input=0, Output=0;
//Specify the links and initial tuning parameters
double Kp=2 , Ki=2, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//PIDController pid(Kp, Ki,  Kd, 0);

//MOTOR PINS 
int ena = 14;
int in1 = 27;
int in2 = 26;
int in3 = 25;
int in4 = 33;
int enb = 32;

// Motor parameters
int min_speed = 150; // Minimum motor speed
int max_speed = 230; // Maximum motor speed

void keep_balance(void* pvParameters );

void bluetooth_input(void* pvParameters );


void motor_A_forward(int speed){
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  analogWrite(ena, speed);
}

void motor_B_forward(int speed){
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  analogWrite(enb, speed);
}

void motor_A_backward(int speed){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  analogWrite(ena, speed);
}

void motor_B_backward(int speed){
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  analogWrite(enb, speed);
}

void stop_motors(){
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
  
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
  Setpoint = 0;

  uint32_t variable = 1000;
  // Set up two tasks to run independently.
  // ARFGS: function name, name for humans, the stack size,  Task parameter, priority - higher-better, Task 
  xTaskCreate(
  keep_balance
  ,  "Balancing" // A name just for humans
  ,  8192        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
  ,  (void*) &variable // Task parameter which can modify the task behavior. This must be passed as pointer to void.
  ,  1  // Priority
  ,  NULL // Task handle is not used here - simply pass NULL
  );
  xTaskCreate(
  drive_motors
  ,  "Motor_drive" // A name just for humans
  ,  8192        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
  ,  (void*) &variable // Task parameter which can modify the task behavior. This must be passed as pointer to void.
  ,  2  // Priority
  ,  NULL // Task handle is not used here - simply pass NULL
  );
}

void loop() {

}

float gyro_reading = 0.0;
int motor_speed = 0;
bool robot_do_balance = false;
float action_point=1.2;
float stop_angle=6.0;

void keep_balance(void* pvParameters ){
    //for button
    int lastState = HIGH;  // the previous state from the input pin
    int currentState = HIGH;     // the current reading from the input pin
    unsigned long pressedTime  = 0;
    unsigned long releasedTime = 0;
    uint32_t variable = *((uint32_t*)pvParameters);
    int sleep_t = 0;
    float last_y = 0.0;
    float offset = 0.1; //deviation from perfect water level
    for(;;){
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
      // Compute PID output
      Input = a.acceleration.y;
      gyro_reading = a.acceleration.y;
      myPID.Compute();
      motor_speed = Output;
      if(0.2 > gyro_reading && gyro_reading > -0.2){
        //pid.reinitialize();
      }
    }
}

void drive_motors(void* pvParameters ){
  int sleep_t = 0;
  for(;;){
      if (motor_speed < min_speed && motor_speed != 0) {
        sleep_t = round((min_speed / motor_speed) * 10);
        motor_speed = min_speed;
        
      } else if (motor_speed > max_speed) {
        motor_speed = max_speed;
        sleep_t = 0;
      }
      else{
        sleep_t = 0;
      }
    if (robot_do_balance && gyro_reading > action_point && gyro_reading < stop_angle) {
      delay(sleep_t);
      motor_A_backward(motor_speed);
      motor_B_backward(motor_speed); 
      delay(sleep_t);
      printf("%f, Backw, speed %d, Output: %f\n", gyro_reading, motor_speed, Output);
      Terminal.print("Backw, speed");
      Terminal.print(motor_speed);
    }
    else if (robot_do_balance && gyro_reading < -action_point && gyro_reading > -stop_angle){ 
      delay(sleep_t);  
      motor_A_forward(motor_speed);
      motor_B_forward(motor_speed);
      delay(sleep_t);
      printf("%f, Forw, speed %d, Output: %f\n", gyro_reading, motor_speed, Output);
      Terminal.print(" Forw, speed");
      Terminal.print(motor_speed);
    } 
    else {
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
