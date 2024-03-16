//Bluetooth
#include <DabbleESP32.h>
//Gyroscope
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <stdio.h>
Adafruit_MPU6050 mpu;


//macros
#define LED_PIN 23 
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//MOTOR PINS 
int ena = 14;
int in1 = 27;
int in2 = 26;
int in3 = 25;
int in4 = 33;
int enb = 32;
int speed = 256;
int lower_speed = 200;

int LED_STATE = false;

// Motor parameters
int base_speed = 200; // Base motor speed
int min_speed = 180; // Minimum motor speed
int max_speed = 255; // Maximum motor speed

void keepbalance(void* pvParameters );

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
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}

// Define PID gains
float Kp = 5.0;   // Proportional gain
float Ki = 0.05;  // Integral gain
float Kd = 0.01;  // Derivative gain

// Define variables for PID control
float previous_error = 0;
float integral = 0;
// Function to calculate PID control signal
int calculate_pid(float error) {
    // Calculate proportional error
    float proportional_error = error * Kp;

    // Calculate integral error
    integral += error;
    float integral_error = integral * Ki;

    // Calculate derivative error
    float derivative_error = (error - previous_error) * Kd;
    previous_error = error;

    // Calculate control signal
    float control_signal = proportional_error + integral_error + derivative_error;

    // Convert control signal to motor speed
    int motor_speed = min_speed + int(abs(control_signal) * (max_speed - min_speed) / 14.0); // 14 is the range of your acceleration values
    // Ensure motor speed is within limits
    if (motor_speed < min_speed) {
        motor_speed = min_speed;
    } else if (motor_speed > max_speed) {
        motor_speed = max_speed;
    }

    return motor_speed;
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

  // Try to initialize! MPU gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  digitalWrite (LED_PIN, HIGH);	// turn on the LED
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
  uint32_t variable = 1000;
  // Set up two tasks to run independently.
  // ARFGS: function name, name for humans, the stack size,  Task parameter, priority - higher-better, Task 
  xTaskCreate(
  keep_balance
  ,  "Task for keeping the robot in equilibium" // A name just for humans
  ,  4096        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
  ,  (void*) &variable // Task parameter which can modify the task behavior. This must be passed as pointer to void.
  ,  2  // Priority
  ,  NULL // Task handle is not used here - simply pass NULL
  );
  // xTaskCreate(
  // taskFunction2
  // ,  "For Controlling robot with bluetooth" // A name just for humans
  // ,  4096        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
  // ,  (void*) &variable // Task parameter which can modify the task behavior. This must be passed as pointer to void.
  // ,  2  // Priority
  // ,  NULL // Task handle is not used here - simply pass NULL
  // );
}


void loop() {

}


void keep_balance(void* pvParameters ){
    uint32_t variable = *((uint32_t*)pvParameters);
    for(;;){
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      float gyro_reading = a.acceleration.y;
      float error = -gyro_reading;  // Reverse the sign here
      // Calculate PID control signal
      int motor_speed = calculate_pid(error);

      if (motor_speed < min_speed) {
        motor_speed = min_speed;
      } else if (motor_speed > max_speed) {
        motor_speed = max_speed;
      }
      Serial.print(gyro_reading);
      Serial.println();
      if (gyro_reading < -0.2 && gyro_reading > -8) {
        motor_A_forward(motor_speed);
        motor_B_forward(motor_speed);
        printf("%d, Moving forward, speed %d\n", gyro_reading, motor_speed);
        Terminal.print("Moving forward, speed");

      } else if (gyro_reading > 0.2 && gyro_reading < 8) {
        motor_A_backward(motor_speed);
        motor_B_backward(motor_speed); 
        printf("%d, Moving Backward, speed %d\n", gyro_reading, motor_speed);
        Terminal.print("Moving Backward, speed");
      } else {
        stop_motors();
      }
    }
}

void bluetooth_input(void* pvParameters ){
    (void) pvParameters;
    for(;;){
        Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.
      if (GamePad.isUpPressed()){
        Serial.print("Up");
      }

      if (GamePad.isDownPressed()){
        Serial.print("Down");
      }

      if (GamePad.isLeftPressed()){
        Serial.print("Left");
      }

      if (GamePad.isRightPressed()){
        Serial.print("Right");
      }

      if (GamePad.isSquarePressed()){
        Serial.print("Square");
      }

      if (GamePad.isCirclePressed()){
        Serial.print("Circle");
        
      }

      if (GamePad.isCrossPressed()){

      }

      if (GamePad.isTrianglePressed()){
        Serial.print("Triangle");
      }

      if (GamePad.isStartPressed()){
        Serial.print("Start");
      }

      if(GamePad.isSelectPressed()){
        Serial.print("Select");
      }

    }
}
