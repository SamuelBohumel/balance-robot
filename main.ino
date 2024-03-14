//Bluetooth
#include <DabbleESP32.h>
//Gyroscope
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;


//macros
#define LED_PIN 23 

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
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

// Constants for PID control
float Kp = 10; // Proportional gain
float Ki = 0.1; // Integral gain
float Kd = 0.01; // Derivative gain
// Variables for PID control
float previous_error = 0;
float integral = 0;
// Motor parameters
int base_speed = 200; // Base motor speed
int min_speed = 180; // Minimum motor speed
int max_speed = 255; // Maximum motor speed

void loop() {
  Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  int gyro_reading = a.acceleration.y;
  // Calculate deviation from desired position (0 degrees)
  float deviation = 0 - gyro_reading;

  // Calculate proportional error
  float proportional_error = deviation * Kp;

  // Calculate integral error
  integral = integral + deviation;
  float integral_error = integral * Ki;

  // Calculate derivative error
  float derivative_error = (deviation - previous_error) * Kd;
  previous_error = deviation;

  // Calculate control signal
  float control_signal = proportional_error + integral_error + derivative_error;

  // Calculate motor speed
  int motor_speed = base_speed + int(control_signal);
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
    Serial.print("Moving forward, speed ");
    Serial.print(motor_speed);
  } else if (gyro_reading > 0.2 && gyro_reading < 8) {
    motor_A_backward(motor_speed);
    motor_B_backward(motor_speed); 
    Serial.print("Moving backward, speed ");
    Serial.print(motor_speed); 
  } else {
    stop_motors();
  }
  Serial.println();

  if (GamePad.isUpPressed())
  {
    Serial.print("Up");
  }

  if (GamePad.isDownPressed())
  {
    Serial.print("Down");
  }

  if (GamePad.isLeftPressed())
  {
    Serial.print("Left");
  }

  if (GamePad.isRightPressed())
  {
    Serial.print("Right");
  }

  if (GamePad.isSquarePressed())
  {
    Serial.print("Square");
  }

  if (GamePad.isCirclePressed())
  {
    Serial.print("Circle");
    
  }

  if (GamePad.isCrossPressed())
  {

  }

  if (GamePad.isTrianglePressed())
  {
    Serial.print("Triangle");
  }

  if (GamePad.isStartPressed())
  {
    Serial.print("Start");
  }

  if (GamePad.isSelectPressed())
  {
    Serial.print("Select");
  }

  // int a = GamePad.getAngle();
  // Serial.print("Angle: ");
  // Serial.print(a);
  // Serial.print('\t');
  // int b = GamePad.getRadius();
  // Serial.print("Radius: ");
  // Serial.print(b);
  // Serial.print('\t');
  // float c = GamePad.getXaxisData();
  // Serial.print("x_axis: ");
  // Serial.print(c);
  // Serial.print('\t');
  // float d = GamePad.getYaxisData();
  // Serial.print("y_axis: ");
  // Serial.println(d);
  // Serial.println();
}
