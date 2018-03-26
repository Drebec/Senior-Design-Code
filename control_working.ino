#include <Servo.h>

Servo steeringServo;  //create steeringServo
int currentSpeed = 210; //enumerate speed and acceleration
int acceleration = 20;

//enumerate ports for motor and sensors
int motorPort = 8;
int leftSensorPort = 7;
int middleSensorPort = 6;
int rightSensorPort = 5;

//variables to store sensor readings
int leftSensor = 0;
int middleSensor = 0;
int rightSensor = 0;

float leftSensorRunning = 0;
float middleSensorRunning = 0;
float rightSensorRunning = 0;

int previousSensor = 0;

void setup() {
  pinMode(motorPort, OUTPUT); //set motor control port to output mode

  steeringServo.attach(4);    //attach the steeringServo to its control port

  Serial.begin(9600);

  //setMotorSpeed(120);
}

void loop() {

  for(int i = 0; i < 10; i++) {
    readSensors();
    leftSensorRunning += leftSensor;
    middleSensorRunning += middleSensor;
    rightSensorRunning += rightSensor;    
  }

  if (leftSensorRunning > 0.5){
    leftSensor = 1;
  }
  if (middleSensorRunning > 0.5) {
    middleSensor = 1;
  }
  if (rightSensorRunning > 0.5) {
    rightSensor = 1;
  }
  
//  readSensors();
//  Serial.print(leftSensor);
//  Serial.print(middleSensor);
//  Serial.print(rightSensor);
//  Serial.print("\n");

    if (middleSensor == 1 && rightSensor == 0 && leftSensor == 0) {
      //straight ahead
      steeringServo.write(90);
      analogWrite(motorPort, currentSpeed);
    } else if (middleSensor == 1 && rightSensor == 1 && leftSensor == 0) {
      //correct right
      steeringServo.write(95);
      analogWrite(motorPort, currentSpeed);
    } else if (middleSensor == 1 && rightSensor == 0 && leftSensor == 1) {
      //correct left
      steeringServo.write(85);
      analogWrite(motorPort, currentSpeed);
    } else if (middleSensor == 0 && rightSensor == 1 && leftSensor == 0) {
      //turn right
      steeringServo.write(120);
      analogWrite(motorPort, currentSpeed - 15);
      previousSensor = 2;
    } else if (middleSensor == 0 && rightSensor == 0 && leftSensor == 1) {
      //turn left
      steeringServo.write(65);
      analogWrite(motorPort, currentSpeed - 15);
      previousSensor = 1;
    } else if (rightSensor == 1 && leftSensor == 1) {
      //stop
      analogWrite(motorPort, 0);
      //exit(0);
    } else if (rightSensor == 0 && leftSensor == 0 && middleSensor == 0) {
      if(previousSensor == 1) {
        //seek left
        steeringServo.write(65);
        analogWrite(motorPort, currentSpeed - 15);
      } else if (previousSensor == 2) {
        //seek right
        steeringServo.write(120);
        analogWrite(motorPort, currentSpeed - 15);
      } else {
        //Serial.println("No Previous Sensor Data");
        steeringServo.write(90);
      }
    } else {
      Serial.println("Not valid");
      analogWrite(motorPort, 0);
    }

  leftSensor = 0;
  middleSensor = 0;
  rightSensor = 0;
  leftSensorRunning = 0.0;
  middleSensorRunning = 0.0;
  rightSensorRunning = 0.0;

}

//-----------FUNCTIONS-----------------

//sets sensor ports to outputs, writes them high and holds for 500 us
//sets them to inputs, waits for 500 us, reads sensor ports and stores
//results in corresponding variables for use in control algorithm
void readSensors() {
  pinMode(rightSensorPort, OUTPUT);
  pinMode(middleSensorPort, OUTPUT);
  pinMode(leftSensorPort, OUTPUT);
  
  digitalWrite(rightSensorPort, HIGH);
  digitalWrite(middleSensorPort, HIGH);
  digitalWrite(leftSensorPort, HIGH);
  delayMicroseconds(400);

  pinMode(rightSensorPort, INPUT);
  pinMode(middleSensorPort, INPUT);
  pinMode(leftSensorPort, INPUT);
  delayMicroseconds(400);

  leftSensor = digitalRead(leftSensorPort);
  middleSensor = digitalRead(middleSensorPort);
  rightSensor = digitalRead(rightSensorPort);
}

//changes the speed of the motor to desired speed by stepping up 'currentSpeed'
//by 'acceleration' every half-second
void setMotorSpeed(int targetSpeed) {
  while (currentSpeed <= targetSpeed) {
    currentSpeed += acceleration;
    analogWrite(motorPort, currentSpeed);
    delay(1000);
  }
}

