
/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Pins used for buttons and sensors
const int inButtonPin = 5; 
const int speedButtonPin = 6;
const int outButtonPin = 9;
const int heaterButtonPin = 10;

int inButtonState = 0; 
int outButtonState = 0;
int speedButtonState = 0;
int heaterButtonState = 0;


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *myStepper = AFMS.getStepper(513, 2);
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);


//
int currentPos = 0; //current position in steps
int fastMove = 0; // 0 = slow move, 1 = fast move
int defaultSpeed = 5; //rpm
int highSpeed = 30; //rpm
int moveIncrement = 10; //Number of steps to move per increment
int buttonReleaseTime = 400; //delay in ms

int heaterLevel = 0;

int currentSpeed = defaultSpeed;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("");
  Serial.println("******** DIY Focuser Test");

  // initialize motorshield
  AFMS.begin();  // create with the default frequency 1.6KHz

  myStepper->setSpeed(currentSpeed);  // 5 rpm   
      
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
 
  // initialize the button and sensor pins as input:
  pinMode(inButtonPin, INPUT);
  digitalWrite(inButtonPin, HIGH); // turns on pull-up resistor after input
 
  pinMode(outButtonPin, INPUT);
  digitalWrite(outButtonPin, HIGH); // turns on pull-up resistor after input
  
  pinMode(speedButtonPin, INPUT);
  digitalWrite(speedButtonPin, HIGH); // turns on pull-up resistor after input
  
  pinMode(heaterButtonPin, INPUT);
  digitalWrite(heaterButtonPin, HIGH); // turns on pull-up resistor after input
}

void loop() {
  // handle buttons: flat brightness, flap movement
  handleButton(); 
  handleSerial();
}

void handleButton() {
  // update button status
  inButtonState = digitalRead(inButtonPin);
  outButtonState = digitalRead(outButtonPin);
  speedButtonState = digitalRead(speedButtonPin);
  heaterButtonState = digitalRead(heaterButtonPin);

  if (inButtonState == LOW){
    if (speedButtonState == LOW){
        currentSpeed = highSpeed;
        buttonInwards();  
      } else {
        currentSpeed = defaultSpeed;
        buttonInwards();
      }
    }
    //delay(buttonReleaseTime/2);


  if (outButtonState == LOW){
    if (speedButtonState == LOW){
        currentSpeed = highSpeed;
        buttonOutwards();  
      } else {
        currentSpeed = defaultSpeed;
        buttonOutwards();
      }
    }
    //delay(buttonReleaseTime/2);


  if (heaterButtonState == LOW){
    heaterButton();

  }

}

void heaterButton() {

    if (heaterLevel < 255-51) {
      heaterLevel = heaterLevel + 50;
      
    } else {
      heaterLevel = 0;
    } 
    
    myMotor->setSpeed(heaterLevel);
    myMotor->run(FORWARD);
    myMotor->run(RELEASE);
    delay(buttonReleaseTime);
    Serial.print("Heater button pressed, current level: ");
    Serial.println(heaterLevel);
  
}

void buttonInwards() {
  
  Serial.print("Start move FORWARD:   ");
  Serial.print(currentSpeed); 
  Serial.print("rpm;   ");
  Serial.print(moveIncrement);
  Serial.print(" #steps");
  
  digitalWrite(LED_BUILTIN, HIGH); 
   
  myStepper->setSpeed(currentSpeed);
  myStepper->step(moveIncrement, FORWARD, DOUBLE); 
  currentPos = currentPos + moveIncrement;
  
  int neededDelay = moveIncrement / (currentSpeed * 513 / 60) * 1000; //in ms
  delay(neededDelay);
  
  myStepper->release();
  
  Serial.print(";   Move completed, current pos:   ");
  Serial.println(currentPos);
  Serial.println("********");

  digitalWrite(LED_BUILTIN, LOW);  

}
void buttonOutwards() {
  
  Serial.print("Start move BACKWARD:   ");
  Serial.print(currentSpeed); 
  Serial.print("rpm;   ");
  Serial.print(moveIncrement);
  Serial.print(" #steps");
  
  digitalWrite(LED_BUILTIN, HIGH); 

  myStepper->setSpeed(currentSpeed);
  myStepper->step(moveIncrement, BACKWARD, DOUBLE); 
  currentPos = currentPos + moveIncrement;  
  
  int neededDelay = moveIncrement / (currentSpeed * 513 / 60) * 1000; //in ms
  delay(neededDelay);
  
  myStepper->release();
  
  Serial.print(";   Move completed, current pos:   ");
  Serial.println(currentPos);
  Serial.println("********");

  digitalWrite(LED_BUILTIN, LOW);  
  
}
void handleSerial() {
  if(Serial.available()) // if there is data comming
  {
    String command = Serial.readStringUntil('\n'); // read string until meet newline character

    if(command == "IN")
    {
        Serial.println("Inwards standard speed"); // send action to Serial Monitor
        currentSpeed = defaultSpeed;
        buttonInwards();  
    }
    else
    if(command == "OUT")
    {
        Serial.println("Outwards standard speed"); // send action to Serial Monitor
        currentSpeed = defaultSpeed;
        buttonOutwards();  
    }
    else
    if(command == "IN_F")
    {
        Serial.println("Inwards high speed"); // send action to Serial Monitor
        currentSpeed = highSpeed;
        buttonInwards();  
    }
    else
    if(command == "OUT_F")
    {
        Serial.println("Outwards high speed"); // send action to Serial Monitor
        currentSpeed = highSpeed;
        buttonOutwards();  
    }
    else
    if(command == "Heat")
    {
        Serial.println("Heater pressed"); // send action to Serial Monitor
        heaterButton(); 
    }
  }
}
