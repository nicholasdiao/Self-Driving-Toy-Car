
#include <AFMotor.h>
/*
 * We include the AF motor library, which provides direct commands to 
 * set motor speed and direction. Below we initilize four motors, specifying 
 * their specific connection to the motor shield. 
 */
AF_DCMotor motor1(1); // Back Right
AF_DCMotor motor2(2); // Front Right
AF_DCMotor motor3(3); // Back Left
AF_DCMotor motor4(4); // Front Left

int roadMotorSpeed = 80;  // Set default power to send to motors
int slowMotorSpeed = 30;  // Set slow speed
//int highwayMotorSpeed = 120;  
int turnSpeed = 30; // set turn speed

bool objectFront;
bool forward;
bool backward;
bool turnRight;
bool turnLeft;
int backUpTime = 1300; // set time in milliseconds for car to backup and turn
int forTurnTime = 850; // set time in milliseconds for car to turn forward


/*
 * Below we include the Ultrasonic library, which provides controls for our 
 * Ultrasonic HC-SR04 sensors. Each sensor has a trigger pin and an echo pin,
 * both of which are connected to a digital I/O pin on the arduino board. 
 * We use four sensors, so the 8 integers below describe which I/O pins each 
 * set of trigger and echo pins are connected to. 
 * The trigger pin is used to turn the sensor on, sending out a sound wave
 * The echo pin relays the time in microseconds that the sound wave travels.
 */
#include "Ultrasonic.h"
int trigPin1 = 24;    //Trig - green Jumper
int echoPin1 = 25;    //Echo - yellow Jumper
int trigPin2 = 26;    //Trig - green Jumper
int echoPin2 = 27;    //Echo - yellow Jumper
int trigPin3 = 28;    //Trig - green Jumper
int echoPin3 = 29;    //Echo - yellow Jumper
int trigPin4 = 30;    //Trig - green Jumper
int echoPin4 = 31;    //Echo - yellow Jumper

/*
 * Below we use a class from Ultrasonic.h called ultrasonic,
 * instanstiating for different instances of it, one for each sensor. 
 * This class allows us to continually turn on the sensors, and automatically
 * converts the millisecond output from the echo pin into a distance measure. 
 * Therefore, we get four integers, each of which describing the distance in
 * cm of the nearest object on all four sides of the car. 
 * Side note, the max distance these sensors can read is 51 cm. Therefore,
 * if there are no objects within 51cm, the output remains at 51cm. 
 */

Ultrasonic ultrasonic1(trigPin1,echoPin1);
Ultrasonic ultrasonic2(trigPin2,echoPin2);
Ultrasonic ultrasonic3(trigPin3,echoPin3);
Ultrasonic ultrasonic4(trigPin4,echoPin4);
int cm1 = (ultrasonic1.Ranging(CM)); // back
int cm2 = (ultrasonic2.Ranging(CM)); // right
int cm3 = (ultrasonic3.Ranging(CM)); // front
int cm4 = (ultrasonic4.Ranging(CM)); //left


/*
 * This function sets the speed of all four motors to the same speed. 
 */
void roadSpeedSet(int speedIn) {
    motor1.setSpeed(speedIn);
    motor2.setSpeed(speedIn);
    motor3.setSpeed(speedIn);
    motor4.setSpeed(speedIn);
    return;
}

/*
 * This function tells the car to move forward at whatever speed is 
 * already set
 */
void roadSpeedRun() {
     motor1.run(FORWARD);
     motor2.run(FORWARD);
     motor3.run(FORWARD);
     motor4.run(FORWARD);
     forward = true;
     delay(20);
     return;
}

/*
 * This function tells the car to move backward at whatever speed is 
 * already set
 */
void roadSpeedBack() {
     motor1.run(BACKWARD);
     motor2.run(BACKWARD);
     motor3.run(BACKWARD);
     motor4.run(BACKWARD);
     forward = false;
     backward = true;
     delay(1300);
     return;
}

/*
 * This function tells the car to gradually transition from a high speed
 * to a low speed.
 */
void slowDownSpeed(int speedIn, int speedOut, int dist) {
     motor1.run(FORWARD);
     motor2.run(FORWARD);
     motor3.run(FORWARD);
     motor4.run(FORWARD);
     forward = true;

     while (dist < 10 && speedIn > speedOut){
      speedIn --; 
      roadSpeedSet(speedIn); 
      delay(10);
    }
     return;
}

/*
 * This function tells the car to gradually transition from a low speed
 * to a high speed.
 */
void speedUpSpeed(int speedIn, int speedOut) {
     motor1.run(FORWARD);
     motor2.run(FORWARD);
     motor3.run(FORWARD);
     motor4.run(FORWARD);
     forward = true;
     for (int i=speedIn; i< speedOut; i++) {
       roadSpeedSet(i); 
       delay(10);
    }
     return;
}

/*
 * This function tells the car to stop by releasing all the motors
 */
void stopRun() {
     motor1.run(RELEASE);
     motor2.run(RELEASE);
     motor3.run(RELEASE);
     motor4.run(RELEASE);
     forward = false;
     backward = false;
     turnRight = false;
     turnLeft = false;
     delay(20);
     return;
}

/*
 * This function tells the car to turn right, by setting the wheels on the
 * left to a high speed, and the wheels on the right to a low speed. 
 * These functions are meant to be slight turns, while the car is still 
 * moving forward. 
 */

int turnStraightHigh = 50;
int turnStraightLow = 10;
void turnRightSpeed(int speedIn) {
     int turnSpeed = speedIn + turnStraightHigh;
     motor4.setSpeed(turnSpeed);
     motor3.setSpeed(turnSpeed);
     motor1.setSpeed(speedIn-turnStraightLow);
     motor2.setSpeed(speedIn-turnStraightLow);
     motor1.run(FORWARD);
     motor2.run(FORWARD);
     motor3.run(FORWARD);
     motor4.run(FORWARD);
     turnRight = true;
     forward = true;
     delay(20);
     return;
}

/*
 * Tells the car to turn left
 */
void turnLeftSpeed(int speedIn) {
     int turnSpeed = speedIn + turnStraightHigh;
     motor1.setSpeed(turnSpeed);
     motor2.setSpeed(turnSpeed);
     motor4.setSpeed(speedIn-turnStraightLow);
     motor3.setSpeed(speedIn-turnStraightLow);
     motor1.run(FORWARD);
     motor2.run(FORWARD);
     motor3.run(FORWARD);
     motor4.run(FORWARD);
     turnLeft = true;
     forward = true;
     delay(20);
     return;
}

/*
 * The following four functions are all intended for the car to be stopped,
 * and then execute a 90 degree turn. 
 * The car can turn left and right, going forward and backward. 
 * The delay in each of the function is intended to be calibrated to ensure
 * the turn lasts until a 90 degree turn is made. 
 */
 int turnHigh = 120;
 int turnLow = 40;
void backLeft(int speedIn) {
     int turnSpeed = speedIn + turnHigh;
     motor1.setSpeed(turnSpeed);
     motor2.setSpeed(turnSpeed);
     motor3.setSpeed(speedIn-turnLow);
     motor4.setSpeed(speedIn-turnLow);
     motor1.run(BACKWARD);
     motor2.run(BACKWARD);
     motor3.run(BACKWARD);
     motor4.run(BACKWARD);
     forward = false;
     backward = true;
     delay(backUpTime);
     return;
}


void backRight(int speedIn) {
     int turnSpeed = speedIn + turnHigh;
     motor3.setSpeed(turnSpeed);
     motor4.setSpeed(turnSpeed);
     motor1.setSpeed(speedIn-turnLow);
     motor2.setSpeed(speedIn-turnLow);
     motor1.run(BACKWARD);
     motor2.run(BACKWARD);
     motor3.run(BACKWARD);
     motor4.run(BACKWARD);
     forward = false;
     backward = true;
     delay(backUpTime);
     return;
}

void frontLeft(int speedIn) {
     int turnSpeed = speedIn + turnHigh;
     motor1.setSpeed(turnSpeed);
     motor2.setSpeed(turnSpeed);
     motor3.setSpeed(speedIn-turnLow);
     motor4.setSpeed(speedIn-turnLow);
     motor1.run(FORWARD);
     motor2.run(FORWARD);
     motor3.run(FORWARD);
     motor4.run(FORWARD);
     forward = false;
     backward = true;
     delay(forTurnTime);
     return;
}



void frontRight(int speedIn) {
     int turnSpeed = speedIn + turnHigh;
     motor3.setSpeed(turnSpeed);
     motor4.setSpeed(turnSpeed);
     motor1.setSpeed(speedIn-turnLow);
     motor2.setSpeed(speedIn-turnLow);
     motor1.run(FORWARD);
     motor2.run(FORWARD);
     motor3.run(FORWARD);
     motor4.run(FORWARD);
     forward = false;
     backward = true;
     delay(forTurnTime);
     return;
}

/*
 * This function was built to test if the car could turn in place 180 degrees. 
 * It failed in practice because there was not enough friction between the 
 * ground and the wheels. 
 */

void turn180(int speedIn) {
     int turnSpeed = speedIn+ 50;
     motor4.setSpeed(turnSpeed);
     motor3.setSpeed(turnSpeed);
     motor1.setSpeed(turnSpeed);
     motor2.setSpeed(turnSpeed);
     motor1.run(BACKWARD);
     motor2.run(BACKWARD);
     motor3.run(FORWARD);
     motor4.run(FORWARD);
     turnRight = true;
     forward = true;
     delay(1000);
     return;
}


/*
 * The following two functions relay if there is an object within
 * 15 cm on the right of left side of the car. 
 */

bool objectRight(int cmR){
  if (cmR <= 15){
      return true;
  }
  else {
    return false;
  }
}

bool objectLeft(int cmL){
  if (cmL <= 15){
      return true;
  }
  else {
    return false;
  }
}



void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    roadSpeedSet(roadMotorSpeed);
}

void loop() {
  // put your main code here, to run repeatedly:


  /*
   * We start by reading an analog value, which is connected to our On/Off
   * switch. If there is a large value, we start the car. If the value is
   * less than a cutoff level, the don't start the car. 
   * Note our switch used is not perfect and there is sometimes enough bleed 
   * through for the car to turn on briefly when it should be off. 
   */
  int start = analogRead(7); 
  Serial.println(start);

  if (start > 400){
      
      /*
       * Now we start reading the sensor values. 
       */
      cm1 = (ultrasonic1.Ranging(CM)); //back
      cm2 = (ultrasonic2.Ranging(CM)); //right
      cm3 = (ultrasonic3.Ranging(CM)); // front 
      cm4 = (ultrasonic4.Ranging(CM)); //left
       // Full cruise mode, no objects ahead
      Serial.println(cm3);
      Serial.println(cm2);
      Serial.println(cm4);
      if (cm3 >45){
         Serial.println("forward");
         turnLeft = false;
         turnRight = false;
         roadSpeedSet(roadMotorSpeed);
         roadSpeedRun();
      }
      /*
       * If an object is really close to the back, we stop. This function
       * is intended for the car to enter a parking mode, and then stop. 
       */
      if (cm1 <= 10){
        stopRun();
        delay(1000);
      }
  
      /*
       * Below we have a situtation where an object is between 25 and 
       * 45 cm ahead. If there is no object to the right, the car will
       * slowly drift to the right. If no object to the left, but an
       * object on the right, it will drift to the left. If there is an
       * object on both sides, it will go straight at a slower speed. 
       * 
       * Note, in practice, the effects of this function are hard to see
       * because the car traverses the 20 cm range of this function quite 
       * quickly. 
       */
      else if (cm3 >25 && cm3<=45 && cm1 > 10){
        roadSpeedSet(slowMotorSpeed);
        if (!objectRight(cm2)){
          turnRightSpeed(slowMotorSpeed);
        }
        else if (!objectLeft(cm4)){
          turnLeftSpeed(slowMotorSpeed);
        }
        else if (objectLeft(cm4) && objectRight(cm2)){
          roadSpeedRun();
        }
      }


      /*
       * Below is where the accident prevention and turning capabilities
       * come into play. 
       * This section of the loop only activates if an object is within 
       * 25cm of the front sensor, telling the car to stop and then turn
       * a certain direction depending on its surroundings. 
       */
      else if (cm3<=25 && cm1 > 10){
        Serial.println("object ahead");

        stopRun();
        delay(300);

        /*
         * Here is the case for no objects beings on the left or right.
         * If there car recently turned left, it will turn right and vice
         * verse. 
         */
        if (!objectRight(cm2) && !objectLeft(cm4)){

          if (turnLeft){
            Serial.println("Simple turn Left");
            backRight(90);
            turnLeft = false;
            turnRight = true;
          }
          else{
            backLeft(90);
            Serial.println("Simple turn Right");
            turnLeft = true;
            turnRight = false; 
          }
        }

        /*
         * Here is the case where an object is on the left.
         * The car will back up, pause, and then move forward and right. 
         */
        else if (objectLeft(cm4) && !objectRight(cm2)){ 
          roadSpeedSet(roadMotorSpeed);
          roadSpeedBack();
          stopRun();
          delay(1000);
          frontRight(70);
          stopRun();
          delay(500);
          turnRight = true;
          Serial.println("object left");
        }
        /*
         * Here is the case where an object is on the right. 
         * The car will back up, pause, and then move forward and left. 
         */
        else if (objectRight(cm2) && !objectLeft(cm4)){
          roadSpeedSet(roadMotorSpeed);
          roadSpeedBack();
          stopRun();
          delay(1000);
          frontLeft(70);
          stopRun();
          delay(500);
          turnLeft = true;
          Serial.println("object right");
  
        }
        /*
         * If an object is on both sides, the car just moves backwards and pauses. 
         */
        else if (objectLeft(cm4) && objectRight(cm2)){
          roadSpeedSet(roadMotorSpeed);
          roadSpeedBack();
          delay(1000);
          Serial.println("Object Both Sides");
  
        }

      }
  }
  else if (start < 400){
    stopRun();
  }
    

}




