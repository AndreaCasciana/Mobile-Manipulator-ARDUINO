#include <Arduino.h>
#include <SR04.h>
#include <pitches.h>
#include <Wire.h>
#include <IRremote.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal.h>
#define TRIG_PIN 12
#define ECHO_PIN 11
#define SERVOMIN 150
#define SERVOMAX 600
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
long dist;
bool isArmOn = true;
bool isArmResting = false;
bool isLaserOn = true;
bool isEndEffectorRotated = false;
int contrast = 80;
LiquidCrystal lcd(13, 10, 5, 4, 3, 2);
int servoSpeed = 1;
int lowSpeed = 3; // Adjust for desired low speed
int midSpeed = 2; // Adjust for desired mid speed
int highSpeed = 1; // Adjust for desired high speed
int servoPos0 = 350;

void robotArmInitialization()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initial position");
    Serial.println("Initial position");
    pwm.setPWM(0, 0, 350);
    servoPos0 = 350;
    delay(1000);
    pwm.setPWM(2, 0, 500);
    delay(50);
    pwm.setPWM(1, 0, 150);
    delay(50);
    pwm.setPWM(3, 0, 420);
    delay(500);
    pwm.setPWM(4, 0, 100);
    delay(500);
    pwm.setPWM(5, 0, 200);
}

void pickAndMoveRight()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Robot arm demo...");
    Serial.println("Robot arm demo...");
    pwm.wakeup();
    delay(100);
    pwm.setPWM(0, 0, 350); // turn center
    delay(1000);
    pwm.setPWM(5, 0, 100); // open end effector
    delay(600);
    pwm.setPWM(2, 0, 550); // go down
    delay(50);
    pwm.setPWM(1, 0, 295); // go down
    delay(800);
    pwm.setPWM(3, 0, 420); // go down
    delay(100);
    pwm.setPWM(5, 0, 280); // grab
    delay(500);
    pwm.setPWM(1, 0, 150); // go up
    delay(50);
    pwm.setPWM(2, 0, 500); // go up
    delay(2000);
    pwm.setPWM(4, 0, 340);
    delay(1000);
    pwm.setPWM(0, 0, 100); // turn left
    delay(1000);
    pwm.setPWM(2, 0, 550); // go down
    delay(800);
    pwm.setPWM(1, 0, 300); // go down
    delay(800);
    pwm.setPWM(5, 0, 100); // open end effector
    delay(1000);
    pwm.setPWM(1, 0, 150); // go up
    delay(800);
    pwm.setPWM(2, 0, 480); // go up
    delay(2000);
    robotArmInitialization();
}

void robotMovementOn()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Robot moving...");
    Serial.println("Robot moving forward...");
    analogWrite(52, 130);
    analogWrite(42, 255);
    digitalWrite(50, LOW);
    digitalWrite(48, HIGH);
    digitalWrite(46, LOW);
    digitalWrite(44, HIGH);
}

void robotMovementRight()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Robot moving...");
    Serial.println("Robot turning right...");
    analogWrite(52, 255);
    analogWrite(42, 255);
    digitalWrite(50, LOW);
    digitalWrite(48, HIGH);
    digitalWrite(46, HIGH);
    digitalWrite(44, LOW);
}

void robotMovementLeft()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Robot moving...");
    Serial.println("Robot turning left...");
    analogWrite(52, 255);
    analogWrite(42, 255);
    digitalWrite(50, HIGH);
    digitalWrite(48, LOW);
    digitalWrite(46, LOW);
    digitalWrite(44, HIGH);
}

void robotMovementBackward()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Backward moving...");
    Serial.println("Robot moving backward...");
    analogWrite(52, 150);
    analogWrite(42, 255);
    digitalWrite(50, HIGH);
    digitalWrite(48, LOW);
    digitalWrite(46, HIGH);
    digitalWrite(44, LOW);
}

void robotMovementOff()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Robot stopped...");
    Serial.println("Robot stopped...");
    analogWrite(52, 0);
    analogWrite(42, 0);
    /*
    digitalWrite(50, LOW);
    digitalWrite(48, LOW);
    digitalWrite(46, LOW);
    digitalWrite(44, LOW);
    */
    //delay(1000);
}

void robotArmActivation()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Arm activation...");
    Serial.println("Arm activation...");
    pwm.wakeup();
    delay(100);
    pwm.setPWM(0, 0, 350);
    servoPos0 = 350;
    delay(1000);
    pwm.setPWM(2, 0, 500);
    delay(50);
    pwm.setPWM(1, 0, 150);
    delay(50);
    pwm.setPWM(3, 0, 420);
    delay(500);
    pwm.setPWM(4, 0, 100);
    delay(500);
    pwm.setPWM(5, 0, 100);
    delay(800);
    digitalWrite(33, HIGH);
    isArmOn = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Arm activated!");
    Serial.println("Arm activated...");
}

void robotArmDeactivation()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Arm deactivation...");
    Serial.println("Arm deactivation...");
    pwm.setPWM(0, 0, 350);
    servoPos0 = 350;
    delay(800);
    pwm.setPWM(4, 0, 340);
    delay(1000);
    pwm.setPWM(1, 0, 90);
    delay(50);
    pwm.setPWM(2, 0, 550);
    delay(50);
    pwm.setPWM(3, 0, 650);
    delay(200);
    pwm.sleep();
    digitalWrite(33, LOW);
    isArmOn = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Arm deactivated!");
    Serial.println("Arm deactivated...");
}

void robotArmRestPosition(){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Arm resting...");
    Serial.println("Arm resting...");
    pwm.setPWM(0, 0, 350);
    servoPos0 = 350;
    delay(800);
    pwm.setPWM(4, 0, 340);
    delay(1000);
    pwm.setPWM(1, 0, 90);
    delay(50);
    pwm.setPWM(2, 0, 550);
    delay(50);
    pwm.setPWM(3, 0, 650);
}

void robotArmTurnLeft()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moving arm...");
    Serial.println("Moving arm to the left...");
    for (servoPos0; servoPos0 >= 100; servoPos0 -= servoSpeed) {
      pwm.setPWM(0, 0, servoPos0);
      delay(1); // Adjust delay for desired speed
    }
    //pwm.setPWM(0, 0, 100);
}

void robotArmTurn(int angle)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moving arm...");
    Serial.print("Rotating arm to pos=");
    Serial.println(angle);
    if(angle > servoPos0) {
      for (servoPos0; servoPos0 <= angle; servoPos0 += servoSpeed) {
        pwm.setPWM(0, 0, servoPos0);
        delay(1); // Adjust delay for desired speed
      }
    } else{
      for (servoPos0; servoPos0 >= angle; servoPos0 -= servoSpeed) {
        pwm.setPWM(0, 0, servoPos0);
        delay(1); // Adjust delay for desired speed
      }
    }    

    Serial.println("Arm rotated to desired position");
    //pwm.setPWM(0, 0, angle);
}

void robotArmGraduallyTurn(int angle) {
  int totalMovement = abs(angle - servoPos0); // Total range of movement
  
  // Handle small movements directly
  if (totalMovement < 5) {
    servoPos0 = angle;
    pwm.setPWM(0, 0, servoPos0);
    return;
  }

  // Speed ranges
  int accelerationRange = totalMovement * 0.10; // First 10% for acceleration
  int decelerationRange = totalMovement * 0.10; // Last 10% for deceleration
  int highSpeedRange = totalMovement * 0.20;    // Middle 20% for high speed
  int constantRange = totalMovement - (accelerationRange + decelerationRange + highSpeedRange);

  // Determine movement direction (1 for forward, -1 for backward)
  int direction = (servoPos0 < angle) ? 1 : -1;

  // Accelerating phase
  for (int i = 0; i < accelerationRange; i++) {
    servoPos0 += direction; // Increment/decrement position
    pwm.setPWM(0, 0, servoPos0);
    delay(lowSpeed);
  }

  // High-speed phase
  for (int i = 0; i < highSpeedRange; i++) {
    servoPos0 += direction; // Increment/decrement position
    pwm.setPWM(0, 0, servoPos0);
    delay(highSpeed);
  }

  // Constant speed phase
  for (int i = 0; i < constantRange; i++) {
    servoPos0 += direction; // Increment/decrement position
    pwm.setPWM(0, 0, servoPos0);
    delay(midSpeed);
  }

  // Decelerating phase
  for (int i = 0; i < decelerationRange; i++) {
    servoPos0 += direction; // Increment/decrement position
    pwm.setPWM(0, 0, servoPos0);
    delay(lowSpeed);
  }

  // Ensure the final position is accurate
  servoPos0 = angle;
  pwm.setPWM(0, 0, servoPos0);
}


void robotArmTurnRight()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moving arm...");
    Serial.println("Moving arm to the right...");
    for (servoPos0; servoPos0 <= 600; servoPos0 += servoSpeed) {
      pwm.setPWM(0, 0, servoPos0);
      delay(1); // Adjust delay for desired speed
    }
    //pwm.setPWM(0, 0, 600);
}

void robotArmCenteredPosition()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Centering arm...");
    Serial.println("Centering arm...");
    if(350 > servoPos0) {
      for (servoPos0; servoPos0 <= 350; servoPos0 += servoSpeed) {
        pwm.setPWM(0, 0, servoPos0);
        delay(1); // Adjust delay for desired speed
      }
    } else{
      for (servoPos0; servoPos0 >= 350; servoPos0 -= servoSpeed) {
        pwm.setPWM(0, 0, servoPos0);
        delay(1); // Adjust delay for desired speed
      }
    }
    //pwm.setPWM(0, 0, 350); // turn center
}

void triggerAlarm(int dist)
{
    analogWrite(52, 0);
    analogWrite(42, 0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WARNING!");
    lcd.setCursor(0, 1);
    lcd.print("OBJECT DETECTED");
    /*
    tone(8, NOTE_D5, 100);
    delay(10);
    tone(8, NOTE_G5, 100);
    delay(10);
    tone(8, NOTE_F5, 100);
    delay(10);
    tone(8, NOTE_A5, 100);
    */
    delay(10);
}

void turnLaserOn()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Laser on!");
    Serial.println("Turning laser on...");
    digitalWrite(33, HIGH);
}

void turnLaserOff()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Laser off!");
    Serial.println("Turning laser off...");
    digitalWrite(33, LOW);
}

void controlRobotLaser()
{
    if (!isLaserOn)
    {
        turnLaserOn();
    }
    else
    {
        turnLaserOff();
    }
    isLaserOn = !isLaserOn;
}

void robotArmMoveUp()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Arm up...");
    Serial.println("Moving arm up...");
    pwm.setPWM(1, 0, 150); // go up
    delay(400);
    pwm.setPWM(2, 0, 500); // go up
}

void robotArmMoveDown()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Arm down...");
    Serial.println("Moving arm down...");
    pwm.setPWM(2, 0, 650); // go down
    delay(50);
    pwm.setPWM(1, 0, 345); // go down
}

void robotEndEffectorOpen()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Opening EOAT...");
    Serial.println("Opening end effector...");
    pwm.setPWM(5, 0, 100); // open end effector
}

void robotEndEffectorClose()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Closing EOAT...");
    Serial.println("Closing end effector...");
    pwm.setPWM(5, 0, 250); // grab
}

void robotEndEffectorRotate()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Rotating EOAT...");
    Serial.println("Rotating end effector...");
    if (!isEndEffectorRotated)
    {
        pwm.setPWM(4, 0, 340);
    }
    else
    {
        pwm.setPWM(4, 0, 100);
    }
    isEndEffectorRotated = !isEndEffectorRotated;
}

void toggleRobotArm(){
    if(!isArmResting){
        robotArmRestPosition();
    }else{
        robotArmInitialization();
    }
    isArmResting = !isArmResting;
}

void robotStartUp()
{
    Serial1.begin(4800);
    Serial1.setTimeout(10);
    Serial.begin(9600);
    pinMode(33, OUTPUT);
    analogWrite(6, contrast);
    lcd.begin(16, 2);
    IrReceiver.begin(29);
    pwm.begin();
    pwm.setPWMFreq(60);
    pinMode(50, OUTPUT);
    pinMode(48, OUTPUT);
    pinMode(46, OUTPUT);
    pinMode(44, OUTPUT);
    pinMode(52, OUTPUT);
    pinMode(42, OUTPUT);
    digitalWrite(50, LOW);
    digitalWrite(48, LOW);
    digitalWrite(46, LOW);
    digitalWrite(44, LOW);
    analogWrite(52, 0);
    analogWrite(42, 0);
    robotArmActivation();
    lcd.clear();
    turnLaserOn();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ARDM2 ROBOT");
    lcd.setCursor(0, 2);
    lcd.print("IS OPERATIONAL.");
    Serial.println("Robot fully operational...");
}
