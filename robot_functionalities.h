#include <Arduino.h>
#include <SR04.h>
#include <pitches.h>
#include <Wire.h>
#include <IRremote.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal.h>
#include "robot_constants.h"

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
int lowSpeed = 3;
int midSpeed = 2;
int highSpeed = 1;
int servoPos0 = ANGLE_CENTER;
int redPin= 53;
int greenPin = 51;
int bluePin = 49;
bool stoppedFront = false;
bool stoppedBack = false;

void setRearLightsColor(int redValue, int greenValue,  int blueValue) 
{
  analogWrite(redPin, redValue);
  analogWrite(greenPin,  greenValue);
  analogWrite(bluePin, blueValue);
}

void lcdPrint(uint8_t line, String message)
{
  if(line == 0){
    lcd.clear();
  }
  lcd.setCursor(0, line);
  lcd.print(message);
}

void rotateRevoluteJoint(uint8_t joint, uint16_t angle)
{
  pwm.setPWM(joint, 0, angle);
}

void sendToESP32(String message)
{
  Serial.println(message);
}

void speakerBeep()
{
  tone(8, NOTE_D5, 100);
  delay(10);
  tone(8, NOTE_G5, 100);
  delay(10);
  tone(8, NOTE_F5, 100);
  delay(10);
  tone(8, NOTE_A5, 100);
}

void robotArmInitialization()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initial position");
    sendToESP32("Initial position");
    rotateRevoluteJoint(ARM_BASE, ANGLE_CENTER);
    servoPos0 = ANGLE_CENTER;
    delay(1000);
    rotateRevoluteJoint(ARM_JOINT_2, 500);
    delay(50);
    rotateRevoluteJoint(ARM_JOINT_1, 150);
    delay(50);
    rotateRevoluteJoint(ARM_JOINT_3, 420);
    delay(500);
    rotateRevoluteJoint(ARM_JOINT_4, 100);
    delay(500);
    rotateRevoluteJoint(ARM_EOAT, 200);
}

void robotMovementOn()
{
    if(!stoppedFront){
      stoppedBack = false;
      lcdPrint(0, "Robot moving...");
      digitalWrite(CATERPILLAR_LEFT, 130);
      digitalWrite(CATERPILLAR_RIGHT, 180);
      digitalWrite(CATERPILLAR_LEFT_BACKWARD, LOW);
      digitalWrite(CATERPILLAR_LEFT_FORWARD, HIGH);
      digitalWrite(CATERPILLAR_RIGHT_BACKWARD, LOW);
      digitalWrite(CATERPILLAR_RIGHT_FORWARD, HIGH);
      setRearLightsColor(255, 0, 0);
    }
}

void robotMovementRight()
{
    stoppedFront = false;
    lcdPrint(0, "Robot moving...");
    digitalWrite(CATERPILLAR_LEFT, 255);
    digitalWrite(CATERPILLAR_RIGHT, 255);
    digitalWrite(CATERPILLAR_LEFT_BACKWARD, LOW);
    digitalWrite(CATERPILLAR_LEFT_FORWARD, HIGH);
    digitalWrite(CATERPILLAR_RIGHT_BACKWARD, HIGH);
    digitalWrite(CATERPILLAR_RIGHT_FORWARD, LOW);
}

void robotMovementLeft()
{
    stoppedFront = false;
    lcdPrint(0, "Robot moving...");
    digitalWrite(CATERPILLAR_LEFT, 255);
    digitalWrite(CATERPILLAR_RIGHT, 255);
    digitalWrite(CATERPILLAR_LEFT_BACKWARD, HIGH);
    digitalWrite(CATERPILLAR_LEFT_FORWARD, LOW);
    digitalWrite(CATERPILLAR_RIGHT_BACKWARD, LOW);
    digitalWrite(CATERPILLAR_RIGHT_FORWARD, HIGH);
}

void robotMovementBackward()
{
    if(!stoppedBack){
      stoppedFront = false;
      lcdPrint(0, "Robot moving...");
      digitalWrite(CATERPILLAR_LEFT, 180);
      digitalWrite(CATERPILLAR_RIGHT, 130);
      digitalWrite(CATERPILLAR_LEFT_BACKWARD, HIGH);
      digitalWrite(CATERPILLAR_LEFT_FORWARD, LOW);
      digitalWrite(CATERPILLAR_RIGHT_BACKWARD, HIGH);
      digitalWrite(CATERPILLAR_RIGHT_FORWARD, LOW);
      setRearLightsColor(244, 255, 168);
    }
}

void robotMovementOff()
{
    digitalWrite(CATERPILLAR_LEFT, 0);
    digitalWrite(CATERPILLAR_RIGHT, 0);
    setRearLightsColor(0, 0, 255);
    lcdPrint(0, "Robot stopped...");
}

void robotArmActivation()
{
    lcdPrint(0, "Arm activation...");
    sendToESP32("Arm activation...");
    pwm.wakeup();
    delay(100);
    rotateRevoluteJoint(ARM_BASE, ANGLE_CENTER);
    servoPos0 = ANGLE_CENTER;
    delay(1000);
    rotateRevoluteJoint(ARM_JOINT_2, 500);
    delay(50);
    rotateRevoluteJoint(ARM_JOINT_1, 150);
    delay(50);
    rotateRevoluteJoint(ARM_JOINT_3, 420);
    delay(500);
    rotateRevoluteJoint(ARM_JOINT_4, GRIPPER_HORIZONTAL);
    delay(500);
    rotateRevoluteJoint(ARM_EOAT, GRIPPER_OPEN);
    delay(800);
    digitalWrite(LASER_PIN, HIGH);
    isArmOn = true;
    lcdPrint(0, "Arm activated...");
    sendToESP32("Arm activated...");
}

void robotArmDeactivation()
{
    lcdPrint(0, "Arm deactivation...");
    sendToESP32("Arm deactivation...");
    rotateRevoluteJoint(ARM_BASE, ANGLE_CENTER);
    servoPos0 = ANGLE_CENTER;
    delay(800);
    rotateRevoluteJoint(ARM_JOINT_4, GRIPPER_VERTICAL);
    delay(1000);
    rotateRevoluteJoint(ARM_JOINT_1, 90);
    delay(50);
    rotateRevoluteJoint(ARM_JOINT_2, 550);
    delay(50);
    rotateRevoluteJoint(ARM_JOINT_3, 650);
    delay(200);
    pwm.sleep();
    digitalWrite(LASER_PIN, LOW);
    isArmOn = false;
    lcdPrint(0, "Arm deactivated...");
    sendToESP32("Arm deactivated...");
}

void robotArmRestPosition()
{
    lcdPrint(0, "Arm resting...");
    sendToESP32("Arm resting...");
    rotateRevoluteJoint(ARM_BASE, ANGLE_CENTER);
    servoPos0 = ANGLE_CENTER;
    delay(800);
    rotateRevoluteJoint(ARM_JOINT_4, GRIPPER_VERTICAL);
    delay(1000);
    rotateRevoluteJoint(ARM_JOINT_1, 90);
    delay(50);
    rotateRevoluteJoint(ARM_JOINT_2, 550);
    delay(50);
    rotateRevoluteJoint(ARM_JOINT_3, 650);
}

void robotArmTurn(int angle)
{
    lcdPrint(0, "Moving arm...");
    Serial.print("Rotating arm to pos=");
    sendToESP32(String(angle));
    if(angle > servoPos0) {
      for (servoPos0; servoPos0 <= angle; servoPos0 += servoSpeed) {
        rotateRevoluteJoint(ARM_BASE, servoPos0);
        delay(1);
      }
    } else{
      for (servoPos0; servoPos0 >= angle; servoPos0 -= servoSpeed) {
        rotateRevoluteJoint(ARM_BASE, servoPos0);
        delay(1);
      }
    }    

    sendToESP32("Arm rotated to desired position");
}

void robotArmMoveVertically(int angle)
{
  int pwm1 = map(angle, 0, 100, ANGLE_JOINT_1_UP, ANGLE_JOINT_1_DOWN);
  int pwm2 = map(angle, 0, 100, ANGLE_JOINT_2_UP, ANGLE_JOINT_2_DOWN);
  sendToESP32("Rotating arm vertically to servo1 pos=" + String(pwm1) + " and servo2 pos=" + String(pwm2));
  rotateRevoluteJoint(ARM_JOINT_1, pwm1);
  delay(100);
  rotateRevoluteJoint(ARM_JOINT_2, pwm2);
}

void robotArmTurnLeft()
{
    lcdPrint(0, "Moving arm...");
    sendToESP32("Moving arm to the left...");
    for (servoPos0; servoPos0 >= ANGLE_LEFT; servoPos0 -= servoSpeed) {
      rotateRevoluteJoint(ARM_BASE, servoPos0);
      delay(1);
    }
}


void robotArmTurnRight()
{
    lcdPrint(0, "Moving arm...");
    sendToESP32("Moving arm to the right...");
    for (servoPos0; servoPos0 <= ANGLE_RIGHT; servoPos0 += servoSpeed) {
      rotateRevoluteJoint(ARM_BASE, servoPos0);
      delay(1);
    }
}

void robotArmCenteredPosition()
{
    lcdPrint(0, "Moving arm...");
    sendToESP32("Centering arm...");
    if(ANGLE_CENTER > servoPos0) {
      for (servoPos0; servoPos0 <= ANGLE_CENTER; servoPos0 += servoSpeed) {
        rotateRevoluteJoint(ARM_BASE, servoPos0);
        delay(1);
      }
    } else{
      for (servoPos0; servoPos0 >= ANGLE_CENTER; servoPos0 -= servoSpeed) {
        rotateRevoluteJoint(ARM_BASE, servoPos0);
        delay(1);
      }
    }
}

void triggerAlarm(int dist)
{
    if(!stoppedFront){
      stoppedFront = true;
      robotMovementOff();
      lcdPrint(0, "WARNING!");
      lcdPrint(1, "OBJECT DETECTED");
      setRearLightsColor(255, 255, 255);
      speakerBeep();
      delay(300);
      setRearLightsColor(0, 0, 0);
      delay(300);
      setRearLightsColor(255, 255, 255);
      speakerBeep();
      delay(300);
      setRearLightsColor(0, 0, 0);
      delay(300);
      setRearLightsColor(255, 255, 255);
      speakerBeep();
    }

    delay(10);
}

void turnLaserOn()
{
    lcdPrint(0, "Laser on!");
    sendToESP32("Turning laser on...");
    digitalWrite(LASER_PIN, HIGH);
}

void turnLaserOff()
{
    lcdPrint(0, "Laser off!");
    sendToESP32("Turning laser off...");
    digitalWrite(LASER_PIN, LOW);
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
    lcdPrint(0, "Arm up...");
    sendToESP32("Moving arm up...");
    rotateRevoluteJoint(ARM_JOINT_2, 580);
    delay(200);
    rotateRevoluteJoint(ARM_JOINT_1, ANGLE_JOINT_1_UP);
    delay(100);
    rotateRevoluteJoint(ARM_JOINT_2, ANGLE_JOINT_2_UP);
}

void robotArmMoveDown()
{
    lcdPrint(0, "Arm down...");
    sendToESP32("Moving arm down...");
    rotateRevoluteJoint(ARM_JOINT_1, ANGLE_JOINT_1_DOWN);
    delay(100);
    rotateRevoluteJoint(ARM_JOINT_2, ANGLE_JOINT_2_DOWN);
}

void robotEndEffectorOpen()
{
    lcdPrint(0, "Opening EOAT...");
    sendToESP32("Opening end effector...");
    rotateRevoluteJoint(ARM_EOAT, GRIPPER_OPEN);
}

void robotEndEffectorClose()
{
    lcdPrint(0, "Closing EOAT...");
    sendToESP32("Closing end effector...");
    rotateRevoluteJoint(ARM_EOAT, GRIPPER_CLOSED);
}

void robotEndEffectorRotate()
{
    lcdPrint(0, "Rotating EOAT...");
    sendToESP32("Rotating end effector...");
    if (!isEndEffectorRotated)
    {
        rotateRevoluteJoint(ARM_JOINT_4, GRIPPER_VERTICAL);
    }
    else
    {
        rotateRevoluteJoint(ARM_JOINT_4, GRIPPER_HORIZONTAL);
    }
    isEndEffectorRotated = !isEndEffectorRotated;
}

void pickAndMoveRight()
{
    lcdPrint(0, "Robot arm demo...");
    sendToESP32("Robot arm demo...");
    pwm.wakeup();
    delay(100);
    robotArmCenteredPosition();
    delay(1000);
    robotEndEffectorOpen();
    delay(600);
    robotArmMoveDown();
    delay(100);
    robotEndEffectorClose();
    delay(500);
    robotArmMoveUp();
    delay(2000);
    robotEndEffectorRotate();
    delay(1000);
    robotArmTurnLeft();
    delay(1000);
    robotArmMoveDown();
    delay(800);
    robotEndEffectorOpen();
    delay(1000);
    robotArmMoveUp();
    delay(2000);
    robotArmInitialization();
}

void toggleRobotArm()
{
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
    Serial1.setTimeout(5);
    Serial.begin(9600);
    pinMode(LASER_PIN, OUTPUT);
    analogWrite(6, contrast);
    lcd.begin(16, 2);
    IrReceiver.begin(29);
    pwm.begin();
    pwm.setPWMFreq(60);
    pinMode(CATERPILLAR_LEFT_BACKWARD, OUTPUT);
    pinMode(CATERPILLAR_LEFT_FORWARD, OUTPUT);
    pinMode(CATERPILLAR_RIGHT_BACKWARD, OUTPUT);
    pinMode(CATERPILLAR_RIGHT_FORWARD, OUTPUT);
    pinMode(CATERPILLAR_LEFT, OUTPUT);
    pinMode(CATERPILLAR_RIGHT, OUTPUT);
    pinMode(redPin,  OUTPUT);              
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    digitalWrite(CATERPILLAR_LEFT_BACKWARD, LOW);
    digitalWrite(CATERPILLAR_LEFT_FORWARD, LOW);
    digitalWrite(CATERPILLAR_RIGHT_BACKWARD, LOW);
    digitalWrite(CATERPILLAR_RIGHT_FORWARD, LOW);
    digitalWrite(CATERPILLAR_LEFT, 0);
    digitalWrite(CATERPILLAR_RIGHT, 0);
    robotArmActivation();
    turnLaserOn();
    lcdPrint(0, "ARDM2 ROBOT");
    lcdPrint(1, "IS OPERATIONAL.");
    setRearLightsColor(0, 0, 255);
    sendToESP32("Robot fully operational...");
}
