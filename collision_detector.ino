#include "robot_functionalities.h"

void setup()
{
  robotStartUp();
}

void decodeIRSignal() 
{
  switch (IrReceiver.decodedIRData.command)
  {

  case 68: //rewind
    robotArmTurnLeft();
    break;

  case 67: //fast forward
    robotArmTurnRight();
    break;

  case 25: //EQ
    pickAndMoveRight();
    break;

  case 70: //vol+
    robotEndEffectorOpen();
    break;

  case 21: //vol-
    robotEndEffectorClose();
    break;

  case 22: //0
    toggleRobotArm();
    break;

  case 13: //ST/REPT
    robotEndEffectorRotate();
    break;

  case 7: //arrow down
    robotArmMoveDown();
    break;

  case 9: //arrow up
    robotArmMoveUp();
    break;

  case 64: //pause
    robotArmCenteredPosition();
    break;

  case 71: //func/stop
    controlRobotLaser();
    break;

  case 69: //power
    if (isArmOn)
    {
      robotArmDeactivation();
      //robotMovementOn();
    }
    else
    {
      //robotMovementOff();
      robotArmActivation();
    }
    break;

  case 28: //5
    robotMovementOff();
    break;

  case 24: //2
    robotMovementOn();
    break;

  case 82: //8
    robotMovementBackward();
    break;

  case 8: //4
    robotMovementLeft();
    break;
  
  case 90: //6
    robotMovementRight();
    break;

  default:
    break;
   /* if (!isArmOn)
    {
      IrReceiver.stop();
      IrReceiver.begin(29, ENABLE_LED_FEEDBACK);
    }
    else
    {
      break;
    }*/
  
  }
}

void loop()
{

  if(Serial1.available()){
    String received = Serial1.readString();
    received.trim();
    if(received == "laserToggle"){
        controlRobotLaser();
    } else if (received == "o") {
        robotEndEffectorOpen();
    } else if (received == "c") {
        robotEndEffectorClose();
    } else if (received == "rotateEndEffector") {
        robotEndEffectorRotate();
    } else if (received == "armUp") {
        robotArmMoveUp();
    } else if (received == "armDown") {
        robotArmMoveDown();
    } else if (received == "armLeft") {
        robotArmTurnLeft();
    } else if (received == "armRight") {
        robotArmTurnRight();
    } else if (received == "armCentered") {
        robotArmCenteredPosition();
    } else if (received == "armResting") {
        toggleRobotArm();
    } else if (received == "armDemoMode") {
        pickAndMoveRight();
    } else if (received == "armPowerToggle") {
      if (isArmOn){
        robotArmDeactivation();
      } else {
        robotArmActivation();
      }
    } else if (received.startsWith("rotateArm")) {
        int angle = received.substring(10).toInt();
        //robotArmTurn(angle);
        robotArmGraduallyTurn(angle);
    } else if (received.startsWith("moveArmVertically")) {
        int value = received.substring(17).toInt();
        Serial.println("Attempted to move the arm vertically using the slider");
        //robotArmMoveVertically(angle);
    }
  }

  if (IrReceiver.decode()){
    lcd.clear();
    decodeIRSignal();
    IrReceiver.resume();
  }

  dist = sr04.Distance();

  if (dist < 20){
    Serial.print("OBSTACLE DETECTED! (");
    Serial.print(dist);
    Serial.println("cm)");
    triggerAlarm(dist);
  }
}