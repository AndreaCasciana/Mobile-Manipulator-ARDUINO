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
    String received = Serial1.readStringUntil('\n');
    received.trim();
    if(received == "0"){
        controlRobotLaser();
    } else if (received == "1") {
        robotEndEffectorOpen();
    } else if (received == "2") {
        robotEndEffectorClose();
    } else if (received == "3") {
        robotEndEffectorRotate();
    } else if (received == "4") {
        robotArmMoveUp();
    } else if (received == "5") {
        robotArmMoveDown();
    } else if (received == "6") {
        robotArmTurnLeft();
    } else if (received == "7") {
        robotArmTurnRight();
    } else if (received == "8") {
        robotArmCenteredPosition();
    } else if (received == "9") {
        toggleRobotArm();
    } else if (received == "10") {
        pickAndMoveRight();
    } else if (received == "11") {
      if (isArmOn){
        robotArmDeactivation();
      } else {
        robotArmActivation();
      }
    } else if (received.startsWith("rth")) {
        int angle = received.substring(3).toInt();
        //robotArmTurn(angle);
        robotArmGraduallyTurn(angle);
    } else if (received.startsWith("rtv")) {
        int value = received.substring(3).toInt();
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