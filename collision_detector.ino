#include "robot_functionalities.h"

void setup()
{
  robotStartUp();
}

void decodeIRSignal() 
{
  switch (IrReceiver.decodedIRData.command)
  {

    case REMOTE_REWIND:
      robotArmTurnLeft();
      break;

    case REMOTE_FAST_FORWARD:
      robotArmTurnRight();
      break;

    case REMOTE_EQ:
      pickAndMoveRight();
      break;

    case REMOTE_VOL_UP:
      robotEndEffectorOpen();
      break;

    case REMOTE_VOL_DOWN:
      robotEndEffectorClose();
      break;

    case REMOTE_NUMPAD_0:
      toggleRobotArm();
      break;

    case REMOTE_ST_REPT:
      robotEndEffectorRotate();
      break;

    case REMOTE_ARROW_DOWN:
      robotArmMoveDown();
      break;

    case REMOTE_ARROW_UP:
      robotArmMoveUp();
      break;

    case REMOTE_PAUSE:
      robotArmCenteredPosition();
      break;

    case REMOTE_FUNC_STOP:
      controlRobotLaser();
      break;

    case REMOTE_POWER:
      if (isArmOn)
      {
        robotArmDeactivation();
      }
      else
      {
        robotArmActivation();
      }
      break;

    case REMOTE_NUMPAD_5:
      robotMovementOff();
      break;

    case REMOTE_NUMPAD_2:
      robotMovementOn();
      break;

    case REMOTE_NUMPAD_8:
      robotMovementBackward();
      break;

    case REMOTE_NUMPAD_4:
      robotMovementLeft();
      break;
    
    case REMOTE_NUMPAD_6:
      robotMovementRight();
      break;

  default:
    break;
  }
}

void loop()
{
  if(Serial1.available()){
    String received = Serial1.readStringUntil('\n');
    received.trim();
    if(received == ROBOT_LASER_TOGGLE){
        controlRobotLaser();
    } else if (received == ROBOT_CATERPILLAR_STOP) {
        robotMovementOff();
    } else if (received == ROBOT_END_EFFECTOR_OPEN) {
        robotEndEffectorOpen();
    } else if (received == ROBOT_END_EFFECTOR_CLOSE) {
        robotEndEffectorClose();
    } else if (received == ROBOT_END_EFFECTOR_ROTATE) {
        robotEndEffectorRotate();
    } else if (received == ROBOT_ARM_MOVE_UP) {
        robotArmMoveUp();
    } else if (received == ROBOT_ARM_MOVE_DOWN) {
        robotArmMoveDown();
    } else if (received == ROBOT_ARM_MOVE_LEFT) {
        robotArmTurnLeft();
    } else if (received == ROBOT_ARM_MOVE_RIGHT) {
        robotArmTurnRight();
    } else if (received == ROBOT_ARM_CENTERED) {
        robotArmCenteredPosition();
    } else if (received == ROBOT_ARM_REST_TOGGLE) {
        toggleRobotArm();
    } else if (received == ROBOT_ARM_DEMO) {
        pickAndMoveRight();
    } else if (received == ROBOT_ARM_POWER_TOGGLE) {
      if (isArmOn){
        robotArmDeactivation();
      } else {
        robotArmActivation();
      }
    } else if (received.startsWith(ROBOT_ARM_CUSTOM_HORIZONTAL_MOVEMENT)) {
        int angle = received.substring(4, received.indexOf(' ')).toInt();
        robotArmTurn(angle);
    } else if (received.startsWith(ROBOT_ARM_CUSTOM_VERTICAL_MOVEMENT)) {
        int value = received.substring(4, received.indexOf(' ')).toInt();
        sendToESP32("Attempted to move the arm vertically using the slider");
        //robotArmMoveVertically(angle);
    } else if (received == ROBOT_CATERPILLAR_MOVE_FORWARD) {
        robotMovementOn();
    } else if (received == ROBOT_CATERPILLAR_MOVE_BACKWARD) {
        robotMovementBackward();
    } else if (received == ROBOT_CATERPILLAR_MOVE_LEFT) {
        robotMovementLeft();
    } else if (received == ROBOT_CATERPILLAR_MOVE_RIGHT) {
        robotMovementRight();
    }
  }

  if (IrReceiver.decode()){
    lcd.clear();
    decodeIRSignal();
    IrReceiver.resume();
  }

  dist = sr04.Distance();

  if (dist < COLLISION_DISTANCE){
    triggerAlarm(dist);
  }
}