void modeBasicMovement() {
  switch (function) {
    case 0:
      //stopBasicMovement();
      rc_state = RCSTATE_INIT;
      break;
    case 1:
      goStraight();
      break;
    case 2:
      goRight();
      break;
    case 3:
      goLeft();
      break;
    case 4:
      turnRight();
      break;
    case 5:
      turnLeft();
      break;
    default:
      break;
  }

}
void stopBasicMovement() {
  brakeAllRelease();
  servoControl(SERVO_STRAIGHT);
  stopActuator();
}

// 255

void goStraight() {
  switch (rc_state) {
    case RCSTATE_INIT:
      stopActuator();
      brakeAllRelease();
      rc_state = RCSTATE_PUSHFORWARD;
      break;

    case RCSTATE_STANDUP:
      if (sensor_value < sensor_max) {
//        decelMax();
        brakeAllRelease();
        servoControl(SERVO_STRAIGHT);
        pullActuator();
      } else {
        rc_state = RCSTATE_PUSHFORWARD;
      }
      break;

    case RCSTATE_PUSHFORWARD:
      if (sensor_value > sensor_min) {
//        decelMin();
        brake34();
        servoControl(SERVO_STRAIGHT);
        pushActuator();
      } else {
        rc_state = RCSTATE_PULLFORWARD;
      }
      break;

    case RCSTATE_PULLFORWARD:
      if (sensor_value < sensor_max) {
//        decelMax();
        brake12();
        servoControl(SERVO_STRAIGHT);
        pullActuator();

      } else {
        rc_state = RCSTATE_PUSHFORWARD;
      }

      break;

    default:
      break;

  }
}

void goBack() {
  switch (rc_state) {
    case RCSTATE_INIT:
      stopActuator();
      brakeAllRelease();
      rc_state = RCSTATE_PUSHFORWARD;
      break;

    case RCSTATE_STANDUP:
      if (sensor_value < sensor_max) {
//        decelMax();
        brakeAllRelease();
        servoControl(SERVO_STRAIGHT);
        pullActuator();
      } else {
        rc_state = RCSTATE_PUSHFORWARD;
      }
      break;

    case RCSTATE_PUSHFORWARD:
      if (sensor_value > sensor_min) {
//        decelMin();
        brake12();
        servoControl(SERVO_STRAIGHT);
        pushActuator();
      } else {
        rc_state = RCSTATE_PULLFORWARD;
      }
      break;

    case RCSTATE_PULLFORWARD:
      if (sensor_value < sensor_max) {
//        decelMax();
        brake34();
        servoControl(SERVO_STRAIGHT);
        pullActuator();

      } else {
        rc_state = RCSTATE_PUSHFORWARD;
      }

      break;

    default:
      break;

  }
}

void goLeft() {
  switch (rc_state) {
    case RCSTATE_INIT:
      rc_state = RCSTATE_STANDUP;
      break;

    case RCSTATE_STANDUP:
      if (sensor_value < sensor_max) {
        decelMax();
        brakeAllRelease();
        servoControl(SERVO_LEFT);
        pullActuator();
      } else {
        rc_state = RCSTATE_PUSHFORWARD;
      }
      break;

    case RCSTATE_PUSHFORWARD:
      if (sensor_value > sensor_min) {
        decelMin();
        brake34();
        servoControl(SERVO_LEFT);
        pushActuator();
      } else {
        rc_state = RCSTATE_PULLFORWARD;
      }
      break;

    case RCSTATE_PULLFORWARD:
      if (sensor_value < sensor_max) {
        decelMax();
        brake12();
        servoControl(SERVO_LEFT);
        pullActuator();

      } else {
        rc_state = RCSTATE_PUSHFORWARD;
      }

      break;

    default:
      break;

  }
}

void goRight() {
  switch (rc_state) {
    case RCSTATE_INIT:
      rc_state = RCSTATE_STANDUP;
      break;

    case RCSTATE_STANDUP:
      if (sensor_value < sensor_max) {
        decelMax();
        brakeAllRelease();
        servoControl(SERVO_RIGHT);
        pullActuator();
      } else {
        rc_state = RCSTATE_PUSHFORWARD;
      }
      break;

    case RCSTATE_PUSHFORWARD:
      if (sensor_value > sensor_min) {
        decelMin();
        brake34();
        servoControl(SERVO_RIGHT);
        pushActuator();
      } else {
        rc_state = RCSTATE_PULLFORWARD;
      }
      break;

    case RCSTATE_PULLFORWARD:
      if (sensor_value < sensor_max) {
        decelMax();
        brake12();
        servoControl(SERVO_RIGHT);
        pullActuator();

      } else {
        rc_state = RCSTATE_PUSHFORWARD;
      }

      break;

    default:
      break;

  }
}

void turnRight() {
  switch (rc_state) {
    case RCSTATE_INIT:
      stopActuator();
      brakeAllRelease();
      rc_state = RCSTATE_PUSH_TURN;
      break;

    case RCSTATE_STANDUP:
      if (sensor_value < sensor_max) {
//        decelMax();
        brakeAllRelease();
        servoControl(SERVO_RIGHT);
        pullActuator();
      } else {
        rc_state = RCSTATE_PUSH_TURN;
      }
      break;

    case RCSTATE_PUSH_TURN:
      if (sensor_value > sensor_min) {
//        decelMin();
        brake34();
        servoControl(SERVO_RIGHT);
        if(!rcServoOnly(getValue(5)))  pushActuator();
      } else {
        rc_state = RCSTATE_PULL_TURN;
      }
      break;

    case RCSTATE_PULL_TURN:
      if (sensor_value < sensor_max) {
//        decelMax();
        brake34();
        servoControl(SERVO_LEFT);
        if(!rcServoOnly(getValue(5)))  pullActuator();

      } else {
        rc_state = RCSTATE_PUSH_TURN;
      }

      break;

    default:
      break;

  }
}
void turnLeft() {
  switch (rc_state) {
    case RCSTATE_INIT:
      stopActuator();
      brakeAllRelease();
      rc_state = RCSTATE_PUSH_TURN;
      break;

    case RCSTATE_STANDUP:
      if (sensor_value < sensor_max) {
//        decelMax();
        brakeAllRelease();
        servoControl(SERVO_LEFT);
        pullActuator();
      } else {
        rc_state = RCSTATE_PUSH_TURN;
      }
      break;

    case RCSTATE_PUSH_TURN:
      if (sensor_value > sensor_min) {
//        decelMin();
        brake34();
        servoControl(SERVO_LEFT);
        if(!rcServoOnly(getValue(5)))  pushActuator();
      } else {
        rc_state = RCSTATE_PULL_TURN;
      }
      break;

    case RCSTATE_PULL_TURN:
      if (sensor_value < sensor_max) {
//        decelMax();
        brake34();
        servoControl(SERVO_RIGHT);
        if(!rcServoOnly(getValue(5)))  pullActuator();

      } else {
        rc_state = RCSTATE_PUSH_TURN;
      }

      break;

    default:
      break;

  }
}
