void modeAutonomous() {


  switch (state) {
    case STATE_INIT: // 0
      reached_scene = false;
      if (distance < MOVE_THRESHOLD) {
        // it started at the destination point
        state = STATE_INIT;
        //  reached_destination = true;
        //  reached_scene = true;

      } else {
        reached_destination = false;
        complete = INCOMPLETE;
        state = STATE_TURN_DECISION;
      }
      break;

    case STATE_TURN_DECISION: //1
      //oled.clear();

      //oled.println("Move Forward");
      // check if LPS1_y is under or above the line between destination and my_coordinate
      if (isAbove(bx_norm, by_norm)) {

        //oled.println("Above, so turn clockwise");
        turn_type = TURN_RIGHT;
      } else {
        //oled.print("Below, so turn counterclockwise");
        turn_type = TURN_LEFT;
      }


      state = STATE_STANDUP_BEFORE_TURN;
      break;

    case STATE_STANDUP_BEFORE_TURN: //2
      if (sensor_value < sensor_max) {
        decelMax();
        brakeAllRelease();
        pullActuator();
      } else {
        state = STATE_CHECK_ANGLE;
      }
      break;

    case STATE_CHECK_ANGLE: //3
      // check the distance and get out of here

      // check angle
      if (xrange_filtered < X_THRESHOLD && yrange_filtered < Y_THRESHOLD) { // if theta is small enouth and facing forward to the destination,
        state = STATE_CHECK_DESTINATION;
      } else {
        state = STATE_PUSH_TURN;
      }
      break;

    case STATE_PUSH_TURN: //4
      // if (!sentBow) {
      //   sendBow();
      //   sentBow = true;
      // }

      //
      if (sensor_value > sensor_min) {
        decelMin();
        brake34();
        pushActuator();
        if (turn_type == TURN_LEFT) {
          servoControl(SERVO_LEFT);
        } else {
          servoControl(SERVO_RIGHT);
        }

        if (xrange_filtered < X_THRESHOLD && yrange_filtered < Y_THRESHOLD) { // if theta is small enouth and facing forward to the destination,
          state = STATE_CHECK_DESTINATION;
        } else {
          state = STATE_PUSH_TURN;
        }

      } else {
        state = STATE_PULL_TURN;
      }
      
      // to escape from here
      if (distance < MOVE_THRESHOLD) {
        state = STATE_FINISH;
      }

      break;

    case STATE_PULL_TURN: //5
      if (sentBow)  sentBow = false;

      if (sensor_value < sensor_max) {
        decelMax();
        brake34();
        pullActuator();
        if (turn_type == TURN_LEFT) {
          servoControl(SERVO_RIGHT);
        } else {
          servoControl(SERVO_LEFT);
        }
      } else {
        state = STATE_TURN_DECISION;
      }
      break;


    case STATE_CHECK_DESTINATION: //6
      if (distance < MOVE_THRESHOLD) {
        state = STATE_FINISH;
      } else {
        state = STATE_CHECK_THETA;
      }
      break;

    case STATE_CHECK_THETA: // 7 while moving foraward to the destination, check theta if it's going right direction.
      if (xrange_filtered < X_THRESHOLD && yrange_filtered < Y_THRESHOLD) { // if theta is small enouth and facing forward to the destination,
        state = STATE_PUSH_FORWARD;
      } else {
        state = STATE_TURN_DECISION; // We are off the track so goback to turn decision
      }
      break;



    case STATE_PUSH_FORWARD: // 8
      // if (!sentBow) {
      //   sendBow();
      //   sentBow = true;
      // }
      if (sensor_value > sensor_min) {
        decelMin();
        brake34();
        servoControl(SERVO_STRAIGHT);
        pushActuator();
      } else {
        state = STATE_PULL_FORWARD;
      }
      break;

    case STATE_PULL_FORWARD: // 9
      if (sentBow)  sentBow = false;
      if (sensor_value < sensor_max) {
        decelMax();
        brake12();
        servoControl(SERVO_STRAIGHT);
        pullActuator();

      } else {
        state = STATE_CHECK_DESTINATION;
      }

      break;

    case STATE_FINISH: // 10
      if (sensor_value < sensor_max) {
        decelMin();
        brakeAllRelease();
        pullActuator();
      } else {
        complete = COMPLETE;
        reached_destination = true;
        reached_scene = true;
        state = STATE_INIT;
      }


      break;

    default:
      break;

  }


}
