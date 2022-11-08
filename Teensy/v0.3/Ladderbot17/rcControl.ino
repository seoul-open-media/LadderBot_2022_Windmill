void rcControl() {
  rcServo(getValue(0), getValue(1));
  rcBrake(getValue(2), getValue(3));
  rcActuator(getValue(4));
  rcActuatorRange(getValue(5));
}

void rcServo(const int &ch_value1, const int &ch_value2) {
  uint16_t value = (SERVOMAX - SERVOMIN) / 2;
  uint16_t value1 = (SERVOMAX - SERVOMIN) / 2;
  
  int16_t servo_value = map(ch_value1, 0, 150, FRONT_MIN, FRONT_MAX); // calculate from ch_value1
  int16_t servo_value1 = map(ch_value1, 0, 150, REAR_MAX, REAR_MIN); // calculate from ch_value1

  value = ((SERVOMAX - SERVOMIN) / 2 + SERVOMIN) + (servo_value * (SERVOMAX - SERVOMIN) / 100);
  value1 = ((SERVOMAX - SERVOMIN) / 2 + SERVOMIN) + (servo_value1 * (SERVOMAX - SERVOMIN) / 100);
  
  pwm1_.setPWM(SERVO_1_PIN_, 0, value);
  pwm1_.setPWM(SERVO_2_PIN_, 0, value1);
//  Serial.print("ch_value1: "); Serial.println(ch_value1);
//  Serial.println(servo_value);
//  if (ch_value2 - OFFSET > MIDDLE_VALUE) {
////    Serial.println("front servo");
//    pwm1_.setPWM(SERVO_1_PIN_, 0, value);
//  } else if (ch_value2 + OFFSET < MIDDLE_VALUE) {
////    Serial.println("rear servo");
//    pwm1_.setPWM(SERVO_2_PIN_, 0, value);
//  } else {
//    // do nothing
//  }
}

void rcBrake(const int &ch_value3, const int &ch_value4) {
  if (ch_value3 - OFFSET > MIDDLE_VALUE) {
    
    // front brake
    if (ch_value4 - OFFSET > MIDDLE_VALUE) {
//      brake1(); // brake right
    } else if (ch_value4 + OFFSET < MIDDLE_VALUE) {
//      brake2(); // brake left
    } else {
      brake12();
//      Serial.println("front brake");
    }
  } else if (ch_value3 + OFFSET < MIDDLE_VALUE) {
    // rear brake
    brake34();
//    Serial.println("rear brake");
  } else {
    brakeAllRelease();
//    Serial.println("no brake");
  }
}

void rcActuator(const int &ch_value5) {
  if (ch_value5 < MIDDLE_VALUE) {
    runActuator();
  } else {
    stopActuator();
  }
}

void rcActuatorRange(const int &ch_value6) {
  if (ch_value6 > MIDDLE_VALUE) {
    sensor_min = DOWN_MIN;
    sensor_max = DOWN_MAX;
  } else {
    sensor_min = UP_MIN;
    sensor_max = UP_MAX;
  }
}

void runActuator() {
  switch (a_state) {
    case STATE_GOING_DOWN:
      if (sensor_value > sensor_min) {
//        decelMin();
        pushActuator();
      } else {
        a_state = STATE_GOING_UP;
      }
      break;

    case STATE_GOING_UP:
      if (sensor_value < sensor_max) {
//        decelMax();
        pullActuator();
      } else {
        a_state = STATE_GOING_DOWN;
      }
      break;

    default:
      break;
  }
}
