void rcControl() {
  rcServo(getValue(0), getValue(1));
  rcBrake(getValue(2), getValue(3));
  rcActuator(getValue(4));
  rcActuatorRange(getValue(5));
}

void rcServo(const int &ch_value1, const int &ch_value2) {
  uint16_t value = (SERVOMAX - SERVOMIN) / 2;
  int16_t servo_value = map(ch_value1, 0, 100, -50, 50); // calculate from ch_value1

  value = ((SERVOMAX - SERVOMIN) / 2 + SERVOMIN) + (servo_value * (SERVOMAX - SERVOMIN) / 100);

  if (ch_value2 - OFFSET > MIDDLE_VALUE) {
    pwm1_.setPWM(SERVO_1_PIN_, 0, value);
  } else if (ch_value2 + OFFSET < MIDDLE_VALUE) {
    pwm1_.setPWM(SERVO_2_PIN_, 0, value);
  } else {
    // do nothing
  }
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
    }
  } else if (ch_value3 + OFFSET < MIDDLE_VALUE) {
    // rear brake
    brake34();
  } else {
    brakeAllRelease();
  }
}

void rcActuator(const int &ch_value5) {
  if (ch_value5 < MIDDLE_VALUE) {
    runActuator();
  } else {
    brakeAllRelease();
    stopActuator();
  }
}

void rcActuatorRange(const int &ch_value6) {
  if (ch_value6 > MIDDLE_VALUE) {
    sensor_min = 30;
    sensor_max = 65;
  } else {
    sensor_min = 45;
    sensor_max = 65;
  }
}

void runActuator() {
  switch (a_state) {
    case STATE_GOING_DOWN:
      if (sensor_value > sensor_min) {
        decelMin();
        pushActuator();
      } else {
        a_state = STATE_GOING_UP;
      }
      break;

    case STATE_GOING_UP:
      if (sensor_value < sensor_max) {
        decelMax();
        pullActuator();
      } else {
        a_state = STATE_GOING_DOWN;
      }
      break;

    default:
      break;
  }
}
