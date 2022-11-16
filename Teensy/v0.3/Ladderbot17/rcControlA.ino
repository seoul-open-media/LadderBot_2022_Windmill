void rcControlA(){
  if(!rcStraight(getValue(1))) rcTurn(getValue(0));
  rcSensorVal(getValue(2));
  rcMode(getValue(4));
  if (actuatorVelMetro.check() == 1) rcActuatorVel(getValue(3));
//  Serial.print("STRIGHT_MIN: "); Serial.println(STRIGHT_MIN);
//  Serial.print("STRIGHT_MAX: "); Serial.println(STRIGHT_MAX);
  
}

void rcTurn(const int &ch_value1){
  if (ch_value1 - OFFSET > MIDDLE_VALUE) {
    turnLeft();
  } else if (ch_value1 + OFFSET < MIDDLE_VALUE) {
    turnRight();
  } else {
    rc_state = RCSTATE_INIT;
  }
}

bool rcStraight(const int &ch_value2) {  
  if (ch_value2 - OFFSET > MIDDLE_VALUE) {
    // go forward
    sensor_min = map(ch_value2, MIDDLE_VALUE + OFFSET, CH2_MAX, STRIGHT_MAX, STRIGHT_MIN);
    goStraight();
//    rc_state = RCSTATE_INIT;
    return true;
  } else if (ch_value2 + OFFSET < MIDDLE_VALUE) {
    // go back
    sensor_min = map(ch_value2, MIDDLE_VALUE + OFFSET, 0, STRIGHT_MAX, STRIGHT_MIN);
    goBack();
//    rc_state = RCSTATE_INIT;
    return true;
  } else {
    // stop
    return false;
  }
}

void rcSensorVal(const int &ch_value3){
  sensor_max = map(ch_value3, 0, CH3_MAX, SENSOR_MAX_LOW, SENSOR_MAX_HIGH);
}

void rcActuatorVel(const int &ch_value4){
  if (ch_value4 - OFFSET > MIDDLE_VALUE) {
    if(actuatorPwm_ < 255) actuatorPwm_++;
  } else if (ch_value4 + OFFSET < MIDDLE_VALUE) {
    if(actuatorPwm_ > 0) actuatorPwm_--;
  } else {
    // do nothing
  }
}

void rcMode(const int &ch_value5){
  if (ch_value5 - OFFSET > MIDDLE_VALUE) {
    STRIGHT_MIN = MODE_LOW_MIN;
    STRIGHT_MAX = MODE_LOW_MAX;
  } else if (ch_value5 + OFFSET < MIDDLE_VALUE) {
    STRIGHT_MIN = MODE_HIGH_MIN;
    STRIGHT_MAX = MODE_HIGH_MAX;
  } else {
    STRIGHT_MIN = MODE_MID_MIN;
    STRIGHT_MAX = MODE_MID_MAX;
  }
}

bool rcServoOnly(const int &ch_value6) {  
  if (ch_value6 - OFFSET > MIDDLE_VALUE) {
    return false;
  } else if (ch_value6 + OFFSET < MIDDLE_VALUE) {
    return true;
  }
}
