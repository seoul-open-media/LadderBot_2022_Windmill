void rcControlA(){
  if(!rcStraight(getValue(1))) rcTurn(getValue(0));
  rcSensorVal(getValue(2));
  rcActuatorVel(getValue(3));
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
    sensor_min = map(ch_value2, MIDDLE_VALUE + OFFSET, CH2_MAX, 45, 20);
    goStraight();
//    rc_state = RCSTATE_INIT;
    return true;
  } else if (ch_value2 + OFFSET < MIDDLE_VALUE) {
    // go back
    sensor_min = map(ch_value2, MIDDLE_VALUE + OFFSET, 0, 45, 20);
    goBack();
//    rc_state = RCSTATE_INIT;
    return true;
  } else {
    // stop
    return false;
  }
}

void rcSensorVal(const int &ch_value3){
  sensor_max = map(ch_value3, 0, CH3_MAX, 30, 60);
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
