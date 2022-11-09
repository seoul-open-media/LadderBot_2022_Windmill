void rcControlA(){
  if(!rcStraight(getValue(0))) rcTurn(getValue(1));
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
  // add a value to control sensor min
  // sensor max default
  if (ch_value2 - OFFSET > MIDDLE_VALUE) {
    // go forward
    rc_state = RCSTATE_INIT;
    return true;
  } else if (ch_value2 + OFFSET < MIDDLE_VALUE) {
    // go back
    rc_state = RCSTATE_INIT;
    return true;
  } else {
    // stop
    return false;
  }
}

void rcSensorVal(const int &ch_value3){
  if (ch_value3 - OFFSET > MIDDLE_VALUE) {
    // sensor min and max low
  } else if (ch_value3 + OFFSET < MIDDLE_VALUE) {
    // sensor min and max high
  } else {
    // sensor min and max default
  }
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
