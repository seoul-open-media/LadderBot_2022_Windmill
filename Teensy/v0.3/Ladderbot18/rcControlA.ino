void rcControlA(){
//  if(!rcStraight(getValue(1))) rcTurn(getValue(0));
//  Serial.print("rc_state= "); Serial.println(rc_state);
//  Serial.print("sensor_value= "); Serial.println(sensor_value);
//  Serial.print("sensor_min= "); Serial.println(sensor_min);
//  Serial.print("sensor_max= "); Serial.println(sensor_max);

    if(!rcStraight(getValue(1))){
    if(!isRcTurn){
//      Serial.print("isRcStraight= "); Serial.println(isRcStraight);
    
      if (isRcStraight && ((rc_state == RCSTATE_PUSHFORWARD) || (rc_state == RCSTATE_PUSH_TURN))){
        sensor_min = SENSOR_MAX_LOW;
//        Serial.print("_sensor_min= "); Serial.println(sensor_min);
        if(sensor_value > sensor_min){
          pushActuator();
        }else{
          stopActuator();
          rc_state = RCSTATE_INIT;
          isRcStraight = false;
//            isRcTurn = true;
        }
      } else if(isRcStraight && ((rc_state == RCSTATE_PULLFORWARD) || (rc_state == RCSTATE_PULL_TURN))){
        sensor_max = SENSOR_MAX_HIGH;
//        Serial.print("_sensor_max= "); Serial.println(sensor_max);
        if(sensor_value < sensor_max){
//          Serial.println("pull");
          pullActuator();
        }else{
          stopActuator();
          rc_state = RCSTATE_INIT;
          isRcStraight = false;
//            isRcTurn = true;
        }
      }
    }
    
    rcTurn(getValue(0));
  }
  

  
  rcSensorVal(getValue(2));
  rcMode(getValue(4));
  if (actuatorVelMetro.check() == 1) rcActuatorVel(getValue(3));
//  Serial.print("straight_min: "); Serial.println(straight_min);
//  Serial.print("straight_max: "); Serial.println(straight_max);
  
}

void rcTurn(const int &ch_value1){
  if (ch_value1 - OFFSET > MIDDLE_VALUE) {
    turnLeft();
    isRcStraight = true;
    isRcTurn = true;
  } else if (ch_value1 + OFFSET < MIDDLE_VALUE) {
    turnRight();
    isRcStraight = true;
    isRcTurn = true;
  } else {
    isRcTurn = false;
//    rc_state = RCSTATE_INIT;
  }
}

bool rcStraight(const int &ch_value2) {  
  if (ch_value2 - OFFSET > MIDDLE_VALUE) {
    // go forward
    sensor_min = map(ch_value2, MIDDLE_VALUE + OFFSET, CH2_MAX, straight_max, straight_min);
    goStraight();
    isRcStraight = true;
//    rc_state = RCSTATE_INIT;
    return true;
  } else if (ch_value2 + OFFSET < MIDDLE_VALUE) {
    // go back
    sensor_min = map(ch_value2, MIDDLE_VALUE + OFFSET, 0, straight_max, straight_min);
    goBack();
    isRcStraight = true;
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
    straight_min = MODE_LOW_MIN;
    straight_max = MODE_LOW_MAX;
  } else if (ch_value5 + OFFSET < MIDDLE_VALUE) {
    straight_min = MODE_HIGH_MIN;
    straight_max = MODE_HIGH_MAX;
  } else {
    straight_min = MODE_MID_MIN;
    straight_max = MODE_MID_MAX;
  }
}

bool rcServoOnly(const int &ch_value6) {  
  if (ch_value6 - OFFSET > MIDDLE_VALUE) {
    return false;
  } else if (ch_value6 + OFFSET < MIDDLE_VALUE) {
    return true;
  }
}
