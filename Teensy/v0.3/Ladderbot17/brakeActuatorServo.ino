//void runActuator() {
//  ////////////////////// Run actuator only when we didn't reached the destination and boundary
//  if (reached_destination != true && reached_boundary != true) {
//
//    if (sensor_value < sensor_min&& a_state == STATE_GOING_DOWN) { // we reached the bottom
//      pullActuator();
//      a_state = STATE_GOING_UP;
//    } else if (sensor_value > sensor_max && a_state == STATE_GOING_UP) { // we reached top
//      pushActuator();
//      a_state = STATE_GOING_DOWN;
//    }
//  } else {
//    stopActuator();
//  }
//}

void decelMax() {
  if (abs((abs(sensor_value) - sensor_max)) < 10){
        actuatorPwm_ = pow(abs((abs(sensor_value) - sensor_max)),2) + 120;
        // Serial.println("decel: ");
        }
        else actuatorPwm_ = 255;
}

void decelMin() {
  if (abs((abs(sensor_value) - sensor_min)) < 10){
        actuatorPwm_ = pow(abs((abs(sensor_value) - sensor_min)),2) + 120;
        // Serial.println("decel: ");
        }
        else actuatorPwm_ = 255;
}

void pushActuator() {
  analogWrite(M1_PWM_PIN, actuatorPwm_);
  digitalWrite(ACTUATOR_CONTROL_PIN1, HIGH);
  digitalWrite(ACTUATOR_CONTROL_PIN2, LOW);
}
void pullActuator() {
  analogWrite(M1_PWM_PIN, actuatorPwm_);
  digitalWrite(ACTUATOR_CONTROL_PIN1, LOW);
  digitalWrite(ACTUATOR_CONTROL_PIN2, HIGH);
}

void stopActuator() {
  if(actuatorPwm_ < 255) actuatorPwm_++;
  analogWrite(M1_PWM_PIN, actuatorPwm_);
  digitalWrite(ACTUATOR_CONTROL_PIN1, LOW);
  digitalWrite(ACTUATOR_CONTROL_PIN2, LOW);
}


void brakeAll() {
  analogWrite(M3_PWM_PIN, 255);
  digitalWrite(M3_A_PIN, HIGH);
  digitalWrite(M3_B_PIN, LOW);

  analogWrite(M4_PWM_PIN, 255);
  digitalWrite(M4_A_PIN, HIGH);
  digitalWrite(M4_B_PIN, LOW);

  //for 4ch transistor circuit
//  digitalWrite(BRAKE_1_PIN, HIGH);
//  digitalWrite(BRAKE_2_PIN, HIGH);
//  digitalWrite(BRAKE_3_PIN, HIGH);
//  digitalWrite(BRAKE_4_PIN, HIGH);

}
void brakeAllRelease() {
//  Serial.println("no brake");
  analogWrite(M3_PWM_PIN, 0); 
  digitalWrite(M3_A_PIN, LOW);
  digitalWrite(M3_B_PIN, LOW);

  analogWrite(M4_PWM_PIN, 0);
  digitalWrite(M4_A_PIN, LOW);
  digitalWrite(M4_B_PIN, LOW);

  //for 4ch transistor circuit
//  digitalWrite(BRAKE_1_PIN, LOW);
//  digitalWrite(BRAKE_2_PIN, LOW);
//  digitalWrite(BRAKE_3_PIN, LOW);
//  digitalWrite(BRAKE_4_PIN, LOW);

}

void brake12() {
//  Serial.println("front brake");
  analogWrite(M3_PWM_PIN, 255);
  digitalWrite(M3_A_PIN, HIGH);
  digitalWrite(M3_B_PIN, LOW);

  analogWrite(M4_PWM_PIN, 0);
  digitalWrite(M4_A_PIN, LOW);
  digitalWrite(M4_B_PIN, LOW);

  //for 4ch transistor circuit
//  digitalWrite(BRAKE_1_PIN, HIGH);
//  digitalWrite(BRAKE_2_PIN, HIGH);
}

void brake34() {
//  Serial.println("rear brake");
  analogWrite(M3_PWM_PIN, 0);
  digitalWrite(M3_A_PIN, LOW);
  digitalWrite(M3_B_PIN, LOW);

  analogWrite(M4_PWM_PIN, 255);
  digitalWrite(M4_A_PIN, HIGH);
  digitalWrite(M4_B_PIN, LOW);

  //for 4ch transistor circuit
//  digitalWrite(BRAKE_3_PIN, HIGH);
//  digitalWrite(BRAKE_4_PIN, HIGH);

}

void servoControl(byte servo_mode) {
  uint16_t value = (SERVOMAX - SERVOMIN)/2;  
  uint16_t value1 = (SERVOMAX - SERVOMIN)/2;
  int16_t servo_value = 0;
  int16_t servo_value1 = 0;
  
  if (servo_mode != previous_servo_mode) {
    previous_servo_mode = servo_mode;
    switch (servo_mode) {
      case SERVO_STRAIGHT:
        servo_value = -12;
        servo_value1 = 15;
        value = ((SERVOMAX - SERVOMIN)/2 + SERVOMIN) + (servo_value * (SERVOMAX - SERVOMIN)/100);
        value1 = ((SERVOMAX - SERVOMIN)/2 + SERVOMIN) + (servo_value1 * (SERVOMAX - SERVOMIN)/100);
        pwm_.setPWM(SERVO_1_PIN_, 0, value);
        pwm_.setPWM(SERVO_2_PIN_, 0, value1);
        break;

      case SERVO_LEFT:
        servo_value = -50;
        servo_value1 = 50;
        value = ((SERVOMAX - SERVOMIN)/2 + SERVOMIN) + (servo_value * (SERVOMAX - SERVOMIN)/100);
        value1 = ((SERVOMAX - SERVOMIN)/2 + SERVOMIN) + (servo_value1 * (SERVOMAX - SERVOMIN)/100);
        pwm_.setPWM(SERVO_1_PIN_, 0, value);
        pwm_.setPWM(SERVO_2_PIN_, 0, value1);
        break;

      case SERVO_RIGHT:
        servo_value = 30;
        servo_value1 = -23;
        value = ((SERVOMAX - SERVOMIN)/2 + SERVOMIN) + (servo_value * (SERVOMAX - SERVOMIN)/100);
        value1 = ((SERVOMAX - SERVOMIN)/2 + SERVOMIN) + (servo_value1 * (SERVOMAX - SERVOMIN)/100);
        pwm_.setPWM(SERVO_1_PIN_, 0, value);
        pwm_.setPWM(SERVO_2_PIN_, 0, value1);
        break;

      default:
        break;
    }


  }
}


//void moveForwardPush() {
//  digitalWrite(BRAKE12_PIN, LOW);
//
//  digitalWrite(BRAKE34_PIN, HIGH);
//
//}
//
//void moveForwardPull() {
//  digitalWrite(BRAKE12_PIN, HIGH);
//
//  digitalWrite(BRAKE34_PIN, LOW);
//
//}

//void moveBackwardPush() {
//  digitalWrite(BRAKE12_PIN, HIGH);
//
//  digitalWrite(BRAKE34_PIN, LOW);
//
//}
//
//void moveBackwardPull() {
//  digitalWrite(BRAKE12_PIN, LOW);
//
//  digitalWrite(BRAKE34_PIN, HIGH);
//
//}


//void turnForwardRight() {
//  digitalWrite(BRAKE12_PIN, LOW);
//
//  digitalWrite(BRAKE34_PIN, LOW);
//
//  analogWrite(SERVO_PWM_PIN, SERVO_RIGHT_VALUE);
//}
//
//void turnForwardLeft() {
//  digitalWrite(BRAKE12_PIN, HIGH);
//
//  digitalWrite(BRAKE34_PIN, LOW);
//
//  analogWrite(SERVO_PWM_PIN, SERVO_LEFT_VALUE);
//}
//void turnBackwardRight() {
//  digitalWrite(BRAKE12_PIN, LOW);
//
//  digitalWrite(BRAKE34_PIN, LOW);
//
//}
//void turnBackwadLeft() {
//  digitalWrite(BRAKE12_PIN, LOW);
//
//  digitalWrite(BRAKE34_PIN, HIGH);
//
//}

//void releaseBrakes() {
//  digitalWrite(BRAKE12_PIN, LOW);
//
//  digitalWrite(BRAKE34_PIN, LOW);
//
//}
