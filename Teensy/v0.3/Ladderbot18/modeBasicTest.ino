void modeBasicTest() {
  if (function != previous_function) {
    previous_function = function;
    switch (function) {
      case 0://
        brakeAllRelease();
        servoControl(SERVO_STRAIGHT);

        stopActuator();
        break;

      case 1://
        pullActuator();

        break;

      case 2://
        pushActuator();

        break;

      case 3://
        brakeAll();
        break;

      case 4://
        brakeAllRelease();
        break;

      case 5://
        brake12();
        break;

      case 6://
        brake34();
        break;

      case 7://
        servoControl(SERVO_STRAIGHT);

        break;

      case 8://
        servoControl(SERVO_LEFT);
        break;

      case 9://
        servoControl(SERVO_RIGHT);
        break;


      default:
        break;

    }
  }
}
