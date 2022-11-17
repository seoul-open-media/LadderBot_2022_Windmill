////////////////// mode
// #define MODE_AUTONOMOUS 0
// #define MODE_BASIC_TEST 1
// #define MODE_BASIC_MOVEMENT 2

void scene() {
  // sequence(sceneNum, second, destination_x, destination_y, sensor_min, sensor_max)
  // sequence(1, 15, 3, 3, 90, 160); // Scene1


  // /* automatically change next scene arrived at the destination
  switch (state_seq)
  {
    case 0:
      sequenceTrigger(1, 1.8, 2.5, 30, 65);
      if (completeScene[0]) {
        state_seq = 1;
      }
      break;

    case 1:
      sequenceTrigger(2, 1.8, 4, 30, 65);
      if (completeScene[1]) {
        state_seq = 2;
      }
      break;

    case 2:
      sequenceTrigger(3, 2.5, 3, 30, 65);
      if (completeScene[2]) {
        state_seq = 3;
        rockTimer.restart();
        sendRockTimer.restart();
        pullActuator();
        brakeAllRelease();
      }
      break;

    case 3:
      mode = 2; // 0, 1, 2
      pullActuator();
      brakeAllRelease();
      function = 0; // stop
      if (rockTimer.hasPassed(1000)) {
        sendRockIsland();
        rockTimer.restart();

      }

      if (sendRockTimer.hasPassed(40000)) {
        state_seq = 4;
        WaitrockTimer.restart();
      };

      break;

    case 4:
      if (WaitrockTimer.hasPassed(410000)) {
        clearCompleteScene();
        state_seq = 0;
        state = STATE_INIT;
      }

      break;

    default:
      break;
  }
  // */
}

void sceneStop(){
  mode = 2; // 0, 1, 2
    pullActuator();
    brakeAllRelease();
    function = 0; // stop
}

void sceneCharge(){
  switch (state_charge)
  {
    case 0:
      sequenceTrigger(10, 1.8, 2.5, 30, 65);
      if (completeScene[9]) {
        state_charge = 1;
      }
      break;

    case 1:
      mode = 2; // 0, 1, 2
      pullActuator();
      brakeAllRelease();
      function = 0; // stop
      break;

    default:
      break;
  }
}

void sequence(int _num, int _sec, float _x, float _y, int _min, int _max) {
  int time = _sec * 1000;
  if (!passedScene[_num - 1] && sceneTimer[_num - 1].hasPassed(time)) {
    destination_x = _x;
    destination_y = _y;
    mode = 0; // 0, 1, 2
    sensor_min = _min;
    sensor_max = _max;
    Serial.print("Scene"); Serial.println(_num - 1);
    passedScene[_num] = true;
  }
}

void sequenceTrigger(int _num, float _x, float _y, int _min, int _max) {
  if (!completeScene[_num - 1]) {
    destination_x = _x;
    destination_y = _y;
    mode = 0; // 0, 1, 2
    sensor_min = _min;
    sensor_max = _max;
    if (reached_scene) {
      completeScene[_num - 1] = true;
    }
  }
}

void clearCompleteScene() {
  for (int i; i < NUM_SCENE; i++) {
    completeScene[i] = false;
  }
}
