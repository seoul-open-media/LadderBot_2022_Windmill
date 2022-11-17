void getHWSerial() {
  if (HWSERIAL.available() > 6) {
    //    // check my bow state


    //  Serial.println("Got Final Result");
    // digitalWrite(13, HIGH);
    byte first_byte = HWSERIAL.read();
    byte second_byte = HWSERIAL.read();
    // Serial.print("first_byte ="); Serial.println(first_byte);
    // Serial.print("second_byte ="); Serial.println(second_byte);
    if (first_byte == 255) {
      if (second_byte == FINAL_RESULT) {
        // read the raging rusult
        for (int i = 0; i < 5; i++)
        {
          r_data[i] = HWSERIAL.read();
          // if (distance_result[i] == 0)MY_ADDRESS = (i + 1);
      //    Serial.print(r_data[i]);
      //    Serial.print(", ");
        }
      //  Serial.println();
        // if the number is above 250(error), set the timer to report achor error
        if (r_data[2] < 250 ) {
          anchor1ErrorTimer.restart();
        }
        if (r_data[3] < 250 ) {
          anchor2ErrorTimer.restart();
        }
        if (r_data[4] < 250 ) {
          anchor3ErrorTimer.restart();
        }

        if (anchor1ErrorTimer.hasPassed(5000)) {
          pullActuator();
          brakeAllRelease();
          displayAnchorError(1);
          Serial.print("Check Anchor1");
          while (1);
        }
        if (anchor2ErrorTimer.hasPassed(5000)) {
          pullActuator();
          brakeAllRelease();
          displayAnchorError(2);
          Serial.print("Check Anchor2");
          while (1);
        }
        if (anchor3ErrorTimer.hasPassed(5000)) {
          pullActuator();
          brakeAllRelease();
          displayAnchorError(3);
          Serial.print("Check Anchor3");
          while (1);
        }

        serial_flag = true;
      }
    } else {
      HWSERIAL.clear();
      while (HWSERIAL.available()) {
        HWSERIAL.read();
      }
    }
  }
}

void sendBow() {
  Serial.println("sendBow");
  byte s_lenData = 3;
  byte s_data[s_lenData];

  s_data[0] = 255;
  s_data[1] = SEND_SONG_INFO;
  s_data[2] = BOW;

  HWSERIAL.write(s_data, s_lenData);
}

void sendRockIsland() {
  Serial.println("sendRockIsland");
  byte s_lenData = 3;
  byte s_data[s_lenData];

  s_data[0] = 255;
  s_data[1] = SEND_SONG_INFO;
  s_data[2] = RockIsland;

  HWSERIAL.write(s_data, s_lenData);
}

void sendRobot(){
  Serial.println("sendRobot");
  byte s_lenData = 3;
  byte s_data[s_lenData];

  s_data[0] = 255;
  s_data[1] = SEND_SONG_INFO;
  s_data[2] = RobotMaster;

  HWSERIAL.write(s_data, s_lenData);
}

void sendPosition(){
  Serial.println("sendPosition");
  byte s_lenData = 3;
  byte s_data[s_lenData];

  s_data[0] = 255;
  s_data[1] = SEND_POSITION;
  s_data[2] = (int) my_coordinate_x*10;
  s_data[3] = (int) my_coordinate_y*10;

  HWSERIAL.write(s_data, s_lenData);
}
