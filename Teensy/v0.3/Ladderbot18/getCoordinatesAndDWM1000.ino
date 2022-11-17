void callLPSGetDataCalcCoord() {
  ////////////////////// every other 50ms. send DWM1000 packet to initiate LPS1, 2 , receive data, caculate crruent position and theta
  if (timer2.hasPassed(100)) {
    timer2.restart();

    ///////////////////////////////////////////////////////  theta, my_coordinate, distance update interval 50ms.
    ///////////// calcluate Theta
    //    getThetaInDeg(destination_x, destination_y);

    ///////////// calcluate Unit Vector
    getUvector(destination_x, destination_y);
    //    Serial.println(theta_deg);

    ///////////// caclulate current position
    my_coordinate_x = (LPS1_x + LPS2_x) / 2;
    my_coordinate_y = (LPS1_y + LPS2_y) / 2;

    // get the distance between the destination and my_coordinate
    distance = sqrt(pow((destination_x - my_coordinate_x), 2) + pow((destination_y - my_coordinate_y), 2));
    ////////////////////////////////////////////////////////
  }

  ///////////////////// if we get the data from LPS1, 2, calculate coordinates of LPS, my_coordinate and Theta
  if (serial_flag) {
    //    Serial.println("receivedAck");
    serial_flag = false;

    ///////////// get the distance data of LPS1,2 and update it
    byte from_address = r_data[0];
    byte to_message = r_data[1];



    //  oled.clear();
    if (to_message == MY_ADDRESS) {
      //            for (int i = 0; i < r_lenData; i++) {
      //            oled.print(r_data[i]);
      //            if(i == 23){
      //              oled.println();
      //            }else{
      //              oled.print(",");
      //            }
      //
      //          }

      switch (from_address) {
        case 1: // data from LPS1
          //        Serial.println("1");
          received_distance_result[0][0] = r_data[2];
          received_distance_result[0][1] = r_data[3];
          received_distance_result[0][2] = r_data[4];
          // received_elapsed_time[0] = r_data[23];
          // Get coordinates of LPS1
          getPosition(1);
          LPS1_x = coordinates_LPS[0][0];
          LPS1_y = coordinates_LPS[0][1];
          lpsTag1Timer.restart();
          break;

        case 2: // data from LPS2
          received_distance_result[1][0] = r_data[2];
          received_distance_result[1][1] = r_data[3];
          received_distance_result[1][2] = r_data[4];
          // received_elapsed_time[1] = r_data[23];
          // Get coordinates of LPS2
          getPosition(2);
          LPS2_x = coordinates_LPS[1][0];
          LPS2_y = coordinates_LPS[1][1];
          lpsTag2Timer.restart();
          break;


        default:
          break;

      }

    }
  }


  // if LPS is off for more than 5 sec, report it in OLED and STOP LADDERBOT
  if (lpsTag1Timer.hasPassed(5000) ) {

    pullActuator();
    brakeAllRelease();

    display.clearDisplay();
    display.println("LPS Tag1(Ladder Front) is not working!");
    display.println("1. Check the power LED and loose cable");
    display.println("2. Otherwise change the Tag1");
    display.display();

    Serial.println("LPS Tag1(Ladder Front) is not working!");
    Serial.println("1. Check the power LED and loose cable");
    Serial.println("2. Otherwise change the Tag1");
    while (1);
  }

  if (lpsTag2Timer.hasPassed(5000) ) {
    
    pullActuator();
    brakeAllRelease();

    display.clearDisplay();
    display.println("LPS Tag2(Ladder Rear) is not working!");
    display.println("1. Check the power LED and loose cable");
    display.println("2. Otherwise change the Tag2");
    display.display();

    Serial.println("LPS Tag2(Ladder Rear) is not working!");
    Serial.println("1. Check the power LED and loose cable");
    Serial.println("2. Otherwise change the Tag2");
    while (1);
  }

}
