void displaySensorY() {
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print("EBIMU: ");
  display.println(sensor_value);

  display.print("state_seq: ");
  display.println(state_seq);

  display.print("x: ");display.print(my_coordinate_x);display.print(", ");
  display.print("y: ");display.println(my_coordinate_y);
  display.print("dest x: ");display.print(destination_x);display.print(", ");
  display.print("y: ");display.println(destination_y);
  display.print("distance: ");display.println(distance);
  display.print("reached: ");display.println(reached_destination);
  display.print("completeScene: ");display.println(completeScene[0]);
  display.print("state: ");display.print(state);
  display.display();

  
}

void displayData01() {
  int reading;
  float bat_voltage;
  //char report[80];


  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.println("Robot Control Board");
  display.print("v.");display.println(RCB_VERSION);

  display.println();

  display.print("Software v. ");
  display.println(SOFT_VERSION);
  display.println();
  display.print("Jon Ladderbot ");
 
  display.println();

  display.display();
}

void displayError03() {  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("EBIMU Not Detected");
  display.display();
}

void displayAnchorError(int anchor_num){
 display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("LPS Anchor ");display.print(anchor_num);display.print(" is not Working!");
  display.print("Reset or change the USB adaptor. Otherwise Replace the anchor. ");
  display.display(); 
}
  
