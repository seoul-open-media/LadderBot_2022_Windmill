void neoKey() {
  //Serial.println("neoKey");
  for (uint8_t i = 0; i < 4; i++) {
    debouncers[i].update();
  }
  
  if (debouncers[0].rose()) {
    Serial.println("Play preseed.");
    neoKeyNum = 1;
  }
  if (debouncers[1].rose()) {
    Serial.println("Stop preseed.");
    neoKeyNum = 2;
  }
  if (debouncers[2].rose()) {
    Serial.println("Robot preseed.");
    neoKeyNum = 3;
  }
  if (debouncers[3].rose()) {
    Serial.println("Charge preseed.");
    neoKeyNum = 4;
  }

  neoKeyAct();
}

void neoKeyAct(){
  if(neoKeyNum == 1){
    isScenePlaying = true;
    isSceneStop = false;
    toCharge = false;
    isSentRobot = false;
  }else if(neoKeyNum == 2){
    isScenePlaying = false;
    isSceneStop = true;
    toCharge = false;
    isSentRobot = false;
    mode = 2;
    function = 0;
  }else if(neoKeyNum == 3){
    isScenePlaying = false;
    isSceneStop = true;
    toCharge = false;
    mode = 2;
    function = 0;
    if(!isSentRobot){
      sendRobot();
      isSentRobot = true;
    }
  }else if(neoKeyNum == 4){
    isScenePlaying = false;
    isSceneStop = false;
    toCharge = true;
    isSentRobot = false;
  }
}
