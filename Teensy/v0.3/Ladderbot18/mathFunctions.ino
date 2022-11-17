void getThetaInDeg(float des_x, float des_y) {
  P12 = sqrt(pow((des_x - LPS1_x), 2) + pow((des_y - LPS1_y), 2));
  P13 = sqrt(pow((des_x - LPS2_x), 2) + pow((des_y - LPS2_y), 2));
  P23 = sqrt(pow((LPS1_x - LPS2_x), 2) + pow((LPS1_y - LPS2_y), 2));
  

  theta_rad = acos((pow(P12, 2) + pow(P13, 2) - pow(P23, 2)) / (2 * P12 * P13));
  theta_deg = theta_rad * 57296 ;
  theta_deg_filtered = thetaFilter.filter(theta_deg) / 1000; // smoothe out the noise

  //Serial.println(theta_deg);
}

void getUvector(float des_x, float des_y) {
  P23 = sqrt(pow((LPS1_x - LPS2_x), 2) + pow((LPS1_y - LPS2_y), 2));
  P14 = sqrt(pow((des_x - my_coordinate_x), 2) + pow((des_y - my_coordinate_y), 2));
  
  ax_norm = (LPS1_x - LPS2_x)/P23;
  ay_norm = (LPS1_y - LPS2_y)/P23;
  bx_norm = (des_x - my_coordinate_x)/P14;
  by_norm = (des_y - my_coordinate_y)/P14;

  xrange = abs(bx_norm - ax_norm) * 10000;
  yrange = abs(by_norm - ay_norm) * 10000;
  
  xrange_filtered = thetaFilter.filter(xrange) / 10000;
  yrange_filtered = thetaFilter.filter(yrange) / 10000;

//  //theta from dot product
//  theta_rad = acos(ax_norm * bx_norm + ay_norm * by_norm);
//  theta_deg = theta_rad * 57296 ;
//  theta_deg_filtered = thetaFilter.filter(theta_deg) / 1000;
  
}

boolean isAbove(float x, float y) {

//  float value = ((destination_y - my_coordinate_y) / (destination_x - my_coordinate_x)) * (x - my_coordinate_x) + my_coordinate_y;
//  float value = (ay_norm * x) / ax_norm;

  // z component from cross product
  float determ = ax_norm * y - ay_norm * x;
  //Serial.print("value = "); Serial.println(value);
  //Serial.print("y = "); Serial.println(y);
//  if (y > value ) {
  if (determ > 0) {
    return true;
  } else {
    return false;
  }
}

void getPosition(byte num_LPS) { // 1 or 2
  //https://en.wikipedia.org/wiki/Trilateration

  float max_distance[3] = {25.1, 25.1, 25.1}; //maximum distance from anchor 1, 2, 3
  float min_distance[3] = {0.1, 0.1, 0.1};

  float r1 = received_distance_result[num_LPS - 1][0] / 10.0; // distance between this tag and anchor1
  float r2 = received_distance_result[num_LPS - 1][1] / 10.0; // distance between this tag and anchor2
  float r3 = received_distance_result[num_LPS - 1][2] / 10.0; // distance between this tag and anchor3

  if ((r1 < max_distance[0] && r1 > min_distance[0]) && (r2 < max_distance[1] && r2 > min_distance[1]) && (r3 < max_distance[2] && r3 > min_distance[2])) {
    float  x = ((r1 * r1) - (r2 * r2) + (d * d)) / (2 * d);
    float  y =  ((r1 * r1) - (r3 * r3) + (p3_i * p3_i) + (p3_j * p3_j)) / (2 * p3_j) - (p3_i * x / p3_j);
    float  z = sqrt(r1 * r1 - x * x - y * y );
    coordinates_LPS[num_LPS - 1][0] = x;
    coordinates_LPS[num_LPS - 1][1] = y;
    //    if (num_LPS == 1) {
    //      smooth_LPS1_x.add(x);
    //      smooth_LPS1_y.add(y);
    //      coordinates_LPS[num_LPS - 1][0] = smooth_LPS1_x.get();
    //      coordinates_LPS[num_LPS - 1][1] = smooth_LPS1_y.get();
    //    }
    //    if (num_LPS == 2) {
    //      smooth_LPS2_x.add(x);
    //      smooth_LPS2_y.add(y);
    //      coordinates_LPS[num_LPS - 1][0] = smooth_LPS2_x.get();
    //      coordinates_LPS[num_LPS - 1][1] = smooth_LPS2_y.get();
    //    }

    // displayData05();
  } else {
    // display error for debug
    // displayData06();
  }

}
