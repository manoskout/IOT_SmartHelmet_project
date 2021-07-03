void getIMUReadings(){
  /**
  Read the sensor values. 
  Every time add in a feature table the metrics in order to classify
  any gesture related to right or left turn
  */
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);
  // ax = a.acceleration.x + 0.22;
  // ay = a.acceleration.y + 0.10;
  // az = a.acceleration.z - 10.33;
  ax = a.acceleration.x - baseline[0];
  ay = a.acceleration.y - baseline[1];
  az = a.acceleration.z - baseline[2];
  // Get current gyroscope values
  gx= g.gyro.x;
  gy= g.gyro.y;
  gz= g.gyro.z;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    // ax = constrain(ax - baseline[0], -TRUNCATE, TRUNCATE);
    // ay = constrain(ay - baseline[1], -TRUNCATE, TRUNCATE);
    // az = constrain(az - baseline[2], -TRUNCATE, TRUNCATE);
    features[i * NUM_AXES + 0] = ax;
    features[i * NUM_AXES + 1] = ay;
    features[i * NUM_AXES + 2] = az;
  }
  
}

void getRollPitch() {
  /**
  Calculate Roll and pitch according to acceleromete values
  */
  //Roll & Pitch Equations
	roll  = (atan2(-ay, az)*180.0)/M_PI;
	pitch = (atan2(ax, sqrt(ay*ay + az*az))*180.0)/M_PI;
}

void getLDRReadings(){
  /**
  Just a simple function that reads the values of LDR sensor and save it into the message slave funtion
  (not used)
  */
  msgToSlave.lightSensor=analogRead(ldrPin);
}
