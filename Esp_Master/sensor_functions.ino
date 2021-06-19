void getIMUReadings(){
  
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  // Get current gyroscope values
  gx= g.gyro.x;
  gy= g.gyro.y;
  gz= g.gyro.z;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    ax = constrain(ax - baseline[0], -TRUNCATE, TRUNCATE);
    ay = constrain(ay - baseline[1], -TRUNCATE, TRUNCATE);
    az = constrain(az - baseline[2], -TRUNCATE, TRUNCATE);
    features[i * NUM_AXES + 0] = ax;
    features[i * NUM_AXES + 1] = ay;
    features[i * NUM_AXES + 2] = az;
  }
  
}

void getRollPitch() {
  /*
  Calculate Roll and pitch and save them into the structure message
  */
  roll = atan2(ax, az) * 180/M_PI;
  pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180/M_PI;
}

void getLDRReadings(){
  // Read the current light Levels
  msgToSlave.lightSensor=analogRead(ldrPin);
}
