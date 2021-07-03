
// #include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
// #define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

// Kalman kalmanX; // Create the Kalman instances
// Kalman kalmanY;
// // -------------------------- DEBUG FUNCTIONS --------------------------
void serialPrint(){
  // kalmanX.setAngle(roll);
  // kalmanY.setAngle(pitch);
  Serial.print("  Speed: ");
  Serial.print(msgToSlave.curSpeed);
  // Serial.print("gyroX: ");
  // Serial.print(gx);
  // Serial.print("  gyroY: ");
  // Serial.print(gy);
  // Serial.print("  gyroZ: ");
  // Serial.print(gz);
  Serial.print(" accX: ");
  Serial.print(ax);
  Serial.print("  accY: ");
  Serial.print(ay);
  Serial.print("  accZ: ");
  Serial.print(az);
  Serial.print("  Roll: ");
  Serial.print(roll);
  Serial.print("  Pitch: ");
  Serial.print(pitch);
  
  Serial.println();

}

// void serialPlotter(){
//   // Just for plotting
//   Serial.print(accX);
//   Serial.print(","); Serial.print(accY);
//   Serial.print(","); Serial.print(accZ);
//   Serial.print(",");

//   Serial.print(gyroX);
//   Serial.print(","); Serial.print(gyroY);
//   Serial.print(","); Serial.print(gyroZ);
//   Serial.println();
//   delayMicroseconds(10000);

// }
// // -------------------------- END OF DEBUG FUNCTIONS --------------------------
