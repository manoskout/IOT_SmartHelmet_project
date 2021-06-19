
// -------------------------- WIFI INIT FUNCTION --------------------------
void connectToWifi(){
  WiFi.begin(ssid,password);
  Serial.print("Connecting....");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected - IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
// -------------------------- ENDOF WIFI INIT FUNCTION --------------------------

// -------------------------- MQTT INIT FUNCTIONS --------------------------
void callback(char* topic, byte* payload, unsigned int length){
  for (int i = 0; i < length; i++){
    message += (char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(String(topic));
  Serial.print("/");
  Serial.print(String(message));
  Serial.println("] ");
  message = "";
}
void reconnect(){
  // Loop until we're reconnected
  while (!client.connected()){
    Serial.println("Attempting MQTT connection...");
    if (client.connect("SmartHelmet")){
      // TODO-> the subscribes need to be changed !!!
      Serial.println("Connected");
      client.subscribe("esp32/HELMET_INFO");
      client.subscribe("esp32/TURNS");
    }
    else{
      Serial.print(client.state());
      Serial.println("Failed - Try again in 5 seconds");
      delay(5000);
    }
  }
}
// -------------------------- ENDOF MQTT INIT FUNCTIONS --------------------------

// -------------------------- SENSORS INIT FUNCTIONS --------------------------
void initIMU(){
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit LSM6DS33 test!");
  if (!imu.begin_I2C()) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  }  
  
  imu.configInt1(false, false, true); // accelerometer DRDY on INT1
  imu.configInt2(false, true, false); // gyro DRDY on INT2
}
void calibrate() {
  Serial.println("Callibrating........");
  // float ax, ay, az,gx,gy,gz, curSpeed;
  for (int i = 0; i < 10; i++) {
    getIMUReadings();
    // TODO -> How to run function without worrying for the unused values
    delay(100);
  }
  baseline[0] = ax;
  baseline[1] = ay;
  baseline[2] = az;
  Serial.println("Ready!");
}

void initLDRSensor(){
  pinMode(ldrPin, INPUT);
}
// -------------------------- ENDOF SENSORS INIT FUNCTIONS --------------------------

// -------------------------- ESP-NOW FUNCTIONS --------------------------
void initESPNOW(){
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  //esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  // Callback Function that sents message to the slave
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  // Callback Function that triggered when a new packet arrives from slave
  memcpy(&msgFromSlave,incomingData, sizeof(msgFromSlave));
  //Just for debug
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  // Serial.println("Received Message-> seatStatus: " + String(msgFromSlave.seatStatus)+ " brake: " +String(msgFromSlave.brake));
}
// -------------------------- ENDOF ESP-NOW FUNCTIONS --------------------------
