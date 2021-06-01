// For IMU
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_Sensor.h>
//For WiFi and TwoWayCom
#include <esp_now.h>
//For MQTT
#include <PubSubClient.h>
#include <WiFi.h>

//  Create sensor object
Adafruit_LSM6DS33 imu;
//  Create sensor object
sensors_event_t a,g,temp;
float accX, accY, accZ, gyroX, gyroY, gyroZ,
      accRoll,      accPitch,     accYaw;            // units degrees (roll and pitch noisy, yaw not possible)


// LDR sensor pin
unsigned long last = 0; // change that
//For gestures
unsigned long lastLeftTurn = 0; // change that
unsigned long lastRightTurn = 0; // change that

unsigned int gestureCnt=0;
const int ldrPin=34;
// Flashed pins
const int rightPin=13;
const int leftPin=14;
const int lightPin=12;
int lightInit; // initial value
//Receiver MAC Address
//C4:4F:33:6B:0F:E1
uint8_t broadcastAddress[] = {0xC4, 0x4F, 0x33, 0x6B, 0x0F, 0xE1};

// Define the struct that contains the message content
// for our purposes we import the accelerometer readings 
typedef struct masterMessage {
  float roll;
  float pitch;
  int lightSensor;
} masterMessage;
//not used
typedef struct receivedMessage{
  bool imuUsage;
  //String rec_message;
}receivedMessage;

// Define the masterMessage
masterMessage msgToSlave;
// TO_DO -> Change the struct because we will get different content (ie, pin to trigger the flashes)
receivedMessage messageFromSlave;

// String receivedString; 
// Variable to store if sending data was successful
String success;
// Should be global ... ( TODO -> Check why ???)
esp_now_peer_info_t peerInfo;

// WiFi credentials
const char* ssid = "Koutoulas";
const char* password = "1312acab";

// MQTT IP server
const char* mqtt_server = "192.168.43.29";
int mqtt_port=1883;
// message to communicate with mqtt server
String message;
bool state=false;

WiFiClient espClient;
PubSubClient client(espClient);

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
  
  // if (String(topic) == "esp32/out"){
  //   if (message == "on") {
  //     digitalWrite(ledPin, HIGH);
  //   }
  //   else if (message == "off"){
  //     digitalWrite(ledPin, LOW);
  //   }
  // }
  // if (String(topic) == "esp32/ldr"){
  //   ledcWrite(0, message.toInt());
  // }
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
    // if (!lsm6ds33.begin_SPI(LSM_CS)) {
    // if (!lsm6ds33.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  }  
  Serial.println("Adafruit LSM6DS33 FOUND!");
    // lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (imu.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (imu.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DS33
  }

  // lsm6ds33.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (imu.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // lsm6ds33.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (imu.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  imu.configInt1(false, false, true); // accelerometer DRDY on INT1
  imu.configInt2(false, true, false); // gyro DRDY on INT2
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
  memcpy(&messageFromSlave,incomingData, sizeof(messageFromSlave));
  //Just for debug
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  // Write the data that have been sent
  // receivedString = messageFromSlave.rec_message;
}
// -------------------------- ENDOF ESP-NOW FUNCTIONS --------------------------

// -------------------------- SENSOR READINGS FUNCTIONS --------------------------
void getIMUReadings(){
  imu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  // Get current gyroscope values
  gyroX= g.gyro.x;
  gyroY= g.gyro.y;
  gyroZ= g.gyro.z;
}
void getRollPitch() {
  /*
  Calculate Roll and pitch and save them into the structure message
  */
  msgToSlave.roll = atan2(accY, accZ) * 180/M_PI;
  msgToSlave.pitch = atan2(-accX, sqrt(accY*accY + accZ*accZ)) * 180/M_PI;
}

void getLDRReadings(){
  // Read the current light Levels
  msgToSlave.lightSensor=analogRead(ldrPin);
}
// -------------------------- ENDOF SENSOR READINGS FUNCTIONS --------------------------

// -------------------------- DECISION-MAKING FUNCTIONS --------------------------
void blinking(int pin){
  /*
  Blinking the alarm 5 times (about 1 second procedure)
  */
  for(int k=0; k<=5; k++){
    digitalWrite(pin,HIGH);
    vTaskDelay(200/portTICK_PERIOD_MS);
    digitalWrite(pin,LOW);
    vTaskDelay(300/portTICK_PERIOD_MS);
    
  }
}

// Set a bool - flag when it finds any gesture to turn left or right
bool leftTurnFlag=false;
bool rightTurnFlag=false;

void checkAlarms(){
  String turnMsg=""; // Move this to the main string ? 
  // Serial.print("Roll: ");
  // Serial.print(msgToSlave.roll);
  // Serial.print(" Pitch: ");
  // Serial.println(msgToSlave.pitch);
  // Should I need gestures for light or it is useless????
  if (msgToSlave.lightSensor< 800){
    digitalWrite(lightPin,HIGH);
  }else{
    digitalWrite(lightPin,LOW);
  }
  // gestureCnt must be 1 because it start from 0
  if (msgToSlave.roll<-20 && leftTurnFlag ==true && gestureCnt>=1 && millis()<=lastLeftTurn + 2000/portTICK_PERIOD_MS){ 
    Serial.println("[LEFT] 2nd gesture");
    leftTurnFlag = false;
    gestureCnt=0;
    turnMsg+="turn=left";
    //Enable Ligh LEFT
    client.publish("esp32/TURNS", turnMsg.c_str());
    blinking(leftPin);    
  }else if (msgToSlave.roll<-20){
    // Also set a timer in 5 seconds (you could do the second gesture into 5 seconds)
    Serial.println("[LEFT] 1st gesture");
    lastLeftTurn = millis();
    gestureCnt+=1;
    leftTurnFlag= true;
  }

  if (msgToSlave.roll>20 && rightTurnFlag ==true && gestureCnt>=1 && millis()<=lastRightTurn + 2000/portTICK_PERIOD_MS){ 
    Serial.println("[RIGHT] 2nd gesture");
    turnMsg+="turn=right"; 
    rightTurnFlag = false;
    gestureCnt=0;
    client.publish("esp32/TURNS", turnMsg.c_str());
    blinking(rightPin);    
    
  }else if (msgToSlave.roll>20){
    // Also set a timer in 5 seconds (you could do the second gesture into 5 seconds)
    Serial.println("[RIGHT] 1st gesture");
    lastRightTurn = millis();
    gestureCnt+=1;
    rightTurnFlag= true;
  }



  
}
// -------------------------- ENDOF DECISION-MAKING FUNCTIONS --------------------------


// -------------------------- DEBUG FUNCTIONS --------------------------
void serialPrint(){
  Serial.print("gyroX: ");
  Serial.print(gyroX);
  Serial.print("  gyroY: ");
  Serial.print(gyroY);
  Serial.print("  gyroZ: ");
  Serial.print(gyroZ);
  Serial.print("accX: ");
  Serial.print(accX);
  Serial.print("  accY: ");
  Serial.print(accY);
  Serial.print("  accZ: ");
  Serial.print(accZ);
  Serial.print("  Roll: ");
  Serial.print(msgToSlave.roll);
  Serial.print("  Pitch: ");
  Serial.print(msgToSlave.pitch);
  Serial.print("  Luminosity: ");
  Serial.print(msgToSlave.lightSensor);
  Serial.println();

}

void serialPlotter(){
  // Just for plotting
  Serial.print(accX);
  Serial.print(","); Serial.print(accY);
  Serial.print(","); Serial.print(accZ);
  Serial.print(",");

  Serial.print(gyroX);
  Serial.print(","); Serial.print(gyroY);
  Serial.print(","); Serial.print(gyroZ);
  Serial.println();
  delayMicroseconds(10000);

}
// -------------------------- END OF DEBUG FUNCTIONS --------------------------

void task1(void * parameters){
  for(;;){
    
    // Serial.print("Task 1: ");
    //Get accelation readings
    getIMUReadings();
    getLDRReadings();
    getRollPitch();
    if (!client.connected()) {
      reconnect();
    }
    if (millis() > last + 1000/portTICK_PERIOD_MS ){
      // Publish the message to the mqtt server every 1 seconds
      String message = "";
      last = millis();
      message = "ldr="+String(analogRead(ldrPin))+";";
      message += "roll="+String(msgToSlave.roll)+";";
      message += "pitch="+String(msgToSlave.pitch)+";";
      message += "accX="+String(accX)+";";
      message += "accY="+String(accY)+";";
      message += "accZ="+String(accZ)+";";
      message += "gyroX="+String(gyroX)+";";
      message += "gyroY="+String(gyroY)+";";
      message += "gyroZ="+String(gyroZ)+";";
      //Serial.println(message);
      client.publish("esp32/HELMET_INFO", message.c_str());

    }
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &msgToSlave, sizeof(msgToSlave));
    
    // if (result == ESP_OK) {
    //   Serial.println("Sent with success");
    // }
    // else {
    //   Serial.println("Error sending the data");
    // }
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
}

void task2(void * parameters){
  // Second task check which alarm should be triggered 
  // or open the headlights according to the luminosity (ldr sensor)
  for(;;){  
    //Check if catch a gesture 
    checkAlarms();

    vTaskDelay(500/portTICK_PERIOD_MS);
  }  
}

void setup(){
  Serial.begin(115200);
  // Initializations
  initIMU();  
  connectToWifi();
  initLDRSensor();
  initESPNOW();
  pinMode(leftPin,OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(lightPin,OUTPUT);

  // Create the first task
  xTaskCreate(
    task1, // function name
    "Task1", // task name
    4000, // stack size
    NULL, // task parameters 
    1, // task priority
    NULL // task handle
    );
  delay(500);
  // create the second task
  xTaskCreate(
    task2, // function name
    "Task2", // task name
    2000, // stack size
    NULL, // task parameters 
    1, // task priority
    NULL // task handle
    );
}

void loop(){}
