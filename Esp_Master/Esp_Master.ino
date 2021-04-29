//#include <Arduino.h>
// For IMU
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_Sensor.h>

//For WiFi and TwoWayCom
#include <esp_now.h>
#include <WiFi.h>

//  Create sensor object
Adafruit_LSM6DS33 imu;

//  Create sensor object
sensors_event_t a,g,temp;
float incomingAccX, incomingAccY, incomingAccZ,
      accRoll,      accPitch,     accYaw;            // units degrees (roll and pitch noisy, yaw not possible)


// LDR sensor pin
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
typedef struct struct_message {
  float accX;
  float accY;
  float accZ;
  int light;
} struct_message;

typedef struct struct_message_Rec{
  String rec_message;
}struct_message_Rec;

// Define the struct_message
struct_message imuReadings;
// TO_DO -> Change the struct because we will get different content (ie, pin to trigger the flashes)
struct_message_Rec receivedMessage;
String receivedString; 
// Variable to store if sending data was successful
String success;
esp_now_peer_info_t peerInfo;

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


void initIMU(){
  Serial.begin(115200);
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
}

void initLDRSensor(){
  //we will take a single reading from the light sensor and store it in the lightCal        
  //variable. This will give us a prelinary value to compare against in the loop
  pinMode(ldrPin, INPUT);
  // lightInit=analogRead(ldrPin);
}


void setup()
{
  initIMU();
  initLDRSensor();
  initESPNOW();

  // Init alarms
  pinMode(leftPin,OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(lightPin,OUTPUT);
}




void serialPrint(){
  Serial.print("X: ");
  Serial.print(imuReadings.accX);
  Serial.print("  Y: ");
  Serial.print(imuReadings.accY);
  Serial.print("  Z: ");
  Serial.print(imuReadings.accZ);
  Serial.print("  Roll: ");
  Serial.print(accRoll);
  Serial.print("  Pitch: ");
  Serial.print(accPitch);
  Serial.print("  Light: ");
  Serial.print(imuReadings.light);
  Serial.println();

}

// Callback Function that sents message
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}
// Callback Function that triggered when a new packet arrives
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  memcpy(&receivedMessage,incomingData, sizeof(receivedMessage));
  //Just for debug
  //Serial.print("Bytes received: ");
  //Serial.println(len);

  // Write the data that have been sent
  receivedString = receivedMessage.rec_message;
  
}
void getAccReadings(){
  imu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  imuReadings.accX = a.acceleration.x;
  imuReadings.accY = a.acceleration.y;
  imuReadings.accZ = a.acceleration.z;
}


void getLDRReadings(){
  // Read the current light Levels
  // lightInit=
  imuReadings.light=analogRead(ldrPin);
}

void doCalculations() {
  accRoll = atan2(imuReadings.accY, imuReadings.accZ) * 180/M_PI;
  accPitch = atan2(-imuReadings.accX, sqrt(imuReadings.accY*imuReadings.accY + imuReadings.accZ*imuReadings.accZ)) * 180/M_PI;
}

void blinking(int pin){
  /*
  Blinking the alarm 5 times (about 1 second procedure)
  */
  for(int k=0; k<=5; k++){
    digitalWrite(pin,HIGH);
    delay(200);
    digitalWrite(pin,LOW);
    delay(200);
    
  }
}

void checkAlarms(){

  if (accRoll<-30){
    //Enable Ligh LEFT
    blinking(leftPin);
  }
  else if (accRoll>30){
    // Enable light Right     
    blinking(rightPin);
  }
  if (imuReadings.light< 600){
    digitalWrite(lightPin,HIGH);
  }else{
    digitalWrite(lightPin,LOW);
  
  }
}
void loop()
{
  //Get accelation readings
  getAccReadings();
  getLDRReadings();
  doCalculations();
  serialPrint();
  checkAlarms();
  // CheckLeft();
  // checkTheLight();
  // if (imuReadings.accY>1){
  //   Serial.print("right");
  // }
  // if (imuReadings.accY<-1){
  //   Serial.print("left");
  // }
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &imuReadings, sizeof(imuReadings));
   
  if (result == ESP_OK) {
    // Serial.println("Sent with success");
  }
  else {
    // Serial.println("Error sending the data");
  }
  delay(500);
}