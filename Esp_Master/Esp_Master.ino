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
float accX, accY, accZ,
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
typedef struct masterMessage {
  float roll;
  float pitch;
  int lightSensor;
} masterMessage;

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

void serialPrint(){
  Serial.print("X: ");
  Serial.print(accX);
  Serial.print("  Y: ");
  Serial.print(accY);
  Serial.print("  Z: ");
  Serial.print(accZ);
  Serial.print("  Roll: ");
  Serial.print(msgToSlave.roll);
  Serial.print("  Pitch: ");
  Serial.print(msgToSlave.pitch);
  Serial.print("  Light: ");
  Serial.print(msgToSlave.lightSensor);
  Serial.println();

}

void serialPlotter(){
  // Serial.print(accX);
  // Serial.print(accY);
  // Serial.print(accZ);
  Serial.print(msgToSlave.roll);
  Serial.print("\t");
  Serial.println(msgToSlave.pitch);
  
  // Serial.print("  Light: ");
  // Serial.print(msgToSlave.lightSensor);
  // Serial.println();

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
  memcpy(&messageFromSlave,incomingData, sizeof(messageFromSlave));
  //Just for debug
  //Serial.print("Bytes received: ");
  //Serial.println(len);

  // Write the data that have been sent
  // receivedString = messageFromSlave.rec_message;
  
}
void getAccReadings(){
  imu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
}


void getLDRReadings(){
  // Read the current light Levels
  // lightInit=
  msgToSlave.lightSensor=analogRead(ldrPin);
}

void doCalculations() {
  /*
  Calculate Roll and pitch and save them into the structure message
  */
  msgToSlave.roll = atan2(accY, accZ) * 180/M_PI;
  msgToSlave.pitch = atan2(-accX, sqrt(accY*accY + accZ*accZ)) * 180/M_PI;
}

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

void checkAlarms(){
  if (msgToSlave.lightSensor< 800){
    digitalWrite(lightPin,HIGH);
  }else{
    digitalWrite(lightPin,LOW);
  
  }
  if (msgToSlave.roll<-30){
    //Enable Ligh LEFT
    blinking(leftPin);
  }
  else if (msgToSlave.roll>30){
    // Enable light Right     
    blinking(rightPin);
  }
  
}


void task1(void * parameters){
  for(;;){
    Serial.print("Task 1: ");
    //Get accelation readings
    getAccReadings();
    getLDRReadings();
    doCalculations();
    
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &msgToSlave, sizeof(msgToSlave));
    
    if (result == ESP_OK) {
      // Serial.println("Sent with success");
    }
    else {
      // Serial.println("Error sending the data");
    }
    // Delay should be reduced ? 
    // serialPrint();
    serialPlotter();
    checkAlarms();
    // Serial.print(" Overflow Stack :");
    // Serial.println(uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
}

void task2(void * parameters){
  for(;;){
    checkAlarms();
    vTaskDelay(500/portTICK_PERIOD_MS);
    // Serial.print(" Overflow Stack :");
    // Serial.println(uxTaskGetStackHighWaterMark(NULL));
  }
  
}

void setup(){
  Serial.begin(115200);
  initIMU();
  initLDRSensor();
  initESPNOW();

  // Init alarms
  pinMode(leftPin,OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(lightPin,OUTPUT);

  xTaskCreate(
    task1, // function name
    "Task1", // task name
    1400, // stack size
    NULL, // task parameters 
    1, // task priority
    NULL // task handle
    );
  delay(500);
  xTaskCreate(
    task2, // function name
    "Task2", // task name
    1024, // stack size
    NULL, // task parameters 
    1, // task priority
    NULL // task handle
    );
}



void loop(){

}
