// For WIFI and TwoWayCom
#include <esp_now.h>
#include <WiFi.h>

const int rightPin=27;
const int leftPin=25;
const int lightPin=26;
const int brakePin=14;

// TURN Flags
bool rightTurnFlag = false;
bool leftTurnFlag = false;
int gestureCnt=0;

// Pressure sensor
const int pressurePin=32;
float lastSpeedTime=0,lastSpeed=0, lastRightTurn=0, lastLeftTurn=0;
//Receiver MAC Address
// MASTER : 10:52:1C:67:C5:2C
uint8_t broadcastAddress[] = {0x10, 0x52, 0x1C, 0x67, 0xC5, 0x2C};

// Define the struct that contains the message content
// for our purposes we import the accelerometer readings 
typedef struct messageFromMaster {
  String turn;
  int lightSensor;
  float curSpeed;
} messageFromMaster;

typedef struct msgForMaster {
  bool brake;
  bool seatStatus;
} msgForMaster;
// Define the msgForMaster
msgForMaster slaveMessage; // more accurate name of this structure (the message that sends to the Master)
// TO_DO -> Change the struct because we will get different content (ie, pin to trigger the flashes)
messageFromMaster masterMessage;
// Variable to store if sending data was successful
String success;
// String turn;
// int receivedLight;


// Callback Function that sents message
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    // success = "Delivery Success :)";
  }
  else{
    // success = "Delivery Fail :(";
  }
}
// Callback Function that triggered when a new packet arrives
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  memcpy(&masterMessage,incomingData, sizeof(masterMessage));
  //Just for debug
  // Serial.print("Bytes received: ");
  // Serial.println(len);

  // Write the data that have been sent
  // turn = masterMessage.turn;
  // receivedLight = masterMessage.lightSensor;
  // Serial.println("Received Message-> Speed: " + String(masterMessage.curSpeed) + "Turn: " + String(masterMessage.turn)+" , Light Sensor: " + masterMessage.lightSensor);
}

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
  esp_now_peer_info_t peerInfo; // TODO MOVE IT AS GLOBAL
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



void blinking(int pin){
  /*
  Blinking the alarm 5 times (about 1 second procedure)
  */
  for(int k=0; k<=5; k++){
    digitalWrite(pin,HIGH);
    vTaskDelay(200/portTICK_PERIOD_MS);
    // delay(200);
    digitalWrite(pin,LOW);
    // delay(300);
    vTaskDelay(300/portTICK_PERIOD_MS);

    
  }
}
void checkLeft(){
  if (masterMessage.turn=="right"){
    if (rightTurnFlag == true && gestureCnt >= 1 && millis() <= lastRightTurn + 2000 / portTICK_PERIOD_MS) {
      Serial.println("[RIGHT] 2nd gesture");
      rightTurnFlag = false;
      gestureCnt = 0;
      blinking(rightPin);
    } else {
      // Also set a timer in 5 seconds (you could do the second gesture into 5 seconds)
      Serial.println("[RIGHT] 1st gesture");
      lastRightTurn = millis();
      gestureCnt += 1;
      rightTurnFlag = true;
    }
  }
}
void checkRight(){
  if (masterMessage.turn=="left"){
    if (leftTurnFlag == true && gestureCnt >= 1 && millis() <= lastLeftTurn + 2000 / portTICK_PERIOD_MS) {
      Serial.println("[LEFT] 2nd gesture");
      leftTurnFlag = false;
      gestureCnt = 0;
      blinking(leftPin);
    } else {
      // Also set a timer in 5 seconds (you could do the second gesture into 5 seconds)
      Serial.println("[LEFT] 1st gesture");
      lastLeftTurn = millis();
      gestureCnt += 1;
      leftTurnFlag = true;
    }
  }
}
void checkAlarms(){
  checkLeft();
  checkRight();
  
  
  if (masterMessage.lightSensor< 800){
    digitalWrite(lightPin,HIGH);
  }else{
    digitalWrite(lightPin,LOW);
  
  }
}
void checkBrakes(){
  //by recognizing the velocity reduction   
  if (millis() > lastSpeedTime + (200/portTICK_PERIOD_MS) ){
    lastSpeedTime=millis();
      Serial.println("Last speed: "+String(lastSpeed)+ " CurSpeed: "+ String(masterMessage.curSpeed));

    if (masterMessage.curSpeed<(3*lastSpeed)/4 || masterMessage.curSpeed==0.0){
      slaveMessage.brake = true;
      digitalWrite(brakePin,HIGH);
    }else{
    digitalWrite(brakePin,LOW);
    slaveMessage.brake = false;
   }
  lastSpeed=masterMessage.curSpeed;
  }
}

void alarmTask(void * parameters){
  for(;;){
    checkAlarms();
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
  
}
void seatTask(void * parameters){
  /*
  This task reads the values of the pressure sensor
  The basic logic is to discetize the values to 0 or 1 
  in order to know whether the driver is sit or not
  */
  for(;;){
    checkBrakes();
    if (touchRead(pressurePin) >=20){
      slaveMessage.seatStatus=false;
    }else{
      slaveMessage.seatStatus=true;
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

void masterCommunication(void* parameters){
  for(;;){
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &slaveMessage, sizeof(slaveMessage));
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

void setup()
{
  pinMode(brakePin, OUTPUT);
  Serial.begin(115200);
  initESPNOW();
  // Init alarms
  pinMode(leftPin,OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(lightPin,OUTPUT);
  xTaskCreate(
    alarmTask, 
    "AlarmTask", 
    1024, 
    NULL, 
    1, 
    NULL
  );
  xTaskCreate(
    seatTask, 
    "seatTask", 
    1024, 
    NULL, 
    1, 
    NULL
  );
  xTaskCreate(
    masterCommunication,
    "masterCommunication",
    2000,
    NULL,
    1,
    NULL
  );  
}



void loop(){}