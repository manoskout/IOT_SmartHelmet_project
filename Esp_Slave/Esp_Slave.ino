#include <SPI.h>
#include <Adafruit_GFX.h>

// For WIFI and TwoWayCom
#include <esp_now.h>
#include <WiFi.h>

const int rightPin=27;
const int leftPin=25;
const int lightPin=26;

//Receiver MAC Address
// MASTER : 10:52:1C:67:C5:2C
uint8_t broadcastAddress[] = {0x10, 0x52, 0x1C, 0x67, 0xC5, 0x2C};

// Define the struct that contains the message content
// for our purposes we import the accelerometer readings 
typedef struct messageFromMaster {
  float roll;
  float pitch;
  int lightSensor;
} messageFromMaster;

typedef struct struct_message {
  bool imuUsage;
} struct_message;
// Define the struct_message
struct_message slaveMessage; // more accurate name of this structure (the message that sends to the Master)
// TO_DO -> Change the struct because we will get different content (ie, pin to trigger the flashes)
messageFromMaster masterMessage;
// Variable to store if sending data was successful
String success;
float receivedRoll,receivedPitch;
int receivedLight;


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
  receivedRoll = masterMessage.roll;
  receivedPitch = masterMessage.pitch;
  receivedLight = masterMessage.lightSensor;
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

void checkAlarms(){
  if (masterMessage.roll<-30){
    //Enable Ligh LEFT
    blinking(leftPin);
  }
  else if (masterMessage.roll>30){
    // Enable light Right     
    blinking(rightPin);
  }
  if (masterMessage.lightSensor< 800){
    digitalWrite(lightPin,HIGH);
  }else{
    digitalWrite(lightPin,LOW);
  
  }
}

void alarmTask(void * parameters){
  for(;;){
    Serial.print("Roll: ");
    Serial.print(masterMessage.roll);
    Serial.print("    Pitch: ");
    Serial.print(masterMessage.pitch);
    Serial.print("    Light: ");
    Serial.print(masterMessage.lightSensor);
    Serial.println("");
    checkAlarms();
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
  
}

void setup()
{
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
}



void loop()
{

}