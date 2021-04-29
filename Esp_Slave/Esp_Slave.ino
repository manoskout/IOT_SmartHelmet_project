#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// For WIFI and TwoWayCom
#include <esp_now.h>
#include <WiFi.h>

/* Declare LCD object for SPI
 Adafruit_PCD8544(CLK,DIN,D/C,CE,RST); */
Adafruit_PCD8544 display = Adafruit_PCD8544(18, 23, 4, 15, 2);
int contrastValue = 60; // Default Contrast Value
const int adcPin = 34;
int adcValue = 0;


//Receiver MAC Address
// MASTER : 10:52:1C:67:C5:2C
uint8_t broadcastAddress[] = {0x10, 0x52, 0x1C, 0x67, 0xC5, 0x2C};

// Define the struct that contains the message content
// for our purposes we import the accelerometer readings 
typedef struct struct_message_rec {
  float accX;
  float accY;
  float accZ;
  int light;
} struct_message_rec;

typedef struct struct_message {
  char message[10];
} struct_message;
// Define the struct_message
struct_message slaveMessage; // more accurate name of this structure (the message that sends to the Master)
// TO_DO -> Change the struct because we will get different content (ie, pin to trigger the flashes)
struct_message_rec imuReadings;
// Variable to store if sending data was successful
String success;
float receivedAccX;
float receivedAccY;
float receivedAccZ;
int receivedLight;


// Callback Function that sents message
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}
// Callback Function that triggered when a new packet arrives
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  memcpy(&imuReadings,incomingData, sizeof(imuReadings));
  //Just for debug
  // Serial.print("Bytes received: ");
  // Serial.println(len);

  // Write the data that have been sent
  receivedAccX = imuReadings.accX;
  receivedAccY = imuReadings.accY;
  receivedAccZ = imuReadings.accZ;
  receivedLight = imuReadings.light;
}

void initLCD(){
  // Needs update (How to write the data into the LCD?)
  /* Initialize the Display*/
  display.begin();

  /* Change the contrast using the following API*/
  display.setContrast(contrastValue);

  /* Clear the buffer */
  display.clearDisplay();
  display.display();
  delay(1000);
  
  /* Now let us display some text */
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,1);
  display.setTextSize(2);
  display.println("|ESP32|");
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(22,20);
  display.println("|Nokia|");
  display.setCursor(22,32);
  display.println("|5110|");
  display.display();
  delay(2000);
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
  esp_now_peer_info_t peerInfo;
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

void setup()
{
  Serial.begin(115200);
  initESPNOW();
  initLCD();
  
}

void loop()
{
  /* You can implement your own display logic here*/  
  // Display Readings in Serial Monitor
  Serial.print("X: ");
  Serial.print(imuReadings.accX);
  Serial.print("    Y: ");
  Serial.print(imuReadings.accY);
  Serial.print("    Z: ");
  Serial.print(imuReadings.accZ);
  Serial.print("    Light: ");
  Serial.print(imuReadings.light);
  Serial.println();
  delay(1000);
}