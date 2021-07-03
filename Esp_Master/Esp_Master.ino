// For IMU
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_Sensor.h>
//For WiFi and TwoWayCom
#include <esp_now.h>
//For MQTT
#include <PubSubClient.h>
#include <WiFi.h>
// Model for gesture detection
#include "model.h"

//Constants Definition
#define NUM_SAMPLES 30  // The sample that collect to identify the head's gestures
#define NUM_AXES 3
// #define TRUNCATE 20  // To prevent the "spikes throughout the data collection"
#define ACCEL_THRESHOLD 0
#define INTERVAL 30

//Model definition
Eloquent::ML::Port::RandomForest clf;

//  Create sensor object
Adafruit_LSM6DS33 imu;

//  Create sensor object
//variables definition
float baseline[NUM_AXES];
float features[NUM_SAMPLES * NUM_AXES];
float ax, ay, az, gx, gy, gz;
float lastSpeed = 0, roll, pitch;  // units degrees (roll and pitch noisy, yaw not possible)


// LDR sensor pin
unsigned long last = 0;           // change that
unsigned long lastSpeedTime = 0;  // change that
//For gestures
unsigned long lastLeftTurn = 0;   // change that
unsigned long lastRightTurn = 0;  // change that
// Set a bool - flag when it finds any gesture to turn left or right
bool rightTurnFlag = false;
bool leftTurnFlag = false;
unsigned int gestureCnt = 0;
// LDR Sensor pin
const int ldrPin = 34;
// Flashed pins
const int rightPin = 13;
const int leftPin = 14;
const int lightPin = 12;
const int brakePin = 27;
//Receiver MAC Address
//C4:4F:33:6B:0F:E1
uint8_t broadcastAddress[] = { 0xC4, 0x4F, 0x33, 0x6B, 0x0F, 0xE1 };

// Define the struct that contains the message content
// for our purposes we import the accelerometer readings
typedef struct masterMessage {
  String turn;
  int lightSensor;
  float curSpeed;
} masterMessage;
//not used
typedef struct receivedMessage {
  bool brake;
  bool seatStatus;
  //String rec_message;
} receivedMessage;

// Define the masterMessage
masterMessage msgToSlave;
// TO_DO -> Change the struct because we will get different content (ie, pin to trigger the flashes)
receivedMessage msgFromSlave;
// Variable to store if sending data was successful
String success;
// Should be global ... ( TODO -> Check why ???)
esp_now_peer_info_t peerInfo;

// WiFi credentials
const char* ssid = "Koutoulakis";
const char* password = "2810751032";

// MQTT IP server
const char* mqtt_server = "192.168.1.20";
int mqtt_port = 1883;
// message to communicate with mqtt server
String message;
bool state = false;

WiFiClient espClient;
PubSubClient client(espClient);


// -------------------------- DECISION-MAKING FUNCTIONS --------------------------
void blinking(int pin) {
  /**
  Blinking the alarm 5 times (in a certain period of time)
  */
  for (int k = 0; k <= 5; k++) {
    digitalWrite(pin, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(pin, LOW);
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

String classify() {
  /**
  Make tha classification of a gesture according to the trained model that used
  @return String The name of the gesture 
  */
  // Serial.print("Detected gesture: ");
  // Serial.println(clf.predictLabel(features));
  return clf.predictLabel(features);
}

void checkLeftTurn() {
  /**
  Check the gesture to the left have been triggered more than once (into a certain period of time)
  The publish the turn into the MQTT broker and the alarm start blinking
  */
  String msg = "turn="+msgToSlave.turn;
  if (leftTurnFlag == true && gestureCnt >= 1 && millis() <= lastLeftTurn + 2000 / portTICK_PERIOD_MS) {
    Serial.println("[LEFT] 2nd gesture");
    client.publish("esp32/TURNS", msg.c_str());
    leftTurnFlag = false;
    gestureCnt = 0;
    blinking(leftPin);
  } else {
    Serial.println("[LEFT] 1st gesture");
    lastLeftTurn = millis();
    gestureCnt += 1;
    leftTurnFlag = true;
  }
}

void checkRightTurn() {
  /**
  Check the gesture to the right have been triggered more than once (into a certain period of time)
  The publish the turn into the MQTT broker and the alarm start blinking
  */
  String msg = "turn="+msgToSlave.turn;
  if (rightTurnFlag == true && gestureCnt >= 1 && millis() <= lastRightTurn + 2000 / portTICK_PERIOD_MS) {
    Serial.println("[RIGHT] 2nd gesture");
    client.publish("esp32/TURNS", msg.c_str());
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


bool motionDetected(float ax, float ay, float az) {
  /**
  Detects motion (not used)
  */
  return (abs(ax) + abs(ay) + abs(az)) > ACCEL_THRESHOLD;
}

void checkBrakes(){
  /**
  If the return message from the Slave is true then it lights up the brakes
  */
}

void checkAlarms() {
  /**
  The main function that investigates the decision of sensros
  */
  checkBrakes();
  // turnMsg="";
  if (msgToSlave.lightSensor < 800) {
    digitalWrite(lightPin, HIGH);
  } else {
    digitalWrite(lightPin, LOW);
  }

  msgToSlave.turn = classify();  // Move this to the main string ?
  // Serial.print(msgToSlave.turn);
  if (msgToSlave.turn == "left") {
  
    checkLeftTurn();
  } else if (msgToSlave.turn == "right") {
    checkRightTurn();
  }
}
// -------------------------- ENDOF DECISION-MAKING FUNCTIONS --------------------------

void task1(void* parameters) {
  /**
   The main task that collects the sensors data and send them to the MQTT server
  */
  for (;;) {
    // Serial.print("Heap Size (BEFORE) : " + String(xPortGetFreeHeapSize()));
    getIMUReadings();
    if (ax > -0.5 && ax <0.5) {
      // This rule is to prevent the noise
      msgToSlave.curSpeed = 0;
    } else {
      msgToSlave.curSpeed = abs(ax * 3.6);
    }
    // Serial.println("Current Speed: " + String(msgToSlave.curSpeed));
    //Get accelation readings
    getRollPitch();
    getLDRReadings();
    if (!client.connected()) {
      reconnect();
    }
    if (millis() > last + 500 / portTICK_PERIOD_MS) {
      // Publish the message to the mqtt server every 1 seconds
      String message = "";
      last = millis();
      serialPrint();
      message = "ldr=" + String(msgToSlave.lightSensor) + ";";
      message += "speed=" + String(msgToSlave.curSpeed) + ";";
      message += "roll=" + String(roll) + ";";
      message += "pitch=" + String(pitch) + ";";
      message += "accX=" + String(ax) + ";";
      message += "accY=" + String(ay) + ";";
      message += "accZ=" + String(az) + ";";
      message += "gyroX=" + String(gx) + ";";
      message += "gyroY=" + String(gy) + ";";
      message += "gyroZ=" + String(gz) + ";";
      message += "seatState=" + String(msgFromSlave.seatStatus) + ";";
      client.publish("esp32/HELMET_INFO", message.c_str());
      // Serial.print("accX (m/s2): ");Serial.print(accX);Serial.print(" accX (km/h): ");Serial.println(accX*3.6);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void task2(void* parameters) {
  /**
  The second task runs simultanuously with the Task1 and it is responsible for the seamless helmet interaction
  according to the sensors' values 
  */
  // Second task check which alarm should be triggered
  // or open the headlights according to the luminosity (ldr sensor)
  for (;;) {
    //Check if catch a gesture
    checkAlarms();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void slaveCommunication(void* parameters) {
  /**
  Yet another task that runs seamlessly. Sends all the essential sensors' data to the slave
  The communication with the slave is established through ESP-NOW protocol
  */
  for (;;) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&msgToSlave, sizeof(msgToSlave));
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  // Initializations
  initIMU();
  calibrate();
  connectToWifi();
  initLDRSensor();
  initESPNOW();
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  // Create the first task
  xTaskCreate(
    task1,    // function name
    "Task1",  // task name
    4000,     // stack size
    NULL,     // task parameters
    1,        // task priority
    NULL      // task handle
  );
  // create the second task
  xTaskCreate(
    task2,    // function name
    "Task2",  // task name
    3000,     // stack size
    NULL,     // task parameters
    1,        // task priority
    NULL      // task handle
  );
  xTaskCreate(
    slaveCommunication,    // function name
    "slaveCommunication",  // task name
    2000,     // stack size
    NULL,     // task parameters
    1,        // task priority
    NULL      // task handle
  );
  vTaskDelay(500 / portTICK_PERIOD_MS);

}

void loop() {}
