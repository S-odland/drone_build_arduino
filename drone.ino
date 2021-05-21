#include <i2c.h>
#include <imu.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager
#include "ESC.h" // RC_ESP library installed by Library Manager

/************** PIN DEFINITIONS *************/

#define LED_PIN 22
#define MOTOR_1 26
#define MOTOR_2 12
#define MOTOR_3 14
#define MOTOR_4 27
#define VR_X 34
#define VR_Y 35 
#define LED_WEB 21

/***************** CONSTANTS ****************/

#define MIN_SPEED 900
#define MAX_SPEED 1700 

const char *ssid = "drone_wifi";
const char *password =  "drone123";
const char *msg_toggle_led = "toggleLED";
const char *msg_get_led = "getLEDState";
const int dns_port = 53;
const int http_port = 80;
const int ws_port = 1337;
const int led_pin = 15;

/************* OBJECT INSTANCES ************/

ESC ESC_1 (MOTOR_1, 1000, 2000, 500); // ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)
ESC ESC_2 (MOTOR_2, 1000, 2000, 500); 
ESC ESC_3 (MOTOR_3, 1000, 2000, 500); 
ESC ESC_4 (MOTOR_4, 1000, 2000, 500); 
IMU_device imu;
TwoWire i2c_lsm6ds33 = TwoWire(0);
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(1337);

/********* VARIABLE DECLARATIONS *********/

int vr_x = 0;
int vr_y = 0;
char msg_buf[10];
int led_state = 0;
bool stop;
signed short accel[3] = {0,0,0};
signed short tilt_loc[10] = {0,0,0,0,0,0,0,0,0,0};
int loc = 10;
int cur_rpm = 0;

/**************** SETUP *****************/

void setup() {
  
  Serial.begin(115200); 
  delay(1000);
  
  IMU_setup();
  client_setup();
  ESC_setup();

}

/**************** LOOP *****************/

void loop() {
  
  webSocket.loop();

}

/************** HELPERS ***************/

void command_speed(int rpm){
  ESC_1.speed(rpm);
  ESC_2.speed(rpm);
  ESC_3.speed(rpm);
  ESC_4.speed(rpm);
}

void ESC_setup() {
  pinMode(VR_X, INPUT);
  pinMode(VR_Y, INPUT);
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(MOTOR_3, OUTPUT);
  pinMode(MOTOR_4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // set led to on to indicate arming
  ESC_1.arm(); // Send the Arm command to ESC
  ESC_2.arm();
  ESC_3.arm();
  ESC_4.arm();
  delay(5000); // Wait a while
  digitalWrite(LED_PIN, LOW); // led off to indicate arming completed
  
  // the following loop turns on the motor slowly, so get ready
  for (int i=0; i<350; i++){ // run speed from 840 to 1190
    ESC_1.speed(MIN_SPEED-200+i); // motor starts up about half way through loop
    ESC_2.speed(MIN_SPEED-200+i); 
    ESC_3.speed(MIN_SPEED-200+i); 
    ESC_4.speed(MIN_SPEED-200+i); 
    delay(10);
  }
}

void IMU_setup() {
  i2c_lsm6ds33.begin(I2C_MASTER_SDA_IO,I2C_MASTER_SCL_IO,I2C_MASTER_FREQ_HZ);
  imu.begin_I2C(LSM6DS33_ID, &i2c_lsm6ds33);
}

void client_setup() {
  if( !SPIFFS.begin()){
    Serial.println("Error mounting SPIFFS");
    while(1);
  }

  WiFi.softAP(ssid,password);

  Serial.println();
  Serial.println("AP running");
  Serial.print("My IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, onIndexRequest);

  // On HTTP request for style sheet, provide style.css
  server.on("/style.css", HTTP_GET, onCSSRequest);

  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);

  // Start web server
  server.begin();

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  Serial.println("Websocket Start");  
}

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:

      // Print out raw message
      Serial.printf("[%u] Received text: %s\n", client_num, payload);

      // Toggle LED
      if ( strcmp((char *)payload, "increaseRPM") == 0 ) {
        
        cur_rpm += 25;
        Serial.printf("Incrementing RPM by 25");
        command_speed(cur_rpm);

      // Report which led is on 
      } else if ( strcmp((char *)payload, "decreaseRPM") == 0) {
        
        cur_rpm -= 25;
        Serial.printf("Decrementing RPM by 25");
        command_speed(cur_rpm);
      } else if ( strcmp((char *)payload, "shutdown") == 0) {
        cur_rpm = 1000;
        Serial.printf("Shutting off motors");
        command_speed(cur_rpm);
      } else if ( strcmp((char *)payload, "getRPM") == 0 ) {
        
        sprintf(msg_buf, "%d", cur_rpm);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      // Message not recognized
      } else {
        Serial.println("[%u] Message not recognized");
      }
      break;

    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

// Callback: send homepage
void onIndexRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/index.html", "text/html");
}

// Callback: send style sheet
void onCSSRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/style.css", "text/css");
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}
