#include <i2c.h>
#include <imu.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager
#include "ESC.h" // RC_ESP library installed by Library Manager
#include <math.h>

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
signed short gyro[3] = {0,0,0};
int cur_rpm = 0;
bool confirm;
float acc_x_ave = 0,acc_y_ave = 0,acc_z_ave = 0;
float gyr_x_ave = 0,gyr_y_ave = 0,gyr_z_ave = 0;
float Ax=0,Ay=0,Az=0,Gx=0,Gy=0,Gz=0;
float roll,pitch,yaw;
float dT = 0.002;
float pitch_accel,roll_accel;
float recipNorm;
float ki,kp,kd;
float e_roll,e_roll_prev,e_roll_dif,e_roll_int,e_pitch,e_pitch_prev,e_pitch_int,e_pitch_dif,e_yaw,e_yaw_prev,e_yaw_dif,e_yaw_int;
float roll_ave,pitch_ave,yaw_ave,cntrl_roll,cntrl_pitch,cntrl_yaw;
float a;
float filter_const;
float thrust_const;
float t_roll[4] = {0,0,0,0};
float t_pitch[4] = {0,0,0,0};
float t_yaw[4] = {0,0,0,0};
float w_roll[4] = {0,0,0,0};
float w_pitch[4] = {0,0,0,0};
float w_yaw[4] = {0,0,0,0};
float cmd_thrust[4] = {0,0,0,0};
float cmd_speed[4] = {0,0,0,0};
/**************** SETUP *****************/

void setup() {
  
  Serial.begin(115200); 
  delay(1000);
  
  IMU_setup();
  client_setup();
  ESC_setup();

  ki = 0.1;
  kp = 3;
  kd = 12;

  a = 0.0921;
  thrust_const = pow(7.7141,-7);
  filter_const = 0.5;
// averaging summed accelerometer values
  roll_ave = 1.58;
  pitch_ave = 0.02;
  yaw_ave = 0.00;

  Serial.print("Roll, Pitch, Yaw Values: ");
  Serial.print("\t");
  Serial.print(roll_ave);
  Serial.print("\t");
  Serial.print(pitch_ave);
  Serial.print("\t");
  Serial.println(yaw_ave);

  delay(5000);
  
  roll = 0;
  pitch = 0;
  yaw = 0;

}

/**************** LOOP *****************/

void loop() {

  thrust_const = 0.5;
  
  read_accelerometer((accel),(accel+1),(accel+2));
  read_gyroscope((gyro),(gyro+1),(gyro+2));

// low pass filter on accelerometer readings
  Ax = (float) Ax + 0.1*(accel[0] - Ax);
  Ay = (float) Ay + 0.1*(accel[1] - Ay);
  Az = (float) Az + 0.1*(accel[2] - Az);

  pitch_accel = atan2(-Ax,sqrt(Ay * Ay + Az * Az));
  roll_accel = atan2(Ay,Az);

// integrating gyro values to git roll pitch and yaw
  roll = (float) gyro[0]*0.0174533 * dT;
  pitch = (float) gyro[1]*0.0174533  * dT;
  yaw = (float) gyro[2]*0.0174533  * dT;

  pitch = (1-thrust_const)*pitch + thrust_const*pitch_accel;
  roll = (1-thrust_const)*pitch + thrust_const*roll_accel;

//  Serial.print("Roll Pitch Yaw: ");
//  Serial.print("\t");
//  Serial.print(roll);
//  Serial.print("\t");
//  Serial.print(pitch);
//  Serial.print("\t");
//  Serial.println(yaw);

  webSocket.loop();

  if (roll < 0 && roll >= -1.59) {
    roll_ave = -1.55;
  } else if (roll >= 0 && roll <= 1.59) {
    roll_ave = 1.55;
  }
  e_roll = roll_ave - roll;
  e_pitch = pitch_ave - pitch;
  e_yaw = yaw_ave - yaw;

  e_roll_dif = e_roll - e_roll_prev;
  e_roll_int += e_roll*dT; 
  e_roll_prev = e_roll;

//  if (e_roll_int >= 50) {
//    e_roll_int = 0;
//  }

  e_pitch_dif = e_pitch - e_pitch_prev;
  e_pitch_int += e_pitch*dT; 
  e_pitch_prev = e_pitch;

//  if (e_pitch_int >= 50) {
//    e_pitch_int = 0;
//  }
  
  e_yaw_dif = e_yaw - e_yaw_prev;
  e_yaw_int += e_yaw*dT; 
  e_yaw_prev = e_yaw;

//  if (abs(e_yaw_int) >= 50) {
//    e_yaw_int = ;
//  }

  cntrl_roll = kp*e_roll + kd*e_roll_dif + ki*e_roll_int;
  cntrl_pitch = kp*e_pitch + kd*e_pitch_dif + ki*e_pitch_int;
  cntrl_yaw = kp*e_yaw + kd*e_yaw_dif + ki*e_yaw_int;

  t_roll[0] += 1/a * cntrl_roll;
  t_roll[1] += 1/a * cntrl_roll;
  t_roll[2] -= 1/a * cntrl_roll;
  t_roll[3] -= 1/a * cntrl_roll;

  t_pitch[0] += 1/a * cntrl_pitch;
  t_pitch[1] -= 1/a * cntrl_pitch;
  t_pitch[2] -= 1/a * cntrl_pitch;
  t_pitch[3] += 1/a * cntrl_pitch;

  t_yaw[0] += cntrl_yaw;
  t_yaw[1] -= cntrl_yaw;
  t_yaw[2] += cntrl_yaw;
  t_yaw[3] -= cntrl_yaw;

  w_roll[0] = sqrt(t_roll[0] / thrust_const);
  w_roll[0] = sqrt(t_roll[1] / thrust_const);
  w_roll[2] = sqrt(t_roll[2] / thrust_const);
  w_roll[3] = sqrt(t_roll[3] / thrust_const);

  w_pitch[0] = sqrt(t_pitch[0] / thrust_const);
  w_pitch[1] = sqrt(t_pitch[1] / thrust_const);
  w_pitch[2] = sqrt(t_pitch[2] / thrust_const);
  w_pitch[3] = sqrt(t_pitch[3] / thrust_const);

  w_yaw[0] = sqrt(t_yaw[0] / thrust_const);
  w_yaw[1] = sqrt(t_yaw[1] / thrust_const);
  w_yaw[2] = sqrt(t_yaw[2] / thrust_const);
  w_yaw[3] = sqrt(t_yaw[3] / thrust_const);

  cmd_speed[0] = w_roll[0] + w_pitch[0] + w_yaw[0];
  cmd_speed[1] = w_roll[1] + w_pitch[1] + w_yaw[1];
  cmd_speed[2] = w_roll[2] + w_pitch[2] + w_yaw[2];
  cmd_speed[3] = w_roll[3] + w_pitch[3] + w_yaw[3];

  cmd_thrust[0] = t_roll[0] + t_pitch[0] + t_yaw[0];
  cmd_thrust[1] = t_roll[1] + t_pitch[1] + t_yaw[1];
  cmd_thrust[2] = t_roll[2] + t_pitch[2] + t_yaw[2];
  cmd_thrust[3] = t_roll[3] + t_pitch[3] + t_yaw[3];

  command_speed(w_roll[0],1);
  command_speed(w_roll[1],2);
  command_speed(w_roll[2],3);
  command_speed(w_roll[3],4);

  command_speed(w_pitch[0],1);
  command_speed(w_pitch[1],2);
  command_speed(w_pitch[2],3);
  command_speed(w_pitch[3],4);

  command_speed(w_yaw[0],1);
  command_speed(w_yaw[1],2);
  command_speed(w_yaw[2],3);
  command_speed(w_yaw[3],4);

  Serial.print("Command Speeds [roll]: ");
  Serial.print("\t");
  Serial.print(cmd_thrust[0]);
  Serial.print("\t");
  Serial.print(cmd_thrust[1]);
  Serial.print("\t");
  Serial.print(cmd_thrust[2]);
  Serial.print("\t");
  Serial.println(cmd_thrust=[3]);

//  Serial.print(cntrl_roll);
//  Serial.print("\t");
//  Serial.print(cntrl_pitch);
//  Serial.print("\t");
//  Serial.println(cntrl_yaw);

  delay(dT*1000); // delay 2 ms -- known timing for integral calculation

}

/************** HELPERS ***************/

void read_accelerometer(signed short *acc_x,signed short *acc_y,signed short *acc_z){

 imu.read_data(acc_x,IMU_OUTX_L_XL);
 imu.read_data(acc_y,IMU_OUTY_L_XL);
 imu.read_data(acc_z,IMU_OUTZ_L_XL);
  
}

void read_gyroscope(signed short *gyr_x, signed short *gyr_y, signed short *gyr_z){
 
 imu.read_data(gyr_x,IMU_OUTX_L_G);
 imu.read_data(gyr_y,IMU_OUTY_L_G);
 imu.read_data(gyr_z,IMU_OUTZ_L_G);
  
}

void command_speed(int rpm, int motor){
  if (motor == 1){
    ESC_1.speed(rpm);
  } else if (motor == 2){
    ESC_2.speed(rpm);
  } else if (motor == 3){
    ESC_3.speed(rpm);
  } else if (motor == 4){
    ESC_4.speed(rpm);
  }
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
  Serial.println(""); 
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
        command_speed(cur_rpm,1);
        command_speed(cur_rpm,2);
        command_speed(cur_rpm,3);
        command_speed(cur_rpm,4);

      // Report which led is on 
      } else if ( strcmp((char *)payload, "decreaseRPM") == 0) {
        
        cur_rpm -= 25;
        Serial.printf("Decrementing RPM by 25");
        command_speed(cur_rpm,1);
        command_speed(cur_rpm,2);
        command_speed(cur_rpm,3);
        command_speed(cur_rpm,4);
      } else if ( strcmp((char *)payload, "shutdown") == 0) {
        cur_rpm = 1000;
        Serial.printf("Shutting off motors");
        command_speed(cur_rpm,1);
        command_speed(cur_rpm,2);
        command_speed(cur_rpm,3);
        command_speed(cur_rpm,4);
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

/************ IMU CALIBRATION ***********/
//
//  acc_x_ave = 0;
//  acc_y_ave = 0;
//  acc_z_ave = 0;
//
//  gyr_x_ave = 0;
//  gyr_y_ave = 0;
//  gyr_z_ave = 0;
//
//  Serial.println("Lay IMU flat");
//  confirm = 0;
//  
//  while (not confirm){
//    if (Serial.available() > 0){
//      confirm = Serial.read();
//    }
//    delay(200);
//  }
//  
//  confirm = 0;
//  Serial.println("Calibrating...");
//  Serial.println("");
//  
//  for (int i = 0; i < 1000; i++) {
//    
//    read_accelerometer((accel),(accel+1),(accel+2));
//    read_gyroscope((gyro),(gyro+1),(gyro+2));
//
//// summing and normalizationg accelerometer readings from 16 bit word in two's complement
//    acc_x_ave += (float) accel[0]/32768;
//    acc_y_ave += (float) accel[1]/32768;
//    acc_z_ave += (float) accel[2]/32768;
//
//// summing and normalizing gyroscope readings from 16 bit word in two's complement
//    gyr_x_ave += (float) gyro[0]/32768;
//    gyr_y_ave += (float) gyro[1]/32768;
//    gyr_z_ave += (float) gyro[2]/32768;
//    
//  }
//
//// averaging summed accelerometer values
//  acc_x_ave = (float) acc_x_ave/1000;
//  acc_y_ave = (float) acc_y_ave/1000;
//  acc_z_ave = (float) acc_z_ave/1000;
//
//// averaging summed gyroscope values
//  gyr_x_ave = (float) gyr_x_ave/1000;
//  gyr_y_ave = (float) gyr_y_ave/1000;
//  gyr_z_ave = (float) gyr_z_ave/1000;
//
//  Serial.print("Accelerometer Values: ");
//  Serial.print("\t");
//  Serial.print(acc_x_ave);
//  Serial.print("\t");
//  Serial.print(acc_y_ave);
//  Serial.print("\t");
//  Serial.println(acc_z_ave);
//
//  Serial.print("Gyroscope Values: ");
//  Serial.print("\t");
//  Serial.print(gyr_x_ave);
//  Serial.print("\t");
//  Serial.print(gyr_y_ave);
//  Serial.print("\t");
//  Serial.println(gyr_z_ave);
