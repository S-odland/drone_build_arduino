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
float dT = 0.002;
float roll,pitch,yaw,pitch_accel,roll_accel,recipNorm,ki,kp,kd,roll_ave,pitch_ave,yaw_ave,cntrl_roll,cntrl_pitch,cntrl_yaw,a,filter_const,
      thrust_const,command_m1,command_m2,command_m3,command_m4,e_roll,e_roll_prev,e_roll_dif,e_roll_int,e_pitch,e_pitch_prev,e_pitch_int,
      e_pitch_dif,e_yaw,e_yaw_prev,e_yaw_dif,e_yaw_int;
bool motorOn = 0;
int ave_speed = 1100;
/**************** SETUP *****************/

void setup() {
  
  Serial.begin(115200); 
  delay(1000);
  
  IMU_setup();
  client_setup();
  ESC_setup();

  ki = 0.05;
  kp = 1.5;
  kd = 5;

  a = 0.0921;
  thrust_const = pow(7.7141,-7);
  filter_const = 0.5;
// averaging summed accelerometer values

  read_accelerometer((accel),(accel+1),(accel+2));
  read_gyroscope((gyro),(gyro+1),(gyro+2));

  for (int i = 0; i < 1000; i ++) {

    read_accelerometer((accel),(accel+1),(accel+2));
    read_gyroscope((gyro),(gyro+1),(gyro+2));

    // low pass filter on accelerometer readings
    Ax = (float) Ax + 0.1*(accel[0] - Ax);
    Ay = (float) Ay + 0.1*(accel[1] - Ay);
    Az = (float) Az + 0.1*(accel[2] - Az);
  
    pitch_accel = 180 * atan2(Ax,sqrt(Ay * Ay + Az * Az))/M_PI;
    roll_accel = 180 * atan2(Ay,sqrt(Ax * Ax + Az * Az))/M_PI;
  
    // integrating gyro values to git roll pitch and yaw
    roll = (float) gyro[0]*0.0174533 * dT;
    pitch = (float) gyro[1]*0.0174533  * dT;
    yaw = (float) gyro[2]*0.0174533  * dT;
  
    pitch = (1-thrust_const)*pitch + thrust_const*pitch_accel;
    roll = (1-thrust_const)*pitch + thrust_const*roll_accel;

    roll_ave += roll_accel;
    pitch_ave += pitch_accel;
    yaw_ave += yaw;
    
  }

  roll_ave = roll_ave/1000;
  pitch_ave = pitch_ave/1000;
  yaw_ave = yaw_ave/1000;

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
  
  webSocket.loop();

  if (motorOn) {
    
    thrust_const = 0.5;
      
    read_accelerometer((accel),(accel+1),(accel+2));
    read_gyroscope((gyro),(gyro+1),(gyro+2));
  
  // low pass filter on accelerometer readings
    Ax = (float) Ax + 0.1*(accel[0] - Ax);
    Ay = (float) Ay + 0.1*(accel[1] - Ay);
    Az = (float) Az + 0.1*(accel[2] - Az);
  
    pitch_accel = 180 * atan2(Ax,sqrt(Ay * Ay + Az * Az))/M_PI;
    roll_accel = 180 * atan2(Ay,sqrt(Ax * Ax + Az * Az))/M_PI;
  
  // integrating gyro values to git roll pitch and yaw
    roll = (float) gyro[0]*0.0174533 * dT;
    pitch = (float) gyro[1]*0.0174533  * dT;
    yaw = (float) gyro[2]*0.0174533  * dT;
  
    pitch = (1-thrust_const)*pitch + thrust_const*pitch_accel;
    roll = (1-thrust_const)*pitch + thrust_const*roll_accel;
  
    if (roll < 0 && roll >= -1.59) {
      roll_ave = -roll_ave;
    } else if (roll >= 0 && roll <= 1.59) {
      roll_ave = roll_ave;
    }
    
    e_roll = roll_ave - roll;
    e_pitch = pitch_ave - pitch;
    e_yaw = yaw_ave - yaw;
  
    e_roll_dif = e_roll - e_roll_prev;
    e_roll_int += e_roll*dT; 
    e_roll_prev = e_roll;
  
    e_pitch_dif = e_pitch - e_pitch_prev;
    e_pitch_int += e_pitch*dT; 
    e_pitch_prev = e_pitch;
    
    e_yaw_dif = e_yaw - e_yaw_prev;
    e_yaw_int += e_yaw*dT; 
    e_yaw_prev = e_yaw;
    
    cntrl_roll = kp*e_roll + kd*e_roll_dif + ki*e_roll_int;
    cntrl_pitch = kp*e_pitch + kd*e_pitch_dif + ki*e_pitch_int;
    cntrl_yaw = kp*e_yaw + kd*e_yaw_dif + ki*e_yaw_int;

/* Need to relate control to motor speeds symetrical about the drone frame

        (0) MOTOR 1         (0) MOTOR 2
        
                   +X (+roll right )
                   ^
                   | 
                   | 
                    --- > +Y (+pitch down)
               

        (0) MOTOR 4         (0) MOTOR 3

        NEGATIVE ROLL  :: MOTOR 1,4 UP, MOTOR 2,3 DOWN
        POSITIVE ROLL  :: MOTOR 2,3 UP, MOTOR 1,4 DOWN

        NEGATIVE PITCH :: MOTOR 1,2 UP, MOTOR 3,4 DOWN
        POSITIVE PITCH :: MOTOR 3,4 UP, MOTOR 1,2 DOWN


    FUNCTION TO MAP ANGLE CONTROLS TO MOTOR SPEEDS:

      - Have average speed --> set by user through web interface
      - Lower sensitivity at smaller error, higher sensitivity at lower error
      - Print out control values to get an idea for the mapping 
      - Mapping should be of the form +/- RPM * Weight where weight has a relation to control values
      - Look to find what asymmetries there are in the motors --> drone seems to be tilting towards motor 3,4
      
*/

  command_m1 = ave_speed + (-cntrl_roll + cntrl_pitch);
  command_m2 = ave_speed + (+cntrl_roll + cntrl_pitch);
  command_m3 = ave_speed + (+cntrl_roll - cntrl_pitch);
  command_m4 = ave_speed + (-cntrl_roll - cntrl_pitch);

  delay(dT*1000); // delay 2 ms -- known timing for integral calculation

    
  } else {
    command_m1 = 0;
    command_m2 = 0;
    command_m3 = 0;
    command_m4 = 0;
  }

  Serial.print("COMMAND SPEEDS: M1 M2 M3 M4 ");
  Serial.print("\t");
  Serial.print(command_m1);
  Serial.print("\t");
  Serial.print(command_m2);
  Serial.print("\t");
  Serial.print(command_m3);
  Serial.print("\t");
  Serial.println(command_m4);

  command_speed(command_m1,1);
  command_speed(command_m2,2);
  command_speed(command_m3,3);
  command_speed(command_m4,4);

//  Serial.println("Commands Sent to Motors");
  
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
      if ( strcmp((char *)payload, "enMotors") == 0 ) {
        
        motorOn = 1;
        Serial.printf("Motors Enabled");

      // Report which led is on 
      } else if ( strcmp((char *)payload, "disMotors") == 0 ) {

        motorOn = 0;
        Serial.printf("Motors Disabled");
        
      } else if ( strcmp((char *)payload, "incBaseMotorSpeed") == 0 ) {

        ave_speed += 5;
        Serial.printf("Increasing baseline motor speed by 1 RPM");
        
      } else if ( strcmp((char *)payload, "decBaseMotorSpeed") == 0 ) {

        ave_speed -= 5;
        Serial.printf("Decreasing baseline motor speed by 1 RPM");

      } else if ( strcmp((char *)payload, "shutdown") == 0) {
        
        ave_speed = 1200;
        Serial.printf("Shutting off motors");
        
      } else if ( strcmp((char *)payload, "getRPM") == 0 ) {
        
        sprintf(msg_buf, "%d", ave_speed);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if (isdigit(*payload) == 1) {

         ave_speed = *payload;
         Serial.println(*payload);
         Serial.println(ave_speed);
      
      } else if ( strcmp((char *)payload, "incKp") == 0 ) {

        kp += 0.01;
        sprintf(msg_buf, "%f", kp);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "decKp") == 0 ) {

        kp -= 0.01;
        sprintf(msg_buf, "%f", kp);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "incKi") == 0 ) {

        ki += 0.01;
        sprintf(msg_buf, "%f", ki);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "decKi") == 0 ) {

        ki -= 0.01;
        sprintf(msg_buf, "%f", ki);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "incKd") == 0 ) {

        kd += 0.01;
        sprintf(msg_buf, "%f", kd);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "decKd") == 0 ) {

        kd -= 0.01;
        sprintf(msg_buf, "%f", kd);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);
      
      } else {
        Serial.println(*payload);
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
