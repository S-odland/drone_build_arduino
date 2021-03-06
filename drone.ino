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
#include <iostream>
#include <fstream>
#include <string>

using namespace std;


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
const int MOVING_AVERAGE_SIZE = 17;

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
signed short accel[3] = {0,0,0};
signed short gyro[3] = {0,0,0};
int cur_rpm = 0;
bool confirm;
float acc_x_ave = 0,acc_y_ave = 0,acc_z_ave = 0;
float gyr_x_ave = 0,gyr_y_ave = 0,gyr_z_ave = 0;
float Ax=0,Ay=0,Az=0,Gx=0,Gy=0,Gz=0;
float dT = 0.005;
float roll,pitch,yaw,pitch_accel,roll_accel,recipNorm,ki,kp,kd,roll_ave,pitch_ave,yaw_ave,cntrl_roll,cntrl_pitch,cntrl_yaw,a,filter_const,
      C,command_m1,command_m2,command_m3,command_m4,e_roll,e_roll_prev,e_roll_dif,e_roll_int,e_pitch,e_pitch_prev,e_pitch_int,
      e_pitch_dif,e_yaw,e_yaw_prev,e_yaw_dif,e_yaw_int;
bool motorOn = 0;
int ave_speed = 1100;
int ind;
int m1_speed = 0;
int m2_speed = 0;
int m3_speed = 0;
float roll_fus = 0;
float pitch_fus = 0;
int m4_speed = 0;
int i = 0;
float pitch_raw,roll_raw,pitch_low,roll_low,pitch_mov,roll_mov;
int iter = 0;
float roll_h = 0;
float pitch_h = 0;
float yaw_h = 0;
float high_const;
float g_low_const;
float int_low_roll = 0;
float int_high_roll = 0;
float int_raw_roll = 0;
float int_low_pitch = 0;
float int_high_pitch = 0;
float int_raw_pitch = 0;
float zero_offset_z = 194.3688;
float zero_offset_y = 198.5680;
float zero_offset_x = 206.1039;
float ave_x = 1.6232;
float ave_y = 288.4133;
float ave_z = -8068.7;
float zero_x,zero_y,zero_z,pitch_zero,roll_zero,pitch_low_zero,roll_low_zero,acc_vec;

/**************** SETUP *****************/

void setup() {
  
  Serial.begin(115200); 
  delay(1000);
  
  IMU_setup();
  client_setup();
  ESC_setup();

  ki = 0.1;
  kp = 0.025;
  kd = 1;

  iter = 7000;

  for (int i = 0; i < iter; i ++) {

    read_accelerometer((accel),(accel+1),(accel+2));
    read_gyroscope((gyro),(gyro+1),(gyro+2));

    Ax = (float) Ax + 0.03*(accel[0] - Ax);
    Ay = (float) Ay + 0.03*(accel[1] - Ay);
    Az = (float) Az + 0.03*(accel[2] - Az);

    pitch_low = atan2(Ax,sqrt(Ay * Ay + Az * Az));
    roll_low = atan2(Ay,sqrt(Ax * Ax + Az * Az));

    roll_ave += roll_low;
    pitch_ave += pitch_low;
    yaw_ave += yaw;
    
  }

  roll_ave = roll_ave/iter;
  pitch_ave = pitch_ave/iter;
  yaw_ave = yaw_ave/iter;

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

  ki = 26.7;
  kp = 12.3;
  kd = 40;
  
}

/**************** LOOP *****************/

void loop() {
  
  webSocket.loop();
  
  if (motorOn) {
    
    read_accelerometer((accel),(accel+1),(accel+2));
    read_gyroscope((gyro),(gyro+1),(gyro+2));
    
//    acc_vector = sqrt(pow(accel[0],2) + pow(accel[1],2) + pow(accel[2],2));

    pitch_raw = atan2(accel[0],sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
    roll_raw = atan2(accel[1],sqrt(accel[0] * accel[0] + accel[2] * accel[2]));

    // low pass filter on accelerometer readings
    Ax = (float) Ax + 0.03*(accel[0] - Ax);
    Ay = (float) Ay + 0.03*(accel[1] - Ay);
    Az = (float) Az + 0.03*(accel[2] - Az);

    pitch_low = atan2(Ax,sqrt(Ay * Ay + Az * Az));
    roll_low = atan2(Ay,sqrt(Ax * Ax + Az * Az));

    C = 0.1;
    
    roll_fus =  (1-C)*(roll_fus-gyro[0]*0.0174533*dT) + C*roll_low;
    pitch_fus =  (1-C)*(pitch_fus+gyro[1]*0.0174533*dT) + C*pitch_low;
   
    e_roll = roll_fus - roll_ave;
    e_pitch = pitch_fus - pitch_ave;
    e_yaw = yaw - yaw_ave;
  
    e_roll_dif = e_roll - e_roll_prev;
    e_roll_int += e_roll*dT; 
    e_roll_prev = e_roll;
  
    e_pitch_dif = e_pitch - e_pitch_prev;
    e_pitch_int += e_pitch*dT; 
    e_pitch_prev = e_pitch;
    
    e_yaw_dif = e_yaw - e_yaw_prev;
    e_yaw_int += e_yaw*dT; 
    e_yaw_prev = e_yaw;

    if (abs(e_roll_int)>2) {
      e_roll_int = 0; 
    }

    if (abs(e_pitch_int)>2) {
      e_pitch_int = 0;
    }

    if (roll_ave < -0.1 or roll_ave > 0.1) {
      cntrl_roll = kp*e_roll + kd*e_roll_dif + 0*e_roll_int;
      cntrl_pitch = kp*e_pitch + kd*e_pitch_dif + ki*e_pitch_int;
      cntrl_yaw = kp*e_yaw + kd*e_yaw_dif + ki*e_yaw_int;
    } else if (pitch_ave < -0.1 or pitch_ave > 0.1){
      cntrl_roll = kp*e_roll + kd*e_roll_dif + ki*e_roll_int;
      cntrl_pitch = kp*e_pitch + kd*e_pitch_dif + 0*e_pitch_int;
      cntrl_yaw = kp*e_yaw + kd*e_yaw_dif + ki*e_yaw_int;
    } else {
      cntrl_roll = kp*e_roll + kd*e_roll_dif + ki*e_roll_int;
      cntrl_pitch = kp*e_pitch + kd*e_pitch_dif + ki*e_pitch_int;
      cntrl_yaw = kp*e_yaw + kd*e_yaw_dif + ki*e_yaw_int;
    }

    

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
      
*/

    command_m1 = m1_speed + cntrl_roll - cntrl_pitch;
    command_m2 = m2_speed - cntrl_roll - cntrl_pitch;
    command_m3 = m3_speed - cntrl_roll + cntrl_pitch;
    command_m4 = m4_speed + cntrl_roll + cntrl_pitch;
  
    Serial.print(accel[0]);
    Serial.print("\t");
    Serial.print(accel[1]);
    Serial.print("\t");
    Serial.print(accel[2]);
    Serial.print("\t");
    Serial.print(gyro[0]);
    Serial.print("\t");
    Serial.print(gyro[1]);
    Serial.print("\t");
    Serial.print(gyro[2]);
    Serial.print("\t");
    Serial.print(kp);
    Serial.print("\t");
    Serial.print(ki);
    Serial.print("\t");
    Serial.print(kd);
    Serial.print("\t");
    Serial.print(roll_raw);
    Serial.print("\t");
    Serial.print(pitch_raw);
    Serial.print("\t");
    Serial.print(roll_low);
    Serial.print("\t");
    Serial.print(pitch_low);
    Serial.print("\t");
    Serial.print(roll_fus);
    Serial.print("\t");
    Serial.print(pitch_fus);
    Serial.print("\t");
    Serial.print(e_roll);
    Serial.print("\t");
    Serial.print(e_pitch);
    Serial.print("\t");
    Serial.print(e_roll_int);
    Serial.print("\t");
    Serial.print(e_pitch_int);
    Serial.print("\t");
    Serial.print(e_roll_dif);
    Serial.print("\t");
    Serial.print(e_pitch_dif);
    Serial.print("\t");
    Serial.print(cntrl_roll);
    Serial.print("\t");
    Serial.print(cntrl_pitch);
    Serial.print("\t");
    Serial.print(command_m1);
    Serial.print("\t");
    Serial.print(command_m2);
    Serial.print("\t");
    Serial.print(command_m3);
    Serial.print("\t");
    Serial.println(command_m4);

    
  } else {
    command_m1 = 0;
    command_m2 = 0;
    command_m3 = 0;
    command_m4 = 0;
  }

  command_speed(command_m1,1);
  command_speed(command_m2,2);
  command_speed(command_m3,3);
  command_speed(command_m4,4);

  // delay(dT*1000); // delay 2 ms -- known timing for integral calculation
  
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
    //Serial.println("Error mounting SPIFFS");
    while(1);
  }

  WiFi.softAP(ssid,password);

  //Serial.println();
  //Serial.println("AP running");
  //Serial.print("My IP address: ");
  //Serial.println(WiFi.softAPIP());

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

  //Serial.println("Websocket Start"); 
  //Serial.println(""); 
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
      //Serial.printf("[%u] Disconnected!\n", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        //Serial.printf("[%u] Connection from ", client_num);
        //Serial.println(ip.toString());
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:

      // Print out raw message
      //Serial.printf("[%u] Received text: %s\n", client_num, payload);

      // Toggle LED
      if ( strcmp((char *)payload, "enMotors") == 0 ) {
        
        motorOn = 1;
        //Serial.printf("Motors Enabled");

      // Report which led is on 
      } else if ( strcmp((char *)payload, "disMotors") == 0 ) {

        motorOn = 0;
        //Serial.printf("Motors Disabled");
        
      } else if ( strcmp((char *)payload, "incBaseMotorSpeed") == 0 ) {

        //ave_speed += 5;
        m1_speed += 5;
        m2_speed += 5;
        m3_speed += 5;
        m4_speed += 5;
        //Serial.printf("Increasing baseline motor speed by 1 RPM");
        
      } else if ( strcmp((char *)payload, "decBaseMotorSpeed") == 0 ) {

        //ave_speed -= 5;
        m1_speed -= 5;
        m2_speed -= 5;
        m3_speed -= 5;
        m4_speed -= 5;
        //Serial.printf("Decreasing baseline motor speed by 1 RPM");

      } else if ( strcmp((char *)payload, "incMotor1") == 0 ) {

        m1_speed += 1;
        Serial.printf("Increasing baseline motor speed by 1 RPM");
        sprintf(msg_buf, "%d", m1_speed);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);
        
      } else if ( strcmp((char *)payload, "decMotor1") == 0 ) {

        m1_speed -= 1;
        Serial.printf("Decreasing baseline motor speed by 1 RPM");
        sprintf(msg_buf, "%d", m1_speed);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "incMotor2") == 0 ) {

        m2_speed += 1;
        Serial.printf("Increasing baseline motor speed by 1 RPM");
        sprintf(msg_buf, "%d", m2_speed);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);
        
      } else if ( strcmp((char *)payload, "decMotor2") == 0 ) {

        m2_speed -= 1;
        Serial.printf("Decreasing baseline motor speed by 1 RPM");
        sprintf(msg_buf, "%d", m2_speed);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);


      } else if ( strcmp((char *)payload, "incMotor3") == 0 ) {

        m3_speed += 1;
        Serial.printf("Increasing baseline motor speed by 1 RPM");
        sprintf(msg_buf, "%d", m3_speed);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);
        
      } else if ( strcmp((char *)payload, "decMotor3") == 0 ) {

        m3_speed -= 1;
        Serial.printf("Decreasing baseline motor speed by 1 RPM");
        sprintf(msg_buf, "%d", m3_speed);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);


      } else if ( strcmp((char *)payload, "incMotor4") == 0 ) {

        m4_speed += 1;
        Serial.printf("Increasing baseline motor speed by 1 RPM");
        sprintf(msg_buf, "%d", m4_speed);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);
        
      } else if ( strcmp((char *)payload, "decMotor4") == 0 ) {

        m4_speed -= 1;
        Serial.printf("Decreasing baseline motor speed by 1 RPM");
        sprintf(msg_buf, "%d", m4_speed);
        Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "shutdown") == 0) {
        
        ave_speed = 1200;
        m1_speed = 1200;
        m2_speed = 1200;
        m3_speed = 1200;
        m4_speed = 1200;
        e_roll_int = 0;
        e_pitch_int = 0;
        //Serial.printf("Shutting off motors");
        
      } else if ( strcmp((char *)payload, "getRPM") == 0 ) {
        
        sprintf(msg_buf, "%d", ave_speed);
        //Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if (isdigit(*payload) == 1) {
        
        String stringOne = String((char *)payload);
        //Serial.printf("%s\n",stringOne);
        stringOne.toInt();
        m1_speed = stringOne.toInt();
        m2_speed = stringOne.toInt();
        m3_speed = stringOne.toInt();
        m4_speed = stringOne.toInt();
        //Serial.printf("Motor speeds set");
        sprintf(msg_buf, "%d", m2_speed);
        //Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);
      
      } else if ( strcmp((char *)payload, "incKp") == 0 ) {

        kp += 0.1;
        sprintf(msg_buf, "%f", kp);
        //Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "decKp") == 0 ) {

        kp -= 0.1;
        sprintf(msg_buf, "%f", kp);
        //Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "incKi") == 0 ) {

        ki += 0.1;
        sprintf(msg_buf, "%f", ki);
        //Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "decKi") == 0 ) {

        ki -= 0.1;
        sprintf(msg_buf, "%f", ki);
        //Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "incKd") == 0 ) {

        kd += 0.1;
        sprintf(msg_buf, "%f", kd);
        //Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);

      } else if ( strcmp((char *)payload, "decKd") == 0 ) {

        kd -= 0.1;
        sprintf(msg_buf, "%f", kd);
        //Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
        webSocket.sendTXT(client_num, msg_buf);
        
      } else if ( strcmp((char *)payload, "hoverLeft") == 0 ) {

        roll_ave = roll_ave + 0.3;

      } else if ( strcmp((char *)payload, "hoverRight") == 0 ) {

        roll_ave = roll_ave - 0.3;

      } else if ( strcmp((char *)payload, "hoverForward") == 0 ) {

        pitch_ave = pitch_ave - 0.3;

      } else if ( strcmp((char *)payload, "hoverBackward") == 0 ) {

        pitch_ave = pitch_ave + 0.3;
        
      } else {
     
        //Serial.println(*payload);
        //Serial.println("[%u] Message not recognized");
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
  //Serial.println("[" + remote_ip.toString() +
  //                "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/index.html", "text/html");
}

// Callback: send style sheet
void onCSSRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  //Serial.println("[" + remote_ip.toString() +
  //                "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/style.css", "text/css");
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  //Serial.println("[" + remote_ip.toString() +
  //                "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}
