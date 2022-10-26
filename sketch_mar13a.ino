#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include "PCF8574.h"
#include <TinyMPU6050.h>

#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.
#define AVC_WIDTH 15 //AVC width in cm

PCF8574 pc1(0x20);
PCF8574 pc2(0x24);
MPU6050 mpu (Wire);

#define AVC_SMALL 10
bool isFB = true;

/*TB6612FNG Dual Motor Driver Carrier*/

#define RightFrontMotor_PWM_CHANNEL 0 
#define LeftFrontMotor_PWM_CHANNEL 1 
#define RightRearMotor_PWM_CHANNEL 2 
#define LeftRearMotor_PWM_CHANNEL 3

#define RightFrontMotor_PWM 16 // pwm output pc1
#define LeftFrontMotor_PWM 18 // pwm output pc2
#define RightRearMotor_PWM 17 // pwm output pc1
#define LeftRearMotor_PWM 5 // pwm output pc2

//Front motors
#define RightFrontMotor_AIN1 3 // control Input AIN1 - right front motor PC1
#define RightFrontMotor_AIN2 4 // control Input AIN2 - right front motor
#define LeftFrontMotor_BIN1 3 // control Input BIN1 - left front motor PC2
#define LeftFrontMotor_BIN2 4 // control Input BIN2 - left front motor
 //Rear motors
#define RightRearMotor_AIN1 2 // control Input AIN1 - right rear motor PC1
#define RightRearMotor_AIN2 1 // control Input AIN2 - right rear motor
#define LeftRearMotor_BIN1 2 // control Input BIN1 - left rear motor PC2
#define LeftRearMotor_BIN2 1 // control Input BIN2 - left rear  motor

#define RELAY_VAC 33 // relay pin
#define MAX_SPEED 255
#define SONAR_NUM 3

//INTERUPT VARIABLES
// #define obsL_pin 19
// #define obsR_pin 23
#define home_pin 32
#define REDLED 5
#define BLUELED 7
#define GREENLED 6

int curspd = 0;
int DEF_SPEED = 192;
int varspeed = 63;
double maxBlockSpeed = 0.00207; //per millisec

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
const char *ssid = "KinesisRoboVAC";
const char *password = "qwerty12";
String routerSSID, routerPASSWORD;
int target, newVal;
bool isMoving = false;
uint8_t isClient = 0;

//STATES
bool isEdging = true;  //is it choosing edge
bool isMovingFB = true; //is it moving forward
volatile bool hasStarted = false; //has it started cleaning
bool isSpotting = false; //has it started spot cleaning
bool hasLocSpot = false;
bool isMapping = false; //has it started mapping
bool isCharging = false; // is it charging
bool turnState = true; //is it forward or backward
bool shouldCharge = false;
// volatile bool shouldML = false;
// volatile bool shouldMR = false;

long etime,oldTime,chargeTime;
int cmod,ncmod,cleanCnt;

const int freq = 30000;
const int resolution = 8;

//BATTERY CONSTANTS
double rr12 = 0.0031;
double batP = 27.7778;
int batteryPercent = 100;
long totalChargeTime = 7200000;

//MAPPING VARIABLES
bool mapTurn = true;
int shouldEnd = 0;

//SPOT VARIABLES
double speedAVC = 0.00155; //in blocks per millisecond
int fx,fy; //block to find
int cx,cy; //current position in block
int ox,oy; //station position or original position
int spotState = 0;
bool isAbove = false;
bool isByRight = false;
int nState,inState,h;

//SEND DATA VARIABLES
long oldSendTime;

// void IRAM_ATTR hitObstacleL(){  
//   shouldMR = true;
// }

// void IRAM_ATTR hitObstacleR(){
// shouldML = true;
// }

void IRAM_ATTR homefunc(){
  hasStarted = !hasStarted;
}

// void notifyClients() {
//   if (isClient > 0) {
//     String a = String(newVal) + "%" + String(isMoving);
// Serial.println(a);
//     ws.textAll(a);
//   }
// }

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    if (strcmp((char *)data, "clean") == 0) { //pause
      hasStarted = !hasStarted;
      if(hasStarted){
        oldSendTime = millis() + 100;
      }
      isSpotting = false; 
      isMapping = false;      
    }else if(strcmp((char *)data, "reclean") == 0){
      isSpotting = false; 
      isMapping = false;      
      isEdging = true;  
      isMovingFB = true; 
      hasStarted = true; 
      oldSendTime = millis() + 100;
      turnState = true; 
    }else if(strcmp((char *)data, "spot") == 0){
      hasStarted = false;
      isMapping = false;
      spotState = 0;
      hasLocSpot = false;      
      isAbove = false;
      isByRight = false;      
    }else if(strcmp((char *)data, "map") == 0){
      hasStarted = false;
      isMapping = true;      
      isEdging = true;
      isSpotting = false;
      mapTurn = true;
      shouldEnd = 0;
      oldSendTime = millis() + 100;
    }else {
      char* newData = strtok((char *)data,"%");
      if((char *)newData[0] == "spot"){
        isSpotting = true;
        curUs();
        fx = newData[1];
        fy = newData[2];      
      }else if(strcmp((char *)newData[0], "changespeed") == 0){
        DEF_SPEED = map((newData[1]-'0'),0,100,128,255);
        varspeed = MAX_SPEED-DEF_SPEED;
        speedAVC = (DEF_SPEED / (MAX_SPEED+1)) * maxBlockSpeed;
      }
    }
  }
}

void wsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.println("Connected");
      isClient++;
      break;
    case WS_EVT_DISCONNECT:
      Serial.println("Disconnected");
      isClient--;
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  initMPU();

  pinMode(RELAY_VAC, OUTPUT);
pinMode(RightFrontMotor_PWM,OUTPUT);
pinMode(LeftFrontMotor_PWM,OUTPUT);
pinMode(RightRearMotor_PWM,OUTPUT);
pinMode(LeftRearMotor_PWM,OUTPUT);
pinMode(35,INPUT);

setPinModes();

  pc1.begin();
  pc2.begin(); 
  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);
  ledcSetup(2, freq, resolution);
  ledcSetup(3, freq, resolution);  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(RightFrontMotor_PWM, 0);
  ledcAttachPin(LeftFrontMotor_PWM, 1);
  ledcAttachPin(RightRearMotor_PWM, 2);
  ledcAttachPin(LeftRearMotor_PWM, 3);  

  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAP(ssid, password);

Serial.println(hasStarted);
  server.on("/chooseMode", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (ON_AP_FILTER(request)) {
      if (setRouterDetails(request)) {
        WiFi.begin(s2c(routerSSID), s2c(routerPASSWORD));

        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.println("Connecting to WiFi..");
        }

        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
      }
      request->send(200, "text/plain", "success");
      return;
    }

    request->send(200, "text/plain", "undefined");
  });

server.on("/toggleClean", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (ON_AP_FILTER(request)) {
      hasStarted = !hasStarted;
      if(hasStarted){
        oldSendTime = millis() + 100;
      }
      request->send(200, "text/plain", "success");
      return;
    }
    request->send(200, "text/plain", "undefined");
  });  

server.on("/checkBattery", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (ON_AP_FILTER(request)) {
      checkBattery();      
      request->send(200, "text/plain", String(batteryPercent));
      return;
    }
    request->send(200, "text/plain", "undefined");
  });    

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  oldTime = millis() + 60000;
  curUs();
  ox = cx;
  oy = cy;
  setInterruptPins();
  server.onNotFound(notFound);

  ws.onEvent(wsEvent);
  server.addHandler(&ws);

  server.begin();
  
}

void loop() {
  //ws.cleanupClients();
if(isCharging){
  showLED(0,1,1);  
  if(chargeTime < millis()){
    isCharging = false;
    batteryFull();    
  }
}else{
  if(hasStarted){
    showLED(1,0,0); 
    automod();
  }else{
    if(isSpotting){
    showLED(1,1,0);       
      setupSpot();
      if(hasLocSpot){
        spotClean();
        hasLocSpot = false;
        isSpotting = false;
      }
    }else if(isMapping){
      showLED(1,0,1);       
      mapEdge();
    }else{
      showLED(0,0,0); 
      hardStop();    
    }
  }
  if(oldTime < millis()){
    checkBattery();
  }
// if(shouldMR){
// moveRight(0);
//   delay(500);  
//   shouldMR = false;
// }
// if(shouldML){
// moveLeft(0);
//   delay(500);  
//   shouldML = true;
// }
  // if(batteryPercent < 25){
  //   batteryLow();
  //   curUs();
  //   fx = ox;
  //   fy = oy;
  //   while(!hasLocSpot){
  //     setupSpot();
  //   }
  //   autoTurn(0);
  //   isCharging = true;
  //   chargeTime = millis() + totalChargeTime;
  // }
}  
}

///chooseMode --- the user chooses whether he wants internet or no internet connection
///startClean --- the avc starts cleaning automatically
///stopClean --- the avc stops cleaning

///charge --- the avc goes to the charging station
///map --- the avc maps its surroundings
///spot --- the avc goes to a particular location and cleans there
///nozone --- the avc does not clean this area ******

//Autonomous Operation
bool isNotNear(uint8_t pos) {
  return posSpace(pos) > AVC_SMALL;
}

uint16_t posSpace(uint8_t pos) {
  //delay(35);
  uint16_t dist = pingcm(pos);
  return dist == 0  ? 400 : dist;
}

bool isR() {
  return isNotNear(2);
}
 
bool isF() {
  return isNotNear(0);
}

bool anyIsNotNear() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (!isNotNear(i)) {
      return false;
    }
  }
  return true;
}

// uint8_t getLongestDistancePos(){ //get the direction of the longest pos
//   uint16_t maxNo = 0;
//     uint8_t maxNoIndex = 0;
//     for (uint8_t i = 0; i < SONAR_NUM; i++) { //get the largest distance
//       if (posSpace(i) > maxNo) {
//         maxNo = posSpace(i);
//         maxNoIndex = i;
//       }
//     }
//     return maxNoIndex;
// }

// uint8_t* getFreeSpace() {
//   static uint8_t freeSpace[3] = {0, 0, 0};
//   for (uint8_t i = 0; i < SONAR_NUM; i++) {
//     freeSpace[i] = isNotNear(i) ? 1 : 0;
//   }
//   return freeSpace;
// }

// uint8_t* getSmallSpace() {
//   uint8_t smallSpace[3] = {1, 1, 1};
//   if(posSpace(1) < posSpace(2)){
//     smallSpace[2] = 0;
//   }else{
//     smallSpace[0] = 0;
//   }
//   // uint8_t j = 0;
//   // while (j < 2) {
//   //   int maxNo = 0;
//   //   uint8_t maxNoIndex = 0;
//   //   for (uint8_t i = j; i < SONAR_NUM; i += 2) { //get the largest distance
//   //     if (posSpace(i) > maxNo) {
//   //       maxNo = posSpace(i);
//   //       maxNoIndex = i;
//   //     }
//   //   }
//   //   smallSpace[maxNoIndex] = 0;
//   //   j++;
//   // }
//   return smallSpace;
// } 

void chooseEdge() {
int  a = pingcm(0);
int b = pingcm(1);
int c = pingcm(2);
Serial.print("\t");
Serial.print(a);
Serial.print("\t");
Serial.print(b);
Serial.print("\t");
Serial.print(c);
   if (anyIsNotNear()) {
     moveRange(posSpace(1) < posSpace(2) ? 1 : 2);
     target = absolute();
  }else{
    if (isNotNear(0)) { //move forward until it meets another edge
      correctMotion();
      movePos(0);
    }else{
      if(posSpace(1) == posSpace(2)){
        autoTurn(180);
      }else{
        turn90(posSpace(1) < posSpace(2));
      }
      cmod = getCmod();
      ncmod = getCmod();  
      cleanCnt = 0;
      target = absolute();
      isEdging = false;
    Serial.println("Found edge");      
    }    
  }
}

int getCmod() {
  return isR() ? 2 : 1;
}

void startClean() {
  if(isMovingFB){
    if (isNotNear(0) ) {
      checkPosV1(true,false); 
      correctSpeed();
      //curspd = (int)(abs(newVal - target) * varspeed / (180 + abs(target)));      
      movePos(0);
    }else{
      isMovingFB = false;
      ncmod = turnState ? cmod : 3-cmod;
      etime = millis() + 2000;
      turnPos(ncmod);
    }      
  }else{
    if (isNotNear(0) && (millis() < etime)  ) {
      checkPosV1(false,false); 
      movePos(0);
    }else{
      turnPos(ncmod);
      turnState = !turnState;  
      target = absolute();      
      isMovingFB = true;
       
    }           
  }
}

void checkPosV1(bool a,bool hasEnded){
  sendClientMotionData(a,turnState,hasEnded);
}

void automod() {
  if(isEdging){
    Serial.println("Looking for edge");
    chooseEdge();
  }else{
    switchVac(true);    
    startClean();
  }
}

void movePos(uint8_t pos) {
  switch (pos) {
    case 0:
      goForward(curspd);
      break;
    case 1:
      moveLeft(0);
      break;
    case 2:
      moveRight(0);
      break;
  }
}

void turnPos(uint8_t pos) {
  switch (pos) {
    case 1:
      turn90(false);
      break;
    case 2:
      turn90(true);
      break;
  }
}

void moveRange(uint8_t a) {
  switch (a) {
    case 1:
      moveLeftForward(MAX_SPEED);
      break;
    case 2:
      moveRightForward(MAX_SPEED);
      break;
  }
}

/*++++++++++++MOTOR CONTROL++++++++++++++*/
void motorControl(String motorStr, int8_t mdirection, uint8_t mspeed) {
  uint8_t IN1;
  uint8_t IN2;
  uint8_t motorPWM;
  bool isPC1;
  if (motorStr == "rf") {       //right front
    IN1 = RightFrontMotor_AIN1;
    IN2 = RightFrontMotor_AIN2;
    motorPWM = RightFrontMotor_PWM_CHANNEL;
    isPC1 = true;
  }
  else if (motorStr == "lf") { //left front
    IN1 = LeftFrontMotor_BIN1;
    IN2 = LeftFrontMotor_BIN2;
    motorPWM = LeftFrontMotor_PWM_CHANNEL;
    isPC1 = false;
  }
  else if (motorStr == "rr") {
    IN1 = RightRearMotor_AIN1;
    IN2 = RightRearMotor_AIN2;
    motorPWM = RightRearMotor_PWM_CHANNEL;
    isPC1 = true;
  }
  else if (motorStr == "lr") {
    IN1 = LeftRearMotor_BIN1;
    IN2 = LeftRearMotor_BIN2;
    motorPWM = LeftRearMotor_PWM_CHANNEL;
    isPC1 = false;

  }
  if (mdirection == 1) {
    if (isPC1) {
      pc1.write(IN1, HIGH);
      pc1.write(IN2, LOW);
    } else {
      pc2.write(IN1, LOW);
      pc2.write(IN2, HIGH);
    }    
    
  }
  else if (mdirection == -1) {
    if (isPC1) {
      pc1.write(IN1, LOW);
      pc1.write(IN2, HIGH);
    } else {
      pc2.write(IN1, HIGH);
      pc2.write(IN2, LOW);
    }
  }
  ledcWrite(motorPWM, mspeed);
}

/*++++++++++++MOTOR FUNCTIONS++++++++++++++*/
void goForward(uint8_t mspeed) {
  motorControl("rf", 1, DEF_SPEED + mspeed);
  motorControl("lf", 1, DEF_SPEED - mspeed);
  motorControl("rr", -1, DEF_SPEED + mspeed);//rr = 1
  motorControl("lr", 1, DEF_SPEED - mspeed);
  isMoving = true;  
}// void goForward(uint8_t mspeed)

void goBackward(uint8_t mspeed) {
  motorControl("rf", -1, DEF_SPEED - mspeed);
  motorControl("lf", -1, DEF_SPEED + mspeed);
  motorControl("rr", 1, DEF_SPEED - mspeed);//rr = -1
  motorControl("lr", -1, DEF_SPEED + mspeed);
  isMoving = true;
}// void goBackward(uint8_t mspeed)

void moveRight(uint8_t mspeed) {
  motorControl("rf", -1, DEF_SPEED - mspeed);
  motorControl("lf", 1, DEF_SPEED + mspeed);
  motorControl("rr", -1, DEF_SPEED + mspeed);//rr = 1
  motorControl("lr", -1, DEF_SPEED - mspeed);
  isMoving = true;
}// void moveRight(uint8_t mspeed)

void moveLeft(uint8_t mspeed) {
  motorControl("rf", 1, DEF_SPEED + mspeed);
  motorControl("lf", -1, DEF_SPEED - mspeed);
  motorControl("rr", 1, DEF_SPEED - mspeed); //rr = -1
  motorControl("lr", 1, DEF_SPEED + mspeed);
  isMoving = true;
}// void moveLeft(uint8_t mspeed)

void moveRightForward(uint8_t mspeed) {
  motorControl("rf", 1, 0);
  motorControl("lf", 1, mspeed);
  motorControl("rr", -1, mspeed);//rr = 1
  motorControl("lr", 1, 0);
  isMoving = true;
}// void  moveRightForward(uint8_t mspeed)

void moveRightBackward(uint8_t mspeed) {
  motorControl("rf", -1, mspeed);
  motorControl("lf", 1, 0);
  motorControl("rr", -1, 0);//rr = 1
  motorControl("lr", -1, mspeed);
  isMoving = true;
}// void  moveRightBackward(uint8_t mspeed)

void moveLeftForward(uint8_t mspeed) {
  motorControl("rf", 1, mspeed);
  motorControl("lf", 1, 0);
  motorControl("rr", -1, 0);//rr = 1
  motorControl("lr", 1, mspeed);
  isMoving = true;
}// void  moveLeftForward(uint8_t mspeed)

void moveLeftBackward(uint8_t mspeed) {
  motorControl("rf", 1, 0);
  motorControl("lf", -1, mspeed);
  motorControl("rr", 1, mspeed);//rr = -1
  motorControl("lr", 1, 0);
  isMoving = true;
}// void  moveLeftBackward(uint8_t mspeed)

void turnRight(uint8_t mspeed) {
  motorControl("rf", -1, mspeed);
  motorControl("lf", 1, mspeed);
  motorControl("rr", 1, mspeed);//rr = -1
  motorControl("lr", 1, mspeed);
isMoving = false;  
}// void turnRight(uint8_t mspeed)

void turnLeft(uint8_t mspeed) {
  motorControl("rf", 1, mspeed);
  motorControl("lf", -1, mspeed);
  motorControl("rr", -1, mspeed);//rr = 1
  motorControl("lr", -1, mspeed);
isMoving = false;  
}// void turnRight(uint8_t mspeed)

// how to find if something is moving
// in turn and off state - moving is false
// in move forward,left,right = moving is true

void turn90(bool isRight){
  turn(isRight,90);
}

void autoTurn(int newAngle){
  target = absolute(); 
  int tSpace = target-newAngle;
  int rSpace = abs(tSpace);
  if(rSpace < 180){
    turn(tSpace > 0,rSpace);
  }else{
    turn(tSpace < 0,360-rSpace);    
  }    
}

void turn(bool isRight,int angle){
  target = absolute();
  int newTarget = isRight ? (target + angle): (target - angle); //was - +
  if(newTarget > 179){
    newTarget = -359+newTarget;
  }
  if(newTarget < -179){
    newTarget = 359+newTarget;
  }
  while(abs(absolute()-newTarget) > 2){
    isRight ? turnRight(MAX_SPEED) : turnLeft(MAX_SPEED);
  }
  hardStop();
}

void stopRobot(int delay_ms) {
  ledcWrite(RightFrontMotor_PWM_CHANNEL, 0);
  ledcWrite(LeftFrontMotor_PWM_CHANNEL, 0);
  ledcWrite(RightRearMotor_PWM_CHANNEL, 0);
  ledcWrite(LeftRearMotor_PWM_CHANNEL, 0); 
  delay(delay_ms);
isMoving = false;   
}// void stopRobot(int delay_ms)

void hardStop() {
  ledcWrite(RightFrontMotor_PWM_CHANNEL, 0);
  ledcWrite(LeftFrontMotor_PWM_CHANNEL, 0);
  ledcWrite(RightRearMotor_PWM_CHANNEL, 0);
  ledcWrite(LeftRearMotor_PWM_CHANNEL, 0);
  isMoving = false;
  switchVac(false);
}// void stopRobot()

void switchVac(bool isOn) {
  digitalWrite(RELAY_VAC, isOn ? HIGH : LOW);
}

/*++++++++++++SONAR FUNCTIONS++++++++++++++*/
//a is the position of the US
//0 - front
//1 - left
//2 - right
int pingcm(int a){
  unsigned long duration = 0;
  int trigPin = 12;
  int echoPin = 2;

  if(a == 2){
    trigPin = 15;
    echoPin = 13;
  }else if(a == 1){
    trigPin = 14;
    echoPin = 4;
  }  
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);  
    
  return ((duration /  2) / 29.1);
}

/*++++++++++++CALLBACKS++++++++++++++*/
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

bool setRouterDetails(AsyncWebServerRequest *request) {
  if (request->hasParam("ssid") && request->hasParam("password")) {
    routerSSID = request->getParam("ssid")->value();
    routerPASSWORD = request->getParam("password")->value();
    return true;
  }
  return false;
}

/*++++++++++++HELPER FUNCTIONS++++++++++++++*/
char *s2c(String comms) {
  if (comms.length() != 0) {
    char *p = const_cast<char *>(comms.c_str());
    return p;
  }
}

/*++++++++++++MPU FUNCTIONS++++++++++++++*/
void initMPU() {
  mpu.Initialize();
  mpu.Calibrate();

  mpu.Execute();
  target = (int)(mpu.GetAngZ() + 0.5);
}

void checkMotion() {
  mpu.Execute();
  newVal = (int)(mpu.GetAngZ() + 0.5);
}

void correctMotion(){
  checkMotion();
  //curspd = (int)(abs(newVal - target) * varspeed / (180 + abs(target)));        
  correctSpeed();
}

void correctSpeed(){
  int diff = abs(newVal-target);
  if(diff >= 300){
    diff -= 300;
  }else if(diff >= 30){
    diff = 30;
  }
  curspd = diff * 2;
}

double getMag(float x, float y) {
  return sqrt(sq(x) + sq(y));
}

int absolute() {
  mpu.Execute();
  return (int)(mpu.GetAngZ()+0.5);
}

int realAngle(){ //left wise  CCW
  int a = absolute();
  return a > 0 ? a : 360+a; 
}

/*++++++++++++LED FUNCTIONS++++++++++++++*/
// 000 - off
// 001 - red low battery
// 010 - green full battery
// 011 - yellow = rg charging
// 100 - blue cleaning
// 101 - magenta = rb mapping
// 110 - cyan =  bg spot cleaning
// 111 - white = rgb
void showLED(int a , int b, int c){
  pc1.write(REDLED, a == 1 ? HIGH : LOW);
  pc1.write(GREENLED, b == 1 ? HIGH : LOW);
  pc1.write(BLUELED, c == 1 ? HIGH : LOW);
}

/*++++++++++++INTERRUPT FUNCTIONS++++++++++++++*/

void setPinModes(){
  // pinMode(obsL_pin,INPUT_PULLUP);
  // pinMode(obsR_pin,INPUT_PULLUP);
  pinMode(home_pin,INPUT_PULLUP);
  pinMode(15,OUTPUT);
  pinMode(14,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,INPUT);
  pinMode(4,INPUT);
  pinMode(2,INPUT);
}

void setInterruptPins(){
  
// attachInterrupt(obsL_pin,hitObstacleL,FALLING);  
// attachInterrupt(obsR_pin,hitObstacleR,FALLING);

attachInterrupt(home_pin,homefunc,FALLING);
}

/*++++++++++++BATTERY MANAGEMENT FUNCTIONS++++++++++++++*/
void checkBattery(){
  double cBattery = rr12 * analogRead(35);
  batteryPercent = (batP * cBattery) - 250;
if(batteryPercent < 0){
batteryPercent = 0;  
}  
}

void batteryLow(){
  showLED(0,0,1);
  hasStarted = false;
  isMapping = false;
  isSpotting = false;
  //save position
}

void batteryFull(){
  showLED(0,1,0);
  //restore state and position
}

/*++++++++++++MAPPING FUNCTIONS++++++++++++++*/
// We assume a 400x400cm room, divided by 10 we have 40x40 blocks
// AVC is 30x30cm hence , it will occupy 3x3 blocks

void mapEdge() {
  if(isEdging){
    Serial.println("Looking for edge");
    chooseEdge();
  }else{
    mapRoom();
  }
}

void mapRoom() { 
  if(isMovingFB){
    if (isNotNear(0) ) {
      checkPosV2(true,false);
      correctSpeed();
      //curspd = (int)(abs(newVal - target) * varspeed / (180 + abs(target)));      
      movePos(0);
    }else{
      isMovingFB = false;
      ncmod = mapTurn ? cmod : 3-cmod;
      turnPos(ncmod);            
      target = absolute();
    }      
  }else{
    if (isNotNear(0)  ) {
      shouldEnd = 1;
      checkPosV2(false,false); 
      correctSpeed();            
      //curspd = (int)(abs(newVal - target) * varspeed / (180 + abs(target)));            
      movePos(0);
    }else{
      if(shouldEnd - 1 == 0){
        shouldEnd = 0;
        turnPos(ncmod);
        mapTurn = !mapTurn;  
        target = absolute();      
        isMovingFB = true;
               
      }else{
        //map has ended
        mapTurn = true;        
        isMovingFB = true;
                
        isMapping = false;        
        checkPosV2(false,true); 
      }      
    }           
  }
}

void checkPosV2(bool a, bool isEnded){
  sendClientMotionData(a,mapTurn,isEnded);
}

/*++++++++++++SPOT FUNCTIONS++++++++++++++*/
// We assume a 400x400cm room, divided by 10 we have 40x40 blocks
// AVC is 30x30cm hence , it will occupy 3x3 blocks

int timeTakenToMove(int b){
  return round(b / speedAVC);
}

void moveBlock(int b){
  movePos(0);
  delay(timeTakenToMove(b));    
}

void setupSpot(){
  if(spotState == 0){
    isAbove = fy > cy;
    isByRight = fx > cx;
    double angle = atan((fy-cy)/(fx-cx)) * 180 / PI;

    int nTarget = isAbove ? 90-angle : 90+angle;
    if(!isByRight){
      nTarget = isAbove ? -90-angle : -90+angle;        
    }

    autoTurn(nTarget);
    h = getMag((fx-cx),(fy-cy));
    nState = 0;
    inState = 0;
    spotState = 1;
  }else{
    if(h < pingcm(0)){
      moveBlock(h);
      hasLocSpot = true;
               
    }else{
      isAbove ? autoTurn(0) : autoTurn(179);
      if(abs(fy-cy)*10 > pingcm(isByRight ? 2 : 1)){ 
        movePos(0);
      }else{
        if(nState == 1){
          if(abs(abs(fx-cx)*10 - pingcm(0)) >= 1){ 
            movePos(0);
          }else{
            if(inState == 1){
              if(abs(abs(fy-cy)*10 - pingcm(0)) >= 1){ 
                movePos(0);
              }else{
                hasLocSpot = true;
                              
              }
            }else{
              isAbove ? autoTurn(0) : autoTurn(179);
              inState = 1;            
            }                        
          }
        }else{
          autoTurn(isByRight ? 90 : -90); 
          nState = 1;    
        }
      }             
    }  
  }
}

void spotClean(){
  switchVac(true);
   moveBlock(2); 
  turn90(true);
   moveBlock(2); 
  turn90(true);
   moveBlock(4); 
  turn90(true);
  moveBlock(4); 
  turn90(true);
  moveBlock(4); 
  turn90(true);
   moveBlock(2); 
  turn90(true);
   moveBlock(2); 
  switchVac(false);   
  checkPosV3(); 
}

void checkPosV3(){
  String spotMsg = "spot-finish";
  if(isClient > 0){
    ws.textAll(spotMsg);    
  }
}

/*++++++++++++CURRENT LOCATION FUNCTION++++++++++++++*/
void curUs(){
  target = absolute();
  int uf = pingcm(0)/10;
  delay(50);  
  int ul = pingcm(1)/10;
  delay(50);
  int ur = pingcm(2)/10; //in blocks

  int f,l,r,b;
    
  if(target > -2 && target < 2){ //target is at 0, normal pos
    f = uf;
    l = ul;
    r = ur;
    b = 0;
  }else if(target > 88 && target < 92){ //target is at 90, left
    f = ur;
    l = uf;
    r = 0;
    b = ul;
  }else if(target > -92 && target < -88){ //target is at -90, right
    f = ul;
    l = 0;
    r = uf;
    b = ur;
  }else if(target > 177 && target < -177){ //target is at 180, back
    f = 0;
    l = ur;
    r = ul;
    b = uf;
  }
  curPos(f,l,r,b); 
}

void curPos(int f, int l , int r, int b){
  if(target > -2 && target < 2){ //target is at 0, normal pos
    cy = f-b;
    if(l+r+3 >= 40){
      cx = l;
    }else{
      cx = 40 - r;                  
    }
  }else if(target > 88 && target < 92){ //target is at 90, left
    cx = l-r;
    if(f+b+3 >= 40){
      cy = f;
    }else{
      cy = 40 - b;                  
    }
  }else if(target > -92 && target < -88){ //target is at -90, right
    cx = r-l;
    if(f+b+3 >= 40){
      cy = f;
    }else{
      cy = 40 - b;                  
    }
  }else if(target > 177 && target < -177){ //target is at 180, back
    cy = b-f;
    if(l+r+3 >= 40){
      cx = l;
    }else{
      cx = 40 - r;                  
    }
  }
}

/*++++++++++++SEND DATA FUNCTION++++++++++++++*/
void sendClientMotionData(bool a, bool curState, bool isEnded){
  checkMotion();  
  int f = pingcm(0)/10;
  int l = pingcm(1)/10;
  int r = pingcm(2)/10; //in blocks

  if(f > 40 || l > 40 || r > 40 ){
    return;
  }
//front - left - right - back
  String smsg = String(f) + "%%" + String(l) + "%%" + String(r)  + "%%0";
  if(a){
    if(!curState){
      smsg = "0%%" + String(r) + "%%" + String(l) + "%%" + String(f);         
    }
  }else{
    smsg = String(r) + "%%" + String(f) + "%%0%%" + String(l);
    if(cmod == 2){
      smsg = String(l) + "%%0%%" + String(f) + "%%" + String(r);
    }
  }

  smsg += "%%" + String(newVal) + "%%" + String(isMoving);

  if(isEnded){
    smsg = "%%";
  }
  
  if(isClient > 0 && millis() > oldSendTime){
    ws.textAll(smsg);    
    oldSendTime = millis() + 100;
  }
}