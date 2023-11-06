
#include <WiFi.h>
#include <WebSocketsServer.h>
//#include <ESP32Servo.h>

//void movingForward();   void movingBackward();    void movingRight();       void movingLeft();

// Motor pins
const int leftFront = 19;
const int leftBack = 18;
const int rightFront = 5;
const int rightBack = 17;

const int enableLeft = 23;
int channel1 = 0;

const int enableRight = 16;
int channel2 = 1;

int freq = 1000;
int Res = 8;

/*Servo gripper;
Servo lift;
int pos0 = 0;
int pos1 = 0;
*/
// Encoder pins
const int encoderPinA = 34;
const int encoderPinB = 35;
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

float kp = 2.0;
float kd = 2.0;
float ki = 2.0;

//network credentials
const char * ssid = "gado";  
const char * password = "21010716*22";
String text;
//Global varialbe defined with port 80
WebSocketsServer webSocket= WebSocketsServer(80);
// Called when websocket server receives any messages
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length){
      //Figure out the type of Websocket Event
      switch(type) {
          // Client has disconnected
          case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected\n", num);
            break;
         
         //New client has connected to the server
         case WStype_CONNECTED:
          {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from: \n", num);
            Serial.println(ip.toString());
          }
          break;
          //Echo the text messages
          case WStype_TEXT:
           //Serial.printf("[%u] Text %s\n", num, payload);
            text = String((char*)payload);
            Serial.print("this is the recieved text: ");
            Serial.println(text);
            webSocket.sendTXT(num, payload);
            if(text == "W"){
            movingForward();
            
            }
            else if(text == "S"){
            movingBackward();
            
            }
            else if(text == "D"){
            movingRight();
            
            }
            else if(text == "A"){
            movingLeft();
            
            }
           /* else if(text == "G"){
            gripping();
            }
            else if(text == "R"){
            releasing();
            }
            else if(text == "L"){
            lifting();
            }
            else if(text == "P"){
            putting();
            }*/
            else{
            noMovement();
            }
           
            
            break;
         // For anything else: do nothing
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

void setup() {
  Serial.begin(115200);    
  Serial.println("Connecting");
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
      delay(500);
      Serial.print(".");
    }
  //print IP Address
  Serial.println("Connected");
  Serial.print("My IP Address: ");
  Serial.println(WiFi.localIP());

  //start Websocket Server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  
  pinMode(leftFront, OUTPUT);
  pinMode(leftBack, OUTPUT);
  //pinMode(enableLeft, OUTPUT);
  pinMode(rightFront, OUTPUT); 
  pinMode(rightBack, OUTPUT);
  //pinMode(enableRight, OUTPUT);


  ledcSetup(channel1, freq, Res);
  ledcSetup(channel2, freq, Res);
  ledcAttachPin(enableLeft, channel1);
  ledcAttachPin(enableRight, channel2);

  /*gripper.attach(32);
  lift.attach(33);*/

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), handleEncoder2, RISING);

}
void loop() {
  int target = encoder1Count;
  float u = pidController(target, kp, kd, ki);
  webSocket.loop();
}


void handleEncoder1() {
  encoder1Count++;
}

void handleEncoder2() {
  encoder2Count++;
}

// PID controller
float pidController(int target, float kp, float kd, float ki) {
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

  int e = encoder2Count - target;                    //
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  float u = (kp * e) + (ki * eIntegral) + (kd * eDerivative);

  previousTime = currentTime;
  ePrevious = e;

  return u;
}

// Motor control functions
void moveLeftMotor(int frontPin, int backPin, float u) {
  float speed = fabs(u);
  if (speed > 255) {
    speed = 255;
  }

  int direction = (u > 0) ? HIGH : LOW;

  digitalWrite(frontPin, direction);
  digitalWrite(backPin, !direction);
  ledcWrite(enableLeft, speed);
}

void moveRightMotor(int frontPin, int backPin, float u) {
  float speed = fabs(u);
  if (speed > 255) {
    speed = 255;
  }

  int direction = (u > 0) ? HIGH : LOW;

  digitalWrite(frontPin, direction);
  digitalWrite(backPin, !direction);
  ledcWrite(enableRight, speed);
}


// Robot motion control
void movingForward() {
  int target = encoder1Count;
  float u = pidController(target, kp, kd, ki);

  moveLeftMotor(leftFront, leftBack, u);
  moveRightMotor(rightFront, rightBack, u);
}

void movingBackward() {
  int target = encoder1Count;
  float u = pidController(target, kp, kd, ki);

  moveLeftMotor(leftFront, leftBack, -u);
  moveRightMotor(rightFront, rightBack, -u);
}

void movingLeft() {
  int target = encoder1Count;
  float u = pidController(target, kp, kd, ki);

  moveLeftMotor(leftFront, leftBack, -u);
  moveRightMotor(rightFront, rightBack, u);
}

void movingRight() {
  int target = encoder1Count;
  float u = pidController(target, kp, kd, ki);

  moveLeftMotor(leftFront, leftBack, u);
  moveRightMotor(rightFront, rightBack, -u);
}

void noMovement() {
  digitalWrite(leftFront, LOW);
  digitalWrite(leftBack, LOW);
  digitalWrite(rightFront, LOW);
  digitalWrite(rightBack, LOW);
}
/*
void gripping() {
  pos0 = pos0 + 5;
  if (pos0 >= 180) {
    pos0 = 180;
  }
}

void releasing() {
  pos0 = pos0 - 5;
  if (pos0 <= 0) {
    pos0 = 0;
  }
}

void lifting() {
  pos1 = pos1 + 5;
  if (pos1 >= 180) {
    pos1 = 180;
  }
}

void putting() {
  pos0 = pos0 - 5;
  if (pos0 <= 0) {
    pos0 = 0;
  }
}*/
