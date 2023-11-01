#include <WiFi.h>
#include <WebSocketsServer.h>
// Replace constants
//Enter the network credentials
const char * ssid = "Afiz Makerspace EXT";  
const char * password = "afiz_makerspace.2022";
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
  Serial.begin(115200);     //Baud rate = 115200
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
}
void loop() {
  webSocket.loop();
}