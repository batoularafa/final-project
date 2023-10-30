#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

const char* ssid = "omadodo";
const char* password = "1972omabato2012";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Serve a simple HTML page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", "<html><body><h1>ESP32 WebSocket Example</h1></body></html>");
  });

  // WebSocket event handler
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      Serial.println("Client connected");
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.println("Client disconnected");
    } else if (type == WS_EVT_DATA) {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (info->opcode == WS_TEXT) {
        String msg = String((char*)data);
        Serial.println("Received: " + msg);

        // Process the received message
        // For this example, we send back the same message
        client->text(msg);
      }
    }
  });

  server.addHandler(&ws);

  // Start the server
  server.begin();
}

void loop() {
  // Your loop code here
}
