#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Change the credentials below, so your ESP8266 connects to your router
const char* ssid = "your ssid";
const char* password = "your password";


// Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
const char* mqtt_server = "pi's ip address";

// Initializes the espClient
WiFiClient espClient;
PubSubClient client(espClient);


// Don't change the function below. This functions connects your ESP8266 to your router
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

// This functions is executed when some device publishes a message to a topic that your ESP8266 is subscribed to
// Change the function below to add logic to your program, so when a device publishes a message to a topic that 
// your ESP8266 is subscribed you can actually do something
void callback(String topic, byte* message, unsigned int length) {
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

  if(topic=="lights"){
      if(messageTemp == "on"){
        Serial.println("[on]");
      }
      else if(messageTemp == "off"){
        Serial.println("[off]");
      }
      else if(messageTemp == "on1"){
        Serial.println("[on1]");
      }
      else if(messageTemp == "off1"){
        Serial.println("[off1]");
      }
      else if(messageTemp == "on2"){
        Serial.println("[on2]");
      }
      else if(messageTemp == "off2"){
        Serial.println("[off2]");
      }
  }
  if(topic=="fans"){
      if(messageTemp == "on3"){
        Serial.println("[on3]");
      }
      else if(messageTemp == "off3"){
        Serial.println("[off3]");
      }
      else if(messageTemp == "on4"){
        Serial.println("[on4]");
      }
      else if(messageTemp == "off4"){
        Serial.println("[off4]");
      }
  }
}

// This functions reconnects your ESP8266 to your MQTT broker
// Change the function below if you want to subscribe to more topics with your ESP8266 
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect

    if (client.connect("ESP8266Client")) {
      Serial.println("connected");  
      // Subscribe or resubscribe to a topic
      // You can subscribe to more topics (to control more LEDs in this example)
      client.subscribe("lights");
      client.subscribe("fans");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
   
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  if(!client.loop())
    client.connect("ESP8266Client");
}
