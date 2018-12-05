#include <WiFi.h>

const char* ssid     = "diogo-positivo";     // your network SSID (name of wifi network)
const char* password = "7uOE8BnV"; // your network password

const char*  server = "10.42.0.241";  // Server URL
const uint16_t port = 23;
WiFiClient client;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  delay(100);

  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    // wait 1 second for re-trying
    delay(1000);
  }

  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.println("\nStarting connection to server...");
  
}

void loop() {
  uint8_t i = 0;
  char buff[20];
  if (!client.connect(server, port)){
    Serial.println("Connection failed!");
    delay(1000);
  }
  else {
    Serial.println("Connected to server!");
    client.write("BRA2018TESTE01");
    delay(1000);
  }
  while (client.available()) {
    if(i == 20){
       Serial.println(buff);
       i = 0;
     }
    char c = client.read();
    buff[i] = c;
    i++;
    //Serial.write(c);
  }
  Serial.println(buff);
  client.stop();
}
