



// ***************************************
// ********** Global Variables ***********
// ***************************************


//Globals for Wifi Setup and OTA
#include <credentials.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//WiFi Credentials
#ifndef STASSID
#define STASSID "your_ssid"
#endif
#ifndef STAPSK
#define STAPSK  "your_password"
#endif
const char* ssid = STASSID;
const char* password = STAPSK;

//MQTT
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#ifndef AIO_SERVER
#define AIO_SERVER      "your_MQTT_server_address"
#endif
#ifndef AIO_SERVERPORT
#define AIO_SERVERPORT  0000 //Your MQTT port
#endif
#ifndef AIO_USERNAME
#define AIO_USERNAME    "your_MQTT_username"
#endif
#ifndef AIO_KEY
#define AIO_KEY         "your_MQTT_key"
#endif
#define MQTT_KEEP_ALIVE 150
unsigned long previousTime;
float mqttConnectFlag = 0.0;

//Initialize and Subscribe to MQTT
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish cb_Power = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/ComputerBrains_PowerButton");
Adafruit_MQTT_Subscribe cb_PowerState = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ComputerBrains_PowerState");

//FastLED
#include "FastLED.h"
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
#define PIN D8
#define UPDATES_PER_SECOND 120
TBlendType currentBlending = LINEARBLEND;

//Input/Output
#define power_button D5

//Variables
int pcPower = 0; //0=Off, 1=On
int lastPcPower = 0;
int rgbFlag = 0; //0=Off, 1=On
int toSend = 0;
unsigned long previousMillis = 0;
unsigned long currentMillis;




// ***************************************
// *************** Setup *****************
// ***************************************


void setup() {

  //Initialize RGB
  FastLED.addLeds<WS2812, PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(200);
  
  //Start Pixels (2 white blinks)
  delay(3000);
  setPixel(0, 255, 255, 255);
  FastLED.show();
  delay(1000);
  setPixel(0, 0, 0, 0);
  FastLED.show();
  delay(1000);
  setPixel(0, 255, 255, 255);
  FastLED.show();
  delay(1000);
  setPixel(0, 0, 0, 0);
  FastLED.show();

  //Initialize Butons
  delay(10);
  pinMode(power_button, INPUT_PULLUP);

  //Initialize Serial, WiFi, & OTA
  wifiSetup();

  //Initialize MQTT
  mqtt.subscribe(&cb_PowerState);  
}




// ***************************************
// ************* Da Loop *****************
// ***************************************


void loop() {
  
  //OTA & MQTT
  ArduinoOTA.handle();
  MQTT_connect();

  //Button Checker
  if (digitalRead(power_button) == LOW) {
    delay(10);
    currentMillis = millis();
    previousMillis = millis();
    while(digitalRead(power_button) == LOW){
      currentMillis = millis();
      if(currentMillis - previousMillis > 1000) {           // Normal Press for Toggling pc power button
        toSend = 100;
        setPixel(0, 236, 247, 30);
        FastLED.delay(1000 / UPDATES_PER_SECOND);
        FastLED.show();
      }
      else if(currentMillis - previousMillis > 3500) {     // Hard Reset Press 
        toSend = 101;
        setPixel(0, 255, 0, 0);
        FastLED.delay(1000 / UPDATES_PER_SECOND);
        FastLED.show();
      }
      delay(100);
    }
    delay(100);
    Serial.println(toSend);
    cb_Power.publish(toSend);
    toSend = 0;
  }

  //Check for PC Power State to reflect button RGB
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(0.01))) {
    uint16_t value = atoi((char *)cb_PowerState.lastread);
    pcPower = value;
    Serial.print("pcPower = ");
    Serial.print(pcPower);
    delay(500);
  }

  //Change RGB Flag to reflect change in state
  if(lastPcPower == 0 && pcPower == 1){ //ON
    setPixel(0, 146, 52, 235);
    FastLED.delay(1000 / UPDATES_PER_SECOND);
    FastLED.show();
  }
  else if(lastPcPower == 1 && pcPower == 0){ //OFF
    setPixel(0, 0, 0, 0);
    FastLED.delay(1000 / UPDATES_PER_SECOND);
    FastLED.show();
  }

  //Ping Timer
  unsigned long currentTime = millis();
  if ((currentTime - previousTime) > MQTT_KEEP_ALIVE * 1000) {
    previousTime = currentTime;
    if (! mqtt.ping()) {
      mqtt.disconnect();
    }
  }

  //Update Last PC Power
  lastPcPower = pcPower;
  delay(10);
}




// ***************************************
// ********** Backbone Methods ***********
// ***************************************




void setPixel(int Pixel, byte red, byte green, byte blue) {
  leds[Pixel].r = red;
  leds[Pixel].g = green;
  leds[Pixel].b = blue;
}

void MQTT_connect() {

  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    if (mqttConnectFlag == 0) {
      //Serial.println("Connected");
      mqttConnectFlag++;
    }
    return;
  }
  Serial.println("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      //while (1);
      Serial.println("Wait 5 secomds to reconnect");
      delay(5000);
    }
  }
}

void wifiSetup() {

  //Serial
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println();
  Serial.println("****************************************");
  Serial.println("Booting");

  //WiFi and OTA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname("SuperDoorbell");                                        
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
