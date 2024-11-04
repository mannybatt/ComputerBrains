



/**
 * 
 *  This is a piece of software designed to turn on a computer wirelessly!
 *  It uses MQTT to control and monitor the computer and a relay connected 
 *  to the pc are able to simulate a power button press. There is a small 
 *  amount of RGB control for various cosmetic components.
 * 
 */


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

//Initialize and Subscribe to MQTT
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe cb_PowerButton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ComputerBrains_PowerButton");
Adafruit_MQTT_Publish cb_PowerState = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/ComputerBrains_PowerState");

//Globals for Relays
#define power_relay D2
#define pc_power_state A0

//Globals for Light Effects
#include "FastLED.h"
#define NUM_LEDS 7
struct CRGB leds[2][NUM_LEDS];
#define PIN D8
#define PIN2 D1
#define UPDATES_PER_SECOND 100
CRGBPalette16 currentPalette = RainbowColors_p;
TBlendType currentBlending = LINEARBLEND;

//Globals for System
int pcPower = 0; //0=Off, 1=On
int lastPcPower = 0;
int rgbFlag = 0; //0=Off, 1=On




// ***************************************
// *************** Setup *****************
// ***************************************


void setup() {
  
  //Initialize Serial
  Serial.begin(115200);
  Serial.println("Booting");

  //WiFi Initialization
  wifiSetup();

  //Initialize MQTT
  mqtt.subscribe(&cb_PowerButton);
  MQTT_connect();

  //Initialize Relays
  delay(1000);
  pinMode(power_relay, OUTPUT);

  //Initialize RGB
  FastLED.addLeds<WS2811, PIN, GRB>(leds[0], NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2811, PIN2, GRB>(leds[1], NUM_LEDS).setCorrection(TypicalLEDStrip);
}




// ***************************************
// ************* Da Loop *****************
// ***************************************


void loop() {

  //OTA code
  ArduinoOTA.handle();

  //MQTT Code
  MQTT_connect();

  //State Manager
  //Listen for Button Press
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    Serial.println("Subscription Recieved");
    uint16_t value = atoi((char *)cb_PowerButton.lastread);
    delay(100);

    //Check for RGB Mode Changes
    if (value == 1 || value == 2) {
      digitalWrite(power_relay, HIGH);
      delay(400);
      digitalWrite(power_relay, LOW);
      delay(1100);
    }
    else if (value == 3) {
      value = 0;
      if (rgbFlag == 0) {
        rgbFlag = 1;
      }
      else if (rgbFlag == 1) {
        rgbFlag = 0;
      }
      Serial.println(rgbFlag);
    }
  }

  //Publish pc Power State
  if (analogRead(pc_power_state) > 500) {
    pcPower = 1;
    if (lastPcPower == 0) {
      rgbFlag = 1;
    }
  }
  else {
    pcPower = 0;
  }
  if (lastPcPower != pcPower) {
    cb_PowerState.publish(pcPower);
  }
  lastPcPower = pcPower;
  Serial.println(pcPower);

  //SSD RGB
  if (rgbFlag == 1) {
    static uint8_t startIndex = 0;
    static uint8_t startIndex2 = 100;
    startIndex = startIndex + 3; /* motion speed */
    startIndex2 = startIndex2 + 3; /* motion speed */
    FillLEDsFromPaletteColors( startIndex);
    FillLEDsFromPaletteColorsBackwords(startIndex2);
    if(pcPower == 0){
      FastLED.clear();
    }
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
  }
  else if (rgbFlag == 0) {
    FastLED.clear();
    FastLED.show();
  }
  if (pcPower == 0) {
    FastLED.clear();
  }
}




// ***************************************
// ********** Backbone Methods ***********
// ***************************************


void wifiSetup() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname("ComputerBrains");
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

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    //Serial.println("Connected");
    return;
  }
  Serial.print("Connecting to MQTT... ");
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
      Serial.println("Wait 10 min to reconnect");
      delay(600000);
    }
  }
  Serial.println("MQTT Connected!");
}

void FillLEDsFromPaletteColors( uint8_t colorIndex) {
  uint8_t brightness = 255;

  for ( int i = 0; i < NUM_LEDS; i++) {
    leds[0][i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
    colorIndex += 3;
  }
}

void FillLEDsFromPaletteColorsBackwords( uint8_t colorIndex) {
  uint8_t brightness = 255;

  for ( int i = NUM_LEDS / 2; i < NUM_LEDS; i++) {
    leds[1][i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
    colorIndex += 3;
  }
}
