#include <Arduino.h>
/**
 * This version is still in use @s4fh-heaterbox, 
 * requires mqttlogger for ADC scaling raw value to temperature
 * Follow on version including webserver: PlatformIO/Projects/esphive
 * 
 * Best Intro: file:///home/horst/Dokumente/elektronik/downloads/A-Beginner's-Guide-to-the-ESP8266.pdf
 * Simple wemos D1 mini  MQTT example
 * See https://makesmart.net/esp8266-d1-mini-mqtt/ for good MQTT introduction
 * API: https://pubsubclient.knolleary.net/api
 * See https://gist.github.com/boverby/d391b689ce787f1713d4a409fb43a0a4  for template of this sketch
 * This sketch demonstrates the capabilities of the pubsub library in combination with the ESP8266 board/library.
 
  - Wemos D1-Mini pins (https://www.wemos.cc/en/latest/d1/d1_mini.html)
    Pin  Function               ESP-8266 Pin
    TX    TXD                   TXD
    RX    RXD                   RXD
    A0  Analog input, max 3.2V  A0
    D0    IO                    GPIO16
    D1    IO, SCL               GPIO5
    D2    IO, SDA               GPIO4
    D3    IO, 10k Pull-up       GPIO0
    D4  IO, 10k Pull-up, BUILTIN_LED  GPIO2
    D5    IO, SCK               GPIO14
    D6    IO, MISO              GPIO12
    D7    IO, MOSI              GPIO13
    D8    IO, 10k Pull-down, SS GPIO15
    G     Ground                GND
    5V    5V                    -
    3V3   3.3V                  3.3V
    RST   Reset                 RST

  ======== OTA  see: https://arduino-esp8266.readthedocs.io/en/latest/ota_updates/readme.html
  Problem with naming resolution, OTA stub offers hostname but avahi fails to provide IP 
  Host esp-0c769f not found: 3(NXDOMAIN)
  Workaround: add currently used one for this MAC to /etc/hosts

  ======== ADS1115
  - Library: see elektronik/S4FH/ads1115-functions.pdf
  - Board: see elektronik/parts/ads1115.pdf
  
  ======== Extensions planned
  - Config file (see arduino example Lolin(Wemos)/ESP8266/ConfigFile including Json code too) to persist settings
  - S4FH shall Message.setRetained(true) for actor settings
  - Use LowPower features (sleep mode, see example), minor priority, power line allways available
  */

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h> // AVAHI for OTA 
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>     // for OTA
#include <ArduinoOTA.h>
#include <PubSubClient.h> // MQTT
#include <Wire.h>  // I2C 
#include <ADS1115_WE.h> 
#define I2C_ADDRESS 0x48

// GPIO16 is not defined ????
#define D0 16

// MQTT server (S4FH) 
const char* mqtt_server = "192.168.1.240";
String clientId;  // part of topic (and, with lowercase mac as OTA stub/host name) 

ESP8266WiFiMulti wifiMulti;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
ADS1115_WE adc(I2C_ADDRESS);
boolean hasadc = false;     // adc is optional

long msDelta = 60 * 1000;   // publish INFO message every 60 sec

long lastMsg = 0;           // timestamp of last publish

//                Relais             1   2   3   4
//                GPIO      2       14  12  13  15      (all D5-D8 might be used for SPI, not used for that)
unsigned int pins[] = {LED_BUILTIN, D5, D6, D7, D8};

ADS1115_MUX channels[] = {ADS1115_COMP_0_GND, ADS1115_COMP_1_GND, ADS1115_COMP_2_GND, ADS1115_COMP_3_GND};

float getAdcValue(int cnr) {
  adc.setCompareChannels(channels[cnr]);
  adc.startSingleMeasurement();
  while(adc.isBusy())
    yield();    // https://forum.arduino.cc/index.php?topic=586711.0
  return adc.getResult_mV(); // alternative: getResult_V for Volt
}


void setup_gpio() {
  Serial.println();
  Serial.println(" ! Setup");
  Serial.print(" ! GPIO Pins: ");
  for (int i = 0; i < 5; i=i + 1) {  
    pinMode(pins[i], OUTPUT);
    Serial.print(" ");
    Serial.print(pins[i]);
    // initial setting
    //if (i > 0)
    //  digitalWrite(pins[i], HIGH);   // initial state of led is on, others are off 
  }
  Serial.println();
}

void setup_i2c() {
  Serial.println(" ! I2C Wire");
  Wire.begin();
  Serial.print(" ! trying ADS1115 ... ");
  hasadc = adc.init();
  if(hasadc){
    adc.setVoltageRange_mV(ADS1115_RANGE_6144);  // gain, +/- 6144 mV
    adc.setConvRate(ADS1115_8_SPS);  // 8 samples / sec
    adc.setMeasureMode(ADS1115_SINGLE);  
    Serial.println(" initialized (single mode)");
  } else {
    Serial.println(" not available!");
  }
}

/**
 * create a unique clientid  (chipid = last 6 hex digits of 12-digit MAC)
 * similar to type-macpart with type lowercase (delock|tasmota...)
 * OTA default hostname is similar but all lowercase
 */
String macToStr(const uint8_t* mac) {
  String result;
  for (int i = 3; i < 6; ++i) {
    String hx = String(mac[i], 16);
    if (hx.length() < 2)
      result += "0";
    result += hx;
  }
  result.toUpperCase();
  return result;
}

String composeClientID() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String clientId;
  clientId += "esp_";
  clientId += macToStr(mac);
  return clientId;
}

void setup_wifi() {
  Serial.print(" ! Setup Wifi");
  wifiMulti.addAP("MyHomeUG", "TJSXAWFNAAWKKYIH");
  wifiMulti.addAP("MyHomeOG", "TJSXAWFNAAWKKYIH");
  Serial.print("  ");
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print(" connected, ");
  Serial.print("AP:");
  Serial.print(WiFi.SSID());
  Serial.print(" IP:");
  Serial.println(WiFi.localIP());
  clientId = composeClientID();   // used as part of topic and (lowercase) as OTA hostname
}

/**
 * To check logging use espscreen (sudo screen /dev/ttyUSB0 115200,cs8)
 * to quit screen: ^a\
 */
void setup_ota() {
  Serial.print(" ! Setup OTA as ");

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  String cid = clientId;
  cid.toLowerCase();
  ArduinoOTA.setHostname(cid.c_str());
  Serial.println(cid);
  
  // No authentication by default
  ArduinoOTA.setPassword((const char *)"ESP");

  // callbacks:
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    Serial.println("   Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
   Serial.println("   End, resetting");
   ESP.reset();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("   Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("  Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println(" ! OTA ready");
}
   
/**
 * MQTT message handling
 */
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("=> ");
  Serial.print(topic);
  Serial.print(" ");
  String dest = String(topic);
  if (dest.startsWith("cmnd/")) {
    String target = dest.substring(dest.lastIndexOf("/") + 1);
    if (target == "DOUT") {
      boolean changed = false;
      String reply = ""; 
      for (unsigned int i = 0; i < 5; i++) {
        int vo = digitalRead(pins[i]);
        if (i < length) {
          char vc = (char)payload[i];  // might be "-"
          Serial.print(vc);
          // set to low or high or leave unchanged
          if (vc == 48  || vc == 49) {
            int vi = vc - 48;
            if (vo != vi) {
              changed = true;
              lastMsg = 0;  // send INFO without delay
            }
            digitalWrite(pins[i], vi);   // Turn the LED on when 0 (open collector logic)
            vo = vi;
          }
        }
        reply += vo;
      }
      Serial.println(); 

      if (length == 0 || changed) {
        Serial.print("<= ");
        dest = "stat" + dest.substring(dest.indexOf("/"));
        Serial.print(dest);
        Serial.print(" ");
        Serial.println(reply);
        mqttClient.publish(dest.c_str(), reply.c_str(), true);
      }
    } else if (target.startsWith("ADC")) {
      Serial.println();
      Serial.print("<= ");
      dest = "stat" + dest.substring(dest.indexOf("/"));
      Serial.print(dest);
      Serial.print(" ");
      String reply;
      if (hasadc) {
        int cnr = target.charAt(3) - 49;   // 0..3
        reply = String(getAdcValue(cnr), 2);
      } else {
        reply = "NaN";
      }
      Serial.print(reply); 
      mqttClient.publish(dest.c_str(), reply.c_str(), true);
    } else if (target == "RST") {
      mqttClient.disconnect();
      Serial.println();
      Serial.println(" ! Restarting");
      //TODO: keep current setting (DIO states) in SPDIFF FS       
      ESP.restart();
    } else {
      Serial.print(" invalid");
    }
    Serial.println();
  }
}


/**
 * ensure MQTT connection
 */
void connect_mqtt() {
  // Loop until we're (re)connected
  while (!mqttClient.connected()) {
    Serial.print(" ! MQTT connection...");
    // Attempt to connect (without randomization as clientId is unique)
    //cid += String(micros() & 0xff, 16); // to randomise. sort of
    if (mqttClient.connect(clientId.c_str())) {
      Serial.print("... connected as ");
      Serial.println(clientId);  // print accepts String objects
      // Once connected resubscribe to receive commands
      String subscription = "cmnd/" + clientId + "/#";
      mqttClient.subscribe(subscription.c_str() );
      Serial.print(" ^ ");
      Serial.println(subscription);
      String topic = "tele/" + clientId + "/INFO";
      // announce availibility (to dedect reboots too)
      String payload = "{\"REVISION\":\"esp-d1-1\"}";
      mqttClient.publish(topic.c_str(), payload.c_str(), false );

      Serial.print("<= ");
      Serial.println(topic + " " + payload); 
    } else {
      Serial.print("... failed, rc=");
      Serial.print(mqttClient.state());
      Serial.print(" wifi=");
      Serial.print(WiFi.status());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/**
 * Main
 */
void setup() {
  Serial.begin(115200);
  delay(1000);
  setup_gpio();
  delay(100);
  setup_i2c();
  delay(100);
  setup_wifi();
  delay(100);
  setup_ota(); // OTA fails with ERROR: No Answer  Reason: device not running, reset!
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);
}

void loop() {
  ArduinoOTA.handle();  // offer updating using OTA
  connect_mqtt();       // (re-)establish connection
  mqttClient.loop();    // handle e.g. incomming messages (subscriptions)
  
  // periodic measure and publish
  long now = millis();    // Attention: wraps (as micros() does)
  if ((now - lastMsg > msDelta ) || (now < lastMsg) || lastMsg == 0) {
    lastMsg = now;
    String payload = "{\"DIO\":\"";   // Json formatted 
    for (int di = 0; di < 5; di++)
      payload += digitalRead(pins[di]);
    payload += "\", \"A0\":";
    payload += analogRead(A0);;
    if (hasadc) {
      for (int ci = 0; ci < 4; ci++) {
        float v = getAdcValue(ci);
        // ADS1115 fails often when 2'nd relais opens, adding capacitor to didnt help 
        if (v < 100.0 || v > 4000.0) {
          Serial.println(" ! ADC overflow, restarting");
          mqttClient.disconnect();
          ESP.restart();
        }
        payload += ", \"ADC";
        payload += (ci + 1); 
        payload += "\":";
        payload += v;  // no unit (mV)
      }
    }
    payload += "}";
    String topic;
    topic += "tele/";
    topic += composeClientID();
    topic += "/INFO";
    Serial.print("<= ");
    Serial.print(topic);
    Serial.print(" ");
    Serial.println(payload);
    mqttClient.publish( (char*)topic.c_str() , (char*)payload.c_str(), true );
  }
}
