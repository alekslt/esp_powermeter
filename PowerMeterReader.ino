/*
  Powermeter Impulse reader -> MQTT

  Aleksander Lygren Toppe
  
  This example code is in the public domain,
  but please send pull requests if you do something cool.
*/

////////////////////////////////
// Headers /////////////////////
////////////////////////////////
extern "C" {
#include "user_interface.h"
}
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

// Passwords and stuff
#include "secret.h"
////////////////////////////////
// Variables that you need to //
// change to make things work //
// for you. ////////////////////
////////////////////////////////

// Which digital port is the light sensor connected to?
const int ldrInPin = D1;

#define USE_ARDUINO_OTA 1
#define OTA_HOSTNAME "esp-powermeter"
#define OTA_PASSWORD CONFIG_OTA_PASSWORD

// WiFi
const char* ssid = CONFIG_WIFI_SSID;
const char* password = CONFIG_WIFI_PASSWORD;

// MQTT
#define MQTT_SERVER      "10.10.0.10"
#define MQTT_SERVERPORT  8883                   // 8883 for MQTTS
#define MQTT_USERNAME    ""
#define MQTT_KEY         ""
#define MQTT_MYNAME      "ESP8266 Client OTA"

#define power_topic "/kdg/sensors/powermeter"
#define power_command_topic "/kdg/sensors/powermeter/commands"

#define CHECK_FINGERPRINT 1
const char* fingerprint = "DD:3E:52:82:D6:56:D3:F6:AD:93:81:B9:7D:D8:30:AF:A3:07:31:96";

// Reporting
#define REPORTING_LIVEPERIOD 5000
#define REPORTING_MINUTEPERIOD 60000

#define REPORTING_RAMP_ACTIVATION_SPAN_MS 5000

////////////////////////////////
// Just nerdy code stuff below /
// Change at your own peril. ///
////////////////////////////////
// Structs /////////////////////
////////////////////////////////
struct reporting_s {
  unsigned long interval;
  unsigned long prevTimeReported;
  unsigned int impulses;
};

////////////////////////////////
// Forward declerations ////////
////////////////////////////////
void verifyFingerprint();
void ICACHE_RAM_ATTR rising1();
void ICACHE_RAM_ATTR falling1();
void callback(char* topic, byte* payload, unsigned int length);
char *f2s(double f, int p);

////////////////////////////////
// Macros //////////////////////
////////////////////////////////
#define MAX(a,b) (((a)>(b))?(a):(b))

////////////////////////////////
// Constants and Variables /////
////////////////////////////////
#define POWER_MIN_LIMIT 0xFFFFFF

// use volatile for shared variables between main code and interrupt handlers
volatile unsigned long currentPulseTime = 0;
volatile unsigned long prevTimePulse;
volatile unsigned long impulsesTotal = 0; 
volatile unsigned long loopCount = 0;
volatile unsigned long prevDelta_uc;
volatile double power = 0.0;
volatile double power_min = 0.0;
volatile double power_max = 0.0;
volatile bool flag_ldr_change = false;
volatile int flag_ldr_value = 0;

unsigned long now;

////////////////////////////////
// Complex Variables
////////////////////////////////
static struct reporting_s reportingLive {
  REPORTING_LIVEPERIOD, 0, 0
};
static struct reporting_s reportingAvg60 {
  REPORTING_MINUTEPERIOD, 0, 0
};

//WiFiClient espClient;
WiFiClientSecure espSecureClient;
PubSubClient mqtt_client(MQTT_SERVER, MQTT_SERVERPORT, callback, espSecureClient);

////////////////////////////////
// Buffers /////////////////////
////////////////////////////////
char outMsg5[512];
char outMsg60[512];

////////////////////////////////
// Inline Functions ////////////
////////////////////////////////
inline void resetPower() {
  power_min = POWER_MIN_LIMIT;
  power_max = 0;
}

////////////////////////////////
// MQTT Functions ///////////////
////////////////////////////////
void verifyFingerprint() {
  const char* host = MQTT_SERVER;

  Serial.print("Connecting to ");
  Serial.println(MQTT_SERVER);

  if (! espSecureClient.connect(host, MQTT_SERVERPORT)) {
    Serial.println("Connection failed. Halting execution.");
    while (1);
  }

  if (espSecureClient.verify(fingerprint, "localhost")) {
    Serial.println("Connection secure.");
  } else {
    Serial.println("Connection insecure! Halting execution.");
    while (1);
  }
  delay(1000);
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect(MQTT_MYNAME)) {
      Serial.println("connected");
      // ... and subscribe to topic
      mqtt_client.subscribe(power_command_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    char receivedChar = (char)payload[i];
    Serial.print(receivedChar);
    if (receivedChar == '0') {
    }
    if (receivedChar == '1') {
    }
  }
  Serial.println();
}

////////////////////////////////
// WiFi Functions ///////////////
////////////////////////////////
void setup_wifi() {
  // Wait for setup to settle a bit before connecting.
  delay(10);
  
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

////////////////////////////////
// Setup - Entry point /////////
////////////////////////////////
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  setup_wifi();
  resetPower();

  // mqtt_client.setServer(AIO_SERVER, AIO_SERVERPORT);
  // mqtt_client.setCallback(callback);

  // make the pushbutton's pin an input:
  pinMode(ldrInPin, INPUT);

  #ifdef USE_ARDUINO_OTA==1
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(OTA_HOSTNAME);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  #endif

  attachInterrupt(ldrInPin, falling1, FALLING);

  #ifdef CHECK_FINGERPRINT==1
  verifyFingerprint();
  #endif
}

////////////////////////////////
// Main Loop ///////////////////
////////////////////////////////
void loop() {
  #ifdef USE_ARDUINO_OTA==1
  ArduinoOTA.handle();
  #endif
  
  if (!mqtt_client.connected()) {
    reconnect();
  }
  mqtt_client.loop();
  // read the input pin:
  if (flag_ldr_change == true) {
    flag_ldr_change = false;
  }
  now = millis();

  if ((unsigned long)(now - reportingLive.prevTimeReported) >= reportingLive.interval) {
    double calcPbp = power_max;
    double calc_power_min = 0;
    
    // Check if we have reported and reset max/min
    if (power_min == POWER_MIN_LIMIT) {
      // Set to last known values
      calcPbp = power;
    } else {
      // Set to current values
      calc_power_min = power_min;
    }
    
    bool ramp = false;
    unsigned long timeSinceLastBlink = micros() - prevTimePulse;
    if ((timeSinceLastBlink/1000) > reportingLive.interval) {
      if ( (timeSinceLastBlink/1000) > REPORTING_RAMP_ACTIVATION_SPAN_MS && timeSinceLastBlink > prevDelta_uc ){
        calcPbp = 3600000000.0 / (timeSinceLastBlink);
        ramp = true;
      }
    }

    // sprintf(outMsg5, "tsb=%lu;timeSinceLastBlink=%lu;prevDelta_uc=%lu;ramp_on=%s;wsb=%lu;", (now / (unsigned long)1000), timeSinceLastBlink, prevDelta_uc, (ramp?"true":"false"), impulsesTotal);
    // mqtt_client.publish(power_topic, outMsg5, true);
    
    sprintf(outMsg5, "tsb=%lu;imps@05=%d;pbp=%s;pbp_min=%s;pbp_max=%s;pbp_calc=%s;wsb=%lu;", (now / (unsigned long)1000), reportingLive.impulses, f2s(power,1), f2s(calc_power_min,1), f2s(power_max,1), f2s(calcPbp,1), impulsesTotal);
    resetPower();
    
    reportingLive.impulses = 0;
    reportingLive.prevTimeReported = now;
    
    Serial.println(outMsg5);
    mqtt_client.publish(power_topic, outMsg5, true);
  }

  if ((unsigned long)(now - reportingAvg60.prevTimeReported) >= reportingAvg60.interval) {
    double usage = (double)reportingAvg60.impulses / ((double)reportingAvg60.interval / 3600000.0);
    sprintf(outMsg60, "tsb=%lu;imps@60=%d;power@60=%s;wsb=%lu;", (now / (unsigned long)1000), reportingAvg60.impulses, f2s(usage,1), impulsesTotal);

    reportingAvg60.impulses = 0;
    reportingAvg60.prevTimeReported = now;

    Serial.println(outMsg60);
    mqtt_client.publish(power_topic, outMsg60, true);
  }

  delay(1);        // delay in between reads for stability
}

////////////////////////////////
// Interrupt functions /////////
////////////////////////////////
void ICACHE_RAM_ATTR rising1()
{
  flag_ldr_change = true;
  currentPulseTime = micros();
  unsigned long current_delta = currentPulseTime - prevTimePulse;
 
  // "Debounce" the readings
  // TODO 20170201 ALT Verify that this time doesn't clip high usage times, e.g. above 10kW/h
  if (current_delta > 100000) {
    prevDelta_uc = current_delta;
    impulsesTotal++;
    reportingLive.impulses++;
    reportingAvg60.impulses++;

    power = 3600000000.0 / prevDelta_uc;
    if (power < power_min) {
      power_min = power;
    }
    if (power > power_max) {
      power_max = power;
    }
    prevTimePulse = currentPulseTime;
  }

  attachInterrupt(ldrInPin, falling1, FALLING);
  //Serial.println("Falling");
}

void ICACHE_RAM_ATTR falling1()
{
  attachInterrupt(ldrInPin, rising1, RISING);
  //Serial.println("Falling");
}


////////////////////////////////
// Utility functions //////////
////////////////////////////////
/* float to string
   f is the float to turn into a string
   p is the precision (number of decimals)
   return a string representation of the float.
*/
char *f2s(double f, int p) {
  char * pBuff;                         // use to remember which part of the buffer to use for dtostrf
  const int iSize = 10;                 // number of bufffers, one for each float before wrapping around
  static char sBuff[iSize][20];         // space for 20 characters including NULL terminator for each float
  static int iCount = 0;                // keep a tab of next place in sBuff to use
  pBuff = sBuff[iCount];                // use this buffer
  if (iCount >= iSize - 1) {            // check for wrap
    iCount = 0;                         // if wrapping start again and reset
  }
  else {
    iCount++;                           // advance the counter
  }
  return dtostrf(f, 0, p, pBuff);       // call the library function
}
