/*
 * ArduinoChargeController
 * 
 * Watch voltage on a battery, and when it gets too low,
 * power on and connect to a wall power supply
 * (one relay for the supply's (presumably 120VAC) input power, one for its 12-14V output)
 * 
 * Designed to run on an ESP8266 with one of the popular '4 Relay Modules'
 * available circa 2017 for ~8$ each.
 * The one I'm using seems to work just fine with ESP8266 voltage levels
 * (3.3V)
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "config.h"

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

#define D0   16
#define D1    5
#define D2    4
#define D3    0
#define D4    2
#define D5   14
#define D6   12
#define D7   13
#define D8   15
#define D9    3
#define D10   1

#define RELAY1 D7
#define RELAY2 D3
#define RELAY3 D2
#define RELAY4 D1

// Wall power to the regulator
#define REGULATOR12_INPUT_RELAY RELAY1
// 12-14V output from the regulator
#define REGULATOR12_OUTPUT_RELAY RELAY2

#define messageBufferSize 96

char hexDigit(int num) {
  num = num & 0xF;
  if( num < 10 ) return '0'+num;
  if( num < 16 ) return 'A'+num;
  return '?'; // Should be unpossible
}

byte macAddressBuffer[6];

char messageBuffer[messageBufferSize];
/** Used by the message_ building functions */
int messageBufferOffset;

void message_clear() {
  messageBufferOffset = 0;
}
void message_appendChar(char c) {
  if( messageBufferOffset < messageBufferSize-1 ) {
    messageBuffer[messageBufferOffset++] = c;
  }
}
void message_appendString(const char *str) {
  while( *str != 0 && messageBufferOffset < messageBufferSize-1 ) {
    messageBuffer[messageBufferOffset] = *str;
    ++str;
    ++messageBufferOffset;
  }
}
void message_separate(const char *separator) {
  if( messageBufferOffset == 0 ) return;
  message_appendString(separator);
}
void message_appendLabel(const char *label) {
  message_separate(" ");
  message_appendString(label);
  message_appendString(":");
}
void message_appendMacAddressHex(byte *macAddress, const char *octetSeparator) {
  for( int i=0; i<6; ++i ) {
    if( i > 0 ) message_appendString(octetSeparator);
    message_appendChar(hexDigit(macAddress[i]>>4));
    message_appendChar(hexDigit(macAddress[i]));
  }
}
void message_appendLong(long v) {
  if( v < 0 ) {
    message_appendChar('+');
    v = -v;
  }
  int printed = snprintf(messageBuffer+messageBufferOffset, messageBufferSize-messageBufferOffset, "%ld", v);
  if( printed > 0 ) messageBufferOffset += printed;
}
void message_appendFloat(float v) {
  if( v < 0 ) {
    message_appendChar('-');
    v = -v;
  }
  int hundredths = (v * 100) - ((int)v) * 100;
  int printed = snprintf(messageBuffer+messageBufferOffset, messageBufferSize-messageBufferOffset, "%d.%02d", (int)v, hundredths);
  if( printed > 0 ) messageBufferOffset += printed;
}
void message_close() {
  if( messageBufferOffset > messageBufferSize-1 ) {
    messageBufferOffset = messageBufferSize-1;
  }
  messageBuffer[messageBufferOffset++] = 0;
}


WiFiClient espClient;
PubSubClient pubSubClient(espClient);

long tickStartTime = -1;

void reportWiFiStatus(int wiFiStatus) {
  Serial.print("# WiFi status: ");
  switch( wiFiStatus ) {
  case WL_CONNECTED:
    Serial.println("WL_CONNECTED");
    Serial.print("# IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("# MAC address: ");
    WiFi.macAddress(macAddressBuffer);
    message_clear();
    message_appendMacAddressHex(macAddressBuffer, ":");
    message_close();
    Serial.println(messageBuffer);
    break;
  case WL_NO_SHIELD:
    Serial.println("WL_NO_SHIELD");
    break;
  case WL_IDLE_STATUS:
    Serial.println("WL_IDLE_STATUS");
    break;
  case WL_NO_SSID_AVAIL:
    Serial.println("WL_NO_SSID_AVAIL");
    break;
  case WL_SCAN_COMPLETED:
    Serial.println("WL_SCAN_COMPLETED");
    break;
  case WL_CONNECT_FAILED:
    Serial.println("WL_CONNECT_FAILED");
    break;
  case WL_CONNECTION_LOST:
    Serial.println("WL_CONNECTION_LOST");
    break;
  case WL_DISCONNECTED:
    Serial.println("WL_DISCONNECTED");
    break;
  default:
    Serial.println(wiFiStatus);
  }
}

long lastWiFiConnectAttempt = -10000;
int previousWiFiStatus = -1;

int maintainWiFiConnection() {
  int wiFiStatus = WiFi.status();
  if( wiFiStatus != previousWiFiStatus ) {
    reportWiFiStatus(wiFiStatus);
    previousWiFiStatus = wiFiStatus;
  }    
  if( wiFiStatus != WL_CONNECTED && tickStartTime - lastWiFiConnectAttempt >= 10000 ) {
    Serial.print("# Attempting to connect to ");
    Serial.print(WIFI_SSID);
    Serial.println("...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWiFiConnectAttempt = tickStartTime;
  }
  return wiFiStatus;
}

long lastMqttConnectAttempt = -10000;

bool maintainMqttConnection() {
  if( pubSubClient.connected() ) return true;
  
  if( tickStartTime - lastMqttConnectAttempt < 10000 ) return false;
  
  message_clear();
  message_appendMacAddressHex(macAddressBuffer, ":");
  message_close();
  if( pubSubClient.connect(messageBuffer) ) {
    Serial.print("# Connected to MQTT server: ");
    Serial.print(MQTT_SERVER);
    Serial.print(":");
    Serial.println(MQTT_PORT);
    message_clear();
    message_appendString("# Hi, I'm ");
    const char *macAddressString = messageBuffer+messageBufferOffset;
    message_appendMacAddressHex(macAddressBuffer, ":");
    message_close();
    pubSubClient.publish(MQTT_TOPIC, messageBuffer);
    return true;
  } else {
    Serial.print("# Failed to connect to MQTT server: ");
    Serial.print(MQTT_SERVER);
    Serial.print(":");
    Serial.println(MQTT_PORT);
    return false;
  }
}

/*
 * Battery charging:
 * 
 * Vmin = voltage below which we want to connect to wall charger -- maybe 11V
 * Vmax = disconnect the wall charger if voltage goes above this -- maybe 14V
 * maxDischargeRate = estimated maximum rate of discharge (Δvoltage/Δtime) -- let's say 1V/minute, which is 1/60000 V/millisecond
 * maxSleepTime = maximum length of time we want to sleep -- maybe 5 minutes
 * 
 * If voltage is < Vmin, connect!  Start connection timer.
 * If connection timer has expired, disconnect.
 * If connected and voltage > Vmax, disconnect.
 * If disconnected and voltage is > minV, and we've been awake for long enough to have reported to MQTT server, sleep for min(maxSleepTime, (V-Vmin)/maxDischargeRate)
 */

const long minRegulator12OutputMillis = 10*60*1000;
const long minRegulator12InputMillis  = 11*60*1000;
const long minMqttLogInterval = 60*1000;
float maxDischargeRate = 1/60000.0;
long maxSleepTime = 10*60*1000;

long regulator12StartTime = -1;
long lastMqttReportTime = -1;
long lastSerialReportTime = -1;

float vMin = 11;
float vMax = 14;

bool regulator12InputOn = false, regulator12OutputOn = false;

#ifdef WIFI_DEBUGGING
void setup() {
  Serial.begin(115200);
  Serial.println("# Let's see if we can get stupid WiFi to work.");
}

void loop() {
  if( maintainWiFiConnection() == WL_CONNECTED ) {
    maintainMqttConnection();
  }
  Serial.print(".");
  delay(500);
}
#else
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(RELAY1, OUTPUT); // Relay 4
  pinMode(RELAY2, OUTPUT); // Relay 3
  pinMode(RELAY3, OUTPUT); // Relay 2
  pinMode(RELAY4, OUTPUT); // Relay 1
  Serial.begin(115200);
  Serial.println("# Hello from ArduinoChargeController!");
  Serial.print("# LED_BUILTIN pin = "); Serial.println(LED_BUILTIN);
  Serial.print("# RELAY1 pin = "); Serial.println(RELAY1);
  Serial.print("# RELAY2 pin = "); Serial.println(RELAY2);
  Serial.print("# RELAY3 pin = "); Serial.println(RELAY3);
  Serial.print("# RELAY4 pin = "); Serial.println(RELAY4);
  pubSubClient.setServer(MQTT_SERVER, MQTT_PORT);
  // Turn all relays to their default position
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);
  digitalWrite(RELAY3, HIGH);
  digitalWrite(RELAY4, HIGH);
}

int tick = 0;
int consecutiveBoringTicks = 0;

struct LogState {
  int a0Val;
  bool regulator12InputOn;
  bool regulator12OutputOn;
  bool mqttConnected;
  long timestamp;
};

struct LogState prevState = {
  a0Val: 0,
  regulator12InputOn: false,
  regulator12OutputOn: false,
  mqttConnected: false,
  timestamp: 0
};
struct LogState lastMqttLogState = prevState;

void setRelayPinIfChanged(int pin, bool &current, bool requested) {
  if( current != requested ) {
    digitalWrite(pin , requested ? LOW : HIGH);
    current = requested;
  }
}

int voltageToA0Val(float v) {
  return 3 + (v * 14.5f);
}
float a0ValToVoltage(int a0Val) {
  return (a0Val - 3) / 14.5f;
}

int absDiff(int a, int b) {
  return abs(a - b);
}

#ifdef SIMULATE_A0VAL
int prevA0Val = 0;
#endif

void loop() {
  struct LogState newState;
  newState.timestamp = tickStartTime = millis();
  
  // Blink the LED *.*.*... so we can tell it's working
  switch( ((tickStartTime >> 8) % 8) ) {
  case 0: case 2: case 4:
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level
    break;
  default:
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    break;
  }
  
  #ifdef SIMULATE_A0VAL
  newState.a0Val = prevA0Val;
  if( regulator12InputOn && regulator12OutputOn ) {
    newState.a0Val = voltageToA0Val(13);
  } else {
    newState.a0Val = std::min(newState.a0Val, int(3 + (12 * 14.5f)));
    if( random(0,50) == 0 ) {
      --newState.a0Val;
    }
  }
  prevA0Val = newState.a0Val;
  newState.a0Val += random(0,1); // simulated noise
  #else
  newState.a0Val = analogRead(A0);
  #endif

  float a0Voltage = a0ValToVoltage(newState.a0Val);
  
  {
    bool regulator12InputShouldBeOn, regulator12OutputShouldBeOn;
    if( a0Voltage < vMin ) {
      // Defs need that power!
      regulator12StartTime = tickStartTime;
      regulator12InputShouldBeOn = regulator12OutputShouldBeOn = true;
    } else {
      regulator12InputShouldBeOn  =                        (regulator12StartTime != -1) && (tickStartTime - regulator12StartTime < minRegulator12InputMillis );
      regulator12OutputShouldBeOn = (a0Voltage <= vMax) && (regulator12StartTime != -1) && (tickStartTime - regulator12StartTime < minRegulator12OutputMillis);
    }
    
    setRelayPinIfChanged( REGULATOR12_INPUT_RELAY , regulator12InputOn , regulator12InputShouldBeOn  );
    setRelayPinIfChanged( REGULATOR12_OUTPUT_RELAY, regulator12OutputOn, regulator12OutputShouldBeOn );
  }

  newState.regulator12InputOn  = regulator12InputOn;
  newState.regulator12OutputOn = regulator12OutputOn;
  
  newState.mqttConnected = (maintainWiFiConnection() == WL_CONNECTED) && maintainMqttConnection();
  
  message_clear();
  // Don't put too much in the log mesasge or our MQTT client will reject it!
  //message_appendLabel("a0RawValue");
  //message_appendLong(a0Val);
  message_appendLabel("batteryVoltage");
  message_appendFloat(a0Voltage);
  message_appendLabel("regPowered");
  message_appendString(newState.regulator12InputOn?"1":"0");
  message_appendLabel("regConnected");
  message_appendString(newState.regulator12OutputOn?"1":"0");
  message_appendLabel("time");
  message_appendLong(newState.timestamp);
  //message_appendLabel("mqttConnected");
  //message_appendString(mqttConnected?"1":"0");
  message_appendLabel("nodeId");
  message_appendMacAddressHex(macAddressBuffer, "-");
  message_close();

  bool publishedToMqtt = false;
  if( newState.mqttConnected ) {
    int valDiff = absDiff(newState.a0Val, lastMqttLogState.a0Val);
    if(
      lastMqttReportTime == -1 ||
      // Attempt to reduce high-frequency, low-amplitude noise in output:
      // Don't log a difference of only one unit except after waiting a while
      (valDiff >= 5                                               ) ||
      (valDiff >= 2 && tickStartTime - lastMqttReportTime >=  1000) ||
      (valDiff >= 1 && tickStartTime - lastMqttReportTime >= 10000) ||
      newState.regulator12InputOn != lastMqttLogState.regulator12InputOn ||
      newState.regulator12OutputOn != lastMqttLogState.regulator12OutputOn ||
      tickStartTime - lastMqttReportTime >= minMqttLogInterval
    ) {
      publishedToMqtt = pubSubClient.publish(MQTT_TOPIC, messageBuffer);
      if( publishedToMqtt ) {
        lastMqttReportTime = tickStartTime;
        lastMqttLogState = newState;
      }
    }
  } else {
    lastMqttReportTime = -1; // Force publish as soon as next connected
  }
  
  if(
    lastSerialReportTime == -1 ||
    tickStartTime - lastSerialReportTime >= 1000 ||
    publishedToMqtt ||
    newState.mqttConnected != prevState.mqttConnected
  ) {
    Serial.print("a0Val:");
    Serial.print(newState.a0Val);
    Serial.print(" ");
    Serial.print(messageBuffer);
    if( !newState.mqttConnected ) {
      Serial.print(" # Not connected to MQTT server");
    } else if( publishedToMqtt ) {
      Serial.print(" # Published to MQTT");
    }
    Serial.println("");
    lastSerialReportTime = tickStartTime;
  }

  ++tick;
  if( !regulator12OutputOn ) {
    ++consecutiveBoringTicks;
  } else {
    consecutiveBoringTicks = 0;
  }

  if( consecutiveBoringTicks >= 100 ) {
    long sleepTime = (a0Voltage - vMin)/maxDischargeRate;
    if( sleepTime > maxSleepTime ) sleepTime = maxSleepTime;
    if( sleepTime < 30000 ) goto nextTick;
    
    message_clear();
    message_appendString("# Power seems okay; time for a ");
    message_appendLong(sleepTime);
    message_appendString("ms nap!");
    message_close();
    Serial.println(messageBuffer);
    pubSubClient.publish(MQTT_TOPIC, messageBuffer);
    delay(1000);
    ESP.deepSleep(1000*(sleepTime-1000));
  }

nextTick:
  // APPARENTLY WIFI WILL NEVER WORK IF YOU DON'T HAVE A DELAY SOMEWHERE
  delay(100);

  prevState = newState;
}
#endif
