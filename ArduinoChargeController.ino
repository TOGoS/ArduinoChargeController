/*
 * ArduinoChargeController
 * 
 * Watch voltage on a battery, and when it gets too low,
 * power on and connect to a wall power supply
 * (one relay for the supply's 120V input, one for its 12-14V output)
 * 
 * Designed to run on an ESP8266 with one of the popular '4 Relay Modules'
 * available circa 2017 for ~8$ each.
 * The one I'm using seems to work just fine with ESP8266 voltage levels
 * (3.3V)
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "config.h"

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

// I'm using a 1M+5M = 6M voltage divider
// The numbers I get seem to indicate 14.5 as a good thing to divide by to get volts
// from the analog reading.
#define A0_VALUE_TO_VOLTS (1.0/14.5)

#define messageBufferSize 128

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
void message_appendString(const char *str) {
  while( *str != 0 && messageBufferOffset < messageBufferSize-1 ) {
    messageBuffer[messageBufferOffset] = *str;
    ++str;
    ++messageBufferOffset;
  }
}
void message_appendMacAddressHex(byte *macAddress, const char *octetSeparator) {
  for( int i=0; i<6; ++i ) {
    if( i > 0 ) message_appendString(octetSeparator);
    messageBuffer[messageBufferOffset++] = hexDigit(macAddress[i]>>4);
    messageBuffer[messageBufferOffset++] = hexDigit(macAddress[i]);
  }
}
void message_separate(const char *separator) {
  if( messageBufferOffset == 0 ) return;
  message_appendString(separator);
}
void message_appendFloat(float v) {
  if( v < 0 ) {
    messageBuffer[messageBufferOffset++] = '-'; // memory unsafety!!
    v = -v;
  }
  int hundredths = (v * 100) - ((int)v) * 100;
  int printed = snprintf(messageBuffer+messageBufferOffset, messageBufferSize-messageBufferOffset, "%d.%02d", (int)v, hundredths);
  if( printed > 0 ) messageBufferOffset += printed;
}
void message_close() {
  messageBuffer[messageBufferOffset++] = 0;
}


WiFiClient espClient;
PubSubClient pubSubClient(espClient);


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
}

int tick = 0;
int previousWiFiStatus = -1;

void reportWiFiStatus(int wiFiStatus) {
  Serial.print("# WiFi status: ");
  switch( wiFiStatus ) {
  case WL_CONNECTED:
    Serial.println("WL_CONNECTED");
    Serial.print("# IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("# MAC address: ");
    WiFi.macAddress(macAddressBuffer);
    message_appendMacAddressHex(macAddressBuffer, ":");
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

int maintainWiFiConnection() {
  long currentTime = millis();
  int wiFiStatus = WiFi.status();
  if( wiFiStatus != previousWiFiStatus ) {
    reportWiFiStatus(wiFiStatus);
    previousWiFiStatus = wiFiStatus;
  }    
  if( wiFiStatus != WL_CONNECTED && lastWiFiConnectAttempt < currentTime - 10000 ) {
    Serial.print("# Attempting to connect to ");
    Serial.print(WIFI_SSID);
    Serial.print("...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWiFiConnectAttempt = currentTime;
  }
  return wiFiStatus;
}

bool maintainMqttConnection() {
  return false;
  // TODO
  if( !pubSubClient.connected() ) {
    message_clear();
    message_appendString("# Hi, I'm ");
    const char *macAddressString = messageBuffer+messageBufferOffset;
    message_appendMacAddressHex(macAddressBuffer, ":");
    pubSubClient.connect(macAddressString);
  }
}

void loop() {
  int connected =
    (maintainWiFiConnection() == WL_CONNECTED) && 
    (maintainMqttConnection());
  
  // Blink the LED *.*.*... so we can tell it's working
  switch( (tick % 8) ) {
  case 0: case 2: case 4:
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level
    break;
  default:
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  }
  
  int a0Val = analogRead(A0);
  float a0Voltage = a0Val * A0_VALUE_TO_VOLTS;
  if( (tick % 10) == 0 ) {
    Serial.print("a0/rawValue:");
    Serial.print(a0Val);
    Serial.print(" a0/voltage:");
    Serial.print(a0Voltage);
    Serial.print("\n");
  }
  
  // Flip the relaze!!
  digitalWrite(RELAY1, (tick & 0x10) ? LOW : HIGH );
  digitalWrite(RELAY2, (tick & 0x20) ? LOW : HIGH );
  digitalWrite(RELAY3, (tick & 0x40) ? LOW : HIGH );
  digitalWrite(RELAY4, (tick & 0x80) ? LOW : HIGH );
  
  if( tick == 100 ) {
    // Take a nap!
    // For it to ever wake up requires that D0 be wired to RST.
    //ESP.deepSleep(30000000);
  }
  
  // Kill time until the next tick
  delay(100);
  ++tick;
}

