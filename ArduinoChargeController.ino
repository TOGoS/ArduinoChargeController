/*
 ESP8266 Blink by Simon Peter
 Blink the blue LED on the ESP-01 module
 This example code is in the public domain
 
 The blue LED on the ESP-01 module is connected to GPIO1 
 (which is also the TXD pin; so we cannot use Serial.print() at the same time)
 
 Note that this sketch uses LED_BUILTIN to find the pin with the internal LED
*/

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

#define RELAY1 D8
#define RELAY2 D3
#define RELAY3 D2
#define RELAY4 D1

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

// the loop function runs over and over again forever
void loop() {
  // Blink the LED *.*.*... so we can tell it's working
  switch( (tick % 8) ) {
  case 0: case 2: case 4:
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level
    break;
  default:
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  }

  // Flip the relaze!!
  digitalWrite(RELAY1, ((tick +  0) % 20) < 10 ? LOW : HIGH );
  digitalWrite(RELAY2, ((tick +  5) % 20) < 10 ? LOW : HIGH );
  digitalWrite(RELAY3, ((tick + 10) % 20) < 10 ? LOW : HIGH );
  digitalWrite(RELAY4, ((tick + 15) % 20) < 10 ? LOW : HIGH );

  // Kill time until the next tick
  delay(100);
  ++tick;
}

