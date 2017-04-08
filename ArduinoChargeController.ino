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

void loop() {
  // Blink the LED *.*.*... so we can tell it's working
  switch( (tick % 8) ) {
  case 0: case 2: case 4:
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level
    break;
  default:
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  }

  int a0Val = analogRead(A0);
  if( (tick % 10) == 0 ) {
    Serial.print("analogRead(A0) = ");
    Serial.println(a0Val);
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

