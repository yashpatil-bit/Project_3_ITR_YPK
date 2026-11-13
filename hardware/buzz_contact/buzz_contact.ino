// Optional objective contact sensor for the buzz-wire (Arduino Uno/Nano).
//
// Wiring (ASK THE TA FIRST - do not modify the shared wire stand without permission):
//   - wire loop         -> Arduino D2   (input with internal pull-up)
//   - probe ring        -> Arduino GND  (thin flexible lead along the robot arm)
//   Touching the wire pulls D2 LOW -> we print "1"; released -> "0".
//   If the stand already has its own battery + buzzer circuit, tap it through an
//   opto-coupler instead of connecting directly, so the two circuits never share current.
//
// Host side: python3 scripts/contact_monitor.py --mode serial --port /dev/ttyACM0

const int WIRE_PIN = 2;
const int LED = 13;
const unsigned long DEBOUNCE_US = 300;   // short: we want to catch brief touches

int state = 0;

void setup() {
  pinMode(WIRE_PIN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  int touching = (digitalRead(WIRE_PIN) == LOW) ? 1 : 0;
  if (touching != state) {
    delayMicroseconds(DEBOUNCE_US);
    if (((digitalRead(WIRE_PIN) == LOW) ? 1 : 0) == touching || touching == 1) {
      state = touching;
      digitalWrite(LED, state);
      Serial.println(state);
    }
  }
  static unsigned long last = 0;           // heartbeat so the host sees the state
  if (millis() - last > 200) {
    last = millis();
    Serial.println(state);
  }
}
