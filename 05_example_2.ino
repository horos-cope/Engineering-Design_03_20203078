//05P12
#define PIN_LED 7
unsigned int toggle;
int i;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  toggle = 1;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  toggle = toggle_state(toggle); // toggle LED value.
  Serial.println(toggle);
  digitalWrite(PIN_LED, toggle); // update LED status.
  delay(1000); // wait for 1,000 milliseconds
  for(i=0;i<11;i++)
  {
    toggle = toggle_state(toggle); // toggle LED value.
    digitalWrite(PIN_LED, toggle);
    delay(100);
  }

  while(1) {}
}

int toggle_state(int toggle) {
  return !toggle;
}
