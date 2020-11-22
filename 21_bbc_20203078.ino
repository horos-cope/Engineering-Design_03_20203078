#include <Servo.h>

Servo myservo;

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9

const float coE[] = {-0.0000279, 0.0185504, -2.1256635, 215.0260079};

int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

  a = 60;
  b = 300;
  myservo.attach(10);
  myservo.writeMicroseconds(1500);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = coE[0] * pow(raw_dist, 3) + coE[1] * pow(raw_dist, 2) + coE[2] * raw_dist + coE[3];
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(dist_cali > 255)
  {
    myservo.writeMicroseconds(1000);
  }
  else
  {
    myservo.writeMicroseconds(1950);
  }
  delay(20);
}
