#include <Servo.h>

Servo myservo;

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9

#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 450 // maximum distance to be measured (unit: mm)

#define _DUTY_MIN 1000 // servo full clockwise position (+ degree)
#define _DUTY_NEU 1460 // servo neutral position (90 degree)
#define _DUTY_MAX 1950 // servo full counterclockwise position (- degree)

const float coE[] = {-0.0000279, 0.0185504, -2.1256635, 215.0260079};
unsigned long last_sampling_time; // unit: ms
int last_section, curr_section;
float section_distance[6] = {0,200,240,330,410,600};
float raw_dist, dist_cali, dist_ema, duty_gap, alpha;
float last_dist;
float servo_angle;


void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

  myservo.attach(10);
  myservo.writeMicroseconds(1500);

  alpha = 0.7;

  servo_angle = _DUTY_NEU;
  last_section = find_section();
  last_sampling_time = 0;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

int find_section(){
  for(int i = 0;i < 5; i++)
  {
    if(section_distance[i] <= dist_ema && section_distance[i+1] >= dist_ema)
      return i;
  }
  return 5;
}

void dist_calcul(void){
  raw_dist = ir_distance();
  dist_cali = coE[0] * pow(raw_dist, 3) + coE[1] * pow(raw_dist, 2) + coE[2] * raw_dist + coE[3];

  // get a distance reading from the USS
  dist_ema = (alpha)*dist_cali + dist_ema*(1-alpha);
}

//상황 8개

int calc_duty(){
  if(curr_section-last_section>0)
  {
    if(curr_section == 1)
    {
      return 75;
    }
    else if(curr_section == 2)
    {
      return 0;
    }
    else if(curr_section == 3)
    {
      return 25;
    }
    else return 50;
  }
  else if(curr_section-last_section<0)
  {
    if(curr_section == 3)
    {
      return -75;
    }
    else if(curr_section == 2)
    {
      return 0;
    }
    else if(curr_section == 1)
    {
      return -25;
    }
    else return -50;
  }
  else
  {
    if(curr_section == 0)
      return -50; 
    else if(curr_section == 4)
      return 50;
  }
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return ;

  // 만약 공이 20~35 구간에 있다면
  // 10~20구간
  // 20~25구간
  // 25~30구간
  // 30~35구간
  // 35~45구간
  
  dist_calcul();
  curr_section = find_section();
  Serial.print("min:100,max:400,dist:");
  Serial.print(raw_dist);
  Serial.print(", dist_cali:");
  Serial.print(dist_cali);
  Serial.print(", dist_ema:");
  Serial.println(dist_ema);
  servo_angle = _DUTY_NEU - calc_duty()*5.5;
  /*
  Serial.print(servo_angle);
  Serial.print(" ");
  Serial.print(last_section);
  Serial.print(" ");
  Serial.println(curr_section);
  */
  last_section = curr_section;
  if(servo_angle < 1000) servo_angle = 1000;
  else if(servo_angle > 1950) servo_angle = 1950;
  myservo.writeMicroseconds(servo_angle);
}
