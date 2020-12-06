#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 // [3097] LED핀 9번으로 설정
#define PIN_SERVO 10 // [3097] 서보핀 9번으로 설정
#define PIN_IR A0 // [3097] 적외선핀 A0번으로 설정

// Framework setting
#define _DIST_TARGET 255 //[0711] 목표값
#define _DIST_MIN 100 //[0711] 측정 최솟값
#define _DIST_MAX 410 //[0711] 측정 최댓값

// Distance sensor
#define _DIST_ALPHA 0.8 // [3095] ema필터의 alpha 값을 설정

// Servo range
#define _DUTY_MIN 1000 // servo full clockwise position (+ degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 1950 // servo full counterclockwise position (- degree)

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 30 

// Event periods
#define _INTERVAL_DIST 25  // [3074] 거리측정주기 (ms)
#define _INTERVAL_SERVO 25 // [3078] 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // [3078] Serial제어주기 (ms)

// PID parameters
#define _KP 3 //[0711] 비례이득
#define _KD 50

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;  // [1615] 센서가 인식한 거리/ema 필터링된 거리

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr = _DUTY_NEU;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
  // initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT); // [3083] PIN_LED 핀을 OUTPUT으로 설정
  myservo.attach(PIN_SERVO); // [3083] PIN_SERVO 핀을 servo를 쓰는 핀으로 설정
  
  // initialize global variables
  
  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); // [3090]
   
  // initialize serial port
  Serial.begin(57600); // [3083] serial port의 baud rate를 57600으로 초기화
  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0)*6; // [3074] 서보 업데이트 1주기에 증감 가능한 duty 의 최댓값

  // [1615] 마지막 이벤트 발생 시간 초기화
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

  event_dist = false;
  event_servo = false;
  event_serial = false;
}
  
void loop() {
/////////////////////
// Event generator //
/////////////////////
// [1615] 거리 측정 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST)
        event_dist = true;
    
  // [1615] 서보 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO)
      event_servo= true;
    
  // [1615] Serial 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL)
      event_serial= true;



////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;
      // get a distance reading from the distance sensor
      dist_raw = ir_distance();

      // PID control logic
      error_curr = dist_raw - _DIST_TARGET; // [3073] 현재 읽어들인 데이터와 기준 값의 차이
      pterm = error_curr * _KP;// [3073] p게인 값인 kp와 error 값의 곱
      dterm = _KD * (error_curr - error_prev);
      
      control = pterm + dterm;
      
      duty_target = _DUTY_NEU - control;
      // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
      if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; // lower limit
      if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; // upper limit
      // update error_prev
      error_prev = error_curr;
    
      // [1615] 마지막 샘플링 시각 업데이트
      last_sampling_time_dist += _INTERVAL_DIST;
  }
  
  
  if(event_servo) {
    event_servo = false;
    
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_curr - duty_target > duty_chg_per_interval)
      duty_curr = duty_curr - duty_chg_per_interval;
    else if(duty_target - duty_curr > duty_chg_per_interval)
      duty_curr = duty_curr + duty_chg_per_interval;
    else
    
      duty_curr = duty_target;

    // update servo position
    myservo.writeMicroseconds(duty_curr); // [1615]

    // [1615] 마지막 서보 조작 시각 업데이트
    last_sampling_time_servo += _INTERVAL_SERVO;
  }
  
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
   }
}

float ir_distance(void){ // return value unit: mm
  // [3088] 19_example_0에서 가져옴
  const float coE[] = {-0.0000101, 0.0063940, 0.1137618, 70.5921117};
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;  // [1615] 변환 식 보정 필요
  val = coE[0] * pow(val, 3) + coE[1] * pow(val, 2) + coE[2] * val + coE[3];
  val = dist_raw * _DIST_ALPHA + (1-_DIST_ALPHA)*val;
  return val;
}
