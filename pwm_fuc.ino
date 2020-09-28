int ledPin = 7;
long period = 10000;
long duty = 100;
int i,j;
int value, key;

void setup() {
  
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  while(!Serial){}
}

void loop() {
  while(Serial.available())
  {
    value = Serial.parseInt();
    if(value)
    {
      Serial.print("value: ");
      Serial.println(value);
      Serial.println("Enter key, period: 1 or duty: 2\n in5 seconds");
      delay(5000);
      key = Serial.parseInt();
      Serial.print("key: ");
      Serial.println(key);
      
      if(key == 1) set_period(value);
      else if(key == 2) set_duty(value);
      else Serial.println("late or invaild key");
    }
  }
  for(i = 0; i <= duty; i++) light(i);
  for(i = duty; i >= 0; i--) light(i);
}

void light(int lightness)
{
  digitalWrite(ledPin,LOW);
  delayMicroseconds(period*lightness/100);
  digitalWrite(ledPin,HIGH);
  delayMicroseconds(period*(100-lightness)/100);
  
  //Serial.println(period*lightness/100);
  //Serial.println(period*(100-lightness)/100);
}

void set_period(int value) // period: 100 to 10000 (unit: us)
{
  if(value >= 100 && value <= 10000)
  {
    period = value;
    Serial.println("Success");
  }
  else Serial.println("invaild value: peroid(100 to 10000)");
}

void set_duty(int value) // duty: 0 to 100 (unit: %)
{  
  if(value >= 1 && value <= 100)
  {
    duty = value;
    Serial.println("Success");
  }
  else Serial.println("invaild vlaue: duty(1 to 100)");
}
