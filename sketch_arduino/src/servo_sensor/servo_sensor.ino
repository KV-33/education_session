#include <Servo.h> //используем библиотеку для работы с сервоприводом

#define SERVO_PIN                  11

#define SENSOR_PIN                 4

#define TIME_PUB_PERIOD_MS         500  // интервал публикации значений

unsigned long last_time_pub; // время последней публикации

Servo arm; //объявляем переменную arm типа Servo
bool state_sensor = false;

void setup(){
  arm.attach(SERVO_PIN);
  pinMode(SENSOR_PIN, INPUT);
  
  Serial.begin(115200);
}

void loop(){
  unsigned long t = millis() - last_time_pub;

  // проверяем нужно ли публиковать
  if (t > TIME_PUB_PERIOD_MS) {
    state_sensor = digitalRead(SENSOR_PIN);
    
    Serial.print(", sensor: ");
    Serial.println(state_sensor);

    Serial.println("--------------------------------");
    last_time_pub = millis(); // фиксируем время последней публикации
  }
  setArm(state_sensor);
}

void setArm(bool value){
  if(value == true)
  {
    arm.write(180); //ставим вал под 0
  }
  else
  {
    arm.write(0); //ставим вал под 0
  }
}
