#include <Servo.h> //используем библиотеку для работы с сервоприводом

#define SERVO_PIN        11

Servo servo; //объявляем переменную servo типа Servo

void setup(){
  servo.attach(SERVO_PIN);
}

void loop(){
  servo.write(0); //ставим вал под 0
  delay(2000); //ждем 2 секунды
  servo.write(180); //ставим вал под 180
  delay(2000); //ждем 2 секунды
}
