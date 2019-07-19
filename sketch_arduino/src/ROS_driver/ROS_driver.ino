// Sketch for ROS
#include <ros.h>
#include <std_msgs/Int32.h>

// выводы к которым подключен драйвер моторов
#define LEFT_MOTOR_PIN_1          5
#define LEFT_MOTOR_PIN_2          6
#define RIGHT_MOTOR_PIN_1         9
#define RIGHT_MOTOR_PIN_2         10

#define MOTOR_VALUE_MAX           255      // максимальное значение подаваемое на драйвер
#define MOTOR_VALUE_MIN           50       // минимальное значение подаваемое на драйвер

#define LEFT                      0
#define RIGHT                     1

#define TIME_PUB_PERIOD_MS        20  // интервал публикации значений

#define COUNT_MOTORS              2

int value_motors[COUNT_MOTORS] = {0, 0};    //значения подаваемые на драйвер моторов
unsigned long last_time;

// функция обработки сообщений топика
void callBackCmdLeftMotor( const std_msgs::Int32& msg){
  value_motors[LEFT] = msg.data;
}
void callBackCmdRightMotor( const std_msgs::Int32& msg){
  value_motors[RIGHT] = msg.data;
}

// создание переменной узла
ros::NodeHandle nh;

// подписчики
ros::Subscriber<std_msgs::Int32> subCmdLeftMotor("motor/left", &callBackCmdLeftMotor );
ros::Subscriber<std_msgs::Int32> subCmdRightMotor("motor/right", &callBackCmdRightMotor );

void setup() {
 nh.getHardware()->setBaud(115200);  // задаем скорость обмена информацией
 nh.initNode();                      // инициализируем узел
 nh.subscribe(subCmdLeftMotor);      // инициализируем подписчика для левого мотора
 nh.subscribe(subCmdRightMotor);     // инициализируем подписчика для правого мотора
 
 // инициализация выходов на драйверы управления моторами
 pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
 pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
 pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
 pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);
}

void loop(){
  unsigned long time = millis() - last_time;

  // проверяем нужно ли менять режим
  if (time > TIME_PUB_PERIOD_MS) {
    moveMotor(value_motors[LEFT], LEFT);
    moveMotor(value_motors[RIGHT], RIGHT);
    last_time = millis(); // фиксируем время последней публикации
  }
 nh.spinOnce();   
}

// управление мотором на определенной стороне робота
void moveMotor(int value, int side){
  // избавляемся от переполнения ШИМ
  if (value>MOTOR_VALUE_MAX)
    value = MOTOR_VALUE_MAX;
  if (value<-MOTOR_VALUE_MAX)
    value = -MOTOR_VALUE_MAX;

  // убираем значения ниже минимального значения при котором моторы могут вращаться
  if (value < 0 && value >= -MOTOR_VALUE_MIN)
    value = -MOTOR_VALUE_MIN;
  if (value > 0 && value <= MOTOR_VALUE_MIN)
    value = MOTOR_VALUE_MIN;

  // определяем направление вращения и передаем значения на драйвер
  if (value>=0) {
    if (value==0){
      // стоп мотор
      analogWrite(side==LEFT ? LEFT_MOTOR_PIN_1 : RIGHT_MOTOR_PIN_1, 0);
      analogWrite(side==LEFT ? LEFT_MOTOR_PIN_2 : RIGHT_MOTOR_PIN_2, 0);
    } else {
      // вращение вперед
      analogWrite(side==LEFT ? LEFT_MOTOR_PIN_1 : RIGHT_MOTOR_PIN_1, abs(value));
      analogWrite(side==LEFT ? LEFT_MOTOR_PIN_2 : RIGHT_MOTOR_PIN_2, 0);
    }
  } else {
    // вращение назад
    analogWrite(side==LEFT ? LEFT_MOTOR_PIN_1 : RIGHT_MOTOR_PIN_1, 0);
    analogWrite(side==LEFT ? LEFT_MOTOR_PIN_2 : RIGHT_MOTOR_PIN_2, abs(value));
  }
}
