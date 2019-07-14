// Скетч для управления моторами используя драйвер (демо)

// выводы к которым подключен драйвер моторов
#define LEFT_MOTOR_PIN_1          5
#define LEFT_MOTOR_PIN_2          6
#define RIGHT_MOTOR_PIN_1         9
#define RIGHT_MOTOR_PIN_2         10

#define TIME_PUB_PERIOD_MS        1000  // интервал публикации значений

#define ROBOT_LINEAR              0.5      // линейная скорость робота
#define ROBOT_ANGULAR             0.0      // угловая скорость робота

#define WHEEL_BASE                0.184     // база колесная в метрах

// стороны робота
#define LEFT                      0
#define RIGHT                     1

// режимы управления
#define STOP                      0
#define MOVE_FRONT                1
#define MOVE_REAR                 2
#define ROTATE_LEFT               3
#define ROTATE_RIGHT              4

unsigned long last_time; // время последней публикации
int mode = STOP;         // режим управления

float linear = ROBOT_LINEAR;
float angular = ROBOT_ANGULAR;

void setup() {
  // инициализация выходов на драйверы управления моторами
  pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);

  Serial.begin(115200);
}

void loop(){
    // демонстрационное вращение моторами с заданной скоростью и сменой направления по таймеру
    switchMode();
    go(mode);
  
    // вычисление требуемой скорости вращения колес
    float speed_left = (linear - WHEEL_BASE * angular);
    float speed_right = (linear + WHEEL_BASE * angular);

    moveMotor(getMotorValue(speed_left), LEFT);
    moveMotor(getMotorValue(speed_right), RIGHT);
}

// управление мотором на определенной стороне робота
void moveMotor(int value, int side){
  // избавляемся от переполнения ШИМ
  if (value>255)
    value = 255;
  if (value<-255)
    value = -255;

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

// изменение режима управления по заданному интервалу
void switchMode(){
  // вычисление интервала времени от последней публикации
  unsigned long time = millis() - last_time;

  // проверяем нужно ли менять режим
  if (time > TIME_PUB_PERIOD_MS) {
    mode++;
    if(mode > ROTATE_RIGHT)
    {
      mode = STOP;
    }
    last_time = millis(); // фиксируем время последней публикации
  }
}

// режимы управления (передача значений драйверу)
void go(int mode){
  switch(mode){
  case MOVE_FRONT:
    angular = 0.0;
    linear = ROBOT_LINEAR;
    break;
  case MOVE_REAR:
    angular = 0.0;
    linear = -ROBOT_LINEAR;
    break;
  case ROTATE_LEFT:
    angular = 3.0;
    linear = 0.0;
    break;
  case ROTATE_RIGHT:
    angular = -3.0;
    linear = 0.0;
    break;
  case STOP:
    angular = 0.0;
    linear = 0.0;
    break;
  }
}

int getMotorValue(float value){
  return map(value*1000, -1000, 1000, -255, 255);
}
