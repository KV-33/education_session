// Скетч для управления моторами используя драйвер (демо)

// выводы к которым подключен драйвер моторов
#define LEFT_MOTOR_PIN_1          5
#define LEFT_MOTOR_PIN_2          6
#define RIGHT_MOTOR_PIN_1         9
#define RIGHT_MOTOR_PIN_2         10

#define TIME_PUB_PERIOD_MS        3000  // интервал публикации значений

#define MOTOR_VALUE               100   // значение подаваемое на драйвер

#define MOTOR_VALUE_MAX           255      // максимальное значение подаваемое на драйвер
#define MOTOR_VALUE_MIN           50       // минимальное значение подаваемое на драйвер

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
    moveMotor(MOTOR_VALUE, LEFT);
    moveMotor(MOTOR_VALUE, RIGHT);
    break;
  case MOVE_REAR:
    moveMotor(-MOTOR_VALUE, LEFT);
    moveMotor(-MOTOR_VALUE, RIGHT);
    break;
  case ROTATE_LEFT:
    moveMotor(-MOTOR_VALUE, LEFT);
    moveMotor(MOTOR_VALUE, RIGHT);
    break;
  case ROTATE_RIGHT:
    moveMotor(MOTOR_VALUE, LEFT);
    moveMotor(-MOTOR_VALUE, RIGHT);
    break;
  case STOP:
    moveMotor(0, LEFT);
    moveMotor(0, RIGHT);
    break;
  }
}
