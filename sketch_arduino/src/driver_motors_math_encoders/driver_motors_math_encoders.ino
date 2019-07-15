// Скетч для управления моторами используя драйвер (демо)

// выводы к которым подключен драйвер моторов
#define LEFT_MOTOR_PIN_1          5
#define LEFT_MOTOR_PIN_2          6
#define RIGHT_MOTOR_PIN_1         9
#define RIGHT_MOTOR_PIN_2         10

#define LEFT_ENCODER_INTERRUPT_NB   0  // номер прерывания
#define RIGHT_ENCODER_INTERRUPT_NB  1  // номер прерывания

#define TIME_PUB_PERIOD_MS        5000  // интервал публикации значений

#define ROBOT_LINEAR              0.5      // линейная скорость робота
#define ROBOT_ANGULAR             0.0      // угловая скорость робота

#define WHEEL_BASE                0.184     // база колесная в метрах
#define WHEEL_DIAMETER            0.084     // диаметр колеса в метрах
#define WHEEL_IMPULSE_COUNT       1225.0    // количество импульсов на оборот колеса

#define COUNT_MOTORS              2         // количество моторов

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

float enc_count[COUNT_MOTORS] = {0.0, 0.0};      // счетчик для левого колеса
float speed_wheel[COUNT_MOTORS] = {0.0, 0.0};    // скорость моторов

void setup() {
  // инициализация выходов на драйверы управления моторами
  pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);

  // инициализация прерываний
  attachInterrupt(LEFT_ENCODER_INTERRUPT_NB,  callBackInterruptLeftEncoder, CHANGE);
  attachInterrupt(RIGHT_ENCODER_INTERRUPT_NB, callBackInterruptRightEncoder, CHANGE);

  Serial.begin(115200);
}

void loop(){
    // демонстрационное вращение моторами с заданной скоростью и сменой направления по таймеру
    switchMode();
    go(mode);
  
    float V = linear;                      //линейная скорость
    float W = angular;                     //угловая скорость
    float r = WHEEL_DIAMETER/2;            //радиус колеса
    float d = WHEEL_BASE;                  //база робота
  
    // вычисление требуемой скорости вращения колес
    float speed_left = r * ((1 / r) * V - (d / r) * W);
    float speed_right = r * ((1 / r) * V + (d / r) * W);

    moveMotor(getMotorValue(speed_wheel[LEFT]), LEFT);
    moveMotor(getMotorValue(speed_wheel[RIGHT]), RIGHT);
    
    Serial.print("L_count: ");
    Serial.print(enc_count[LEFT]);
    Serial.print(", R_count: ");
    Serial.println(enc_count[RIGHT]);

    Serial.print("L_meters: ");
    Serial.print(impulse2meters(enc_count[LEFT]));
    Serial.print(", R_meters: ");
    Serial.println(impulse2meters(enc_count[RIGHT]));

    Serial.println("--------------------------------");
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

float getRotationDir(float value){
  if (value>=0) {
    if (value==0){
      return 0.0;
    }
    else
    {
      return 1.0;
    }
  }
  else
  {
    return -1.0;
  }
}

// обработчик прерывания для левого колеса
inline void callBackInterruptLeftEncoder() {
  enc_count[LEFT] += getRotationDir(speed_wheel[LEFT]);
}

// обработчик прерывания для правого колеса
inline void callBackInterruptRightEncoder() {
  enc_count[RIGHT] += getRotationDir(speed_wheel[RIGHT]);
}

// преобразование импульсов в метры
inline float impulse2meters(float x) {
  return (x / WHEEL_IMPULSE_COUNT) * M_PI * WHEEL_DIAMETER;
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
