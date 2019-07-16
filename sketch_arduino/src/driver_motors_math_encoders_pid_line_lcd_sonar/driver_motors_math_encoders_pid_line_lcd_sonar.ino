// Скетч для управления моторами используя драйвер (ПИД-регулятор) для езды по линии
#include <LiquidCrystalRus.h> //инициализация библиотеки 
#include <Ultrasonic.h>
#include <SimpleKalmanFilter.h>

#define SONAR_TRIG_PIN        7
#define SONAR_ECHO_PIN        8

// выводы к которым подключен драйвер моторов
#define LEFT_MOTOR_PIN_1          5
#define LEFT_MOTOR_PIN_2          6
#define RIGHT_MOTOR_PIN_1         9
#define RIGHT_MOTOR_PIN_2         10

// выводы к которым подключены датчики линии
#define SENSOR_LINE_LEFT          A0
#define SENSOR_LINE_RIGHT         A1

#define LEFT_ENCODER_INTERRUPT_NB   0  // номер прерывания
#define RIGHT_ENCODER_INTERRUPT_NB  1  // номер прерывания

#define TIME_PUB_PERIOD_MS          5  // интервал публикации значений

#define Kp                        630.0    // пропорциональный коэффициент для ПИД регулятора (41.7)
#define Ki                        0.2      // интегральный коэффициент для ПИД регулятора
#define Kd                        0.3      // дифференциальный коэффициент для ПИД регулятора
#define I_SIZE                    1      // длинна выборки последних интегральных состовляющих   
#define K_SPEED                   0.42    // коэффициент поправки неадекватной скорости 

#define Kp_LINE                   0.003    // пропорциональный коэффициент для ПИД регулятора линии
#define Kd_LINE                   0.0005    // дифференциальный коэффициент для ПИД р-егулятора линии

#define MOTOR_VALUE_MAX           255      // максимальное значение подаваемое на драйвер
#define MOTOR_VALUE_MIN           80       // минимальное значение подаваемое на драйвер

#define ROBOT_LINEAR              0.3      // линейная скорость робота
#define ROBOT_ANGULAR             0.0      // угловая скорость робота

#define WHEEL_BASE                0.184     // база колесная в метрах
#define WHEEL_DIAMETER            0.084     // диаметр колеса в метрах
#define WHEEL_IMPULSE_COUNT       1225.0    // количество импульсов на оборот колеса

#define COUNT_MOTORS              2         // количество моторов

// стороны робота
#define LEFT                      0
#define RIGHT                     1

unsigned long last_time_pub; // время последней публикации

float linear = ROBOT_LINEAR;
float angular = ROBOT_ANGULAR;

float enc_count[COUNT_MOTORS] = {0.0, 0.0};      // счетчик для левого колеса
float speed_wheel[COUNT_MOTORS] = {0.0, 0.0};    // скорость моторов

float e_prev[COUNT_MOTORS] = {0.0, 0.0};          //последнее значение разницы скорости движения
float I_prev[COUNT_MOTORS][I_SIZE];               //последние значения выборки интегральной составляющей ПИД регулятора
int i_count = 0;

float e_prev_line = 0.0;

LiquidCrystalRus lcd(12, 13, A2, A3, A4, A5);
Ultrasonic sonar(SONAR_TRIG_PIN, SONAR_ECHO_PIN);
SimpleKalmanFilter kf(1, 1, 0.05);

void setup() {
  // инициализация выходов на драйверы управления моторами
  pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);

  // инициализация прерываний
  attachInterrupt(LEFT_ENCODER_INTERRUPT_NB,  callBackInterruptLeftEncoder, CHANGE);
  attachInterrupt(RIGHT_ENCODER_INTERRUPT_NB, callBackInterruptRightEncoder, CHANGE);

  lcd.begin(16, 2);// описание экрана 1602(16 столбцов,2 строки)

  Serial.begin(115200);
}

void loop() {
  float range = sonar.read() / 100.0;
  float range_kf = kf.updateEstimate(range);

  unsigned long t = millis() - last_time_pub;

  // проверяем нужно ли публиковать
  if (t > TIME_PUB_PERIOD_MS) {

    float sensor_line_left = analogRead(SENSOR_LINE_LEFT);
    float sensor_line_right = analogRead(SENSOR_LINE_RIGHT);
    float angular = pidLine(sensor_line_right, sensor_line_left);

    float V = linear;                      //линейная скорость
    float W = angular;                     //угловая скорость
    float r = WHEEL_DIAMETER / 2.0;          //радиус колеса
    float d = WHEEL_BASE;                  //база робота
    
    // вычисление требуемой скорости вращения колес
    speed_wheel[LEFT] = r * ((1.0 / r) * V - (d / r) * W);
    speed_wheel[RIGHT] = r * ((1.0 / r) * V + (d / r) * W);

    //вычисление текущей скорости вращения колес
    float speed_actual_left = impulse2meters(enc_count[LEFT]) / ((float)t / 1000.0);
    float speed_actual_right = impulse2meters(enc_count[RIGHT]) / ((float)t / 1000.0);

    //вычисление значения для драйвера используя ПИД-регулятор
    float pid_left = linear2driverMotor(speed_wheel[LEFT], speed_actual_left, LEFT);
    float pid_right = linear2driverMotor(speed_wheel[RIGHT], speed_actual_right, RIGHT);

    moveMotor(pid_left, LEFT);
    moveMotor(pid_right, RIGHT);

    lcd.print("ANG:"); //выводим текст
    lcd.print(angular); //выводим текст
    lcd.setCursor(0, 1); //переводим курсор на 0 столбец 1 строку
    lcd.print("SNR:"); //выводим текст
    lcd.print(range_kf); //выводим текст
    lcd.setCursor(0, 0); //переводим курсор на 0 столбец 1 строку

    enc_count[LEFT] = 0.0;
    enc_count[RIGHT] = 0.0;

    last_time_pub = millis(); // фиксируем время последней публикации
  }
}

float pidLine(int sensor_line_1, int sensor_line_2)
{
  //Расчет средней скорости движения между публикациями
  float e = sensor_line_2 - sensor_line_1;          //угол отклонения

  if (e == 0.0) {
    e_prev_line = 0.0;
    return 0.0;
  }

  //ПИД регулятор для расчета значения для драйвера моторов
  float P = Kp_LINE * e;
  float D = Kd_LINE * (e - e_prev_line);
  float value = P + D;

  e_prev_line = e;                           //фиксируем последнее значение угла угла отклонения

  return value;
}

int linear2driverMotor(float linear_speed, float speed_actual, int side)
{
  if (linear_speed == 0) {
    I_prev[i_count][side] = 0.0;
    e_prev[side] = 0.0;
    return 0;
  }

  //Расчет средней скорости движения между публикациями
  float e = -(speed_actual - (linear_speed / K_SPEED));          //разница в скорости текущая в m/s и желаемая m/s

  //ПИД регулятор для рассчета значения для драйвера моторов
  float P = Kp * e;
  float I = sumIprev(side) * Ki;
  float D = Kd * (e - e_prev[side]);
  float motor_value = round(P + I + D);

  if (i_count == I_SIZE) {
    i_count = 0;
  }

  I_prev[i_count][side] = I;                  //фиксируем интегральную составляющую
  e_prev[side] = e;                     //фиксируем последнее значение разницы в скорости
  i_count++;

  if (motor_value < 0 && motor_value >= -MOTOR_VALUE_MIN) {
    motor_value = -MOTOR_VALUE_MIN;
  }
  if (motor_value > 0 && motor_value <= MOTOR_VALUE_MIN) {
    motor_value = MOTOR_VALUE_MIN;
  }

  //Убираем переполнение ШИМ
  if (motor_value > MOTOR_VALUE_MAX) {
    return MOTOR_VALUE_MAX;
  }

  if (motor_value < -MOTOR_VALUE_MAX) {
    return -MOTOR_VALUE_MAX;
  }

  return motor_value;
}

// управление мотором на определенной стороне робота
void moveMotor(int value, int side) {
  // избавляемся от переполнения ШИМ
  if (value > 255)
    value = 255;
  if (value < -255)
    value = -255;

  // определяем направление вращения и передаем значения на драйвер
  if (value >= 0) {
    if (value == 0) {
      // стоп мотор
      analogWrite(side == LEFT ? LEFT_MOTOR_PIN_1 : RIGHT_MOTOR_PIN_1, 0);
      analogWrite(side == LEFT ? LEFT_MOTOR_PIN_2 : RIGHT_MOTOR_PIN_2, 0);
    } else {
      // вращение вперед
      analogWrite(side == LEFT ? LEFT_MOTOR_PIN_1 : RIGHT_MOTOR_PIN_1, abs(value));
      analogWrite(side == LEFT ? LEFT_MOTOR_PIN_2 : RIGHT_MOTOR_PIN_2, 0);
    }
  } else {
    // вращение назад
    analogWrite(side == LEFT ? LEFT_MOTOR_PIN_1 : RIGHT_MOTOR_PIN_1, 0);
    analogWrite(side == LEFT ? LEFT_MOTOR_PIN_2 : RIGHT_MOTOR_PIN_2, abs(value));
  }
}

float getRotationDir(float value) {
  if (value >= 0) {
    if (value == 0) {
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

float sumIprev(int side) {
  float sum_i_prev = 0.0;
  for (int i = 0; i < I_SIZE; i++)
  {
    sum_i_prev += I_prev[side][i];
  }
  return sum_i_prev;
}

float noRear(float value)
{
  if (value < 0)
  {
    return 0.0;
  }
  return abs(value / 2);
}
