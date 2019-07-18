// Скетч для определения угла по датчикам линии

// выводы к которым подключены датчики линии
#define SENSOR_LINE_LEFT          A0
#define SENSOR_LINE_RIGHT         A1

#define TIME_PUB_PERIOD_MS          20  // интервал публикации значений

#define Kp_LINE                   0.003    // пропорциональный коэффициент для ПИД регулятора линии
#define Kd_LINE                   0.0005    // дифференциальный коэффициент для ПИД р-егулятора линии

unsigned long last_time_pub; // время последней публикации

float e_prev_line = 0.0;

void setup() {
  Serial.begin(115200);
}

void loop(){
  unsigned long t = millis() - last_time_pub;

  // проверяем нужно ли публиковать
  if (t > TIME_PUB_PERIOD_MS) {

    float sensor_line_left = analogRead(SENSOR_LINE_LEFT);
    float sensor_line_right = analogRead(SENSOR_LINE_RIGHT);
    float angular = linePID(sensor_line_left, sensor_line_right);
    
    Serial.print("angular: ");
    Serial.print(angular);
    Serial.print(", L_sensor: ");
    Serial.print(sensor_line_left);
    Serial.print(", R_sensor: ");
    Serial.println(sensor_line_right);

    Serial.println("--------------------------------");
    last_time_pub = millis(); // фиксируем время последней публикации
  }
}

float linePID(int sensor_line_1, int sensor_line_2)
{
  //Расчет средней скорости движения между публикациями
  float e = sensor_line_2 - sensor_line_1;          //угол отклонения
  
  if(e == 0.0){
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
