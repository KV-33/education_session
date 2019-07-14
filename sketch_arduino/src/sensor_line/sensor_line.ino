// Скетч для определения угла по датчикам линии

// выводы к которым подключены датчики линии
#define SENSOR_ANALOG                  A0
#define SENSOR_DIGITAL                 4

#define TIME_PUB_PERIOD_MS          200  // интервал публикации значений

unsigned long last_time_pub; // время последней публикации

float e_prev_line = 0.0;

void setup() {
  pinMode(SENSOR_DIGITAL, INPUT);
  
  Serial.begin(115200);
}

void loop(){
  unsigned long t = millis() - last_time_pub;

  // проверяем нужно ли публиковать
  if (t > TIME_PUB_PERIOD_MS) {

    int sensor_analog = analogRead(SENSOR_ANALOG);
    int sensor_digital = digitalRead(SENSOR_DIGITAL);
    
    Serial.print("sensor_analog: ");
    Serial.print(sensor_analog);
    Serial.print(", sensor_digital: ");
    Serial.println(sensor_digital);

    Serial.println("--------------------------------");
    last_time_pub = millis(); // фиксируем время последней публикации
  }
}
