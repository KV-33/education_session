#include <Ultrasonic.h>
#include <SimpleKalmanFilter.h>

#define SONAR_TRIG_PIN        7
#define SONAR_ECHO_PIN        8

#define TIME_PUB_PERIOD_MS          200  // интервал публикации значений

unsigned long last_time_pub; // время последней публикации

Ultrasonic sonar(SONAR_TRIG_PIN, SONAR_ECHO_PIN);
SimpleKalmanFilter kf(1, 1, 0.05);

void setup() {
  Serial.begin(115200);
}

void loop() {
  float range = sonar.read() / 100.0;
  float range_kf = kf.updateEstimate(range);

  unsigned long t = millis() - last_time_pub;

  // проверяем нужно ли публиковать
  if (t > TIME_PUB_PERIOD_MS) {

    Serial.print("Range: ");
    Serial.print(range);
    Serial.print(", Range_filtered: ");
    Serial.println(range_kf);

    Serial.println("--------------------------------");
    last_time_pub = millis(); // фиксируем время последней публикации
  }
}
