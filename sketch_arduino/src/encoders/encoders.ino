#define LEFT_ENCODER_INTERRUPT_NB   0  // номер прерывания
#define RIGHT_ENCODER_INTERRUPT_NB  1  // номер прерывания

#define TIME_PUB_PERIOD_MS        1000  // интервал публикации значений

#define WHEEL_DIAMETER            0.084     // диаметр колеса в метрах
#define WHEEL_IMPULSE_COUNT       1225.0    // количество импульсов на оборот колеса

#define COUNT_MOTORS              2         // количество моторов

// стороны робота
#define LEFT                      0
#define RIGHT                     1

unsigned long last_time; // время последней публикации

float enc_count[COUNT_MOTORS] = {0.0, 0.0};      // счетчик для левого колеса

void setup() {
  // инициализация прерываний
  attachInterrupt(LEFT_ENCODER_INTERRUPT_NB,  callBackInterruptLeftEncoder, CHANGE);
  attachInterrupt(RIGHT_ENCODER_INTERRUPT_NB, callBackInterruptRightEncoder, CHANGE);

  Serial.begin(115200);
}

void loop(){
  unsigned long time = millis() - last_time;

  // проверяем нужно ли менять режим
  if (time > TIME_PUB_PERIOD_MS) {
    Serial.print("L_count: ");
    Serial.print(enc_count[LEFT]);
    Serial.print(", R_count: ");
    Serial.println(enc_count[RIGHT]);

    Serial.print("L_meters: ");
    Serial.print(impulse2meters(enc_count[LEFT]));
    Serial.print(", R_meters: ");
    Serial.println(impulse2meters(enc_count[RIGHT]));

    Serial.println("--------------------------------");
    last_time = millis(); // фиксируем время последней публикации
  }

}

// обработчик прерывания для левого колеса
inline void callBackInterruptLeftEncoder() {
  enc_count[LEFT]++;
}

// обработчик прерывания для правого колеса
inline void callBackInterruptRightEncoder() {
  enc_count[RIGHT]++;
}

// преобразование импульсов в метры
inline float impulse2meters(float x) {
  return (x / WHEEL_IMPULSE_COUNT) * M_PI * WHEEL_DIAMETER;
}
