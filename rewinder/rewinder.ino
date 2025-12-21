#include "FastAccelStepper.h"
#include "HX711.h"

#define STRAIN_DATA_PIN  2
#define STRAIN_CLOCK_PIN 3
#define FIRST_STEPPER_DIR_PIN   8
#define FIRST_STEPPER_STEP_PIN  9
#define SECOND_STEPPER_DIR_PIN  11
#define SECOND_STEPPER_STEP_PIN 10
#define WATER_LEVEL_RELAY_PIN   4
#define FIRST_PUMP_PWM_PIN      5
#define SECOND_PUMP_PWM_PIN     6

#define STRAIN_LEVEL_0    30    // нулевая грацица в граммах - ниже нее - проволока порвана/не протянута
#define STRAIN_LEVEL_1    150   // грацица в граммах - ниже нее - проволока совсем не натянута
#define STRAIN_LEVEL_2    550   // целевая сила натяжения проволоки в граммах
#define STRAIN_LEVEL_3    700   // перегруз
#define STRAIN_DELTA      10    // погрешность натяжения в граммах +-
#define SPEED_DELTA       100

HX711 strainSensor;
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *firstStepper = NULL;
FastAccelStepper *secondStepper = NULL;

float baseSpeed = 350;  // 500 шагов в секунду
uint8_t inPumpPwm = 127;
uint8_t outPumpPwm = 255;

uint32_t strainScale = 579.635498;
float strainOffset= 91019;

float skp = 2.0;
float skd = 0.0;
float ski = 0.03;

float strain = 0;
float speed = 0; 
uint32_t timer = 0;
 
void setup()
{
  engine.init();
  Serial.begin(115200);
  strainSensor.begin(STRAIN_DATA_PIN, STRAIN_CLOCK_PIN);
 
  strainSensor.set_scale(strainScale); 
  strainSensor.set_offset(strainOffset);
  strainSensor.tare();

  firstStepper = engine.stepperConnectToPin(FIRST_STEPPER_STEP_PIN);
  secondStepper = engine.stepperConnectToPin(SECOND_STEPPER_STEP_PIN);

  firstStepper->setDirectionPin(FIRST_STEPPER_DIR_PIN);
  firstStepper->setSpeedInHz(baseSpeed);  
  firstStepper->setAcceleration(1500);    
  
  secondStepper->setDirectionPin(SECOND_STEPPER_DIR_PIN);
  secondStepper->setSpeedInHz(baseSpeed);   
  secondStepper->setAcceleration(1500);    
  
  pinMode(FIRST_PUMP_PWM_PIN, OUTPUT);
  pinMode(SECOND_PUMP_PWM_PIN, OUTPUT);
  pinMode(WATER_LEVEL_RELAY_PIN, INPUT_PULLUP);
  timer = millis();
}
 
void loop()
{
  if(strainSensor.is_ready()){
    strain = strainSensor.get_units(0);   // максимальная частота HX711 по умолчанию - 10Гц, что катастрафически мало. Можно поднять до 80Гц (подрезав дорожку на плате), 
    if(strain < 0) strain = 0;            // но этого все равно мало. Однако, пока будем работать с чем есть...
  }                                       

  if((strain > STRAIN_LEVEL_3) || (strain < STRAIN_LEVEL_0)){    // проволока порвана/не протянута или перегруз - останавливаем перемотку
    firstStepper -> stopMove();
    secondStepper -> stopMove();

    pid(0, 0, 0, 0, 0, 0, 0, 0);
    timer = micros();
  }
  else if(strain <= STRAIN_LEVEL_1) {    // проволока совсем не натянута - крутим только одну катушку
    firstStepper -> runBackward();
    secondStepper -> stopMove();

    pid(0, 0, 0, 0, 0, 0, 0, 0);
    timer = micros();
  } 
  else if(strain > STRAIN_LEVEL_1) {    // проволока находится в пределе регулирования по натяжению
    //float err = strain - STRAIN_LEVEL_2;    // релейный регулятор с медленными ускорениями шаговиков. Лень возиться с нелинейностью и PID регулятором
    float dt = (float)(micros() - timer)/1000000.f;
    float speedOffset = -pid(strain, STRAIN_LEVEL_2, skp, skd, ski, dt, -SPEED_DELTA, SPEED_DELTA);
    //Serial.print(strain);
    //Serial.print(",");
    //Serial.print(dt*1000000);
    //Serial.print(",");
    //Serial.println(baseSpeed + speedOffset);
    secondStepper->setSpeedInHz(baseSpeed + speedOffset);

    //if (abs(err) > STRAIN_DELTA)  secondStepper->setSpeedInHz(baseSpeed + SPEED_DELTA * err/abs(err));  // одна катушка отстает/опережает
    //else  secondStepper->setSpeedInHz(baseSpeed);
    timer = micros();

    firstStepper -> runBackward(); 
    secondStepper -> runForward();
  }

  digitalWrite(FIRST_PUMP_PWM_PIN, HIGH);  
  if (digitalRead(WATER_LEVEL_RELAY_PIN)) digitalWrite(SECOND_PUMP_PWM_PIN, HIGH);
  else digitalWrite(SECOND_PUMP_PWM_PIN, LOW);
                                                  
  //Serial.println(strain);
    //float freq = 1.f/(float)(millis()-timer);
    //timer = millis();
    //Serial.println(freq*1000);
  //delay(100);
}


float pid(float input, float trgt, float kp, float ki, float kd, float dt, float outMin, float outMax){
  static float integral = 0.f;    // храним значение суммы интегральной компоненты
  static float lastError = 0.f;   // и предыдущую ошибку регулирования для дифференциирования

  if(dt == 0.f){
    integral = 0.f;
    lastError = 0.f; 
    return 0.f;
  }
  
  float error = trgt - input;   // получаем ощибку регулирования
  float tIntegral = integral + ki * error * dt;  // пересчитываем интегральную сумму
  float diff = (error - lastError) / dt;  // ищем дифференциал
  lastError = error;

  float out = kp * error + tIntegral + kd * diff;   // считаем и выводим результат
  if((out < outMin) && (error < 0)) return outMin;
  else if ((out > outMax) && (error > 0)) return outMax;

  integral = tIntegral;
  return out;
}