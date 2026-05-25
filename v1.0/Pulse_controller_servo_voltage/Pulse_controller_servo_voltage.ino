#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PWM.h>
#include <Wire.h>

#define PULSE_PIN 9 // только 9 или 10 (расширенный 16-битный ШИМ)

#define FEED_HOLD_PIN     A1
#define FEED_RELEASE_PIN  A2
#define FEED_LED          13
#define CAP_VOLTAGE_SENSE_PIN  A7
//#define CAP_VOLTAGE_MIN_INTERRUPT_PIN 
//#define SET_MIN_CAP_VOLTAGE_PIN 
#define CURRENT_SENSE_PIN A6 
//#define CURRENT_MAX_INTERRUPT_PIN 
//#define SET_MAX_CURRENT_PIN 
#define VOLTAGE_SENSE_COEF    (5*68.0/3.3)/1024.f // Параметры делителя напряжения (5 В * 68 Ом/3.3 Ом)
#define VOLTAGE_SENSE_OFFSET  2             //
#define CURRENT_SENSE_COEF    5*40/1024.f   // Параметры датчика тока (acs758 100В)
#define CURRENT_SENSE_OFFSET  -100          // Параметры датчика тока
#define MIN_FREQ  1000      // в Герцах
#define MAX_FREQ  60000
#define STEP_FREQ 1000
#define MIN_PULSE_TIME 0.1f  // в микросекундах
#define MAX_PULSE_TIME 30.0f   // в микросекундах
#define LEVEL_2_PULSE_TIME 4.0f  // до 4 мкс прибавляться будет по +-0.1 мкс. После 4 мкс по +-1 мкс
#define LEVEL_1_STEP_PULSE_TIME 0.1f
#define LEVEL_2_STEP_PULSE_TIME 1.0f
#define MIN_SC_CURRENT  10    // минимальный ток короткого замыкания (А) 
#define MAX_SC_CURRENT  90    // максимальный ток короткого замыкания (А)
#define STEP_SC_CURRENT 2.5     // шаг изменения тока короткого замыкания (А) 

#define PULT_SCREEN_ADRESS  0X3C
#define PULT_SCREEN_WIDTH   128
#define PULT_SCREEN_HEIGHT  64
#define PULT_BUTTON_1_PIN   2
#define PULT_BUTTON_2_PIN   3
#define PULT_BUTTON_3_PIN   4
#define PULT_BUTTON_4_PIN   5
#define PULT_BUTTON_5_PIN   6
#define PULT_BUTTON_6_PIN   7
#define PULT_BUTTON_7_PIN   11
#define PULT_BUTTON_8_PIN   12


uint16_t calcPulseWidth(uint16_t, float);   // рассчет заполнения ШИМ по временным параметрам (выходной диапазон для 16-битного ШИМ: 0-65536)
void displayParam();  // вывод действующих параметров на дисплей
void displayShortCircuit();
void checkPultButtons();  // проверка кнопок
void setPulse(uint16_t);
void updateParam();
float currentFilter(float, float);
float voltageFilter(float, float);

bool isPulseEnable = false;
bool isShortCircuit = false;
int32_t freq = 5000;   // частота в Гц
float pulseTime = 1.5;  // время импульса в микросекундах

float maxVoltage = 85;   
float servoVoltage = 80;  // при падении до этого напряжения подается сигнал об остановке/замедлении
float voltage = 0;     
float maxPulseCurrent = 20;  // сверх - ток короткого зымыкания
float current = 0; 
float maxPulseEnergy = maxVoltage*maxPulseCurrent*pulseTime/1000.f;  // В мДж
uint16_t pulseWidth = calcPulseWidth(freq, pulseTime);

Adafruit_SSD1306 display(PULT_SCREEN_WIDTH, PULT_SCREEN_HEIGHT, &Wire, -1);

void setup(){
  InitTimersSafe(); 
  SetPinFrequency(PULSE_PIN, freq);   // начальная частота 5 кГц
  setPulse(0);

  display.begin(SSD1306_SWITCHCAPVCC, PULT_SCREEN_ADRESS);
  delay(2000);
  displayParam();
  
  pinMode(PULT_BUTTON_1_PIN, INPUT);
  pinMode(PULT_BUTTON_2_PIN, INPUT);
  pinMode(PULT_BUTTON_3_PIN, INPUT);
  pinMode(PULT_BUTTON_4_PIN, INPUT);
  pinMode(PULT_BUTTON_5_PIN, INPUT);
  pinMode(PULT_BUTTON_6_PIN, INPUT);
  pinMode(PULT_BUTTON_7_PIN, INPUT);
  pinMode(PULT_BUTTON_8_PIN, INPUT);

  pinMode(FEED_HOLD_PIN, OUTPUT);
  pinMode(FEED_RELEASE_PIN, OUTPUT);
  pinMode(FEED_LED, OUTPUT);
}

void loop(){
  float instantVoltage = (float)(analogRead(CAP_VOLTAGE_SENSE_PIN))*VOLTAGE_SENSE_COEF + VOLTAGE_SENSE_OFFSET;  // моментальное напряжение
  float instantCurrent = (float)(analogRead(CURRENT_SENSE_PIN))*CURRENT_SENSE_COEF + CURRENT_SENSE_OFFSET;      // моментальный ток

  voltage = voltageFilter(instantVoltage, 0.06);  // усредненные значения тока и напряжения
  current = currentFilter(instantCurrent, 0.06);

  //if ((instantCurrent >= maxPulseCurrent) || (voltage < maxVoltage*0.5)){   // если короткое замыкание
  if (instantCurrent >= maxPulseCurrent){   // если короткое замыкание
    isShortCircuit = true;  
    setPulse(0);
    moveHold(true);
    displayShortCircuit();    
  }

  moveHold(voltage < servoVoltage);   // останавливаем движение станка, если напряжение упало меньше напряжения слежения

  uint8_t pressedButton = checkButtons();
  switch(pressedButton){
    case 1:
      if (freq > MIN_FREQ) freq -= STEP_FREQ;
      freq = constrain(freq, MIN_FREQ, MAX_FREQ);
      break;
    case 2:
      freq += STEP_FREQ;
      freq = constrain(freq, MIN_FREQ, MAX_FREQ);
      break;
    case 3:
      if ((pulseTime > LEVEL_2_PULSE_TIME) && ((pulseTime-LEVEL_2_STEP_PULSE_TIME) <= LEVEL_2_PULSE_TIME)) pulseTime = LEVEL_2_PULSE_TIME;
      else if (pulseTime <= LEVEL_2_PULSE_TIME) pulseTime -= LEVEL_1_STEP_PULSE_TIME;
      else pulseTime -= LEVEL_2_STEP_PULSE_TIME;
      pulseTime = constrain(pulseTime, MIN_PULSE_TIME, MAX_PULSE_TIME);
      break;
    case 4:
      if ((pulseTime < LEVEL_2_PULSE_TIME) && ((pulseTime+LEVEL_1_STEP_PULSE_TIME) >= LEVEL_2_PULSE_TIME)) pulseTime = LEVEL_2_PULSE_TIME;
      else if (pulseTime < LEVEL_2_PULSE_TIME) pulseTime += LEVEL_1_STEP_PULSE_TIME;
      else pulseTime += LEVEL_2_STEP_PULSE_TIME;
      pulseTime = constrain(pulseTime, MIN_PULSE_TIME, MAX_PULSE_TIME);
      break;
    case 5:   
      maxPulseCurrent -= STEP_SC_CURRENT;
      maxPulseCurrent = constrain(maxPulseCurrent, MIN_SC_CURRENT, MAX_SC_CURRENT);
      break;
    case 6:     
      maxPulseCurrent += STEP_SC_CURRENT;
      maxPulseCurrent = constrain(maxPulseCurrent, MIN_SC_CURRENT, MAX_SC_CURRENT);
      break;
    case 7:   
      isPulseEnable = false;
      isShortCircuit = false;  
      setPulse(0);
      break;
    case 8:     
      isPulseEnable = true;
      isShortCircuit = false;  
      maxVoltage = voltage; 
      break;
    default:
      break;
  }

  if (pressedButton != 0) {
    //Serial.println(pressedButton);
    SetPinFrequency(PULSE_PIN, freq);   // начальная частота 5 кГц
    updateParam();
    setPulse(pulseWidth);
    displayParam();
  }
}

uint16_t calcPulseWidth(uint16_t frequency, float pulseT){
  float temp = 65536*(pulseT*frequency/1000000);
  return constrain(temp, 0, 65536);
}

uint8_t checkButtons(){
  static uint32_t debouncingTimer = 0; 
  static bool buttonPos[8][4] = {false, false, false, false,    // По индексам:
                                 false, false, false, false,    // [0] - событие изменения положения кнопки (при прочтении - стереть!) 
                                 false, false, false, false,    // [1] - действующее положение кнопки
                                 false, false, false, false,    // [2] - прошлое положение кнопки
                                 false, false, false, false,    // [3] - текущее положение кнопки
                                 false, false, false, false,
                                 false, false, false, false,
                                 false, false, false, false};
  

  if ((millis() - debouncingTimer) > 50){  // таймер антидребезга
    buttonPos[0][3] = digitalRead(PULT_BUTTON_1_PIN);   // читаем кнопки
    buttonPos[1][3] = digitalRead(PULT_BUTTON_2_PIN);
    buttonPos[2][3] = digitalRead(PULT_BUTTON_3_PIN);
    buttonPos[3][3] = digitalRead(PULT_BUTTON_4_PIN);
    buttonPos[4][3] = digitalRead(PULT_BUTTON_5_PIN);
    buttonPos[5][3] = digitalRead(PULT_BUTTON_6_PIN);
    buttonPos[6][3] = digitalRead(PULT_BUTTON_7_PIN);
    buttonPos[7][3] = digitalRead(PULT_BUTTON_8_PIN);
    //Serial.println(buttonPos[0][3]);

    for (uint8_t i=0; i<8; i++){
      if (buttonPos[i][2] == buttonPos[i][3]){   // если два последних измерения кнопка была нажата/отжата (защита от дребезга)
        if (buttonPos[i][1] != buttonPos[i][2]) buttonPos[i][0] = true; // записываем событие: положение кнопки было изменено изменено 
        buttonPos[i][1] = buttonPos[i][2];  // изменяем действуещее положение кнопки 
      }
      buttonPos[i][2] = buttonPos[i][3];  // изменяем прошлое положение кнопки для следующего цикла
    }
    debouncingTimer = millis();
  }

  //for (int i = 0; i < 4; i++) Serial.print(buttonPos[0][i]);
  //Serial.println(" ");

  uint8_t pressedButton = 0; 
  for (uint8_t i=0; i<8; i++){
    if (buttonPos[i][0] && buttonPos[i][1]) pressedButton = i+1;
    buttonPos[i][0] = false;
  }
  return pressedButton;   // возвращаем нажатую кнопку, если такая была
}

void updateParam(){
  maxPulseEnergy = maxVoltage*maxPulseCurrent*pulseTime/1000.f;  // В мДж
  pulseWidth = calcPulseWidth(freq, pulseTime);
}

void displayParam(){
  //static char strOut[75];
  //sprintf(strOut, ("Frequency: %u Hz\n pulse time: %F us\n pulse energy: %F mJ\n"), freq, pulseTime, maxPulseEnergy);
  //sprintf(strOut, ("f:%u Hz\nt:%.1f us\nE:%.1f mJ\n"), freq, 0.f, 0.f);
  display.clearDisplay();
  display.setTextSize(2); 
  display.setTextColor(WHITE); 
  display.setRotation(2);
  display.setCursor(0, 0); 
  display.print("f:");
  display.print(freq);
  display.print(" Hz\nt:");
  display.print(pulseTime, 1);
  display.print(" us\nI:");
  display.print(maxPulseCurrent, 1);
  display.print(" A\nE:");
  display.print(maxPulseEnergy, 1);
  display.print(" mJ");

  if (isPulseEnable && (!isShortCircuit)) display.fillRoundRect(115, 35, 12, 12, 3, SSD1306_WHITE);
  else display.drawRoundRect(115, 35, 12, 12, 3, SSD1306_WHITE);

  display.display(); 
}

void displayShortCircuit(){
  display.clearDisplay();
  display.setTextSize(2); 
  display.setTextColor(WHITE); 
  display.setRotation(2);
  display.setCursor(0, 0); 
  display.print("short\ncircuit");
  display.display(); 
}

void setPulse(uint16_t pulse){
  if (isPulseEnable && (!isShortCircuit)) pwmWriteHR(PULSE_PIN, pulse);
  else pwmWriteHR(PULSE_PIN, 0);
}

float voltageFilter(float newVoltage, float Kp){
  static float oldVoltage = 0.0;
  oldVoltage = (1.f - Kp)*oldVoltage + Kp*newVoltage;
  return oldVoltage;
}

float currentFilter(float newCurrent, float Kp){
  static float oldCurrent = 0.0;
  oldCurrent = (1.f - Kp)*oldCurrent + Kp*newCurrent;
  return oldCurrent;
}

void moveHold(bool state){
  if (state) {
    digitalWrite(FEED_HOLD_PIN, LOW);
    digitalWrite(FEED_RELEASE_PIN, HIGH);
    digitalWrite(FEED_LED, HIGH);
  }
  else if (!isShortCircuit)
  {
    digitalWrite(FEED_RELEASE_PIN, LOW);
    digitalWrite(FEED_HOLD_PIN, HIGH);
    digitalWrite(FEED_LED, LOW);
  }
}


