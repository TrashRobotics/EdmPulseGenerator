#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PWM.h>
#include <Wire.h>
#include <Keypad.h>

#define PULSE_PIN 9 // только 9 или 10 (расширенный 16-битный ШИМ)
#define SC_LED_PIN   A3
#define HOLD_PIN 0
#define RESUME_PIN 1

//#define CAP_CAPACITY  100.f   // ескость разрядного конденсатора (в мкФ)
//#define CAP_VOLTAGE_SENSE_PIN  A7   // напряжения, измеряемое на конденсаторе
#define TO_CAP_CURRENT_SENSE_PIN A6  // средний ток (измеряем ток идущий от блока питания до конденсатора) /+предположительно, можно использовать вместо следящего напряжения/ 
#define VOLTAGE_SENSE_COEF    (5*68.0/3.3)/1024.f // Параметры делителя напряжения (5 В * 68 Ом/3.3 Ом)
#define VOLTAGE_SENSE_OFFSET  2             // калибровка ошибки измерения напряжения
#define TO_CAP_CURRENT_SENSE_COEF    (5/1024.f)/0.185   // Параметры датчика тока (acs712 5A)
#define TO_CAP_CURRENT_SENSE_OFFSET  -2.5/0.185 - 0.15         // Параметры датчика тока
#define MAX_SERVO_CURRENT 4 // ограниченик максимального тока блока питания (в Амперах)
#define MIN_FREQ  1000      // в Герцах
#define MAX_FREQ  80000
#define STEP_FREQ 1000
#define MIN_PULSE_TIME 0.1f  // в микросекундах
#define MAX_PULSE_TIME 30.0f   // в микросекундах
#define LEVEL_2_PULSE_TIME 4.0f  // до 4 мкс прибавляться будет по +-0.1 мкс. После 4 мкс по +-1 мкс
#define LEVEL_1_STEP_PULSE_TIME 0.1f
#define LEVEL_2_STEP_PULSE_TIME 1.0f
#define MIN_SERVO_CURRENT  0.25     // в Амперах
#define MAX_SERVO_CURRENT  5.0
#define STEP_SERVO_CURRENT 0.25
#define MIN_SC_PERCENT  0.05     // в процентах
#define MAX_SC_PERCENT  0.95
#define STEP_SC_PERCENT 0.01
#define A_CURRENT_PIN_ENABLE 8    // пины включающие дополнительные резисторы (увеличивающие ток)
#define B_CURRENT_PIN_ENABLE 11   //
#define C_CURRENT_PIN_ENABLE 12   //
#define D_CURRENT_PIN_ENABLE 13   //
#define E_CURRENT_PIN_ENABLE A0   //
#define F_CURRENT_PIN_ENABLE A1   //
#define G_CURRENT_PIN_ENABLE A2    //

#define PULT_SCREEN_ADRESS  0X3C
#define PULT_SCREEN_WIDTH   128
#define PULT_SCREEN_HEIGHT  64
#define PULT_SCREEN_ROWS    4 
#define PULT_SCREEN_COLS    2

float resistanceArray[] = {10.f, 10.f, 10.f, 10.f, 10.f, 10.f,}; // массив токоограничивающих резисторов (включаться будут по порядку) (в Омах)
float halfResistance = 20.f;  // сопротивление дополнительного резистора для уменьшения шага выходного тока (в Омах)

char hKeys[PULT_SCREEN_ROWS][PULT_SCREEN_COLS] = {
  {'f','F'},
  {'p','P'},
  {'i','I'},
  {'e','E'}
};
uint8_t rowPins[PULT_SCREEN_ROWS] = {2, 3, 4, 5}; 
uint8_t colPins[PULT_SCREEN_COLS] = {6, 7};  

uint16_t calcPulseWidth(uint16_t, float);   // рассчет заполнения ШИМ по временным параметрам (выходной диапазон для 16-битного ШИМ: 0-65536)
void displayParam();  // вывод действующих параметров на дисплей
void displayShortCircuit();
void setPulse(uint16_t);
void updateParam();
float currentFilter(float, float);
float voltageFilter(float, float);
bool checkSC_FSM();

static enum {     // состояния конечного автомата движения/остановки механики станка
    F_FLOAT,	// 
    F_HOLD,
    F_RESUME,	// 
} feedState;

static enum {   // состояния конечного автомата настроек (два листа)
    C_FIRST,	// 
    C_SECOND
} configState;

bool isHardHold = false;  // ручная остановка станка

// Задаваемые значения (через пульт, вручную или через измерения) //
int32_t freq = 20000;   // частота в Гц
float pulseTime = 3;  // время импульса в микросекундах

float maxCapVoltage = 85;  // напряжение, выставленное на блоке питания (в вольтах) (можно измерять делителем, а не задавать).
float maxToCapCurrent = 2; // ограничение тока, выставленное на блоке питания (в Амперах) 

// следящее напряжение. При падении до этого напряжения подается сигнал об остановке/замедлении 
//float servoVoltage[] = {0.9*maxVoltage, 0.6*maxVoltage};  // параметры следящего напряжения. (до 1): выше этого напряжения - цепь открыта (искры нет); (между 1-2): рабочий дипазон напряжения (нормальная искра); (после 2): напряжение короткого замыкания.    

// следящий ток (экспериментальное).Тоже самое, что и следящее напряжение, но тольно - это ток от источника питания 
float servoCurrent[] = {0.15, 4.2};  // (в Амперах). Параметры следящего напряжения. (задаются вручную или на пульте)
// (до servoCurrent[0]): меньше этого тока - цепь открыта (искры нет), можно ехать ускоренно;
// (между servoCurrent[0] и servoCurrent[1]): рабочий дипазон тока (нормальная искра), скорость движения можно регулировать линейно по значению тока из диапазона {0.15, 0.85} -> {100% подача, 0% подача} 
// (после servoCurrent[1]): ток короткого замыкания. Движения станка останавливается. Генератор несколько раз отключается и перезаряжает конденсатор (защита от просадки напряжения->неспособности ударной ионизации->неспособности поджечь искру) и каждый проверяет значение тока  (Проверка от статистических КЗ). Если значение не меняется, то генератор падает в состояние ожидания устранения КЗ
float shortCircuitPercent = 0.45;  // процеент коротких замыканий за секунду, после чего генератор уходит в защиту 


// НЕ задаваемые значения
bool isPulseEnable = false;
bool isShortCircuit = false;
bool isCheckSC = false;

uint32_t scPulseWindow[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   // битовое скользящее окно коротких замыканий (битовые операции быстрее). Будем считать количество КЗ, а потом их процентное соотношение к общему числу выборок.  
uint8_t scPulseWindowSize = 10;

uint16_t shortCircuitCounter = 0; // счетчик коротких замыканий
uint16_t shortCircuitCounterNorm = 0; // счетчик общего количества измерений
uint32_t shortCircuitTimer = 0;
// время отключения генератора до полной перезарядки конденсатора t = 5*R*C (нужно, чтоб генератор не захлебывался). (позволяет эмпирически предугадать восстановление напряжения, без его прямого измерения) 
//uint32_t shortCircuitDelay = 5*CAP_CAPACITY*maxCapVoltage/maxToCapCurrent/1000.f;  // (в миллисекундах) но т.к. у нас не RC-генератор, а импульсный источник тока, то задержку можно сильно уменьшить.

int8_t resistanceCounter = 1;  // вспомогательный счетчик переключающий выходные токоограничивающие ресисторы 
float pulsePeakCurrentLimit = 4;   // примерное (максимальное) ограничение пикового тока на разряднике, пересчитанное на основе напряжения и сопротивления ограничивающих резисторов (в Амперах)

float capVoltage = 0; // текущие средние значения тока и напряжения(на конденсаторе). 
float averageCurrent = 0;

uint8_t resistanceNum = 0; // количество токоограничивающих резисторов
uint32_t ledBlinkTimer = 0;
uint32_t ledBlinkDelay = 0;

bool isCheckConfigListChange = false;
uint32_t configListChangeTimer = 0;

float maxPulseEnergy = maxCapVoltage*pulsePeakCurrentLimit*pulseTime/1000.f;  // В мДж
uint16_t pulseWidth = calcPulseWidth(freq, pulseTime);


Adafruit_SSD1306 display(PULT_SCREEN_WIDTH, PULT_SCREEN_HEIGHT, &Wire, -1);
Keypad customKeypad = Keypad(makeKeymap(hKeys), rowPins, colPins, PULT_SCREEN_ROWS, PULT_SCREEN_COLS); 


void setup(){
  resistanceNum = sizeof(resistanceArray)/sizeof(float);  // количиство резисторов в массиве
  resistanceNum = resistanceNum*2 + 1; // дополнительный резистор (halfResistance) увеличиает разрядность комбинаций включения резисторов на (*2) (+1) - это сам доп. резитор. (подробнее см. видео) 

  InitTimersSafe(); 
  SetPinFrequency(PULSE_PIN, freq);   // начальная частота 5 кГц
  setPulse(0);

  pinMode(A_CURRENT_PIN_ENABLE, OUTPUT);
  pinMode(B_CURRENT_PIN_ENABLE, OUTPUT);
  pinMode(C_CURRENT_PIN_ENABLE, OUTPUT);
  pinMode(D_CURRENT_PIN_ENABLE, OUTPUT);
  pinMode(E_CURRENT_PIN_ENABLE, OUTPUT);
  pinMode(F_CURRENT_PIN_ENABLE, OUTPUT);
  pinMode(G_CURRENT_PIN_ENABLE, OUTPUT);
  pinMode(SC_LED_PIN, OUTPUT);
  pinMode(RESUME_PIN, OUTPUT);
  pinMode(HOLD_PIN, OUTPUT);
  digitalWrite(RESUME_PIN, HIGH);
  digitalWrite(HOLD_PIN, HIGH);
  _softResume();  

  display.begin(SSD1306_SWITCHCAPVCC, PULT_SCREEN_ADRESS);
  delay(2000);
  displayParam();
}

void loop(){
  // напряжение нужно мерять изолированно, иначе ардуинка глючит и ребутится (одна даже сгорела)
  float instantVoltage = 85; //(float)(analogRead(VOLTAGE_SENSE_PIN))*VOLTAGE_SENSE_COEF + VOLTAGE_SENSE_OFFSET;  // моментальное напряжение
  //float instantCurrent = (float)(analogRead(CURRENT_SENSE_PIN))*CURRENT_SENSE_COEF + CURRENT_SENSE_OFFSET;      // моментальный ток
  capVoltage = voltageFilter(instantVoltage, 0.06);  // усредненные значения напряжения
  averageCurrent = (float)(analogRead(TO_CAP_CURRENT_SENSE_PIN))*TO_CAP_CURRENT_SENSE_COEF + TO_CAP_CURRENT_SENSE_OFFSET; // среднее значение тока (меряем ток от источника питания)
  averageCurrent = -averageCurrent;   // если перепутана полярность датчика тока
  averageCurrent = currentFilter(averageCurrent, 0.01);   // фильтр нижних частот

  if (!isCheckSC && !isShortCircuit) {   // если не идет проверка на короткое замыкание
    if (averageCurrent <= servoCurrent[0]){   // если схема открыта (нет искы)
      // TODO: двигпемся ускоренно
      
    }
    else if ((servoCurrent[0] < averageCurrent) && (averageCurrent <= servoCurrent[1])) { 
      
      // TODO: двигаемся в рабочем режиме со скоростью пропорциональной averageCurrent в диапазоне {servoCurrent[0], servoCurrent[1]} = {100% подача, 0% подача} 
    }
    else if ((averageCurrent > servoCurrent[1])){  // было поймано КЗ
      isCheckSC = true; 
      shortCircuitCounter = 0;
      shortCircuitCounterNorm = 0;
      blink(150);
      //displayLog(averageCurrent, isCheckSC);
      shortCircuitTimer = millis();
    }
  }

  //displayLog(averageCurrent, (averageCurrent > servoCurrent[1]));

  if (isCheckSC){
    if((millis() - shortCircuitTimer) < 1000){  // считаем количество измерений с КЗ за одну секунду (фильтрация статистических КЗ)
      if(averageCurrent > servoCurrent[1]) {
        shortCircuitCounter++;
        blink(150);
      }
      shortCircuitCounterNorm++;
    }
    else 
    {isCheckSC = false;}

    if (shortCircuitCounterNorm > 400) {
      float percent = (float)shortCircuitCounter/shortCircuitCounterNorm;   // процентное соотношение измерений с КЗ к общему количеству выборок
      //displayLog((float)shortCircuitCounter/shortCircuitCounterNorm, shortCircuitCounter);
      if((percent >= shortCircuitPercent))
      {
        isShortCircuit = true;  
        isCheckSC = false;
        shortCircuitCounter = 0;
        shortCircuitCounterNorm = 0;
        setPulse(0);
        displayShortCircuit(); 
      }
    }
  }

  updateFeed_FSM();

  //displayLog(averageCurrent, isCheckSC);

  //checkSC_FSM();  // конечный автомат: проверка на КЗ и защита от захдебывания
  if (isCheckSC || isShortCircuit)  // если идет проверка на статистическое КЗ или появилось неустранимое КЗ
  {
    blink(150);
  }

  updateLedState();

  if (isCheckConfigListChange) {
    if (customKeypad.findInList('e') == -1) isCheckConfigListChange = false;
    if ((millis() - configListChangeTimer) > 2000){   // если зажимаем кнопку больше 2 секунд - настройки переключаются
      if (configState == C_SECOND) configState = C_FIRST;
      else configState = C_SECOND;
      isCheckConfigListChange = false;
      displayParam();
    }
  }

  //displayLog(isCheckConfigListChange, customKeypad.findInList('e'));

  char pressedButton = customKeypad.getKey();
  switch(pressedButton){
    case 'f':
      if (configState == C_FIRST) {
        if (freq > MIN_FREQ) freq -= STEP_FREQ;
        freq = constrain(freq, MIN_FREQ, MAX_FREQ);
      } 
      break;
    case 'F':
      if (configState == C_FIRST) {
        freq += STEP_FREQ;
        freq = constrain(freq, MIN_FREQ, MAX_FREQ);
      }
      break;
    case 'p':
      if (configState == C_FIRST) {
        if ((pulseTime > LEVEL_2_PULSE_TIME) && ((pulseTime-LEVEL_2_STEP_PULSE_TIME) <= LEVEL_2_PULSE_TIME)) pulseTime = LEVEL_2_PULSE_TIME;
        else if (pulseTime <= LEVEL_2_PULSE_TIME) pulseTime -= LEVEL_1_STEP_PULSE_TIME;
        else pulseTime -= LEVEL_2_STEP_PULSE_TIME;
        pulseTime = constrain(pulseTime, MIN_PULSE_TIME, MAX_PULSE_TIME);
      } else if (configState == C_SECOND) {
        if (shortCircuitPercent > MIN_SC_PERCENT) shortCircuitPercent -= STEP_SC_PERCENT;
        shortCircuitPercent = constrain(shortCircuitPercent, MIN_SC_PERCENT, MAX_SC_PERCENT);        
      }
      break;
    case 'P':
      if (configState == C_FIRST) {
        if ((pulseTime < LEVEL_2_PULSE_TIME) && ((pulseTime+LEVEL_1_STEP_PULSE_TIME) >= LEVEL_2_PULSE_TIME)) pulseTime = LEVEL_2_PULSE_TIME;
        else if (pulseTime < LEVEL_2_PULSE_TIME) pulseTime += LEVEL_1_STEP_PULSE_TIME;
        else pulseTime += LEVEL_2_STEP_PULSE_TIME;
        pulseTime = constrain(pulseTime, MIN_PULSE_TIME, MAX_PULSE_TIME);
      } else if (configState == C_SECOND) {
        shortCircuitPercent += STEP_SC_PERCENT;
        shortCircuitPercent = constrain(shortCircuitPercent, MIN_SC_PERCENT, MAX_SC_PERCENT);      
      }
      break;
    case 'i': 
      if (configState == C_FIRST) {  
        resistanceCounter--; 
        resistanceCounter = constrain(resistanceCounter, 0, resistanceNum);
      } else if (configState == C_SECOND) {
        if (servoCurrent[1] > MIN_SERVO_CURRENT) servoCurrent[1] -= STEP_SERVO_CURRENT;
        servoCurrent[1] = constrain(servoCurrent[1], MIN_SERVO_CURRENT, MAX_SERVO_CURRENT);
      }
      break;
    case 'I':     
      if (configState == C_FIRST) {
        resistanceCounter++; 
        resistanceCounter = constrain(resistanceCounter, 0, resistanceNum);
      } else if (configState == C_SECOND) {
        servoCurrent[1] += STEP_SERVO_CURRENT;
        servoCurrent[1] = constrain(servoCurrent[1], MIN_SERVO_CURRENT, MAX_SERVO_CURRENT);
      }
      break;
    case 'e':   
      _hardHold();
      isPulseEnable = false;
      isShortCircuit = false;  
      isCheckSC = false; 
      shortCircuitCounter = 0;
      shortCircuitCounterNorm = 0;
      setPulse(0);
      isCheckConfigListChange = true;
      configListChangeTimer = millis();
      break;
    case 'E':   
      _softResume();  
      isPulseEnable = true;
      isShortCircuit = false;  
      isCheckSC = false; 
      shortCircuitCounter = 0;
      shortCircuitCounterNorm = 0;
      maxCapVoltage = capVoltage; 
      break;
    default:
      break;
  }

  if (pressedButton != 0) {
    //Serial.println(pressedButton);
    SetPinFrequency(PULSE_PIN, freq);   // начальная частота 5 кГц
    updateParam();
    setPulse(pulseWidth);
    setCurrentLimit(resistanceCounter);
    displayParam();
  }
}

uint16_t calcPulseWidth(uint16_t frequency, float pulseT){
  float temp = 65536*(pulseT*frequency/1000000);
  return constrain(temp, 0, 65536);
}

void updateParam(){
  float allResistanceInv = 0;
  for(uint8_t i = 0; i < (resistanceCounter/2); i++) allResistanceInv += 1/resistanceArray[i];
  allResistanceInv += (resistanceCounter % 2)*1/halfResistance; 

  if (allResistanceInv <= 0) pulsePeakCurrentLimit = 0;
  else pulsePeakCurrentLimit = maxCapVoltage*allResistanceInv; 
  //currentLimit = (resistanceCounter % 2)*maxVoltage/HALF_OUT_CURRENT_LIMIT_RESISTANCE + (resistanceCounter/2)*maxVoltage/GEN_OUT_CURRENT_LIMIT_RESISTANCE;
  maxPulseEnergy = maxCapVoltage*pulsePeakCurrentLimit*pulseTime/1000.f;  // В мДж 
  pulseWidth = calcPulseWidth(freq, pulseTime);
  //shortCircuitDelay = 5*CAP_CAPACITY*maxCapVoltage/maxToCapCurrent/1000.f;
}

void displayParam(){
  //static char strOut[75];
  //sprintf(strOut, ("Frequency: %u Hz\n pulse time: %F us\n pulse energy: %F mJ\n"), freq, pulseTime, maxPulseEnergy);
  //sprintf(strOut, ("f:%u Hz\nt:%.1f us\nE:%.1f mJ\n"), freq, 0.f, 0.f);
  switch(configState){
    case C_FIRST:
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
      display.print(pulsePeakCurrentLimit, 1);
      display.print(" A\nE:");
      display.print(maxPulseEnergy, 1);
      display.print(" mJ");
      break;
    case C_SECOND:
      display.clearDisplay();
      display.setTextSize(2); 
      display.setTextColor(WHITE); 
      display.setRotation(2);
      display.setCursor(0, 0); 
      //display.print(" \n:");
      display.print("Is:");
      display.print(servoCurrent[1], 1);
      display.print(" A\nsc:");
      display.print((uint8_t)(shortCircuitPercent*100));
      display.print(" %");
      break;
  }

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

void setCurrentLimit(int8_t counter){
  // включаем попеременно резисторы в зависимости от счетчика. Резисторы установлены по следующей схеме:
  // |    A       B       C       D       E       F       G    |
  // |  20 Ом   10 ОМ   10 ОМ   10 ОМ   10 ОМ   10 ОМ   10 ОМ  |
  int8_t mod = counter/2;
  digitalWrite(A_CURRENT_PIN_ENABLE, counter % 2);
  digitalWrite(B_CURRENT_PIN_ENABLE, 0 < mod--);
  digitalWrite(C_CURRENT_PIN_ENABLE, 0 < mod--);
  digitalWrite(D_CURRENT_PIN_ENABLE, 0 < mod--);
  digitalWrite(E_CURRENT_PIN_ENABLE, 0 < mod--);
  digitalWrite(F_CURRENT_PIN_ENABLE, 0 < mod--);
  digitalWrite(G_CURRENT_PIN_ENABLE, 0 < mod--);
}

float voltageFilter(float newVoltage, float Kp){
  static float oldVoltage = 0.0;
  oldVoltage = (1.f - Kp)*oldVoltage + Kp*newVoltage;
  return oldVoltage;
}

void blink(uint32_t btime){
  ledBlinkTimer = millis();
  ledBlinkDelay = btime;
}

void updateLedState(){
  if (!ledBlinkDelay) return;

  if ((millis() - ledBlinkTimer) > ledBlinkDelay){
    digitalWrite(SC_LED_PIN, LOW);
    ledBlinkDelay = 0;
  }
  else{
    digitalWrite(SC_LED_PIN, HIGH);
  }
}

float currentFilter(float newCurrent, float Kp){
  static float oldCurrent = 0.0;
  oldCurrent = (1.f - Kp)*oldCurrent + Kp*newCurrent;
  return oldCurrent;
}


void displayLog(float param1, int32_t param2){
  static float prm1 = 0;
  static int32_t prm2 = 0;
  static uint32_t logTimer = 0;

  prm1 = param1;
  prm2 = param2;
  if ((millis() - logTimer) > 500){
    display.clearDisplay();
    display.setTextSize(2); 
    display.setTextColor(WHITE); 
    display.setRotation(2);
    display.setCursor(0, 0); 
    display.print("log:\n");
    display.print("p1: ");
    display.print(prm1, 2);
    display.print("\np2: ");
    display.print(prm2);
    display.display(); 
    logTimer = millis();
  }
}


bool updateFeed_FSM(){
  static uint32_t resumeTimer = 0;

  if (isCheckSC || isShortCircuit || isHardHold) feedState = F_HOLD;

  switch(feedState)
  {
    case F_FLOAT:
      digitalWrite(RESUME_PIN, HIGH);
      digitalWrite(HOLD_PIN, HIGH);
      return false;

    case F_HOLD:
      digitalWrite(HOLD_PIN, LOW);
      digitalWrite(RESUME_PIN, HIGH);
      if (!isCheckSC && !isShortCircuit && !isHardHold) {
        feedState = F_RESUME;
        resumeTimer = millis();
      }
      return false;

    case F_RESUME:
      if (millis() - resumeTimer < 2000){
        digitalWrite(RESUME_PIN, LOW);
        digitalWrite(HOLD_PIN, HIGH);
      }
      else feedState = F_FLOAT;
    return false;
  }
}

void _hardHold(){
  isHardHold = true;
  digitalWrite(HOLD_PIN, LOW);
  digitalWrite(RESUME_PIN, HIGH);
  feedState = F_HOLD;
}

void _softResume(){
  isHardHold = false;
}
