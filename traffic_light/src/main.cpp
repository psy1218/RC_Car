#include <Arduino.h>
#include <TaskScheduler.h>
#include <PinChangeInterrupt.h>

// pin 정의
const int button1 = 2;
const int button2 = 3;
const int button3 = 4;
const int potPin = A0;
const int redLED = 9;
const int yellowLED = 10;
const int greenLED = 11;

// LED 상태 초기화
volatile bool redState = LOW;
volatile bool yellowState = LOW;
volatile bool greenState = LOW;

// LED 상태 변경을 위한 flag
volatile int ledFlag = 0;
// 버튼 flag
volatile int button1flag = 0;
volatile int button2flag = 0;
volatile int button3flag = 0;

// LED 지속 시간  
int RED_TIME    = 2000;
int YELLOW_TIME = 500;
int GREEN_TIME  = 2000;

// led 3번씩 꺼지고 켜지기기
const int GREEN_BLINK = 1000;
const int GREEN_BLINK_INTERVAL = 166;

// LED 깜빡임 주기
const int BLINK = 400;

// 가변저항 값 조절 간격
const int BRIGHTNESS_UPDATE_INTERVAL = 10; 

// LED 상태 - onenable, ondisable
bool redOE();
void redOD();
bool yellowOE();
void yellowOD();
bool greenOE();
void greenOD();
// LED 상태 - LOW 상태로 변경
void reset();

// LED 상태 - Blink
void Toggle();
// 기본동작의 green blink
bool greenBlinkOE();
void greenBlinkCB();
void greenBlinkOD();

// 버튼 ISR 함수
void button1_ISR();
void button2_ISR();
void button3_ISR();

// 가변저항 값 조절
void adjustBrightness();

// send Traffic State
void sendTrafficState();

// 모드 변경
void updateMode(int mode);

// TaskScheduler 객체 생성
Scheduler ts;

// Task 객체 생성
Task red(RED_TIME, TASK_ONCE, NULL, &ts, false, redOE, redOD); // redOE: onenable, redOD: ondisable
Task yellow(YELLOW_TIME, TASK_ONCE, NULL, &ts, false, yellowOE, yellowOD); // yellowOE: onenable, yellowOD: ondisable
Task green(GREEN_TIME, TASK_ONCE, NULL, &ts, false, greenOE, greenOD);   // greenOE: onenable, greenOD: ondisable
Task greenbasicBlink(GREEN_BLINK_INTERVAL, GREEN_BLINK / (GREEN_BLINK_INTERVAL), greenBlinkCB, &ts, false, greenBlinkOE, greenBlinkOD); // greenBlinkOE: onenable, greenBlinkOD: ondisable

// button2 누르면 모든 LED 깜빡임
Task BlinkAll(BLINK, TASK_FOREVER, &Toggle);

// 가변저항 값 조절
Task adjustBrightnessTask(BRIGHTNESS_UPDATE_INTERVAL, TASK_FOREVER, &adjustBrightness, &ts, true);

// 신호등 상태 전송
Task sendTrafficStateTask(50, TASK_FOREVER, &sendTrafficState, &ts, true);


void setup() {
  Serial.begin(9600);

  // pin mode 설정
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);

  pinMode(potPin, INPUT);

  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  // Task 등록
  ts.addTask(BlinkAll);

  // pcint 설정
  attachPCINT(digitalPinToPCINT(button1), button1_ISR, FALLING);
  attachPCINT(digitalPinToPCINT(button2), button2_ISR, FALLING);
  attachPCINT(digitalPinToPCINT(button3), button3_ISR, FALLING);

  //red restart
  red.restartDelayed();
}

void loop() {
  // Task 실행
  ts.execute();
}

// 버튼 1, 2, 3 인터럽트 설정
void button1_ISR() {
  button1flag ^= 1; // 버튼을 누를 때마다 0/1 전환
  Serial.println(button1flag);
  if (button1flag == 1) { //버튼이 눌리면 
    // LED Task 중단
    red.abort();
    yellow.abort();
    green.abort();
    greenbasicBlink.abort();
    BlinkAll.abort();
    
    Serial.println("abort");
    //emergency mode
    redState = HIGH;
    yellowState = LOW;
    greenState = LOW;
  } 
  else {
    reset();
    BlinkAll.abort(); // 모든 LED 깜빡임 중지 (button2 눌린 상태일 때를 방지)
    Serial.println("restart");
    red.restartDelayed(); //기본동작 재시작 
  }
}

void button2_ISR() {
  button2flag ^= 1;
  Serial.println(button2flag); 
  if(button2flag == 1){ //button2 눌렸을 때 
    // LED Task 중단
    red.abort();
    yellow.abort();
    green.abort();
    greenbasicBlink.abort();

    reset();
    // 모든 LED 깜빡임 시작
    BlinkAll.enable();
  }
  else{
    reset();
    BlinkAll.abort();
    red.restartDelayed(); //기본동작 재시작 
    Serial.println("restart");
  }
}

void button3_ISR() {
  button3flag ^= 1;
  if(button3flag == 1){ //button3 눌렸을 때
    // LED Task 중단
    red.abort();
    yellow.abort();
    green.abort();
    greenbasicBlink.abort();
    BlinkAll.abort();

    reset();
  }
  else{
    reset();
    BlinkAll.abort();
    red.restartDelayed(); //기본동작 재시작
    Serial.println("restart");
  }
}

// LED 밝기 조절 함수
void adjustBrightness() {
  int sensorValue = analogRead(potPin); // 가변저항 값 읽기
  int brightness = map(sensorValue, 0, 1023, 0, 255); // 0~1023 → 0~255로 변환

  // LED 밝기 조절: 거짓이면 0, 참이면 brightness 값
  analogWrite(redLED, redState == HIGH ? brightness : 0);
  analogWrite(yellowLED, yellowState == HIGH ? brightness : 0);
  analogWrite(greenLED, greenState == HIGH ? brightness : 0);
}

// 신호등 상태 전송
void sendTrafficState() {
  int sensorValue = analogRead(potPin);
  int brightness = map(sensorValue, 0, 1023, 0, 255);
  
  // 신호등 상태 전송[redState, yellowState, greenState, brightness, mode]
  Serial.print(redState);
  Serial.print(",");
  Serial.print(yellowState);
  Serial.print(",");
  Serial.print(greenState);
  Serial.print(",");
  Serial.print(brightness);
  Serial.print(",");
  Serial.println(button1flag + 2 * button2flag + 3 * button3flag);
}

// 시리얼 통신 이벤트 처리
void serialEvent() {
  if (Serial.available()) { // 데이터가 수신되면
    String data = Serial.readStringUntil('\n');   // 줄바꿈 문자까지 읽기
    data.trim(); // 앞뒤 공백 제거

    // 모드 변경 처리 (p5.js → Arduino)
    if (data.startsWith("mode:")) { // "mode:"로 시작하는 경우
      int newMode = data.substring(5).toInt(); // "mode:1" → 1로 변환
      if (newMode >= 0 && newMode <= 3) { // 0~3 사이인 경우
        button1flag = (newMode == 1); // 버튼1 flag 설정
        button2flag = (newMode == 2); // 버튼2 flag 설정
        button3flag = (newMode == 3); // 버튼3 flag 설정
        updateMode(newMode); // 모드 변경 처리
      }
    } 
    else {
      // 기존 슬라이더 데이터 처리
      int times[3]; // 시간 배열
      int index = 0; 
      char *token = strtok(const_cast<char *>(data.c_str()), ","); // 쉼표로 분리
      while (token != NULL && index < 3) { // 쉼표로 분리한 값이 있고 3개 이하인 경우
        times[index++] = atoi(token); // 정수로 변환하여 배열에 저장
        token = strtok(NULL, ",");    // 다음 토큰 읽기
      }
      if (index == 3) { // 3개의 값이 있는 경우
        RED_TIME = times[0];    // 기본동작 red 시간 설정
        YELLOW_TIME = times[1]; // 기본동작 yellow 시간 설정
        GREEN_TIME = times[2]; // 기본동작 green 시간 설정 
        red.setInterval(RED_TIME); // red Task 시간 설정
        yellow.setInterval(YELLOW_TIME); // yellow Task 시간 설정
        green.setInterval(GREEN_TIME); // green Task 시간 설정
      }
    }
  }
}

// 모드 변경 시 LED 상태 업데이트 (p5.js에서 받은 모드 적용)
void updateMode(int mode) {
  // 기존 실행 중인 Task 중단
  red.abort();
  yellow.abort();
  green.abort();
  greenbasicBlink.abort();
  BlinkAll.abort();

  reset();
  // 모드에 따라 LED 상태 변경
  switch (mode) {
    case 1:
      redState = HIGH;
      break;
    case 2:
      BlinkAll.enable();
      break;
    case 3:
      // 모든 LED를 꺼두고 대기
      break;
    default:
      red.restartDelayed();
      break;
  }
}

// LED 제어 함수
bool redOE() {
  redState = HIGH;
  Serial.println("red on");
  return true;
}

void redOD() {
  redState = LOW;
  ledFlag = 1;
  Serial.println("red off");

  yellow.restartDelayed(); // red 끝나고 yellow Task 시작
}

bool yellowOE() {
  yellowState = HIGH;
  Serial.println("yellow on");
  return true;
}

void yellowOD() {
  yellowState = LOW;
  Serial.println("yellow off");
  if(ledFlag==1){
    green.restartDelayed(); // yellow 끝나고 green Task 시작
  }
  else{
    red.restartDelayed(); // yellow 끝나고 red Task 시작
  }
}

bool greenOE() {
  greenState = HIGH;
  Serial.println("green on");
  return true;
}

void greenOD() {
  greenState = LOW;
  Serial.println("green off");
  ledFlag = 0;
  greenbasicBlink.restartDelayed(); // green 끝나고 greenBlink Task 시작
}


// basic green blink
bool greenBlinkOE() {
  greenState = LOW;
  return true;
}

void greenBlinkCB()  {
  greenState = greenState ^ 1;
}

void greenBlinkOD() {
  greenState = LOW;
  yellow.restartDelayed();
}
// LED Blink
void Toggle(){
  redState = redState ^ 1;
  yellowState = yellowState ^ 1;
  greenState = greenState ^ 1;
}

void reset(){
  redState = LOW;
  yellowState = LOW;
  greenState = LOW;
}