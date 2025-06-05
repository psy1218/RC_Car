#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <math.h>

// 수신기 입력 핀
#define CH3_PIN A0  // 제어 값
#define CH5_PIN A1  // On/Off 모드
#define CH6_PIN A2  // 밝기 조절 모드
#define CH9_PIN A3  // 색상 조절 모드

// 출력 핀
#define RED_LED_PIN     3   // On/Off 전용 빨간색 LED
#define YELLOW_LED_PIN  5   // 밝기 조절 전용 노란색 LED
#define RGB_RED_PIN     6   // RGB 모듈_Red
#define RGB_GREEN_PIN   9   // RGB 모듈_Green
#define RGB_BLUE_PIN    10    // RGB 모듈_Blue

// PWM 측정용 변수
volatile unsigned long ch3_start = 0, ch5_start = 0, ch6_start = 0, ch9_start = 0; // PWM 시작 시간
volatile int ch3_pulse = 1500, ch5_pulse = 1000, ch6_pulse = 1000, ch9_pulse = 1000; // PWM 길이
volatile bool new_ch3 = false, new_ch5 = false, new_ch6 = false, new_ch9 = false;   // 새로운 PWM 수신 여부

// 제어 상태 변수
bool led_on = false; // On/Off 모드 상태
int brightness = 255; // 밝기 조절 모드 상태
int hue = 0; // 색상 조절 모드 상태

//채널 Interrupt service routine 함수 선언
void ch3_ISR();
void ch5_ISR();
void ch6_ISR();
void ch9_ISR();
void applyHSVColor(int h, float s, float v); // HSV 색상 변환 함수

void setup() {
  Serial.begin(115200);

  // 채널 입력 핀 모드 설정
  pinMode(CH3_PIN, INPUT_PULLUP);
  pinMode(CH5_PIN, INPUT_PULLUP);
  pinMode(CH6_PIN, INPUT_PULLUP);
  pinMode(CH9_PIN, INPUT_PULLUP);

  //PIN change interrupt attach - CHANGE이용해서 모든 상태 변화 감지
  attachPCINT(digitalPinToPCINT(CH3_PIN), ch3_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(CH5_PIN), ch5_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(CH6_PIN), ch6_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(CH9_PIN), ch9_ISR, CHANGE);

  // 출력 핀 모드 설정 - LED 제어 
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
}

void ch3_ISR() {
  if (digitalRead(CH3_PIN) == HIGH) ch3_start = micros(); //pwm 시작
  else if (ch3_start) { //pwm 종료
    ch3_pulse = micros() - ch3_start; // PWM 길이 측정
    new_ch3 = true; // 새로운 PWM 수신 여부 설정
    ch3_start = 0; // 시작 시간 초기화
  }
}
void ch5_ISR() {
  if (digitalRead(CH5_PIN) == HIGH) ch5_start = micros();
  else if (ch5_start) {
    ch5_pulse = micros() - ch5_start;
    new_ch5 = true;
    ch5_start = 0;
  }
}
void ch6_ISR() {
  if (digitalRead(CH6_PIN) == HIGH) ch6_start = micros();
  else if (ch6_start) {
    ch6_pulse = micros() - ch6_start;
    new_ch6 = true;
    ch6_start = 0;
  }
}
void ch9_ISR() {
  if (digitalRead(CH9_PIN) == HIGH) ch9_start = micros();
  else if (ch9_start) {
    ch9_pulse = micros() - ch9_start;
    new_ch9 = true;
    ch9_start = 0;
  }
}

void loop() {
  if (new_ch3) { // 새로운 PWM 수신 여부 확인 - 채널3으로 제어어
    int val = ch3_pulse; // PWM 값 저장

    // On/Off 제어
    if (ch5_pulse > 1500) { //pwm 길이가 1500보다 크면 On/Off 모드
      led_on = (val > 1500); //채널 3의 PWM 값에 따라 LED ON/OFF 결정
      digitalWrite(RED_LED_PIN, led_on ? HIGH : LOW); // LED 제어
      Serial.print("[On/Off] PWM:"); Serial.print(val);
      Serial.print(" LED:"); Serial.println(led_on ? "ON" : "OFF");
    } else {
      digitalWrite(RED_LED_PIN, LOW); // 모드 OFF 시 LED OFF
    }

    // 밝기 조절
    if (ch6_pulse > 1500) { //pwm 길이가 1500보다 크면 밝기 조절 모드
      brightness = constrain(map(val, 1000, 2000, 0, 255), 0, 255); //채널 3의 PWM 값에 따라 밝기 조절
      analogWrite(YELLOW_LED_PIN, brightness); // 노란색 LED 밝기 조절
      Serial.print("[Brightness] PWM:"); Serial.print(val);
      Serial.print(" Brightness:"); Serial.println(brightness);
    } else {
      analogWrite(YELLOW_LED_PIN, 0); // 모드 OFF 시 밝기 0
    }

    // 색상 조절
    if (ch9_pulse > 1500) { //pwm 길이가 1500보다 크면 색상 조절 모드
      hue = constrain(map(val, 1000, 2000, 0, 360), 0, 360); //채널 3의 PWM 값에 따라 색상 조절
      applyHSVColor(hue, 1.0, 1.0); // RGB LED 색상 조절 - hue(색상), saturation(채도), value(명도) 
      Serial.print("[Color] PWM:"); Serial.print(val);
      Serial.print(" Hue:"); Serial.println(hue);
    } else {
      // RGB LED OFF
      analogWrite(RGB_RED_PIN, 0);
      analogWrite(RGB_GREEN_PIN, 0);
      analogWrite(RGB_BLUE_PIN, 0);
    }

    new_ch3 = false; // 새로운 PWM 수신 여부 초기화
  }
}

//HSV 색상 변환 함수
void applyHSVColor(int h, float s, float v) {
  float c = v * s; // Chroma
  float x = c * (1 - fabs(fmod(h / 60.0, 2) - 1)); // 보조 색상
  float m = v - c; // Match value
  // RGB 색상 값 초기화
  float r = 0, g = 0, b = 0;

  if (h < 60)      { r = c; g = x; b = 0; } //빨간색
  else if (h < 120){ r = x; g = c; b = 0; } //초록색
  else if (h < 180){ r = 0; g = c; b = x; } //파란색
  else if (h < 240){ r = 0; g = x; b = c; } //청록색
  else if (h < 300){ r = x; g = 0; b = c; } //자홍색
  else             { r = c; g = 0; b = x; } //노란색

  // RGB 색상 값 조정
  analogWrite(RGB_RED_PIN,   (r + m) * 255);
  analogWrite(RGB_GREEN_PIN, (g + m) * 255);
  analogWrite(RGB_BLUE_PIN,  (b + m) * 255);
}
