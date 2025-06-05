#include <Arduino.h>
#include <Servo.h>
#include <PinChangeInterrupt.h>

// === 핀 정의 ===
#define THROTTLE_IN A0
#define STEER_IN A1
#define THROTTLE_OUT 9
#define STEER_OUT 6
#define LED_RED 3
#define LED_GREEN 5
#define CH7_PIN 2        // 모드 전환용 PWM 핀
#define CH6_PIN 4        // 서보 작동용 PWM 핀
#define AUX_SERVO_PIN 10 // 추가 서보 핀

// 상태 저장 변수
bool isStopped = false; // 정지 상태 저장
bool isTurningLeft = false;
bool isTurningRight = false;

// === 객체 및 변수 ===
Servo esc;
Servo steer_servo;
Servo aux_servo; // 추가 서보

// CH7 관련
volatile unsigned long ch7_start = 0;
volatile uint16_t ch7_pulse = 1500;
volatile bool new_ch7 = false;
bool mode = false; // false: 수동, true: 자동

// CH6 관련
volatile unsigned long ch6_start = 0;
volatile uint16_t ch6_pulse = 1500;
volatile bool new_ch6 = false;

// 시리얼 수신 처리용
String inputString = "";
bool stringComplete = false;

int last_angle = 90;

void ch7_ISR();
void ch6_ISR();
//int aut_mode_flag = 0;
void setup()
{
    Serial.begin(9600);
    esc.attach(THROTTLE_OUT);
    steer_servo.attach(STEER_OUT);
    aux_servo.attach(AUX_SERVO_PIN); // aux 서보는 핀 10번에 연결

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(CH7_PIN, INPUT_PULLUP);
    pinMode(CH6_PIN, INPUT_PULLUP);

    // === 초기화 ===
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);

    esc.writeMicroseconds(1500);
    steer_servo.write(90);
    aux_servo.write(0); // 초기값 중립
    delay(2000);

    attachPCINT(digitalPinToPCINT(CH7_PIN), ch7_ISR, CHANGE);
    attachPCINT(digitalPinToPCINT(CH6_PIN), ch6_ISR, CHANGE);

    inputString.reserve(20);
}

void loop()
{
    // === CH7 모드 갱신 ===
    if (new_ch7)
    {
        new_ch7 = false;
        mode = (ch7_pulse > 1500); // true: 자동, false: 수동
    }

    // === CH6 서보 제어 ===
    if (new_ch6)
    {
        new_ch6 = false;
        if (ch6_pulse > 1500)
        {
            aux_servo.write(180); // ON 상태
        }
        else
        {
            aux_servo.write(0); // OFF 상태
        }
    }

    // === 자동 모드 ===
    if (mode)
    {

        while (Serial.available())
        {
            char inChar = (char)Serial.read();
            if (inChar == '\n')
                stringComplete = true;
            else
                inputString += inChar;
        }

        if (stringComplete)
        {
            int steer_val = inputString.toInt();
            steer_val = constrain(steer_val, -50, 50);
            int servo_pos = constrain(90 + steer_val, 40, 140);
            last_angle = servo_pos;
            steer_servo.write(servo_pos);

            if (servo_pos < 90 - 30 || servo_pos > 90 + 30)
            {

                esc.writeMicroseconds(1553); // 전진 빠르게
            }
            else
            {
                esc.writeMicroseconds(1555); // 정지 상태
            }

            inputString = "";
            stringComplete = false;

            int angle = servo_pos;

            if (angle > 93)
            {
                isTurningLeft = true;
                isTurningRight = false;
            }
            else if (angle < 83)
            {
                isTurningRight = true;
                isTurningLeft = false;
            }
            else
            {
                isTurningLeft = false;
                isTurningRight = false;
            }
            // === LED 출력 (자동 모드 회전 방향 기준)
            if (isTurningLeft)
            {
                digitalWrite(LED_RED, 1);
                digitalWrite(LED_GREEN, 0);
            }
            else if (isTurningRight)
            {
                digitalWrite(LED_RED, 0);
                digitalWrite(LED_GREEN, 1);
            }
            else
            {
                digitalWrite(LED_RED, 0);
                digitalWrite(LED_GREEN, 0);
            }
        }
    }
    else
    {

        // === 수동 모드 ===
        int pwm_throttle = pulseIn(THROTTLE_IN, HIGH, 25000);
        int pwm_steer = pulseIn(STEER_IN, HIGH, 25000);

        int neutral = 1500;
        int reduced_pwm;

        if (pwm_throttle > neutral)
            reduced_pwm = neutral + (pwm_throttle - neutral) / 5;
        else if (pwm_throttle < neutral)
            reduced_pwm = neutral - (neutral - pwm_throttle) / 5;
        else
            reduced_pwm = neutral;

        esc.writeMicroseconds(reduced_pwm);


        int angle = map(pwm_steer, 1000, 2000, 40, 140);
        angle = constrain(angle, 40, 140);
        steer_servo.write(angle);
        last_angle = angle;

        // === LED 제어 ===
        // 이전 상태를 기준으로 조건문 분기
        // === isStopped 판별만 따로 처리 ===
        if (reduced_pwm < 1450)
        {
            isStopped = true;
        }
        else if (reduced_pwm >= 1450)
        {
            isStopped = false;
        }

        // === angle에 따라 회전 방향 판단 (정지 여부와 관계 없이) ===
        if (angle > 93)
        {
            isTurningLeft = true;
            isTurningRight = false;
        }
        else if (angle < 80)
        {
            isTurningRight = true;
            isTurningLeft = false;
        }
        else
        {
            isTurningLeft = false;
            isTurningRight = false;
        }

        // === LED 출력 (정지 + 회전 방향 반영)
        if (isStopped)
        {
            digitalWrite(LED_RED, 1);
            digitalWrite(LED_GREEN, 1);
        }
        else if (isTurningLeft)
        {
            digitalWrite(LED_RED, 1);
            digitalWrite(LED_GREEN, 0);
        }
        else if (isTurningRight)
        {
            digitalWrite(LED_RED, 0);
            digitalWrite(LED_GREEN, 1);
        }
        else
        {
            digitalWrite(LED_RED, 0);
            digitalWrite(LED_GREEN, 0);
        }
    }
} //

// === CH7 PWM 측정 인터럽트 ===
void ch7_ISR()
{
    if (digitalRead(CH7_PIN) == HIGH)
        ch7_start = micros();
    else if (ch7_start)
    {
        ch7_pulse = micros() - ch7_start;
        new_ch7 = true;
        ch7_start = 0;
    }
}

// === CH6 PWM 측정 인터럽트 ===
void ch6_ISR()
{
    if (digitalRead(CH6_PIN) == HIGH)
        ch6_start = micros();
    else if (ch6_start)
    {
        ch6_pulse = micros() - ch6_start;
        new_ch6 = true;
        ch6_start = 0;
    }
}
